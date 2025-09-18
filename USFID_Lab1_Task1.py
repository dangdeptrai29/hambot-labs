"""Planner and executor for the HamBot waypoint lap.

The motions are generated from the waypoint geometry so that the robot can
drive every straight at top speed and negotiate the required arcs with the
maximum feasible wheel velocity.  The result is roughly half the runtime of
the previous hard-coded command list while still touching all intermediate
poses.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Iterable, List, Sequence, Tuple

try:  # pragma: no cover - imported when running on the actual robot
    from robot_systems.robot import HamBot
except ImportError:  # pragma: no cover - fallback for local analysis
    class HamBot:  # type: ignore[no-redef]
        def __init__(self, *_: object, **__: object) -> None:
            raise RuntimeError(
                "HamBot hardware library is not available in this environment."
            )

# Robot limits ---------------------------------------------------------------
AXLE_LENGTH = 0.184  # m
MAX_WHEEL_SPEED = 0.81  # m/s (linear speed at the tire)
MAX_BODY_SPIN_RATE = 2.0 * MAX_WHEEL_SPEED / AXLE_LENGTH  # rad/s
RPM_SCALE = 75.0 / MAX_WHEEL_SPEED  # Convert wheel linear speed (m/s) to rpm

# Waypoint list --------------------------------------------------------------


@dataclass(frozen=True)
class Pose:
    """Compact representation of a 2-D pose."""

    x: float
    y: float
    theta: float


@dataclass(frozen=True)
class Waypoint:
    name: str
    pose: Pose


WAYPOINTS: Sequence[Waypoint] = (
    Waypoint("P0", Pose(2.0, -2.0, math.pi)),
    Waypoint("P1", Pose(-1.5, -2.0, math.pi)),
    Waypoint("P2", Pose(-2.0, -1.5, math.pi / 2.0)),
    Waypoint("P3", Pose(-2.0, -0.5, math.pi / 2.0)),
    Waypoint("P4", Pose(-1.0, -0.5, 3.0 * math.pi / 2.0)),
    Waypoint("P5", Pose(-0.5, -1.0, 7.0 * math.pi / 4.0)),
    Waypoint("P6", Pose(2.0, -1.0, 0.0)),
    Waypoint("P7", Pose(2.0, 0.0, math.pi / 2.0)),
    Waypoint("P8", Pose(0.0, 0.0, math.pi)),
    Waypoint("P9", Pose(0.0, 1.0, math.pi / 2.0)),
    Waypoint("P10", Pose(-2.0, 1.0, math.pi)),
    Waypoint("P11", Pose(-1.0, 2.0, 0.0)),
    Waypoint("P12", Pose(1.5, 2.0, 0.0)),
)

# Segments that must be executed as constant-curvature arcs.
ARC_SEGMENTS = {
    ("P1", "P2"),
    ("P3", "P4"),
    ("P10", "P11"),
}

# Optional scripted manoeuvre supplied with the lab hand-out for P12→P13.
FINAL_ARC = (0.80, 0.24, 0.5)  # left_speed, right_speed, duration


@dataclass(frozen=True)
class Command:
    """Container for a single motor command."""

    label: str
    left_speed: float
    right_speed: float
    duration: float


# Math utilities -------------------------------------------------------------


def wrap_pi(angle: float) -> float:
    """Wrap an angle into the (−π, π] interval."""

    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def propagate_pose(pose: Pose, command: Command) -> Pose:
    """Integrate the command to estimate the resulting pose."""

    x, y, theta = pose.x, pose.y, pose.theta
    v = 0.5 * (command.left_speed + command.right_speed)
    omega = (command.right_speed - command.left_speed) / AXLE_LENGTH

    if abs(omega) < 1e-9:
        distance = v * command.duration
        x += distance * math.cos(theta)
        y += distance * math.sin(theta)
    else:
        radius = v / omega
        dtheta = omega * command.duration
        x += radius * (math.sin(theta + dtheta) - math.sin(theta))
        y -= radius * (math.cos(theta + dtheta) - math.cos(theta))
        theta = wrap_pi(theta + dtheta)
    return Pose(x, y, theta)


# Motion primitive builders --------------------------------------------------


def spin(description: str, delta_theta: float) -> Command | None:
    """Return the spin command required for a heading change."""

    delta = wrap_pi(delta_theta)
    if abs(delta) < 1e-6:
        return None
    duration = abs(delta) / MAX_BODY_SPIN_RATE
    direction = math.copysign(1.0, delta)
    left = -direction * MAX_WHEEL_SPEED
    right = direction * MAX_WHEEL_SPEED
    return Command(description, left, right, duration)


def drive_straight(description: str, distance: float) -> Command | None:
    """Return the straight drive command for the supplied distance."""

    if abs(distance) < 1e-6:
        return None
    speed = math.copysign(MAX_WHEEL_SPEED, distance)
    duration = abs(distance) / MAX_WHEEL_SPEED
    return Command(description, speed, speed, duration)


def compute_arc_radius(start: Pose, end: Pose) -> Tuple[float, float]:
    """Solve for the arc radius and heading change between two poses."""

    delta_theta = wrap_pi(end.theta - start.theta)
    if abs(delta_theta) < 1e-6:
        raise ValueError("Arc segment requires a non-zero heading change")

    dx = end.x - start.x
    dy = end.y - start.y
    sin_term = math.sin(start.theta + delta_theta) - math.sin(start.theta)
    cos_term = math.cos(start.theta + delta_theta) - math.cos(start.theta)

    candidates: List[float] = []
    if abs(sin_term) > 1e-9:
        candidates.append(dx / sin_term)
    if abs(cos_term) > 1e-9:
        candidates.append(-dy / cos_term)
    if not candidates:
        raise ValueError("Unable to resolve arc radius from waypoint geometry")
    radius = sum(candidates) / len(candidates)
    return radius, delta_theta


def drive_arc(description: str, start: Pose, end: Pose) -> Command:
    """Return the constant-curvature arc connecting start and end poses."""

    radius, delta_theta = compute_arc_radius(start, end)
    left_radius = radius - AXLE_LENGTH / 2.0
    right_radius = radius + AXLE_LENGTH / 2.0
    max_radius = max(abs(left_radius), abs(right_radius))
    omega = MAX_WHEEL_SPEED / max_radius * math.copysign(1.0, delta_theta)
    left_speed = omega * left_radius
    right_speed = omega * right_radius
    duration = abs(delta_theta / omega)
    return Command(description, left_speed, right_speed, duration)


# Route planner --------------------------------------------------------------


def build_route() -> List[Command]:
    """Compute the full command list for the waypoint run."""

    commands: List[Command] = []
    pose = WAYPOINTS[0].pose

    for start_wp, end_wp in zip(WAYPOINTS, WAYPOINTS[1:]):
        if (start_wp.name, end_wp.name) in ARC_SEGMENTS:
            arc_cmd = drive_arc(
                f"{start_wp.name}→{end_wp.name} arc", pose, end_wp.pose
            )
            commands.append(arc_cmd)
            pose = propagate_pose(pose, arc_cmd)
            continue

        displacement = math.hypot(
            end_wp.pose.x - pose.x, end_wp.pose.y - pose.y
        )
        if displacement > 1e-6:
            travel_heading = math.atan2(
                end_wp.pose.y - pose.y, end_wp.pose.x - pose.x
            )
            spin_cmd = spin(
                f"Spin to face {end_wp.name}", travel_heading - pose.theta
            )
            if spin_cmd is not None:
                commands.append(spin_cmd)
                pose = propagate_pose(pose, spin_cmd)

            straight_cmd = drive_straight(
                f"{start_wp.name}→{end_wp.name} straight", displacement
            )
            if straight_cmd is not None:
                commands.append(straight_cmd)
                pose = propagate_pose(pose, straight_cmd)

        spin_end = spin(
            f"Adjust to {end_wp.name} heading", end_wp.pose.theta - pose.theta
        )
        if spin_end is not None:
            commands.append(spin_end)
            pose = propagate_pose(pose, spin_end)

    # Optional final custom arc to reach P13.
    left_speed, right_speed, duration = FINAL_ARC
    commands.append(Command("P12→P13 arc", left_speed, right_speed, duration))

    return commands


# Robot interface ------------------------------------------------------------


def execute(bot: HamBot, commands: Iterable[Command]) -> None:
    """Stream the motion primitives to HamBot."""

    for index, command in enumerate(commands, start=1):
        left_rpm = command.left_speed * RPM_SCALE
        right_rpm = command.right_speed * RPM_SCALE
        print(
            f"{index:02d}. {command.label}: "
            f"VL={command.left_speed:+.3f} m/s ({left_rpm:+.1f} rpm), "
            f"VR={command.right_speed:+.3f} m/s ({right_rpm:+.1f} rpm) "
            f"for {command.duration:.3f} s"
        )
        bot.set_left_motor_speed(left_rpm)
        bot.set_right_motor_speed(right_rpm)
        if command.duration > 0.0:
            time.sleep(command.duration)
        bot.stop_motors()
        time.sleep(0.05)


def main() -> None:
    commands = build_route()
    total_time = sum(command.duration for command in commands)

    pose = WAYPOINTS[0].pose
    for command in commands:
        pose = propagate_pose(pose, command)

    print("\n=== HamBot waypoint run ===")
    print(f"Total scripted motion time ≈ {total_time:.3f} s")
    print(
        "Estimated final pose ≈ "
        f"({pose.x:.3f} m, {pose.y:.3f} m, {pose.theta:+.3f} rad)"
    )

    bot = HamBot(lidar_enabled=False, camera_enabled=False)
    try:
        execute(bot, commands)
    except KeyboardInterrupt:
        print("\nInterrupted by user. Stopping motors…")
    finally:
        bot.stop_motors()
        if hasattr(bot, "disconnect_robot"):
            bot.disconnect_robot()

    print("\n=== Sequence complete ===\n")


if __name__ == "__main__":
    main()

