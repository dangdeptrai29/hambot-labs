"""Hard-coded HamBot script for the waypoint lap."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Iterable, Tuple

from robot_systems.robot import HamBot

# Robot and actuator limits -------------------------------------------------
AXLE_LENGTH = 0.184           # wheel separation (m)
MAX_WHEEL_SPEED = 0.81        # linear wheel speed at full command (m/s)
CMD_SCALE = 75.0 / MAX_WHEEL_SPEED  # convert m/s wheel targets to BuildHAT RPM

# P0 pose (x, y, theta)
START_POSE: Tuple[float, float, float] = (2.0, -2.0, math.pi)


@dataclass(frozen=True)
class Command:
    """Simple container for a motor command."""

    name: str
    left_speed: float  # m/s at the wheel
    right_speed: float  # m/s at the wheel
    duration: float  # seconds


# Sequence pre-computed from the geometry in the lab description.
COMMANDS: Tuple[Command, ...] = (
    Command("P0→P1 straight", 0.81, 0.81, 4.320987654321),
    Command("P1→P2 arc (R=-0.50 m, Δθ=-π/2)", 0.81, 0.5582432432432434, 1.148038796867381),
    Command("P2→P3 straight", 0.81, 0.81, 1.2345679012345678),
    Command("P3→P4 arc (R=-0.50 m, Δθ=-π)", 0.81, 0.5582432432432434, 2.296077593734762),
    Command("Spin +45° at P4", -0.81, 0.81, 0.08920924163987372),
    Command("P4→P5 straight", 0.81, 0.81, 0.8729713347982069),
    Command("Spin +45° at P5", -0.81, 0.81, 0.08920924163987372),
    Command("P5→P6 straight", 0.81, 0.81, 3.0864197530864197),
    Command("Spin +90° at P6", -0.81, 0.81, 0.17841848327974744),
    Command("P6→P7 straight", 0.81, 0.81, 1.2345679012345678),
    Command("Spin +90° at P7", -0.81, 0.81, 0.17841848327974744),
    Command("P7→P8 straight", 0.81, 0.81, 2.4691358024691357),
    Command("Spin −90° at P8", 0.81, -0.81, 0.17841848327974744),
    Command("P8→P9 straight", 0.81, 0.81, 1.2345679012345678),
    Command("Spin +90° at P9", -0.81, 0.81, 0.17841848327974744),
    Command("P9→P10 straight", 0.81, 0.81, 2.4691358024691357),
    Command("P10→P11 arc (R=-1.00 m, Δθ=-3π/2)", 0.81, 0.6735164835164835, 6.3529984772593595),
    Command("Spin +90° at P11", -0.81, 0.81, 0.17841848327974744),
    Command("P11→P12 straight", 0.81, 0.81, 3.0864197530864197),
    Command("P12→P13 arc (given wheel speeds)", 0.80, 0.24, 0.5),
)


def wrap_pi(angle: float) -> float:
    """Wrap an angle into the (−π, π] range."""

    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def propagate_pose(
    pose: Tuple[float, float, float], left_speed: float, right_speed: float, duration: float
) -> Tuple[float, float, float]:
    """Integrate a differential-drive motion primitive."""

    x, y, theta = pose
    v = 0.5 * (left_speed + right_speed)
    omega = (right_speed - left_speed) / AXLE_LENGTH
    if abs(omega) < 1e-9:
        # Straight line
        x += v * duration * math.cos(theta)
        y += v * duration * math.sin(theta)
    else:
        R = v / omega
        x += R * (math.sin(theta + omega * duration) - math.sin(theta))
        y -= R * (math.cos(theta + omega * duration) - math.cos(theta))
    theta = wrap_pi(theta + omega * duration)
    return x, y, theta


def estimate_final_pose(commands: Iterable[Command]) -> Tuple[float, float, float]:
    pose = START_POSE
    for cmd in commands:
        pose = propagate_pose(pose, cmd.left_speed, cmd.right_speed, cmd.duration)
    return pose


def execute_sequence(bot: HamBot, commands: Iterable[Command]) -> None:
    """Send the hard-coded command sequence to the robot."""

    for index, cmd in enumerate(commands, start=1):
        left_cmd = CMD_SCALE * cmd.left_speed
        right_cmd = CMD_SCALE * cmd.right_speed
        print(
            f"{index:02d}. {cmd.name}: VL={cmd.left_speed:+.3f} m/s, "
            f"VR={cmd.right_speed:+.3f} m/s for {cmd.duration:.3f} s"
        )
        bot.set_left_motor_speed(left_cmd)
        bot.set_right_motor_speed(right_cmd)
        if cmd.duration > 0.0:
            time.sleep(cmd.duration)
        bot.stop_motors()
        time.sleep(0.05)


def main() -> None:
    total_time = sum(cmd.duration for cmd in COMMANDS)
    final_pose = estimate_final_pose(COMMANDS)

    print("\n=== HamBot waypoint run (hard-coded commands) ===")
    print(f"Total scripted motion time ≈ {total_time:.3f} s")
    print(
        f"Estimated final pose P13 ≈ ({final_pose[0]:.4f} m, {final_pose[1]:.4f} m, {final_pose[2]:+.3f} rad)"
    )

    bot = HamBot(lidar_enabled=False, camera_enabled=False)
    try:
        execute_sequence(bot, COMMANDS)
    except KeyboardInterrupt:
        print("\nInterrupted by user. Stopping motors…")
    finally:
        bot.stop_motors()
        if hasattr(bot, "disconnect_robot"):
            bot.disconnect_robot()

    print("\n=== Sequence complete ===\n")


if __name__ == "__main__":
    main()
