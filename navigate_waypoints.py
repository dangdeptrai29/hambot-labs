# navigate_waypoints.py
# Clean, small-room-safe, forward-only navigation for HamBot (RPM-based motors)

import math, time
from robot_systems.robot import HamBot

# ========= 1) Key constants =========
R_WHEEL = 0.045         # [m] wheel radius
AXLE_L  = 0.184         # [m] axle length (wheel separation)
RPM_MAX = 75.0          # [rpm] HamBot.check_speed clamps to ±75

# Hardware top speeds (from RPM_MAX)
V_WHEEL_MAX_HW = (RPM_MAX/60.0) * (2.0*math.pi*R_WHEEL)  # ≈ 0.353 m/s
OMEGA_MAX_HW   = (2.0*V_WHEEL_MAX_HW) / AXLE_L           # rad/s (in-place spin)

# One knob: slow everything (time stretches so geometry is preserved)
SPEED_SCALE = 1.0

# One knob: shrink the whole route for small rooms (distances & radii)
PATH_SCALE = 0.35      # try 0.25–0.50; smaller = shorter route

# ========= 2) Waypoints (original) =========
# These are the *nominal* poses; the runtime geometry will be scaled by PATH_SCALE.
WPTS = [
    ( 2.0, -2.0, math.pi),      # P0
    (-1.5, -2.0, math.pi),      # P1
    (-2.0, -1.5, math.pi/2),    # P2  <- ARC
    (-2.0, -0.5, math.pi/2),    # P3
    (-1.0, -0.5, 3*math.pi/2),  # P4  <- ARC
    (-0.5, -1.0, 7*math.pi/4),  # P5
    ( 2.0, -1.0, 0.0),          # P6
    ( 2.0,  0.0, math.pi/2),    # P7
    ( 0.0,  0.0, math.pi),      # P8
    ( 0.0,  1.0, math.pi/2),    # P9
    (-2.0,  1.0, math.pi),      # P10 <- ARC
    (-1.0,  2.0, 0.0),          # P11
    ( 1.5,  2.0, 0.0),          # P12 <- ARC to P13 (given wheels)
]

# Segments to run as arcs (constant curvature)
ARC_DESTS_FIXED_R = {2, 4, 11}         # P1→P2, P3→P4, P10→P11
ARC_R_FIXED_ABS   = 0.50 * PATH_SCALE  # [m] shrink the radius with the path

# Final required arc P12->P13: GIVEN wheel speeds & time (m/s, s)
GIVEN_P12_P13 = dict(VR=0.24, VL=0.80, T=0.50)

# ========= 3) Tiny math helpers =========
def wrap_pi(a): return (a + math.pi) % (2.0*math.pi) - math.pi
def bearing(p, q): return math.atan2(q[1]-p[1], q[0]-p[0])
def dist(p, q): return math.hypot(q[0]-p[0], q[1]-p[1])
def mps_to_rpm(v): return (v / (2.0*math.pi*R_WHEEL)) * 60.0

# ========= 4) One safe sender: preserves geometry under scaling/limits =========
def run_wheels_for_geom(bot, VL_mps, VR_mps, T_nom):
    """
    Drive wheels at (VL, VR) [m/s] for nominal T_nom seconds, while:
      - Applying SPEED_SCALE uniformly to both wheels,
      - Enforcing ±RPM_MAX by uniform scaling (preserves curvature),
      - Stretching time so distance/angle remain correct.
    """
    # Apply user speed scale
    VL = VL_mps * SPEED_SCALE
    VR = VR_mps * SPEED_SCALE

    # Enforce RPM limit by uniform scale
    peak_rpm = max(abs(mps_to_rpm(VL)), abs(mps_to_rpm(VR)))
    hw_scale = 1.0
    if peak_rpm > RPM_MAX + 1e-9:
        hw_scale = RPM_MAX / peak_rpm
        VL *= hw_scale
        VR *= hw_scale

    # Stretch time so geometry is preserved
    s_total = SPEED_SCALE * hw_scale
    T_eff = T_nom / s_total if s_total > 1e-9 else T_nom

    bot.run_motors_for_seconds(
        T_eff,
        left_speed=mps_to_rpm(VL),     # HamBot flips left internally
        right_speed=mps_to_rpm(VR)
    )

# ========= 5) Motion primitives =========
def do_spin(bot, dtheta):
    """Spin in place by dtheta [rad] using hardware max yaw (forward-safe)."""
    sgn = 1.0 if dtheta >= 0 else -1.0
    T_nom = abs(dtheta) / OMEGA_MAX_HW if OMEGA_MAX_HW > 0 else 0.0
    VR = +sgn * V_WHEEL_MAX_HW
    VL = -sgn * V_WHEEL_MAX_HW
    print(f"[SPIN] Δθ={dtheta:+.3f} rad, T_nom={T_nom:.3f}s")
    run_wheels_for_geom(bot, VL, VR, T_nom)

def do_straight(bot, D):
    """Forward-only straight: distance D [m]; always go forward (D>=0)."""
    D = abs(D) * PATH_SCALE                      # shrink for small room
    v = V_WHEEL_MAX_HW                           # forward
    T_nom = D / v if v > 1e-9 else 0.0
    print(f"[MOVE] D_scaled={D:.3f} m, T_nom={T_nom:.3f}s")
    run_wheels_for_geom(bot, v, v, T_nom)

def feasible_v_for_R(R_abs):
    """Max body speed v so that |VL|,|VR| <= V_WHEEL_MAX_HW for given |R|."""
    a = abs(1.0 - AXLE_L/(2.0*R_abs))
    b = abs(1.0 + AXLE_L/(2.0*R_abs))
    return min(V_WHEEL_MAX_HW/a, V_WHEEL_MAX_HW/b)   # m/s

def do_arc_fixed_R(bot, dtheta, R_abs):
    """
    Forward-only, constant-curvature arc:
      - Use radius |R| (scaled), sign chosen so ω and dθ match,
      - Use the largest forward v allowed by RPM cap,
      - Run for T = |dθ| / |ω|.
    """
    R = math.copysign(R_abs, dtheta)   # sign so ω=v/R has same sign as dθ
    v_max = feasible_v_for_R(R_abs) * 0.98  # small margin
    v = v_max                             # forward (v >= 0)
    omega = v / R
    T_nom = abs(dtheta) / abs(omega) if abs(omega) > 1e-9 else 0.0
    VL = v * (1.0 - AXLE_L/(2.0*R))
    VR = v * (1.0 + AXLE_L/(2.0*R))
    D_arc = abs(R) * abs(dtheta)
    print(f"[ARC ] R={R:+.3f} m, Δθ={dtheta:+.3f} rad, D_scaled≈{D_arc:.3f} m, T_nom={T_nom:.3f}s")
    print(f"       v={v:.3f} m/s, VL={VL:.3f}, VR={VR:.3f}")
    run_wheels_for_geom(bot, VL, VR, T_nom)

def do_arc_given_shrunk(bot, VR, VL, T):
    """
    P12->P13 with given (VR, VL, T), but *shrink distance by PATH_SCALE*
    while keeping the same heading change (Δθ).
    """
    v  = 0.5 * (VR + VL)
    w  = (VR - VL) / AXLE_L
    v2 = PATH_SCALE * v                 # shrink distance
    VR2 = v2 + 0.5*AXLE_L*w
    VL2 = v2 - 0.5*AXLE_L*w
    print(f"[ARC*] given VR={VR:.3f},VL={VL:.3f},T={T:.3f}  -> shrunk VR'={VR2:.3f}, VL'={VL2:.3f}, T'={T:.3f}")
    run_wheels_for_geom(bot, VL2, VR2, T)

# ========= 6) Main: execute the route =========
if __name__ == "__main__":
    bot = HamBot(lidar_enabled=False, camera_enabled=False)
    print(f"\n=== START ===")
    print(f"RPM_MAX=±{RPM_MAX:.0f}, V_WHEEL_MAX_HW={V_WHEEL_MAX_HW:.3f} m/s, "
          f"OMEGA_MAX_HW={OMEGA_MAX_HW:.3f} rad/s, SPEED_SCALE={SPEED_SCALE:.2f}, PATH_SCALE={PATH_SCALE:.2f}")

    # <-- FIX: define theta before using it
    theta = WPTS[0][2]  # assume start at P0 heading

    for i in range(1, len(WPTS)):
        p0, p1 = WPTS[i-1], WPTS[i]
        th0, th1 = p0[2], p1[2]
        print(f"\n-- Segment P{i-1} -> P{i} --")

        # Special case: at P10 (segment P10->P11), spin +90° first, then arc
        if i == 11:
            spin_90 = -math.pi / 2.0   # <-- flip direction: CW 90°
            print("[P10] Pre-arc spin -90° (CW)")
            do_spin(bot, spin_90)
            th0_after = wrap_pi(th0 + spin_90)
            dtheta_rem = wrap_pi(th1 - th0_after)
            print(f"[P10] Remaining Δθ for arc = {dtheta_rem:+.3f} rad")
            do_arc_fixed_R(bot, dtheta_rem, ARC_R_FIXED_ABS)
            theta = th1
            print(f"[DONE] Now at P{i} = ({p1[0]:.2f}, {p1[1]:.2f}, {p1[2]:.2f} rad)")
            continue

        # Regular fixed-radius arcs for the other flagged segments
        if i in ARC_DESTS_FIXED_R:
            dtheta = wrap_pi(th1 - th0)
            do_arc_fixed_R(bot, dtheta, ARC_R_FIXED_ABS)
            theta = th1
            print(f"[DONE] Now at P{i} = ({p1[0]:.2f}, {p1[1]:.2f}, {p1[2]:.2f} rad)")
            continue

        # Otherwise: spin -> forward straight -> spin (with distances scaled)
        hdg = bearing(p0, p1)
        dth1 = wrap_pi(hdg - theta)
        if abs(dth1) > 1e-6: do_spin(bot, dth1)

        D_nom = dist(p0, p1)          # nominal distance before scaling
        do_straight(bot, D_nom)       # applies PATH_SCALE internally

        dth2 = wrap_pi(th1 - hdg)
        if abs(dth2) > 1e-6: do_spin(bot, dth2)

        theta = th1
        print(f"[DONE] Now at P{i} = ({p1[0]:.2f}, {p1[1]:.2f}, {p1[2]:.2f} rad)")

    # Final arc: P12 -> P13 (given wheels) but shrink distance by PATH_SCALE, keep Δθ
    print("\n-- Segment P12 -> P13 (given wheels, distance shrunk) --")
    do_arc_given_shrunk(bot, VR=GIVEN_P12_P13["VR"], VL=GIVEN_P12_P13["VL"], T=GIVEN_P12_P13["T"])

    bot.stop_motors()
    print("\nDone.\n")
