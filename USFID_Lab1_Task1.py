# navigate_waypoints.py
import math, time
from robot_systems.robot import HamBot

# ---------- Robot params ----------
AXLE_L  = 0.184      # m
V_WHEEL_MAX = 0.81   # m/s
V_BODY_MAX  = V_WHEEL_MAX
OMEGA_BODY_MAX = ( V_WHEEL_MAX - (-V_WHEEL_MAX) )/AXLE_L  # 8.804 rad/s

# ---------- Waypoints ----------
WPTS = [
    ( 2.0, -2.0, math.pi),      # P0
    (-1.5, -2.0, math.pi),      # P1
    (-2.0, -1.5, math.pi/2),    # P2  <- arc
    (-2.0, -0.5, math.pi/2),    # P3
    (-1.0, -0.5, 3*math.pi/2),  # P4  <- arc
    (-0.5, -1.0, 7*math.pi/4),  # P5
    ( 2.0, -1.0, 0.0),          # P6
    ( 2.0,  0.0, math.pi/2),    # P7
    ( 0.0,  0.0, math.pi),      # P8
    ( 0.0,  1.0, math.pi/2),    # P9
    (-2.0,  1.0, math.pi),      # P10
    (-1.0,  2.0, 0.0),          # P11
    ( 1.5,  2.0, 0.0),          # P12  <- arc to P13
]

# Hard-code which destinations are arcs
ARC_DESTS = {2, 4, 13}

def wrap_pi(a): return (a + math.pi) % (2*math.pi) - math.pi
def bearing(p,q): return math.atan2(q[1]-p[1], q[0]-p[0])
def dist(p,q): return math.hypot(q[0]-p[0], q[1]-p[1])

def wheel_speeds_from_body(v, omega):
    vr = v + 0.5*AXLE_L*omega
    vl = v - 0.5*AXLE_L*omega
    return vl, vr

# ---------- Motion helpers ----------
def run_spin(bot, dtheta):
    omega = OMEGA_BODY_MAX if dtheta >= 0 else -OMEGA_BODY_MAX
    vl, vr = wheel_speeds_from_body(0.0, omega)
    t = abs(dtheta) / abs(omega)
    bot.run_motors_for_seconds(t, left_speed=vl*100/V_WHEEL_MAX,
                                  right_speed=vr*100/V_WHEEL_MAX)

def run_straight(bot, D):
    v = V_BODY_MAX if D >= 0 else -V_BODY_MAX
    vl, vr = wheel_speeds_from_body(v, 0.0)
    t = abs(D) / abs(v)
    bot.run_motors_for_seconds(t, left_speed=vl*100/V_WHEEL_MAX,
                                  right_speed=vr*100/V_WHEEL_MAX)

def run_arc(bot, R, dtheta):
    """Arc with fixed curvature R until heading changes by dtheta."""
    v = 0.68  # safe body speed so wheels ≤ 0.81
    omega = v / R
    VL, VR = wheel_speeds_from_body(v, omega)
    T = abs(dtheta) / abs(omega)
    print(f"[ARC] R={R:+.2f}, Δθ={dtheta:+.2f}, v={v:.2f}, VL={VL:.2f}, VR={VR:.2f}, T={T:.2f}")
    bot.run_motors_for_seconds(T, left_speed=VL*100/V_WHEEL_MAX,
                                  right_speed=VR*100/V_WHEEL_MAX)

def run_arc_given(bot, VR, VL, T):
    bot.run_motors_for_seconds(T,
        left_speed=VL*100/V_WHEEL_MAX,
        right_speed=VR*100/V_WHEEL_MAX)

# ---------- Main ----------
if __name__ == "__main__":
    bot = HamBot(lidar_enabled=False, camera_enabled=False)
    theta = WPTS[0][2]

    for i in range(1, len(WPTS)):
        p0, p1 = WPTS[i-1], WPTS[i]

        if i in ARC_DESTS and i != 13:
            # Arc movement (P1->P2 and P3->P4)
            dtheta = wrap_pi(p1[2] - theta)
            run_arc(bot, R=-0.50, dtheta=dtheta)
            theta = p1[2]
            continue

        # Normal straight block
        D = dist(p0,p1)
        hdg = bearing(p0,p1)

        dth1 = wrap_pi(hdg - theta)
        if abs(dth1) > 1e-6: run_spin(bot, dth1)

        run_straight(bot, D)

        dth2 = wrap_pi(p1[2] - hdg)
        if abs(dth2) > 1e-6: run_spin(bot, dth2)

        theta = p1[2]

    # Special arc: P12 -> P13
    print("\nSegment P12->P13 (arc with given wheel speeds)")
    run_arc_given(bot, VR=0.24, VL=0.80, T=0.50)

    bot.stop_motors()
    print("Done.")
