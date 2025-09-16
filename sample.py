# navigate_waypoints.py
import math, time
from robot_systems.robot import HamBot  # your provided wrapper around buildhat Motors

# ---------- Robot params (from assignment) ----------
R_WHEEL = 0.05     # m
AXLE_L  = 0.10     # m
OMEGA_MAX = 20.0   # rad/s  (motor)
V_WHEEL_MAX = R_WHEEL * OMEGA_MAX  # 1.0 m/s

# body rates at "max"
V_BODY_MAX = (V_WHEEL_MAX + V_WHEEL_MAX) / 2.0        # straight at 1.0 m/s
OMEGA_BODY_MAX = ( V_WHEEL_MAX - (-V_WHEEL_MAX) )/AXLE_L  # in-place spin at 20 rad/s

# ---------- Waypoints (x, y, theta) ----------
WPTS = [
    ( 2.0, -2.0, math.pi/2),   # P0 (start)
    (-1.5, -2.0, math.pi/2),   # P1
    (-2.0, -1.5, math.pi/2),   # P2
    (-2.0, -0.5, math.pi/2),   # P3
    (-1.0, -0.5, 3*math.pi/2), # P4
    (-0.5, -1.0, 7*math.pi/4), # P5
    ( 2.0, -1.0, 0.0),         # P6
    ( 2.0,  0.0, math.pi/2),   # P7
    ( 0.0,  0.0, math.pi),     # P8
    ( 0.0,  1.0, math.pi/2),   # P9
    (-2.0,  1.0, math.pi),     # P10
    (-1.0,  2.0, 0.0),         # P11
    ( 1.5,  2.0, 0.0),         # P12
]

# ---------- Helpers ----------
def wrap_pi(a):
    return (a + math.pi) % (2*math.pi) - math.pi

def bearing(p, q):
    return math.atan2(q[1]-p[1], q[0]-p[0])

def dist(p, q):
    return math.hypot(q[0]-p[0], q[1]-p[1])

def wheel_speeds_from_body(v, omega):
    # v = (vr+vl)/2 ; omega = (vr - vl)/L  -> solve:
    vr = v + 0.5*AXLE_L*omega
    vl = v - 0.5*AXLE_L*omega
    return vl, vr

def run_spin(bot, dtheta):
    # sign sets direction: CCW positive -> left wheel backward, right wheel forward
    omega = OMEGA_BODY_MAX if dtheta >= 0 else -OMEGA_BODY_MAX
    vl, vr = wheel_speeds_from_body(0.0, omega)
    t = abs(dtheta) / abs(omega)
    # scale to ±100% motor speed (your Motor API uses a percent-like value)
    scale = 100.0 * (abs(vr)/V_WHEEL_MAX)
    # normalize both wheels by the same factor so |vr| maps to 100%
    k = 100.0 / (V_WHEEL_MAX)  # 1.0 -> 100%
    bot.set_left_motor_speed(k*vl)
    bot.set_right_motor_speed(k*vr)

    t0 = time.time()
    try:
        print(f"   [spin] vl={vl:+.2f} m/s, vr={vr:+.2f} m/s, T={t:.3f}s")
    except Exception:
        pass
    time.sleep(t)
    bot.stop_motors()
    print(f"   [spin] measured T≈{time.time()-t0:.3f}s")

def run_straight(bot, distance):
    v = V_BODY_MAX if distance >= 0 else -V_BODY_MAX
    vl, vr = wheel_speeds_from_body(v, 0.0)  # equal wheels
    t = abs(distance) / abs(v) if v != 0 else 0.0
    k = 100.0 / (V_WHEEL_MAX)
    bot.set_left_motor_speed(k*vl)
    bot.set_right_motor_speed(k*vr)

    t0 = time.time()
    print(f"   [move]  vl={vl:+.2f} m/s, vr={vr:+.2f} m/s, D={distance:.3f} m, T={t:.3f}s")
    time.sleep(t)
    bot.stop_motors()
    print(f"   [move]  measured T≈{time.time()-t0:.3f}s")

# ---------- Pre-compute plan ----------
segments = []
theta = WPTS[0][2]
total_D = 0.0
total_T = 0.0

print("\n=== PRE-COMPUTED PLAN (max wheel speed 1.0 m/s) ===")
print("Idx | Segment                | D (m)   | rot→bearing (rad)  Tspin (s) | Tmove (s) | rot→θ_i (rad)  Tspin (s)")
for i in range(1, len(WPTS)):
    p0 = WPTS[i-1]
    p1 = WPTS[i]
    D = dist(p0, p1)
    hdg_to_next = bearing(p0, p1)

    dth1 = wrap_pi(hdg_to_next - theta)              # rotate to face segment
    Tspin1 = abs(dth1) / OMEGA_BODY_MAX

    Tmove = D / V_BODY_MAX                           # straight at 1 m/s

    dth2 = wrap_pi(p1[2] - hdg_to_next)              # rotate to required θ at waypoint
    Tspin2 = abs(dth2) / OMEGA_BODY_MAX

    segment = (D, dth1, Tspin1, Tmove, dth2, Tspin2)
    segments.append(segment)
    total_D += D
    total_T += (Tspin1 + Tmove + Tspin2)

    print(f"{i:>3} | P{i-1}->P{i}  "
          f"| {D:6.3f} | {dth1:+11.6f} -> {Tspin1:6.3f} | {Tmove:7.3f} | {dth2:+11.6f} -> {Tspin2:6.3f}")

    # advance theta so the next leg begins from the waypoint's final heading
    theta = p1[2]

print(f"\nTotal path length D_total = {total_D:.3f} m")
print(f"Total travel time  T_total = {total_T:.3f} s "
      f"(translation {total_D/V_BODY_MAX:.3f}s + rotations ~{total_T - total_D/V_BODY_MAX:.3f}s)")

# ---------- Navigate ----------
bot = HamBot(lidar_enabled=False, camera_enabled=False)
print("\n=== EXECUTION ===")
for i in range(1, len(WPTS)):
    p0 = WPTS[i-1]
    p1 = WPTS[i]
    D, dth1, Tspin1, Tmove, dth2, Tspin2 = segments[i-1]

    print(f"\nSegment P{i-1} -> P{i}")
    # 1) rotate toward bearing
    if abs(dth1) > 1e-6:
        run_spin(bot, dth1)

    # 2) straight move
    run_straight(bot, D)

    # 3) rotate to required final θ_i
    if abs(dth2) > 1e-6:
        run_spin(bot, dth2)

bot.stop_motors()
print("\nDone.\n")

# ---------- REQUIRED: Arc segment P12 -> P13 with given VR, VL, T ----------
print("=== ARC SEGMENT (P12 → P13) — required calculation ===")
# Given:
VR = 0.24  # m/s
VL = 0.80  # m/s
T  = 0.50  # s

v = 0.5 * (VR + VL)                     # body linear speed
omega = (VR - VL) / AXLE_L              # body angular rate (right-handed +)
R = v / omega if abs(omega) > 1e-9 else float('inf')  # curvature radius (signed)

x0, y0, th0 = WPTS[-1]  # P12 pose = (1.5, 2.0, 0)
ICCx = x0 - R * math.sin(th0)
ICCy = y0 + R * math.cos(th0)

theta_travel = omega * T
arc_len = v * T

# endpoint pose after rotating around ICC by theta_travel
x_rel = x0 - ICCx; y_rel = y0 - ICCy
x_end = math.cos(theta_travel)*x_rel - math.sin(theta_travel)*y_rel + ICCx
y_end = math.sin(theta_travel)*x_rel + math.cos(theta_travel)*y_rel + ICCy
th_end = wrap_pi(th0 + theta_travel)

print(f"VR={VR:.2f} m/s, VL={VL:.2f} m/s, T={T:.2f} s")
print(f"ω = (VR−VL)/L = {omega:.3f} rad/s")
print(f"v = (VR+VL)/2 = {v:.3f} m/s")
print(f"R = v/ω = {R:.5f} m (signed; negative => CW turn)")
print(f"ICC = ({ICCx:.5f}, {ICCy:.5f})")
print(f"θ_traveled = ω·T = {theta_travel:.3f} rad")
print(f"Arc distance D = v·T = {arc_len:.3f} m")
print(f"P13 ≈ ({x_end:.5f}, {y_end:.5f}, {th_end:.3f})")
