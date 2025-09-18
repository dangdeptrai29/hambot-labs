# navigate_waypoints.py
import math, time
from robot_systems.robot import HamBot
# from robot_systems.robot import IMU   # (left unused here)

# ---------- Robot params ----------
R_WHEEL = 0.045            # wheel radius (m)
AXLE_L  = 0.184            # axle length (m)
OMEGA_MAX = 18.0           # motor shaft max rad/s (spec)
V_WHEEL_MAX = 0.81         # max wheel linear speed (m/s)

# Body limits at "max"
V_BODY_MAX = V_WHEEL_MAX                     # straight at 0.81 m/s
OMEGA_BODY_MAX = ( V_WHEEL_MAX - (-V_WHEEL_MAX) )/AXLE_L  # ≈ 8.804 rad/s

# ---------- Waypoints (x, y, theta) ----------
WPTS = [
    ( 2.0, -2.0, math.pi),      # P0
    (-1.5, -2.0, math.pi),      # P1
    (-2.0, -1.5, math.pi/2),    # P2
    (-2.0, -0.5, math.pi/2),    # P3
    (-1.0, -0.5, 3*math.pi/2),  # P4
    (-0.5, -1.0, 7*math.pi/4),  # P5
    ( 2.0, -1.0, 0.0),          # P6
    ( 2.0,  0.0, math.pi/2),    # P7
    ( 0.0,  0.0, math.pi),      # P8
    ( 0.0,  1.0, math.pi/2),    # P9
    (-2.0,  1.0, math.pi),      # P10
    (-1.0,  2.0, 0.0),          # P11
    ( 1.5,  2.0, 0.0),          # P12
]

def wrap_pi(a): return (a + math.pi) % (2*math.pi) - math.pi
def bearing(p, q): return math.atan2(q[1]-p[1], q[0]-p[0])
def dist(p, q): return math.hypot(q[0]-p[0], q[1]-p[1])

def wheel_speeds_from_body(v, omega):
    """
    Diff-drive inverse kinematics:
      v     = (VR + VL)/2
      omega = (VR - VL)/L
    Solve:
      VR = v + (L/2)*omega
      VL = v - (L/2)*omega
    """
    vr = v + 0.5*AXLE_L*omega
    vl = v - 0.5*AXLE_L*omega
    return vl, vr

# ---------- Motion executors ----------
def run_spin(bot, dtheta):
    """Turn in place by dtheta radians using max yaw rate."""
    omega = OMEGA_BODY_MAX if dtheta >= 0 else -OMEGA_BODY_MAX
    vl, vr = wheel_speeds_from_body(0.0, omega)
    t = abs(dtheta) / abs(omega) if omega != 0 else 0.0
    k = 100.0 / V_WHEEL_MAX
    bot.set_left_motor_speed(k*vl)
    bot.set_right_motor_speed(k*vr)
    t0 = time.time()
    print(f"   [spin] vl={vl:+.3f} m/s, vr={vr:+.3f} m/s, T={t:.3f}s")
    time.sleep(t)
    bot.stop_motors()
    print(f"   [spin] measured T≈{time.time()-t0:.3f}s")

def run_straight(bot, distance):
    """Drive straight by 'distance' meters at ±V_BODY_MAX."""
    v = V_BODY_MAX if distance >= 0 else -V_BODY_MAX
    vl, vr = wheel_speeds_from_body(v, 0.0)
    t = abs(distance) / abs(v) if v != 0 else 0.0
    k = 100.0 / V_WHEEL_MAX
    bot.set_left_motor_speed(k*vl)
    bot.set_right_motor_speed(k*vr)
    t0 = time.time()
    print(f"   [move]  vl={vl:+.3f} m/s, vr={vr:+.3f} m/s, D={distance:.3f} m, T={t:.3f}s")
    time.sleep(t)
    bot.stop_motors()
    print(f"   [move]  measured T≈{time.time()-t0:.3f}s")

def run_arc_by_R_v(bot, R, v, dtheta):
    """
    Constant-curvature arc defined by (R, v) over heading change dtheta.
    Wheel speeds remain constant (VL != VR).
    """
    if abs(R) < 1e-9:
        print("   [arc] |R|≈0 -> using spin instead")
        run_spin(bot, dtheta); return
    omega = v / R
    VL, VR = wheel_speeds_from_body(v, omega)
    # saturate check (informational)
    if max(abs(VL), abs(VR)) > V_WHEEL_MAX + 1e-6:
        print(f"   [arc] WARN: wheel speed exceeds limit ({max(abs(VL),abs(VR)):.3f} > {V_WHEEL_MAX:.3f})")
    T = abs(dtheta) / abs(omega)
    k = 100.0 / V_WHEEL_MAX
    bot.set_left_motor_speed(k*VL)
    bot.set_right_motor_speed(k*VR)
    t0 = time.time()
    D = abs(R * dtheta)
    print(f"   [arc ] R={R:+.3f} m, v={v:.3f} m/s, ω={omega:+.3f} rad/s, Δθ={dtheta:+.3f} rad, D={D:.3f} m, T={T:.3f}s")
    print(f"          VL={VL:+.3f} m/s, VR={VR:+.3f} m/s")
    time.sleep(T)
    bot.stop_motors()
    print(f"   [arc ] measured T≈{time.time()-t0:.3f}s")

def run_arc_by_VR_VL_T(bot, VR, VL, T):
    """
    Constant (VR, VL) during T seconds (used for P12→P13).
    """
    v = 0.5*(VR + VL)
    omega = (VR - VL) / AXLE_L
    R = v / omega if abs(omega) > 1e-9 else float('inf')
    k = 100.0 / V_WHEEL_MAX
    bot.set_left_motor_speed(k*VL)
    bot.set_right_motor_speed(k*VR)
    t0 = time.time()
    print(f"   [arc*] R={R:+.3f} m, v={v:.3f} m/s, ω={omega:+.3f} rad/s, Δθ={omega*T:+.3f} rad, D={v*T:.3f} m, T={T:.3f}s")
    print(f"          VL={VL:+.3f} m/s, VR={VR:+.3f} m/s")
    time.sleep(T)
    bot.stop_motors()
    print(f"   [arc*] measured T≈{time.time()-t0:.3f}s")

# ---------- Planning ----------
# Mark which segments should be true arcs (by index of the DESTINATION wp)
ARC_DESTS = {2, 4, 11, 13}  # P1->P2, P3->P4, P10->P11, P12->P13

# Policy for the arcs P1->P2 and P3->P4 (and tentative P10->P11):
ARC_R_DEFAULT = -0.50  # meters (CW)
def feasible_v_for_R(R, vmax=V_WHEEL_MAX):
    """Return the max body speed v that keeps |VL|,|VR| <= vmax for this R."""
    # VL = v*(1 - L/(2R)), VR = v*(1 + L/(2R))
    a = abs(1 - AXLE_L/(2*R))
    b = abs(1 + AXLE_L/(2*R))
    return min(vmax/a, vmax/b)

def forward_kinematics_arc(x0, y0, th0, R, dtheta):
    """End pose if we travel a constant-curvature arc (R, dtheta)."""
    if abs(R) < 1e-9:
        return x0, y0, wrap_pi(th0 + dtheta)
    x1 = x0 + R*(math.sin(th0 + dtheta) - math.sin(th0))
    y1 = y0 - R*(math.cos(th0 + dtheta) - math.cos(th0))
    th1 = wrap_pi(th0 + dtheta)
    return x1, y1, th1

# Build a unified plan list with tuples describing each segment
plan = []  # each item: dict with keys {kind, src, dst, ... params ...}

theta_curr = WPTS[0][2]
total_D = 0.0
total_T = 0.0

print("\n=== PRE-COMPUTED PLAN (with curvature segments) ===")
for i in range(1, len(WPTS)+1):
    # P12->P13 is a virtual final segment with given wheel speeds
    if i == len(WPTS):  # P12 -> P13
        src = WPTS[-1]; dst = ("P13",)  # symbolic
        kind = "ARC_GIVEN"
        VR, VL, T_given = 0.24, 0.80, 0.50
        v = 0.5*(VR+VL)
        omega = (VR - VL)/AXLE_L
        R = v/omega if abs(omega)>1e-9 else float('inf')
        D_arc = abs(v*T_given)
        dtheta = omega*T_given
        plan.append(dict(kind=kind, i_from=12, i_to=13, src=src, given=(VR,VL,T_given),
                         R=R, v=v, omega=omega, dtheta=dtheta, D=D_arc, T=T_given))
        print(f"{i:>3} | P12->P13 ARC* | R={R:+.3f} m | v={v:.3f} | ω={omega:+.3f} | Δθ={dtheta:+.3f} | D={D_arc:.3f} | T={T_given:.3f}")
        total_D += D_arc
        total_T += T_given
        break

    src = WPTS[i-1]; dst = WPTS[i]
    x0,y0,th0 = src; x1,y1,th1 = dst

    if (i in ARC_DESTS) and (i != 13):  # ARC for P1->P2, P3->P4, P10->P11 (tentative)
        kind = "ARC"
        dtheta = wrap_pi(th1 - th0)
        R = ARC_R_DEFAULT
        v = feasible_v_for_R(R)  # = 0.6841216 for R=-0.5
        omega = v / R
        D_arc = abs(R * dtheta)
        T_arc = abs(dtheta) / abs(omega) if abs(omega)>1e-9 else 0.0

        # Check if this arc actually reaches (x1,y1) from (x0,y0,th0)
        x_end, y_end, _ = forward_kinematics_arc(x0, y0, th0, R, dtheta)
        pos_err = math.hypot((x_end - x1), (y_end - y1))
        feasible = pos_err < 1e-3  # 1 mm tolerance; tighten/loosen as needed

        if not feasible and i == 11:
            # P10->P11 often cannot be a single constant-curvature arc. Fallback to STR.
            print(f"{i:>3} | P10->P11 ARC?  | R={R:+.3f} would miss by {pos_err:.3f} m -> FALLBACK to STRAIGHT planning")
            # Do standard spin-straight-spin timing for the straight chord (your original policy)
            D_line = dist(src, dst)
            psi = bearing(src, dst)
            dth1 = wrap_pi(psi - th0)
            Tspin1 = abs(dth1)/OMEGA_BODY_MAX
            Tmove = D_line/V_BODY_MAX
            dth2 = wrap_pi(th1 - psi)
            Tspin2 = abs(dth2)/OMEGA_BODY_MAX
            plan.append(dict(kind="STR_BLOCK", i_from=i-1, i_to=i, src=src, dst=dst,
                             D=D_line, dth1=dth1, Tspin1=Tspin1, Tmove=Tmove, dth2=dth2, Tspin2=Tspin2))
            print(f"      STR: D={D_line:.3f} | dθ1={dth1:+.3f}->{Tspin1:.3f}s | move={Tmove:.3f}s | dθ2={dth2:+.3f}->{Tspin2:.3f}s")
            total_D += D_line
            total_T += (Tspin1 + Tmove + Tspin2)
            theta_curr = th1
            continue

        plan.append(dict(kind=kind, i_from=i-1, i_to=i, src=src, dst=dst,
                         R=R, v=v, omega=omega, dtheta=dtheta, D=D_arc, T=T_arc))
        print(f"{i:>3} | P{i-1}->P{i} ARC | R={R:+.3f} m | v={v:.3f} | ω={omega:+.3f} | Δθ={dtheta:+.3f} | D={D_arc:.3f} | T={T_arc:.3f}")
        total_D += D_arc
        total_T += T_arc
        theta_curr = th1
        continue

    # Default segments: your original spin→straight→spin planning
    D_line = dist(src, dst)
    psi = bearing(src, dst)
    dth1 = wrap_pi(psi - theta_curr)
    Tspin1 = abs(dth1)/OMEGA_BODY_MAX
    Tmove = D_line/V_BODY_MAX
    dth2 = wrap_pi(th1 - psi)
    Tspin2 = abs(dth2)/OMEGA_BODY_MAX
    plan.append(dict(kind="STR_BLOCK", i_from=i-1, i_to=i, src=src, dst=dst,
                     D=D_line, dth1=dth1, Tspin1=Tspin1, Tmove=Tmove, dth2=dth2, Tspin2=Tspin2))
    print(f"{i:>3} | P{i-1}->P{i} STR | D={D_line:.3f} | dθ1={dth1:+.3f}->{Tspin1:.3f}s | move={Tmove:.3f}s | dθ2={dth2:+.3f}->{Tspin2:.3f}s")
    total_D += D_line
    total_T += (Tspin1 + Tmove + Tspin2)
    theta_curr = th1

print(f"\nTotal planned path length ≈ {total_D:.3f} m")
print(f"Total planned time        ≈ {total_T:.3f} s")

# ---------- Execute ----------
bot = HamBot(lidar_enabled=False, camera_enabled=False)
print("\n=== EXECUTION (with arcs where specified) ===")
for seg in plan:
    if seg["kind"] == "STR_BLOCK":
        i0, i1 = seg["i_from"], seg["i_to"]
        print(f"\nSegment P{i0} -> P{i1} (STRAIGHT block)")
        if abs(seg["dth1"]) > 1e-6: run_spin(bot, seg["dth1"])
        run_straight(bot, seg["D"])
        if abs(seg["dth2"]) > 1e-6: run_spin(bot, seg["dth2"])

    elif seg["kind"] == "ARC":
        i0, i1 = seg["i_from"], seg["i_to"]
        print(f"\nSegment P{i0} -> P{i1} (ARC)")
        run_arc_by_R_v(bot, seg["R"], seg["v"], seg["dtheta"])

    elif seg["kind"] == "ARC_GIVEN":
        print(f"\nSegment P12 -> P13 (ARC with given VR,VL,T)")
        VR, VL, T_given = seg["given"]
        run_arc_by_VR_VL_T(bot, VR, VL, T_given)

bot.stop_motors()
print("\nDone.\n")

# ---------- (Optional) print final P13 pose from the given arc numbers ----------
VR = 0.24; VL = 0.80; T = 0.50
v = 0.5*(VR + VL)
omega = (VR - VL)/AXLE_L
R = v/omega if abs(omega) > 1e-9 else float('inf')
x0, y0, th0 = WPTS[-1]
ICCx = x0 - R*math.sin(th0)
ICCy = y0 + R*math.cos(th0)
theta_travel = omega*T
x_rel = x0 - ICCx; y_rel = y0 - ICCy
x_end = math.cos(theta_travel)*x_rel - math.sin(theta_travel)*y_rel + ICCx
y_end = math.sin(theta_travel)*x_rel + math.cos(theta_travel)*y_rel + ICCy
th_end = wrap_pi(th0 + theta_travel)
print(f"[Check] P13 ≈ ({x_end:.5f}, {y_end:.5f}, {th_end:.3f}) | R={R:+.3f}, ω={omega:+.3f}, v={v:.3f}")
