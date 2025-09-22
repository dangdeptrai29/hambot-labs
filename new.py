from robot_systems.robot import HamBot
import time
import math
import sys

# === Robot geometry (adjust if yours differs) ===
WHEEL_RADIUS = 0.05   # meters
AXLE_LENGTH = 0.184   # meters (track width)

# Initialize robot
bot = HamBot(lidar_enabled=False, camera_enabled=False)

# ---------- Helpers ----------
def print_encoder_readings(action_name):
    """Print current encoder readings with action context"""
    left_reading, right_reading = bot.get_encoder_readings()
    print(f"{action_name} - Left encoder: {left_reading:.3f} rad, Right encoder: {right_reading:.3f} rad")

def reset_motors_and_encoders():
    """Stop motors, reset encoders, and wait for stabilization"""
    bot.stop_motors()
    time.sleep(0.5)  # Allow motors to fully stop
    bot.reset_encoders()
    time.sleep(0.5)  # Wait for reset to complete
    print("Motors stopped and encoders reset")

def _poll_and_print(prev_L, prev_R, prev_t, label):
    """
    Helper to compute per-wheel velocities and incremental distances from encoders
    and print a compact telemetry line.
    Returns updated (theta_L, theta_R, t) and (vL, vR, dsL, dsR, dt).
    """
    theta_L, theta_R = bot.get_encoder_readings()          # radians
    t = time.monotonic()
    dt = max(1e-6, t - prev_t) #difference between the time and prev

    dtheta_L = theta_L - prev_L
    dtheta_R = theta_R - prev_R

    # Linear wheel velocities from angular velocity * radius
    vL = (dtheta_L / dt) * WHEEL_RADIUS                    # m/s
    vR = (dtheta_R / dt) * WHEEL_RADIUS                    # m/s

    # Incremental linear distances for each wheel
    dsL = dtheta_L * WHEEL_RADIUS                          # m
    dsR = dtheta_R * WHEEL_RADIUS                          # m

    # Report
    avg_s = 0.5 * (dsL + dsR)
    print(f"{label} | vL={vL:+.3f} m/s, vR={vR:+.3f} m/s | Δs_avg={avg_s:+.4f} m over {dt*1000:.0f} ms")

    return (theta_L, theta_R, t), (vL, vR, dsL, dsR, dt)

def settle_and_ramp_to(left_target, right_target, settle_s=0.20, steps=6, step_dt=0.08):
    """
    After a stationary spin, briefly stop to let static friction effects settle,
    then ramp smoothly to the target straight-line commands.
    """
    # brief stop/dwell
    bot.set_left_motor_speed(0)
    bot.set_right_motor_speed(0)
    time.sleep(settle_s)
    # gentle ramp to target
    for i in range(1, steps + 1):
        a = i / steps
        bot.set_left_motor_speed(int(round(left_target * a)))
        bot.set_right_motor_speed(int(round(right_target * a)))
        time.sleep(step_dt)

# ---------- Unified sequencer: ALL phases, no resets between phases ----------
def run_full_sequence(
    speed_outer=50,           # base straight + outer-arc command
    inner_scale_arc=0.5,      # inner = round(outer * scale) during moving arcs
    speed_spin=30,            # speed magnitude used for stationary turns
    poll=0.1
):
    """
    Sequence (all in one flow):
    #changed frequently to fit the maze
    #tested on floor without carpet -> carpet ==> too much friction 
    A) Straight 1.16 m
    B) CW arc 90° or pi/2
    C) Straight 0.33 m
    D) CW arc 180° or pi
    E) Stationary CCW 25°
    F) Straight 0.24 m
    G) Stationary CCW 3°
    H) Straight 0.83 m
    I) Stationary CCW 78°
    J) Straight 0.33 m
    K) Stationary CCW 90°
    L) Straight 0.66 
    M) Stationary CW 90°
    N) Straight 0.33 m
    O) Stationary CCW 90°
    P) Straight 0.66 m
    Q) Stationary CW 90°

    - No encoder resets between phases; we keep integrating ds via prev state.
    - Distances come from encoder deltas (we track each phase’s own totals).
    - For arcs/turns, heading change uses Δψ = (Rw/b) * (Δθ_R - Δθ_L).
    - After each stationary spin (E, G, I, K, M, O), dwell at 0/0 then ramp into the next straight.
    """

    # Start from a known zero, but DO NOT reset until the very end of the whole sequence
    reset_motors_and_encoders()
    print_encoder_readings("Before full sequence")

    # Take a fresh snapshot for integration across phases
    theta_L, theta_R = bot.get_encoder_readings() 
    prev = (theta_L, theta_R, time.monotonic())

    # Convenience for arc motor commands (outer=left for CW forward arcs)
    speed_left_arc = speed_outer  #left wheel will always be the slower one
    speed_right_arc = int(round(speed_outer * inner_scale_arc)) #ratio between inner and outer wheels .5

    # =======================
    # A) Straight 1.16 m
    # =======================
    target_dist = 1.16
    print(f"[A] Straight: target {target_dist:.3f} m at speed {speed_outer}")

    bot.set_left_motor_speed(speed_outer)
    bot.set_right_motor_speed(speed_outer)
    A_sL = A_sR = 0.0
    A_t0 = time.monotonic()

    #segment length for arch
    # how big the arch is quarter circle = pi/2 
    #(distance +- the axil_length/2 )
    # pi would be half a circle 
    # 2pi a circle

    while True:
        time.sleep(poll)
        prev, (_, _, dsL, dsR, _) = _poll_and_print(prev[0], prev[1], prev[2], "[A] Straight")
        A_sL += dsL #total left wheel distance travel
        A_sR += dsR #total right wheel distance travel
        if 0.5 * (A_sL + A_sR) >= target_dist: #the avg of the segment length >= 1.16m
            break

    A_time = time.monotonic() - A_t0 #calculate the time we have traveled so far
    A_dist = 0.5 * (A_sL + A_sR)
    print(f"[A] Done: distance={A_dist:.4f} m (L={A_sL:.4f}, R={A_sR:.4f}), time={A_time:.3f} s")

    # =======================
    # B) CW arc 90°
    # =======================
    target_rad = math.radians(90) #pi/2 the robot rotates quarter of a circle 
    print(f"[B] Arc CW 90°: outer L={speed_left_arc}, inner R={speed_right_arc} (scale={inner_scale_arc:.2f})")
    
    bot.set_left_motor_speed(speed_left_arc) #faster wheel 
    bot.set_right_motor_speed(speed_right_arc) #slower

    #--constructor---
    # Record arc start angles for Δψ
    B_L0, B_R0 = bot.get_encoder_readings()
    B_sL = B_sR = 0.0 #segment/distance traveled 
    B_dpsi = 0.0 # change in rads
    B_t0 = time.monotonic() #start time for section B 
    #end

    while True:
        time.sleep(poll)
        prev_now, (_, _, dsL, dsR, _) = _poll_and_print(prev[0], prev[1], prev[2], "[B] Arc CW")
        prev = prev_now #left wheel 
        B_sL += dsL
        B_sR += dsR

        theta_L_now, theta_R_now = prev[0], prev[1] #left wheel , right wheel 
        B_dpsi = (WHEEL_RADIUS / AXLE_LENGTH) * ((theta_R_now - B_R0) - (theta_L_now - B_L0))
        if abs(B_dpsi) >= target_rad:
            break

    B_time = time.monotonic() - B_t0
    B_dist = 0.5 * (B_sL + B_sR)
    print(f"[B] Done: Δψ={B_dpsi:.3f} rad (~-{target_rad:.3f}), distance={B_dist:.4f} m, time={B_time:.3f} s")

    # =======================
    # C) Straight 0.33 m
    # =======================
    target_dist = 0.33
    print(f"[C] Straight: target {target_dist:.3f} m at speed {speed_outer}")
    bot.set_left_motor_speed(speed_outer)
    bot.set_right_motor_speed(speed_outer)
    C_sL = C_sR = 0.0
    C_t0 = time.monotonic()

    while True:
        time.sleep(poll)
        prev, (_, _, dsL, dsR, _) = _poll_and_print(prev[0], prev[1], prev[2], "[C] Straight")
        C_sL += dsL
        C_sR += dsR
        if 0.5 * (C_sL + C_sR) >= target_dist:
            break

    C_time = time.monotonic() - C_t0
    C_dist = 0.5 * (C_sL + C_sR)
    print(f"[C] Done: distance={C_dist:.4f} m (L={C_sL:.4f}, R={C_sR:.4f}), time={C_time:.3f} s")

    # =======================
    # D) CW arc 180°
    # =======================
    target_rad = math.radians(180)
    print(f"[D] Arc CW 180°: outer L={speed_left_arc}, inner R={speed_right_arc} (scale={inner_scale_arc:.2f})")
    bot.set_left_motor_speed(speed_left_arc)
    bot.set_right_motor_speed(speed_right_arc)
    D_L0, D_R0 = bot.get_encoder_readings()
    D_sL = D_sR = 0.0
    D_dpsi = 0.0
    D_t0 = time.monotonic()

    while True:
        time.sleep(poll)
        prev_now, (_, _, dsL, dsR, _) = _poll_and_print(prev[0], prev[1], prev[2], "[D] Arc CW")
        prev = prev_now
        D_sL += dsL
        D_sR += dsR

        theta_L_now, theta_R_now = prev[0], prev[1]
        D_dpsi = (WHEEL_RADIUS / AXLE_LENGTH) * ((theta_R_now - D_R0) - (theta_L_now - D_L0))
        if abs(D_dpsi) >= target_rad:
            break

    D_time = time.monotonic() - D_t0
    D_dist = 0.5 * (D_sL + D_sR)
    print(f"[D] Done: Δψ={D_dpsi:.3f} rad (~-{target_rad:.3f}), distance={D_dist:.4f} m, time={D_time:.3f} s")

    # =======================
    # E) Stationary CCW 25°
    # =======================
    target_rad = math.radians(25)  # 25 degrees
    print(f"[E] Stationary CCW 25°: spin in place (L=-{speed_spin}, R=+{speed_spin})")
    bot.set_left_motor_speed(-speed_spin)
    bot.set_right_motor_speed(+speed_spin)
    E_L0, E_R0 = bot.get_encoder_readings()
    E_dpsi = 0.0
    E_t0 = time.monotonic()

    while True:
        time.sleep(poll)
        prev_now, (_, _, _, _, _) = _poll_and_print(prev[0], prev[1], prev[2], "[E] Spin CCW")
        prev = prev_now
        theta_L_now, theta_R_now = prev[0], prev[1]
        E_dpsi = (WHEEL_RADIUS / AXLE_LENGTH) * ((theta_R_now - E_R0) - (theta_L_now - E_L0))
        if E_dpsi >= target_rad:
            break

    E_time = time.monotonic() - E_t0
    print(f"[E] Done: Δψ={E_dpsi:.3f} rad (~+{target_rad:.3f}), time={E_time:.3f} s")

    # Apply settle+ramp before starting the next straight
    bot.set_left_motor_speed(0)
    bot.set_right_motor_speed(0)
    time.sleep(0.12)
    settle_and_ramp_to(speed_outer, speed_outer)

    # =======================
    # F) Straight 0.24 m
    # =======================
    target_dist = 0.24
    print(f"[F] Straight: target {target_dist:.3f} m at speed {speed_outer}")
    # Speeds already set by settle_and_ramp_to; just integrate distance.
    F_sL = F_sR = 0.0
    F_t0 = time.monotonic()

    while True:
        time.sleep(poll)
        prev, (_, _, dsL, dsR, _) = _poll_and_print(prev[0], prev[1], prev[2], "[F] Straight")
        F_sL += dsL
        F_sR += dsR
        if 0.5 * (F_sL + F_sR) >= target_dist:
            break

    F_time = time.monotonic() - F_t0
    F_dist = 0.5 * (F_sL + F_sR)
    print(f"[F] Done: distance={F_dist:.4f} m (L={F_sL:.4f}, R={F_sR:.4f}), time={F_time:.3f} s")

    # =======================
    # G) Stationary CCW 3°
    # =======================
    target_rad = math.radians(5)  # 3 -> 5  degrees
    print(f"[G] Stationary CCW 3°: spin in place (L=-{speed_spin}, R=+{speed_spin})")
    bot.set_left_motor_speed(-speed_spin)
    bot.set_right_motor_speed(+speed_spin)
    G_L0, G_R0 = bot.get_encoder_readings()
    G_dpsi = 0.0
    G_t0 = time.monotonic()

    while True:
        time.sleep(poll)
        prev_now, (_, _, _, _, _) = _poll_and_print(prev[0], prev[1], prev[2], "[G] Spin CCW")
        prev = prev_now
        theta_L_now, theta_R_now = prev[0], prev[1]
        G_dpsi = (WHEEL_RADIUS / AXLE_LENGTH) * ((theta_R_now - G_R0) - (theta_L_now - G_L0))
        if G_dpsi >= target_rad:
            break

    G_time = time.monotonic() - G_t0
    print(f"[G] Done: Δψ={G_dpsi:.3f} rad (~+{target_rad:.3f}), time={G_time:.3f} s")

    # Apply settle+ramp before the next straight
    bot.set_left_motor_speed(0)
    bot.set_right_motor_speed(0)
    time.sleep(0.12)
    settle_and_ramp_to(speed_outer, speed_outer)

    # =======================
    # H) Straight 0.83 m
    # =======================
    target_dist = 0.83
    print(f"[H] Straight: target {target_dist:.3f} m at speed {speed_outer}")
    # Speeds already set by settle_and_ramp_to; just integrate distance.
    H_sL = H_sR = 0.0
    H_t0 = time.monotonic()

    while True:
        time.sleep(poll)
        prev, (_, _, dsL, dsR, _) = _poll_and_print(prev[0], prev[1], prev[2], "[H] Straight")
        H_sL += dsL
        H_sR += dsR
        if 0.5 * (H_sL + H_sR) >= target_dist:
            break

    H_time = time.monotonic() - H_t0
    H_dist = 0.5 * (H_sL + H_sR)
    print(f"[H] Done: distance={H_dist:.4f} m (L={H_sL:.4f}, R={H_sR:.4f}), time={H_time:.3f} s")

    # =======================
    # I) Stationary CCW 85°
    # =======================
    target_rad = math.radians(78)  # 85 ->80 -> 78 degrees -> 70
    print(f"[I] Stationary CCW 78°: spin in place (L=-{speed_spin}, R=+{speed_spin})")
    bot.set_left_motor_speed(-speed_spin)
    bot.set_right_motor_speed(+speed_spin)
    I_L0, I_R0 = bot.get_encoder_readings()
    I_dpsi = 0.0
    I_t0 = time.monotonic()

    while True:
        time.sleep(poll)
        prev_now, (_, _, _, _, _) = _poll_and_print(prev[0], prev[1], prev[2], "[I] Spin CCW")
        prev = prev_now
        theta_L_now, theta_R_now = prev[0], prev[1]
        I_dpsi = (WHEEL_RADIUS / AXLE_LENGTH) * ((theta_R_now - I_R0) - (theta_L_now - I_L0))
        if I_dpsi >= target_rad:
            break

    I_time = time.monotonic() - I_t0
    print(f"[I] Done: Δψ={I_dpsi:.3f} rad (~+{target_rad:.3f}), time={I_time:.3f} s")

    # Apply settle+ramp before the next straight
    bot.set_left_motor_speed(0)
    bot.set_right_motor_speed(0)
    time.sleep(0.12)
    settle_and_ramp_to(speed_outer, speed_outer)

    # =======================
    # J) Straight 0.33 m
    # =======================
    target_dist = 0.33  #edited
    print(f"[J] Straight: target {target_dist:.3f} m at speed {speed_outer}")
    # Speeds already set by settle_and_ramp_to; just integrate distance.
    J_sL = J_sR = 0.0
    J_t0 = time.monotonic()

    while True:
        time.sleep(poll)
        prev, (_, _, dsL, dsR, _) = _poll_and_print(prev[0], prev[1], prev[2], "[J] Straight")
        J_sL += dsL
        J_sR += dsR
        if 0.5 * (J_sL + J_sR) >= target_dist:
            break

    J_time = time.monotonic() - J_t0
    J_dist = 0.5 * (J_sL + J_sR)
    print(f"[J] Done: distance={J_dist:.4f} m (L={J_sL:.4f}, R={J_sR:.4f}) in {J_time:.3f} s")

    # =======================
    # K) Stationary CCW 90°
    # =======================
    target_rad = math.radians(70) #smaller 90-> 88 -> 85 ->80 ->78 -> 74 -> 70
    print(f"[K] Stationary CCW 90°: spin in place (L=-{speed_spin}, R=+{speed_spin})")
    bot.set_left_motor_speed(-speed_spin)
    bot.set_right_motor_speed(+speed_spin)
    K_L0, K_R0 = bot.get_encoder_readings()
    K_dpsi = 0.0
    K_t0 = time.monotonic()

    while True:
        time.sleep(poll)
        prev_now, (_, _, _, _, _) = _poll_and_print(prev[0], prev[1], prev[2], "[K] Spin CCW")
        prev = prev_now
        theta_L_now, theta_R_now = prev[0], prev[1]
        K_dpsi = (WHEEL_RADIUS / AXLE_LENGTH) * ((theta_R_now - K_R0) - (theta_L_now - K_L0))
        if K_dpsi >= target_rad:
            break

    K_time = time.monotonic() - K_t0
    print(f"[K] Done: Δψ={K_dpsi:.3f} rad (~+{target_rad:.3f}), time={K_time:.3f} s")

    # Apply settle+ramp before the next straight
    bot.set_left_motor_speed(0)
    bot.set_right_motor_speed(0)
    time.sleep(0.12)
    settle_and_ramp_to(speed_outer, speed_outer)

    # =======================
    # L) Straight 0.66 m
    # =======================
    target_dist = 0.66
    print(f"[L] Straight: target {target_dist:.3f} m at speed {speed_outer}")
    L_sL = L_sR = 0.0
    L_t0 = time.monotonic()

    while True:
        time.sleep(poll)
        prev, (_, _, dsL, dsR, _) = _poll_and_print(prev[0], prev[1], prev[2], "[L] Straight")
        L_sL += dsL
        L_sR += dsR
        if 0.5 * (L_sL + L_sR) >= target_dist:
            break

    L_time = time.monotonic() - L_t0
    L_dist = 0.5 * (L_sL + L_sR)
    print(f"[L] Done: distance={L_dist:.4f} m (L={L_sL:.4f}, R={L_sR:.4f}) in {L_time:.3f} s")

    # =======================
    # M) Stationary CW 90°
    # =======================
    target_rad = math.radians(83) # smaller 90->83 ->87
    print(f"[M] Stationary CW 90°: spin in place (L=+{speed_spin}, R=-{speed_spin})")
    bot.set_left_motor_speed(+speed_spin)
    bot.set_right_motor_speed(-speed_spin)
    M_L0, M_R0 = bot.get_encoder_readings()
    M_dpsi = 0.0
    M_t0 = time.monotonic()

    while True:
        time.sleep(poll)
        prev_now, (_, _, _, _, _) = _poll_and_print(prev[0], prev[1], prev[2], "[M] Spin CW")
        prev = prev_now
        theta_L_now, theta_R_now = prev[0], prev[1]
        M_dpsi = (WHEEL_RADIUS / AXLE_LENGTH) * ((theta_R_now - M_R0) - (theta_L_now - M_L0))
        if M_dpsi <= -target_rad:   # CW should be negative yaw; stop when magnitude reaches 90°
            break

    M_time = time.monotonic() - M_t0
    print(f"[M] Done: Δψ={M_dpsi:.3f} rad (~-{target_rad:.3f}), time={M_time:.3f} s")

    # Apply settle+ramp before the next straight
    bot.set_left_motor_speed(0)
    bot.set_right_motor_speed(0)
    time.sleep(0.12)
    settle_and_ramp_to(speed_outer, speed_outer)

    # =======================
    # N) Straight 0.33 m
    # =======================
    target_dist = 0.33
    print(f"[N] Straight: target {target_dist:.3f} m at speed {speed_outer}")
    N_sL = N_sR = 0.0
    N_t0 = time.monotonic()

    while True:
        time.sleep(poll)
        prev, (_, _, dsL, dsR, _) = _poll_and_print(prev[0], prev[1], prev[2], "[N] Straight")
        N_sL += dsL
        N_sR += dsR
        if 0.5 * (N_sL + N_sR) >= target_dist:
            break

    N_time = time.monotonic() - N_t0
    N_dist = 0.5 * (N_sL + N_sR)
    print(f"[N] Done: distance={N_dist:.4f} m (L={N_sL:.4f}, R={N_sR:.4f}) in {N_time:.3f} s")

    # =======================
    # O) Stationary CCW 90°
    # =======================
    target_rad = math.radians(88)  #90 -> 80 edited
    print(f"[O] Stationary CCW 90°: spin in place (L=-{speed_spin}, R=+{speed_spin})")
    bot.set_left_motor_speed(-speed_spin)
    bot.set_right_motor_speed(+speed_spin)
    O_L0, O_R0 = bot.get_encoder_readings()
    O_dpsi = 0.0
    O_t0 = time.monotonic()

    while True:
        time.sleep(poll)
        prev_now, (_, _, _, _, _) = _poll_and_print(prev[0], prev[1], prev[2], "[O] Spin CCW")
        prev = prev_now
        theta_L_now, theta_R_now = prev[0], prev[1]
        O_dpsi = (WHEEL_RADIUS / AXLE_LENGTH) * ((theta_R_now - O_R0) - (theta_L_now - O_L0))
        if O_dpsi >= target_rad:
            break

    O_time = time.monotonic() - O_t0
    print(f"[O] Done: Δψ={O_dpsi:.3f} rad (~+{target_rad:.3f}), time={O_time:.3f} s")

    # Apply settle+ramp before the next straight
    bot.set_left_motor_speed(0)
    bot.set_right_motor_speed(0)
    time.sleep(0.12)
    settle_and_ramp_to(speed_outer, speed_outer)

    # =======================
    # P) Straight 0.66 m
    # =======================
    target_dist = 0.66
    print(f"[P] Straight: target {target_dist:.3f} m at speed {speed_outer}")
    P_sL = P_sR = 0.0
    P_t0 = time.monotonic()

    while True:
        time.sleep(poll)
        prev, (_, _, dsL, dsR, _) = _poll_and_print(prev[0], prev[1], prev[2], "[P] Straight")
        P_sL += dsL
        P_sR += dsR
        if 0.5 * (P_sL + P_sR) >= target_dist:
            break

    P_time = time.monotonic() - P_t0
    P_dist = 0.5 * (P_sL + P_sR)
    print(f"[P] Done: distance={P_dist:.4f} m (L={P_sL:.4f}, R={P_sR:.4f}) in {P_time:.3f} s")

    # =======================
    # Q) Stationary CW 90° set facing  up before curve
    # =======================
    target_rad = math.radians(90)
    print(f"[Q] Stationary CW 90°: spin in place (L=+{speed_spin}, R=-{speed_spin})")
    bot.set_left_motor_speed(+speed_spin)
    bot.set_right_motor_speed(-speed_spin)
    Q_L0, Q_R0 = bot.get_encoder_readings()
    Q_dpsi = 0.0
    Q_t0 = time.monotonic()

    while True:
        time.sleep(poll)
        prev_now, (_, _, _, _, _) = _poll_and_print(prev[0], prev[1], prev[2], "[Q] Spin CW")
        prev = prev_now
        theta_L_now, theta_R_now = prev[0], prev[1]
        Q_dpsi = (WHEEL_RADIUS / AXLE_LENGTH) * ((theta_R_now - Q_R0) - (theta_L_now - Q_L0))
        if Q_dpsi <= -target_rad:
            break

    Q_time = time.monotonic() - Q_t0
    print(f"[Q] Done: Δψ={Q_dpsi:.3f} rad (~-{target_rad:.3f}), time={Q_time:.3f} s")

    # =======================
    # R) Stationary CW 180° -> 135 curve
    # =======================
    target_rad = math.radians(135)
    print(f"[R] Arc CW 180°: outer L={speed_left_arc}, inner R={speed_right_arc} (scale={inner_scale_arc:.2f})")
    bot.set_left_motor_speed(speed_left_arc)
    bot.set_right_motor_speed(speed_right_arc)
    R_L0, R_R0 = bot.get_encoder_readings()
    R_sL = R_sR = 0.0
    R_dpsi = 0.0
    R_t0 = time.monotonic()

    while True:
        time.sleep(poll)
        prev_now, (_, _, dsL, dsR, _) = _poll_and_print(prev[0], prev[1], prev[2], "[R] Arc CW")
        prev = prev_now
        R_sL += dsL
        R_sR += dsR

        theta_L_now, theta_R_now = prev[0], prev[1]
        R_dpsi = (WHEEL_RADIUS / AXLE_LENGTH) * ((theta_R_now - R_R0) - (theta_L_now - R_L0))
        if abs(R_dpsi) >= target_rad:
            break

    R_time = time.monotonic() - R_t0
    R_dist = 0.5 * (R_sL + R_sR)
    print(f"[R] Done: Δψ={R_dpsi:.3f} rad (~-{target_rad:.3f}), distance={R_dist:.4f} m, time={R_time:.3f} s")

    # Apply settle+ramp before the next straight
    bot.set_left_motor_speed(0)
    bot.set_right_motor_speed(0)
    time.sleep(0.12)
    settle_and_ramp_to(speed_outer, speed_outer)

    # =======================
    # S) Stationary CCW 20°-> before going next staright
    # =======================
    target_rad = math.radians(13)  # 25 -> 20 -> 13 edited 
    print(f"[S] Stationary CCW 38°: spin in place (L=-{speed_spin}, R=+{speed_spin})")
    bot.set_left_motor_speed(-speed_spin)
    bot.set_right_motor_speed(+speed_spin)
    S_L0, S_R0 = bot.get_encoder_readings()
    S_dpsi = 0.0
    S_t0 = time.monotonic()

    while True:
        time.sleep(poll)
        prev_now, (_, _, _, _, _) = _poll_and_print(prev[0], prev[1], prev[2], "[S] Spin CCW")
        prev = prev_now
        theta_L_now, theta_R_now = prev[0], prev[1]
        S_dpsi = (WHEEL_RADIUS / AXLE_LENGTH) * ((theta_R_now - S_R0) - (theta_L_now - S_L0))
        if S_dpsi >= target_rad:
            break

    S_time = time.monotonic() - S_t0
    print(f"[S] Done: Δψ={S_dpsi:.3f} rad (~+{target_rad:.3f}), time={S_time:.3f} s")

    # Apply settle+ramp before the next straight
    bot.set_left_motor_speed(0)
    bot.set_right_motor_speed(0)
    time.sleep(0.12)
    settle_and_ramp_to(speed_outer, speed_outer)

    # =======================
    # T) Straight 0.66 m
    # =======================
    target_dist = 0.70  #edited
    print(f"[T] Straight 0.66: target {target_dist:.3f} m at speed {speed_outer}")
    T_sL = T_sR = 0.0
    T_t0 = time.monotonic()

    while True:
        time.sleep(poll)
        prev, (_, _, dsL, dsR, _) = _poll_and_print(prev[0], prev[1], prev[2], "[T] Straight")
        T_sL += dsL
        T_sR += dsR
        if 0.5 * (T_sL + T_sR) >= target_dist:
            break

    T_time = time.monotonic() - T_t0
    T_dist = 0.5 * (T_sL + T_sR)
    print(f"[T] Done: distance={T_dist:.4f} m (L={T_sL:.4f}, R={T_sR:.4f}) in {T_time:.3f} s")


    # =======================
    # U) Stationary CW 87.17°
    # =======================
    
    target_rad = math.radians(87.7) # 87.17 -> changed to 87.7 for visualization 
    print(f"[U] Arc CW 87.17: outer L={speed_left_arc}, inner R={speed_right_arc} (scale={inner_scale_arc:.2f})")
    bot.set_left_motor_speed(speed_left_arc)
    bot.set_right_motor_speed(speed_right_arc)
    U_L0, U_R0 = bot.get_encoder_readings()
    U_sL = U_sR = 0.0

    U_dpsi = 0.0 
    U_t0 = time.monotonic()
    while True:
        time.sleep(poll)
        prev_now, (_, _, dsL, dsR, _) = _poll_and_print(prev[0], prev[1], prev[2], "[U] Final arch 87.17")
        prev = prev_now
        U_sL += dsL
        U_sR += dsR

        theta_L_now, theta_R_now = prev[0], prev[1]
        U_dpsi = (WHEEL_RADIUS / AXLE_LENGTH) * ((theta_R_now - U_R0) - (theta_L_now - U_L0))
        if abs(U_dpsi) >= target_rad:
            break

    U_time = time.monotonic() - U_t0
    U_dist = 0.5 * (U_sL + U_sR)
    print(f"[U] Done: Δψ={U_dpsi:.3f} rad (~-{target_rad:.3f}), distance={U_dist:.4f} m, time={U_time:.3f} s")
    
    # print(f"[U] Stationary CW 87.17: spin in place")

    # bot.set_left_motor_speed(0.8)
    # bot.set_right_motor_speed(0.24)
    # angular_v = (0.24 - 0.8)/AXLE_LENGTH
    # print(f"[U] Stationary CW ", angular_v ,"angular speed")
    # result = 0.5 * angular_v
    # print(f"[U] Stationary CW ", result ," rads")

    # degrees = result*(180/math.pi)
    # target_rad = math.radians(degrees)
    # print(f"[U] Stationary CW ", degrees ," degrees")
    



    # Summary (optional)
    print("\n=== SUMMARY ===")
    print (f"[0] 0.00s, 0.00m, Robot Before Starting Maze")
    print(f"[A] Straight 1.16m: {A_dist:.4f} m in {A_time:.3f} s")
    print(f"[B] Arc CW 90°: Δψ={B_dpsi:.3f} rad, {B_dist:.4f} m in {B_time:.3f} s")
    print(f"[C] Straight 0.33m: {C_dist:.4f} m in {C_time:.3f} s")
    print(f"[D] Arc CW 180°: Δψ={D_dpsi:.3f} rad, {D_dist:.4f} m in {D_time:.3f} s")
    print(f"[E] Spin CCW 25°: Δψ={E_dpsi:.3f} rad in {E_time:.3f} s")
    print(f"[F] Straight 0.24m: {F_dist:.4f} m in {F_time:.3f} s")
    print(f"[G] Spin CCW 3°: Δψ={G_dpsi:.3f} rad in {G_time:.3f} s")
    print(f"[H] Straight 0.83m: {H_dist:.4f} m in {H_time:.3f} s")
    print(f"[I] Spin CCW 78°: Δψ={I_dpsi:.3f} rad in {I_time:.3f} s")
    print(f"[J] Straight 0.33m: {J_dist:.4f} m in {J_time:.3f} s")
    print(f"[K] Spin CCW 90°: Δψ={K_dpsi:.3f} rad in {K_time:.3f} s")
    print(f"[L] Straight 0.66m: {L_dist:.4f} m in {L_time:.3f} s")
    print(f"[M] Spin CW 90°: Δψ={M_dpsi:.3f} rad in {M_time:.3f} s")
    print(f"[N] Straight 0.33m: {N_dist:.4f} m in {N_time:.3f} s")
    print(f"[O] Spin CCW 90°: Δψ={O_dpsi:.3f} rad in {O_time:.3f} s")
    print(f"[P] Straight 0.66m: {P_dist:.4f} m in {P_time:.3f} s")
    print(f"[Q] Spin CW 90°: Δψ={Q_dpsi:.3f} rad in {Q_time:.3f} s")
    print(f"[R] Spin CW 180°: Δψ={R_dpsi:.3f} rad in {R_time:.3f} s")
    print(f"[S] Spin CCW 45: Δψ={S_dpsi:.3f} rad in {S_time:.3f} s")
    print(f"[T] Straight 0.66: Δψ={T_dist:.3f} rad in {T_time:.3f} s")
    print(f"[U] Spin CW 87.17°: Δψ={U_dpsi:.3f} rad in {U_time:.3f} s")

def main():
    try:
        reset_motors_and_encoders()
        run_full_sequence(          #run
            speed_outer=50,        # keep outer constant for all moving arcs & straights
            inner_scale_arc=0.5,   # adjust radius of the arcs
            speed_spin=30,         # stationary turn speed
            poll=0.1
        )
        reset_motors_and_encoders()
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        bot.stop_motors()
        print("Shutting down robot...")
        bot.disconnect_robot()
        print("Robot disconnected successfully")

if __name__ == "__main__":
    main()