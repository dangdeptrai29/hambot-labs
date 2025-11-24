"""
ham_pf_disambiguation.py
Full HamBot + Particle Filter with shared-pattern disambiguation.

- Rotates in SEARCH phase while updating PF from LIDAR
- Moves to neighbor cells only if rotation is insufficient to resolve ambiguity
- Prints PF outputs (list + table, mode, >=80%)
- Stops when localization or goal reached
"""

import time
import random
import math
from collections import Counter
import numpy as np
from robot_systems.robot import HamBot

# ==========================
# Configuration
# ==========================
GOAL_RGB = (245, 0, 124)
GOAL_TOL = 0.25
GOAL_AREA = 400

FRONT_CENTER = 180
FRONT_SECTOR = 20
RIGHT_CENTER = 270
RIGHT_SECTOR = 15

GOAL_REACHED = 250
OBSTACLE_DETECT = 250
TARGET_WALL = 300
WALL_LOST = 1500

SEARCH_SPEED = 20
FORWARD_SPEED = 35
WALL_SPEED = 30
STEER_GAIN = 0.2
LEFT_BIAS = 1.10
TEST_MODE = True

WALL_P = 0.05
WALL_D = 0.02

CAM_WIDTH = 640
CAM_CENTER = 320

LOOP_DT = 0.1
TWO_ROTATIONS = 720

ROWS = 4
COLS = 4
N_PARTICLES = 250

DETECT_THRESH = 400
SECTOR_HALF = 15

# Maze map: (N,E,S,W) signature per cell
MAZE_MAP = {
    (0,0): (1,1,0,0), (0,1): (1,0,1,0), (0,2): (1,0,1,0), (0,3): (1,0,0,1),
    (1,0): (0,1,0,1), (1,1): (1,1,1,0), (1,2): (1,0,1,0), (1,3): (0,0,1,1),
    (2,0): (0,1,0,0), (2,1): (1,0,1,0), (2,2): (1,0,1,1), (2,3): (1,1,0,1),
    (3,0): (0,1,1,0), (3,1): (1,0,1,0), (3,2): (1,0,1,0), (3,3): (0,0,1,1),
}

landmark_cell_ids = {
    (134,244,96): 1,
    (255,7,132): 4,
    (255,189,1): 13,
    (205,222,217): 16
}

# -------------------------------
# LIDAR sector helpers
# -------------------------------
def sector_min(scan, center_angle, half_width):
    start = (center_angle - half_width) % 360
    end = (center_angle + half_width) % 360
    if start <= end:
        sector = scan[start:end+1]
    else:
        sector = scan[start:] + scan[:end+1]
    vals = [d for d in sector if d > 0]
    return min(vals) if vals else None

def get_front_distance(scan):
    return sector_min(scan, FRONT_CENTER, FRONT_SECTOR)

def get_right_distance(scan):
    return sector_min(scan, RIGHT_CENTER, RIGHT_SECTOR)
# ---------------------------------------------------
# Helper: move approximately one cell in a given direction
# ---------------------------------------------------
def move_to_neighbor(bot, direction="forward", distance_mm=600, speed=30):
    """
    Moves HamBot roughly one cell in the specified direction.
    direction: 'forward', 'backward', 'left', 'right'
    distance_mm: how far to move (tune for your cell size)
    speed: motor speed
    """
    # Convert distance to approximate time (depends on your robot speed)
    # Here we assume speed ~ mm/sec, adjust empirically
    duration = distance_mm / speed / 10.0  # rough scaling factor, tune for your robot

    if direction == "forward":
        bot.set_left_motor_speed(speed)
        bot.set_right_motor_speed(speed)
    elif direction == "backward":
        bot.set_left_motor_speed(-speed)
        bot.set_right_motor_speed(-speed)
    elif direction == "left":
        bot.set_left_motor_speed(-speed)
        bot.set_right_motor_speed(speed)
    elif direction == "right":
        bot.set_left_motor_speed(speed)
        bot.set_right_motor_speed(-speed)
    else:
        print("[WARN] Unknown move direction:", direction)
        return

    time.sleep(duration)
    bot.stop_motors()
    time.sleep(0.05)

# -------------------------------
# Particle Filter
# -------------------------------
class ParticleFilter:
    def __init__(self, rows=ROWS, cols=COLS, N=N_PARTICLES):
        self.rows = rows
        self.cols = cols
        self.N = N
        self.particles = []
        per_cell = max(1, self.N // (rows*cols))
        for r in range(rows):
            for c in range(cols):
                for _ in range(per_cell):
                    self.particles.append((r,c))
        while len(self.particles) < self.N:
            self.particles.append((random.randrange(rows), random.randrange(cols)))
        if len(self.particles) > self.N:
            self.particles = self.particles[:self.N]
        # sensor model
        self.p_z0_s0 = 0.6
        self.p_z1_s0 = 0.4
        self.p_z1_s1 = 0.8
        self.p_z0_s1 = 0.2

    def scan_to_abs_obs(self, scan, robot_heading_deg):
        obs = []
        for abs_deg in (0,90,180,270):
            lidar_idx = int((180 + (abs_deg - robot_heading_deg)) % 360)
            m = sector_min(scan, lidar_idx, SECTOR_HALF)
            obs.append(1 if m and m < DETECT_THRESH else 0 if m else None)
        return tuple(obs)

    def single_likelihood(self, obs_bit, s_bit):
        if obs_bit is None: return 1.0
        if s_bit == 0: return self.p_z0_s0 if obs_bit == 0 else self.p_z1_s0
        return self.p_z0_s1 if obs_bit == 0 else self.p_z1_s1

    def update_and_resample(self, scan, robot_heading_deg):
        obs = self.scan_to_abs_obs(scan, robot_heading_deg)
        lik = np.zeros(self.N, dtype=float)
        for i, (r,c) in enumerate(self.particles):
            sig = MAZE_MAP[(r,c)]
            p = 1.0
            for ob_bit, s_bit in zip(obs, sig):
                p *= self.single_likelihood(ob_bit, s_bit)
            lik[i] = p
        s = lik.sum()
        if s == 0 or np.isnan(s):
            self.particles = [(random.randrange(self.rows), random.randrange(self.cols)) for _ in range(self.N)]
            return
        weights = lik / s
        positions = (np.arange(self.N) + random.random()) / self.N
        cumulative = np.cumsum(weights)
        i=j=0
        new_particles=[]
        while i<self.N:
            while positions[i] > cumulative[j]:
                j+=1
            new_particles.append(self.particles[j])
            i+=1
        self.particles = new_particles

    def print_required(self):
        counts = Counter(self.particles)
        list_entries = []
        for idx in range(self.rows*self.cols):
            r = idx // self.cols
            c = idx % self.cols
            cnt = counts[(r,c)]
            list_entries.append(f"cell {idx+1}: {cnt}")
        print("List form: " + ", ".join(list_entries))
        print("\nTable form (rows top->bottom):")
        for r in reversed(range(self.rows)):
            row_str = " ".join(f"{counts[(r,c)]:3d}" for c in range(self.cols))
            print(row_str)
        mode_cell, mode_count = counts.most_common(1)[0]
        mode_idx = mode_cell[0]*self.cols + mode_cell[1] + 1
        print(f"\nMode cell: {mode_cell} (cell {mode_idx}) with {mode_count}/{self.N} particles")
        localized = mode_count >= 0.8*self.N
        print("Localization achieved (>=80%)?", localized)
        print("-"*60)
        return localized

    def percent_in_mode(self):
        counts = Counter(self.particles)
        _, cnt = counts.most_common(1)[0]
        return cnt / self.N
    
# -------------------------------
# Robot class
# -------------------------------
class ParticleFilterRobot:
    def __init__(self):
        self.bot = HamBot(lidar_enabled=True, camera_enabled=True)
        self.cam = self.bot.camera
        self.cam.set_target_colors([GOAL_RGB], tolerance=GOAL_TOL)
        time.sleep(0.5)
        self.last_left_cmd=0
        self.last_right_cmd=0
        self.pf = ParticleFilter()

    def set_motors(self, left, right):
        self.bot.set_left_motor_speed(left)
        self.bot.set_right_motor_speed(right)
        self.last_left_cmd = left
        self.last_right_cmd = right

    def stop_motors(self):
        self.bot.stop_motors()
        self.last_left_cmd = 0
        self.last_right_cmd = 0

    def run(self):
        goal_found=False
        wall_following=False
        rotation_degrees=0.0
        last_heading=None

        try:
            while True:
                scan = self.bot.get_range_image()
                if scan == -1:
                    time.sleep(LOOP_DT)
                    continue

                front_d = get_front_distance(scan)
                right_d = get_right_distance(scan)
                landmarks = self.cam.find_landmarks(min_area=GOAL_AREA)
                goal_visible = len(landmarks)>0

                # ------------------------
                # SEARCH + shared-pattern disambiguation
                # ------------------------
                if not goal_found and not goal_visible and not wall_following:
                    current_heading = self.bot.get_heading()
                    if last_heading is not None:
                        delta = current_heading - last_heading
                        if delta>180: delta-=360
                        if delta<-180: delta+=360
                        rotation_degrees += abs(delta)
                    last_heading = current_heading

                    self.set_motors(-SEARCH_SPEED, SEARCH_SPEED)
                    self.pf.update_and_resample(scan, current_heading)
                    localized = self.pf.print_required()
                    counts = Counter(self.pf.particles)
                    mode_count = counts.most_common(1)[0][1]
                    ambiguous = (mode_count/self.pf.N)<0.8

                    if ambiguous and rotation_degrees >= TWO_ROTATIONS:
                        print("[INFO] Shared pattern detected → moving to neighbor")
                        self.stop_motors()
                        r,c = counts.most_common(1)[0][0]
                        neighbors=[]
                        for dr,dc,dir_name in [(1,0,'up'),(-1,0,'down'),(0,1,'right'),(0,-1,'left')]:
                            nr,nc=r+dr,c+dc
                            if 0<=nr<ROWS and 0<=nc<COLS:
                                if dir_name=='up' and MAZE_MAP[(r,c)][0]==0: neighbors.append((nr,nc,'forward'))
                                if dir_name=='right' and MAZE_MAP[(r,c)][1]==0: neighbors.append((nr,nc,'right'))
                                if dir_name=='down' and MAZE_MAP[(r,c)][2]==0: neighbors.append((nr,nc,'backward'))
                                if dir_name=='left' and MAZE_MAP[(r,c)][3]==0: neighbors.append((nr,nc,'left'))
                        if neighbors:
                            nr,nc,direction = neighbors[0]
                            print(f"[INFO] Moving {direction} to cell ({nr},{nc})")
                            move_to_neighbor(self.bot, direction=direction, distance_mm=600)
                            scan=self.bot.get_range_image()
                            self.pf.update_and_resample(scan, self.bot.get_heading())
                            self.pf.print_required()
                        rotation_degrees=0.0
                        last_heading=None
                    time.sleep(LOOP_DT)
                    continue

                # ------------------------
                # WALL FOLLOW
                # ------------------------
                if wall_following:
                    if goal_visible:
                        print("[FOUND] Goal during wall follow!")
                        wall_following=False
                        goal_found=True
                        self.stop_motors()
                        time.sleep(0.3)
                        continue
                    if not right_d or right_d>WALL_LOST:
                        self.set_motors(-SEARCH_SPEED, SEARCH_SPEED)
                        time.sleep(LOOP_DT)
                        self.stop_motors()
                        time.sleep(0.05)
                        continue
                    if front_d and front_d<300:
                        self.set_motors(-15,15)
                        time.sleep(0.4)
                        self.stop_motors()
                        time.sleep(0.05)
                        continue
                    error=(right_d-TARGET_WALL) if right_d else 0
                    pd=WALL_P*error
                    speed=WALL_SPEED if not front_d or front_d>=600 else WALL_SPEED*max(0.4,front_d/600)
                    left_rpm=np.clip(speed+pd,-75,75)
                    right_rpm=np.clip(speed-pd,-75,75)
                    self.set_motors(left_rpm,right_rpm)
                    time.sleep(LOOP_DT)
                    continue

                # ------------------------
                # GOAL FOUND / APPROACH
                # ------------------------
                if goal_visible and not goal_found:
                    goal_found=True
                    lm=landmarks[0]
                    print(f"[FOUND] Goal detected at pixel ({lm.x},{lm.y}) size {lm.width}x{lm.height}")
                    self.stop_motors()
                    time.sleep(0.3)
                    continue

                if goal_found:
                    lm=landmarks[0] if landmarks else None
                    if not lm:
                        print("[WARNING] Lost goal while approaching.")
                        self.stop_motors()
                        goal_found=False
                        time.sleep(0.5)
                        continue
                    if front_d and front_d<GOAL_REACHED:
                        print("[SUCCESS] GOAL REACHED!")
                        self.stop_motors()
                        break
                    if front_d and front_d<OBSTACLE_DETECT:
                        print(f"[OBSTACLE] Front blocked at {front_d}mm - stopping")
                        self.stop_motors()
                        break
                    pixel_error = lm.x-CAM_CENTER
                    error_normalized = pixel_error/CAM_CENTER
                    if lm.width>120: speed=FORWARD_SPEED*0.6
                    elif lm.width>80: speed=FORWARD_SPEED*0.8
                    else: speed=FORWARD_SPEED
                    if TEST_MODE:
                        left_rpm=speed*LEFT_BIAS
                        right_rpm=speed
                    else:
                        steer_correction=STEER_GAIN*error_normalized
                        left_rpm=speed*(1.0+steer_correction)*LEFT_BIAS
                        right_rpm=speed*(1.0-steer_correction)
                    left_rpm=np.clip(left_rpm,-75,75)
                    right_rpm=np.clip(right_rpm,-75,75)
                    self.set_motors(left_rpm,right_rpm)
                    time.sleep(LOOP_DT)
                    continue

                time.sleep(LOOP_DT)

        except KeyboardInterrupt:
            print("\n[STOP] User interrupted")
        except Exception as e:
            print("\n[ERROR]", e)
            import traceback
            traceback.print_exc()
        finally:
            self.stop_motors()
            try:
                self.cam.stop()
            except Exception:
                pass
            print("\n[SHUTDOWN] Complete\n")

# ------------------------
# Main
# ------------------------
def main():
    robot = ParticleFilterRobot()
    robot.run()

if __name__ == "__main__":
    main()
