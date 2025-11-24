"""
ham_pf_complete.py
Full corrected HamBot + Particle Filter (Option C: search-only PF)

Usage: run on the robot. The script:
 - creates HamBot(lidar_enabled=True, camera_enabled=True)
 - rotates in the SEARCH phase while updating the PF from LIDAR
 - prints required outputs after each PF update (list + table, mode, >=80%)
 - stops when the localization condition is met

Author: ChatGPT (adapted to your HamBot API)
"""

import time
import random
import math
from collections import Counter
import numpy as np
from robot_systems.robot import HamBot          

# ==========================
# Configuration (your values)
# ==========================
GOAL_RGB = (245, 0, 124)
GOAL_TOL = 0.25
GOAL_AREA = 400

# LIDAR indexing and sectors (your conventions)
FRONT_CENTER = 180
FRONT_SECTOR = 20
RIGHT_CENTER = 270
RIGHT_SECTOR = 15

# Distances (mm)
GOAL_REACHED = 250
OBSTACLE_DETECT = 250
TARGET_WALL = 300
WALL_LOST = 1500

# Motor speeds / timing
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
GOAL_LOST_THRESHOLD = 10

# Particle filter / map settings
ROWS = 4      # row index 0..3 (0 is bottom)
COLS = 4
N_PARTICLES = 250

# LIDAR detection thresholds (tune if needed)
DETECT_THRESH = 400        # mm; below => observe wall
SECTOR_HALF = 15           # degrees half-width for sector min

# ---------------------------------------------------------------------
# MAZE MAP (4x4) from your description; signature = (N,E,S,W)
# ---------------------------------------------------------------------
MAZE_MAP = {

    (0,0):	(1, 1, 0, 0),
    (0,1):	(1, 0, 1, 0),
    (0,2):	(1, 0, 1, 0),
    (0,3):	(1, 0, 0, 1),
    (1,0):	(0, 1, 0, 1),
    (1,1):	(1, 1, 1, 0),
    (1,2):	(0, 1, 1, 0),
    (1,3):	(0, 0, 1, 1),
    (2,0):	(0, 1, 0, 0),
    (2,1):	(1, 0, 1, 0),
    (2,2):	(1, 0, 1, 1),
    (2,3):	(1, 1, 0, 1),
    (3,0):	(0, 1, 1, 0),
    (3,1):	(1, 0, 1, 0),
    (3,2):	(0, 0, 1, 0),
    (3,3):	(0, 0, 1, 1),

}

landmark_cell_ids = {
    (0, 255, 0): 1,      # Green -> cell #1 (0,0)
    (245, 0, 124): 4,    # Pink -> cell #4 (0,3)
    (255, 165, 0): 13,   # Orange -> cell #13 (3,0)
    (0, 0, 255): 16      # Blue -> cell #16 (3,3)
}

# ---------------------------------------------------------------------
# Helpers: lidar sector mins (adopted from your helpers)
# ---------------------------------------------------------------------
def sense_landmarks(self):
    """
    Returns a list of detected landmark colors (as RGB tuples) that match known landmarks.
    """
    landmarks = self.cam.find_landmarks(min_area=GOAL_AREA)
    if not landmarks:
        return []

    # Grab the current camera frame
    img = self.cam.get_image()
    if img is None:
        return []

    detected_colors = []

    for lm in landmarks:
        # Ensure center is within image bounds
        x = int(np.clip(lm.center_x, 0, img.shape[1]-1))
        y = int(np.clip(lm.center_y, 0, img.shape[0]-1))

        # Sample the pixel color at the center
        rgb = tuple(img[y, x, :3])  # OpenCV uses (row, col)
        
        # Compare against known landmark colors
        for known_rgb in landmark_cell_ids.keys():
            if np.allclose(rgb, known_rgb, atol=10):  # allow small tolerance
                detected_colors.append(known_rgb)
                break

    if detected_colors:
        print("Detected landmarks (RGB matching known landmarks):", detected_colors)
    return detected_colors


# -------------------------------
# The weight update remains the same:
# -------------------------------
def update_weights_with_landmark(particles, w, detected_landmarks):
    """
    Boost weights for particles that are in cells corresponding to detected landmarks
    """
    boost = 5.0
    for color in detected_landmarks:
        if color in landmark_cell_ids:
            target_cell = landmark_cell_ids[color]
            for k, particle in enumerate(particles):
                if particle.cell_id == target_cell:
                    w[k] *= boost
    # Normalize
    if np.sum(w) > 0:
        w /= np.sum(w)
    return w

def sector_min(scan, center_angle, half_width):
    """Return min positive reading in [center-half, center+half] sector, else None."""
    start = int((center_angle - half_width) % 360)
    end = int((center_angle + half_width) % 360)
    if start <= end:
        sector = scan[start:end+1]
    else:
        sector = scan[start:] + scan[:end+1]
    vals = [d for d in sector if d > 0]
    return min(vals) if vals else None

def get_front_distance(scan):
    start = (FRONT_CENTER - FRONT_SECTOR) % 360
    end = (FRONT_CENTER + FRONT_SECTOR) % 360
    if start < end:
        sector = scan[start:end+1]
    else:
        sector = scan[start:] + scan[:end+1]
    valid = [d for d in sector if d > 0]
    return min(valid) if valid else None

def get_right_distance(scan):
    start = (RIGHT_CENTER - RIGHT_SECTOR) % 360
    end = (RIGHT_CENTER + RIGHT_SECTOR) % 360
    if start < end:
        sector = scan[start:end+1]
    else:
        sector = scan[start:] + scan[:end+1]
    valid = [d for d in sector if d > 0]
    return min(valid) if valid else None

# ---------------------------------------------------------------------
# Particle Filter (cells only, no orientation) using exact sensor model
# ---------------------------------------------------------------------
class ParticleFilter:
    def __init__(self, rows=ROWS, cols=COLS, N=N_PARTICLES):
        self.rows = rows
        self.cols = cols
        self.N = N
        # initialize particles evenly across cells
        self.particles = []
        per_cell = max(1, self.N // (self.rows * self.cols))
        for r in range(self.rows):
            for c in range(self.cols):
                for _ in range(per_cell):
                    self.particles.append((r,c))
        while len(self.particles) < self.N:
            self.particles.append((random.randrange(self.rows), random.randrange(self.cols)))
        if len(self.particles) > self.N:
            self.particles = self.particles[:self.N]
        # weights (kept implicitly during update/resample)
        # sensor model probabilities (from you)
        self.p_z0_s0 = 0.6   # p(z=0 | s=0)
        self.p_z1_s0 = 0.4   # p(z=1 | s=0)
        self.p_z1_s1 = 0.8   # p(z=1 | s=1)
        self.p_z0_s1 = 0.2   # p(z=0 | s=1)

    # map a full 360 scan + heading -> absolute N,E,S,W observation bits
    def scan_to_abs_obs(self, scan, robot_heading_deg, detect_thresh=DETECT_THRESH, sector_half=SECTOR_HALF):
        # internal helper: sector around lidar index -> min
        def sector_min_at(lidar_idx):
            start = int((lidar_idx - sector_half) % 360)
            end = int((lidar_idx + sector_half) % 360)
            if start <= end:
                sector = scan[start:end+1]
            else:
                sector = scan[start:] + scan[:end+1]
            vals = [d for d in sector if d > 0]
            if not vals:
                return None
            return min(vals)

        obs = []
        # absolute directions: N=0°, E=90°, S=180°, W=270°
        for abs_deg in (0, 90, 180, 270):
            # convert absolute world dir to LIDAR index given your lidar convention (180=front)
            lidar_idx = int((180 + (abs_deg - robot_heading_deg)) % 360)
            m = sector_min_at(lidar_idx)
            if m is None:
                obs.append(None)
            else:
                obs.append(1 if m < detect_thresh else 0)
        return tuple(obs)   # (N,E,S,W) bits or None where no returns

    # single-side likelihood p(z | s)
    def single_likelihood(self, obs_bit, s_bit):
        if obs_bit is None:
            return 1.0
        if s_bit == 0:
            return self.p_z0_s0 if obs_bit == 0 else self.p_z1_s0
        else:
            return self.p_z0_s1 if obs_bit == 0 else self.p_z1_s1

    # update weights from scan + heading, then systematic resample
    def update_and_resample(self, scan, robot_heading_deg):
        obs = self.scan_to_abs_obs(scan, robot_heading_deg)
        # compute likelihoods for each particle
        lik = np.zeros(self.N, dtype=float)
        for i, (r,c) in enumerate(self.particles):
            sig = MAZE_MAP[(r,c)]
            p = 1.0
            for ob_bit, s_bit in zip(obs, sig):
                p *= self.single_likelihood(ob_bit, s_bit)
            lik[i] = p
        s = lik.sum()
        if s == 0 or np.isnan(s):
            # degeneracy fallback: reinitialize uniformly (rare)
            self.particles = [(random.randrange(self.rows), random.randrange(self.cols)) for _ in range(self.N)]
            return
        weights = lik / s
        # systematic resample
        positions = (np.arange(self.N) + random.random()) / self.N
        cumulative = np.cumsum(weights)
        i = 0; j = 0
        new_particles = []
        while i < self.N:
            while positions[i] > cumulative[j]:
                j += 1
            new_particles.append(self.particles[j])
            i += 1
        self.particles = new_particles

    # required printouts
    def print_required(self):
        counts = Counter(self.particles)
        # List form: cell 1..rows*cols in row-major order (row 0 bottom)
        list_entries = []
        for idx in range(self.rows * self.cols):
            r = idx // self.cols
            c = idx % self.cols
            cnt = counts[(r,c)]
            list_entries.append(f"cell {idx+1}: {cnt}")
        print("List form: " + ", ".join(list_entries))

        # Table form (recommended) - print top->bottom for human readability
        print("\nTable form (rows top->bottom):")
        for r in reversed(range(self.rows)):
            row_str = " ".join(f"{counts[(r,c)]:3d}" for c in range(self.cols))
            print(row_str)

        # Mode cell and localization check
        mode_cell, mode_count = counts.most_common(1)[0]
        mode_idx = mode_cell[0] * self.cols + mode_cell[1] + 1
        print(f"\nMode cell: {mode_cell} (cell {mode_idx}) with {mode_count}/{self.N} particles")
        localized = (mode_count >= 0.8 * self.N)
        print("Localization achieved (>=80% in one cell)?", localized)
        print("-"*60)
        return localized

    def percent_in_mode(self):
        counts = Counter(self.particles)
        _, cnt = counts.most_common(1)[0]
        return cnt / self.N

# ---------------------------------------------------------------------
# Robot + PF integration (search-only PF)
# ---------------------------------------------------------------------
class ParticleFilterRobot:
    def __init__(self):
        # create bot and camera
        self.bot = HamBot(lidar_enabled=True, camera_enabled=True)
        self.cam = self.bot.camera
        self.cam.set_target_colors([GOAL_RGB], tolerance=GOAL_TOL)
        time.sleep(0.5)

        # store last commanded motor speeds so code can reference them if needed
        self.last_left_cmd = 0
        self.last_right_cmd = 0

        # PF
        self.pf = ParticleFilter(rows=ROWS, cols=COLS, N=N_PARTICLES) if False else ParticleFilter()

    # wrappers to set motors AND remember last command values
    def set_motors(self, left, right):
        self.bot.set_left_motor_speed(left)
        self.bot.set_right_motor_speed(right)
        self.last_left_cmd = left
        self.last_right_cmd = right

    def stop_motors(self):
        self.bot.stop_motors()
        self.last_left_cmd = 0
        self.last_right_cmd = 0

    # main loop (Option C: PF runs during SEARCH spinning)
    def run(self):
        goal_found = False
        wall_following = False
        rotation_degrees = 0.0
        last_heading = None
        goal_seen_ever = False
        goal_lost_count = 0

        try:
            while True:
                scan = self.bot.get_range_image()
                if scan == -1:
                    time.sleep(LOOP_DT)
                    continue

                front_d = get_front_distance(scan)
                right_d = get_right_distance(scan)
                landmarks = self.cam.find_landmarks(min_area=GOAL_AREA)
                goal_visible = len(landmarks) > 0

                # ------------------------
                # PHASE 1: SEARCH FOR GOAL
                # PF updates ONLY while in this phase (Option C)
                # ------------------------
                if not goal_found and not goal_visible and not wall_following:
                    current_heading = self.bot.get_heading()
                    if last_heading is not None:
                        delta = current_heading - last_heading
                        if delta > 180: delta -= 360
                        if delta < -180: delta += 360
                        rotation_degrees += abs(delta)
                    last_heading = current_heading

                    # rotate in place slowly and remember commanded motors
                    self.set_motors(-SEARCH_SPEED, SEARCH_SPEED)

                    # update PF with current scan (no PF motion update in Option C)
                    self.pf.update_and_resample(scan, self.bot.get_heading())

                    # print required outputs
                    localized = self.pf.print_required()
                    if localized:
                        print("[PF] Localization success during SEARCH phase. Stopping.")
                        self.stop_motors()
                        break

                    # If performed two full rotations without finding goal -> wall follow
                    if rotation_degrees >= TWO_ROTATIONS and not goal_visible:
                        print("[SEARCH] Two rotations complete -> switching to wall-follow")
                        wall_following = True
                        rotation_degrees = 0.0
                        last_heading = None
                        self.stop_motors()
                        time.sleep(0.3)
                        continue

                    time.sleep(LOOP_DT)
                    continue

                # ------------------------
                # WALL FOLLOWING PHASE (PF not running here)
                # ------------------------
                if wall_following:
                    if goal_visible:
                        print("[FOUND] Goal during wall follow!")
                        wall_following = False
                        goal_found = True
                        self.stop_motors()
                        time.sleep(0.3)
                        continue

                    if not right_d or right_d > WALL_LOST:
                        # rotate while searching for wall
                        self.set_motors(-SEARCH_SPEED, SEARCH_SPEED)
                        time.sleep(LOOP_DT)
                        self.stop_motors()
                        time.sleep(0.05)
                        continue

                    if front_d and front_d < 300:
                        # turn left ~90 deg
                        print(f"[WALL] Front {front_d:.0f}mm - turning LEFT")
                        self.set_motors(-15, 15)
                        time.sleep(0.4)
                        self.stop_motors()
                        time.sleep(0.05)
                        continue

                    # PD wall-follow
                    error = (right_d - TARGET_WALL) if right_d else 0
                    d_error = 0  # no history in this simple loop
                    pd = WALL_P * error + WALL_D * d_error

                    if front_d and front_d < 600:
                        speed = WALL_SPEED * max(0.4, front_d / 600)
                    else:
                        speed = WALL_SPEED

                    left_rpm = np.clip(speed + pd, -75, 75)
                    right_rpm = np.clip(speed - pd, -75, 75)
                    print(f"[WALL] R={right_d if right_d else 0:.0f}mm F={front_d if front_d else 0}mm L:{left_rpm:.1f} R:{right_rpm:.1f}")
                    self.set_motors(left_rpm, right_rpm)
                    time.sleep(LOOP_DT)
                    continue

                # ------------------------
                # GOAL FOUND / APPROACH PHASE
                # ------------------------
                if goal_visible and not goal_found:
                    goal_found = True
                    lm = landmarks[0]
                    print(f"[FOUND] Goal detected at pixel ({lm.x}, {lm.y}) size {lm.width}x{lm.height}")
                    self.stop_motors()
                    time.sleep(0.3)
                    continue

                if goal_found:
                    lm = landmarks[0] if landmarks else None
                    if not lm:
                        print("[WARNING] Lost goal while approaching.")
                        self.stop_motors()
                        goal_found = False
                        time.sleep(0.5)
                        continue

                    if front_d and front_d < GOAL_REACHED:
                        print("[SUCCESS] GOAL REACHED!")
                        self.stop_motors()
                        break

                    if front_d and front_d < OBSTACLE_DETECT:
                        print(f"[OBSTACLE] Front blocked at {front_d}mm - stopping")
                        self.stop_motors()
                        break

                    pixel_error = lm.x - CAM_CENTER
                    error_normalized = pixel_error / CAM_CENTER

                    if lm.width > 120:
                        speed = FORWARD_SPEED * 0.6
                    elif lm.width > 80:
                        speed = FORWARD_SPEED * 0.8
                    else:
                        speed = FORWARD_SPEED

                    if TEST_MODE:
                        left_rpm = speed * LEFT_BIAS
                        right_rpm = speed
                    else:
                        steer_correction = STEER_GAIN * error_normalized
                        left_rpm = speed * (1.0 + steer_correction) * LEFT_BIAS
                        right_rpm = speed * (1.0 - steer_correction)

                    left_rpm = np.clip(left_rpm, -75, 75)
                    right_rpm = np.clip(right_rpm, -75, 75)
                    self.set_motors(left_rpm, right_rpm)
                    time.sleep(LOOP_DT)
                    continue

                # default small sleep
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

# ---------------------------------------------------------------------
# main entry point
# ---------------------------------------------------------------------
def main():
    robot = ParticleFilterRobot()
    robot.run()

if __name__ == "__main__":
    main()
