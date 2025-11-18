# task1_trilateration.py
# Lab 4 – Task 1 (HamBot): Trilateration-based localization
# Detect 4 colored landmarks, measure distances, estimate robot position

import time
import math
import numpy as np
from robot_systems.robot import HamBot

# ====== Landmark positions (simulation scale) ======
LANDMARKS = {
    'yellow': (-2.5, 2.5),   # top-left
    'red':    (2.5, 2.5),    # top-right
    'green':  (-2.5, -2.5),  # bottom-left
    'blue':   (2.5, -2.5)    # bottom-right
}

# ====== Landmark RGB colors (averaged measurements) ======
COLORS = {
    'red':    (189, 3, 71),
    'green':  (79, 182, 98),
    'blue':   (109, 184, 206),
    'yellow': (201, 92, 14)
}
COLOR_TOLERANCE = 40  # RGB distance threshold

# ====== Scale factor (reality / simulation) ======
SCALE = 0.38  # 1.9m reality / 5m simulation

# ====== Robot parameters ======
SCAN_RPM = 20.0       # rotation speed for 360° scan
SCAN_DT = 0.05        # sample interval during scan
MM_TO_M = 0.001       # LIDAR mm → meters

# ====== Grid parameters ======
GRID_SIZE = 5         # 5×5 grid
CELL_SIZE = 1.0       # 1m per cell (simulation)

def rgb_distance(c1, c2):
    """Euclidean distance between two RGB colors."""
    return math.sqrt(sum((a - b)**2 for a, b in zip(c1, c2)))

def detect_color(rgb):
    """Match RGB to landmark color name."""
    for name, target_rgb in COLORS.items():
        if rgb_distance(rgb, target_rgb) < COLOR_TOLERANCE:
            return name
    return None

def get_lidar_distance(bot, angle_deg):
    """Get LIDAR distance at specific angle (0-359°)."""
    scan = bot.get_range_image()
    if scan == -1 or len(scan) < 360:
        return None
    
    idx = int(angle_deg) % 360
    dist_mm = scan[idx]
    if dist_mm <= 0 or not math.isfinite(dist_mm):
        return None
    return dist_mm * MM_TO_M

def scan_for_landmarks(bot):
    """
    Rotate 360° and detect landmarks with camera + LIDAR.
    Returns: dict of {color: (angle_deg, distance_m)}
    """
    print("[SCAN] Starting 360° landmark detection...")
    detections = {}
    
    # Start rotation
    bot.set_left_motor_speed(-SCAN_RPM)
    bot.set_right_motor_speed(SCAN_RPM)
    
    start_time = time.time()
    angle = 0.0
    
    try:
        while angle < 360:
            # Get camera frame
            frame = bot.get_image()
            if frame is None:
                time.sleep(SCAN_DT)
                continue
            
            # Check for landmarks in frame
            landmarks = bot.find_landmarks(frame)
            
            for landmark in landmarks:
                # Get average RGB of landmark bounding box
                x, y, w, h = landmark
                if w > 0 and h > 0:
                    roi = frame[y:y+h, x:x+w]
                    avg_rgb = tuple(roi.mean(axis=(0,1)).astype(int))
                    
                    # Match color
                    color = detect_color(avg_rgb)
                    if color and color not in detections:
                        # Get LIDAR distance at current angle
                        dist = get_lidar_distance(bot, angle)
                        if dist and dist > 0.1:  # sanity check
                            detections[color] = (angle, dist)
                            print(f"  Found {color} at {angle:.1f}°, dist={dist:.3f}m")
            
            # Update angle estimate (rough approximation)
            angle += SCAN_RPM * 360 / 60 * SCAN_DT  # rpm → deg/sec → deg
            time.sleep(SCAN_DT)
            
            # Stop if we found at least 3 landmarks
            if len(detections) >= 3 and angle > 180:
                break
    
    finally:
        bot.stop_motors()
    
    print(f"[SCAN] Detected {len(detections)} landmarks: {list(detections.keys())}")
    return detections

def trilaterate(landmark_data):
    """
    Solve for robot position using trilateration.
    Input: dict of {color: (angle_deg, distance_m)}
    Returns: (x, y) in simulation coordinates
    """
    # Build system of equations: (x - x_i)² + (y - y_i)² = r_i²
    points = []
    distances = []
    
    for color, (angle, dist) in landmark_data.items():
        if color in LANDMARKS:
            lx, ly = LANDMARKS[color]
            points.append([lx, ly])
            distances.append(dist / SCALE)  # convert reality → simulation
    
    if len(points) < 3:
        print(f"[ERROR] Need at least 3 landmarks, got {len(points)}")
        return None
    
    # Least-squares solution
    # Expand: x² + y² - 2x·x_i - 2y·y_i = r_i² - x_i² - y_i²
    # Linear system: A·[x, y]ᵀ = b
    
    A = []
    b = []
    p0 = points[0]
    r0 = distances[0]
    
    for i in range(1, len(points)):
        pi = points[i]
        ri = distances[i]
        
        A.append([2*(p0[0] - pi[0]), 2*(p0[1] - pi[1])])
        b.append(r0**2 - ri**2 - p0[0]**2 - p0[1]**2 + pi[0]**2 + pi[1]**2)
    
    A = np.array(A)
    b = np.array(b)
    
    # Solve using least squares
    try:
        x, y = np.linalg.lstsq(A, b, rcond=None)[0]
        return (x, y)
    except:
        print("[ERROR] Trilateration failed")
        return None

def position_to_cell(x, y):
    """
    Convert continuous position (x, y) to grid cell index (1-25).
    Grid center (0, 0) = cell 13
    """
    # Convert to grid coordinates (0-4, 0-4)
    col = int((x + 2.5) / CELL_SIZE)
    row = int((2.5 - y) / CELL_SIZE)  # y increases upward
    
    # Clamp to grid
    col = max(0, min(4, col))
    row = max(0, min(4, row))
    
    # Cell index: row * 5 + col + 1
    cell_idx = row * 5 + col + 1
    return cell_idx

def main():
    bot = HamBot(lidar_enabled=True, camera_enabled=True)
    time.sleep(1.0)
    print("[Task1] Trilateration Localization")
    print(f"Scale factor: {SCALE} (reality/simulation)")
    
    try:
        # Step 1: Scan for landmarks
        detections = scan_for_landmarks(bot)
        
        if len(detections) < 3:
            print(f"[FAIL] Only detected {len(detections)} landmarks (need ≥3)")
            return
        
        # Step 2: Trilateration
        position = trilaterate(detections)
        if position is None:
            print("[FAIL] Trilateration failed")
            return
        
        x, y = position
        print(f"\n[RESULT] Estimated position: ({x:.2f}, {y:.2f})")
        
        # Step 3: Convert to cell
        cell = position_to_cell(x, y)
        print(f"[RESULT] Cell index: {cell}")
        
        # Verification: print distances to all landmarks
        print("\nVerification:")
        for color, (lx, ly) in LANDMARKS.items():
            calc_dist = math.sqrt((x - lx)**2 + (y - ly)**2) * SCALE
            measured = detections.get(color, (None, None))[1]
            status = "✓" if measured else "—"
            print(f"  {color:6s}: calculated={calc_dist:.3f}m, measured={measured:.3f if measured else '—'}m {status}")
    
    except KeyboardInterrupt:
        print("\n[STOP] User interrupted")
    finally:
        bot.stop_motors()

if __name__ == "__main__":
    main()