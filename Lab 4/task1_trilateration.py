# task1_trilateration.py
import time
import math
import numpy as np
from robot_systems.robot import HamBot

LANDMARKS = {
    'yellow': (-2.5, 2.5),
    'red':    (2.5, 2.5),
    'green':  (-2.5, -2.5),
    'blue':   (2.5, -2.5)
}

COLOR_CONFIGS = {
    'blue':   ((80, 183, 220), 0.20),
    'red':    ((254, 4, 170), 0.20),
    'yellow': ((251, 171, 3), 0.20),
    'green':  ((65, 229, 94), 0.28)
}

SCALE = 0.38
MIN_AREA = 300
MAX_AREA = 15000
SCAN_RPM = 12.0
SCAN_DT = 0.12
MM_TO_M = 0.001
FRONT_IDX = 180
LIDAR_SPAN = 20

def is_valid_landmark(lm):
    area = lm.width * lm.height
    if not (MIN_AREA < area < MAX_AREA):
        return False
    aspect = max(lm.width, lm.height) / min(lm.width, lm.height)
    return aspect <= 5.0

def get_landmark_distance(scan, lm):
    pixel_offset = lm.x - 320
    angular_offset = pixel_offset * (60.0 / 640.0)
    center = int(FRONT_IDX + angular_offset)
    
    sector = []
    for i in range(center - LIDAR_SPAN, center + LIDAR_SPAN + 1):
        val = scan[i % 360]
        if val > 0:
            sector.append(val)
    
    if len(sector) < 5:
        return None
    
    sector_arr = np.array(sector)
    mean = sector_arr.mean()
    std = sector_arr.std()
    
    filtered = sector_arr[np.abs(sector_arr - mean) <= 1.5 * std]
    
    if len(filtered) == 0:
        return None
    
    return filtered.min() * MM_TO_M

def scan_for_landmark(bot, cam, color_name):
    print(f"  {color_name}...")
    
    rgb, tol = COLOR_CONFIGS[color_name]
    cam.set_target_colors([rgb], tolerance=tol)
    time.sleep(0.3)
    
    bot.set_left_motor_speed(-SCAN_RPM)
    bot.set_right_motor_speed(SCAN_RPM)
    
    start_heading = bot.get_heading() or 0.0
    angle_traveled = 0.0
    last_heading = start_heading
    
    try:
        while angle_traveled < 370:
            current_heading = bot.get_heading()
            landmarks = cam.find_landmarks(min_area=MIN_AREA)
            
            valid = [lm for lm in landmarks if is_valid_landmark(lm)]
            
            if valid:
                bot.stop_motors()
                time.sleep(0.25)
                
                dists = []
                for _ in range(3):
                    lms = cam.find_landmarks(min_area=MIN_AREA)
                    if lms:
                        lm = lms[0]
                        scan = bot.get_range_image()
                        if scan != -1:
                            d = get_landmark_distance(scan, lm)
                            if d and 0.2 < d < 2.5:
                                dists.append(d)
                    time.sleep(0.1)
                
                if len(dists) >= 2:
                    final_dist = np.median(dists)
                    print(f"    ✓ {current_heading:.0f}°, {final_dist:.3f}m")
                    return (current_heading, final_dist)
                
                bot.set_left_motor_speed(-SCAN_RPM)
                bot.set_right_motor_speed(SCAN_RPM)
            
            if current_heading is not None:
                delta = current_heading - last_heading
                if delta > 180: delta -= 360
                elif delta < -180: delta += 360
                angle_traveled += abs(delta)
                last_heading = current_heading
            
            time.sleep(SCAN_DT)
        
        bot.stop_motors()
        print(f"    ✗ not found")
        return None
    
    except Exception as e:
        bot.stop_motors()
        print(f"    Error: {e}")
        return None

def trilaterate(detections):
    if len(detections) < 3:
        return None
    
    A_rows = []
    b_rows = []
    
    colors = list(detections.keys())
    for i in range(len(colors)):
        for j in range(i+1, len(colors)):
            c1, c2 = colors[i], colors[j]
            p1 = LANDMARKS[c1]
            p2 = LANDMARKS[c2]
            r1 = detections[c1][1] / SCALE
            r2 = detections[c2][1] / SCALE
            
            A_rows.append([2*(p2[0] - p1[0]), 2*(p2[1] - p1[1])])
            b_rows.append(r1**2 - r2**2 - p1[0]**2 - p1[1]**2 + p2[0]**2 + p2[1]**2)
    
    A = np.array(A_rows)
    b = np.array(b_rows)
    
    try:
        x, y = np.linalg.lstsq(A, b, rcond=None)[0]
        return (x, y)
    except:
        return None

def position_to_cell(x, y):
    col = int((x + 2.5))
    row = int((2.5 - y))
    col = np.clip(col, 0, 4)
    row = np.clip(row, 0, 4)
    return row * 5 + col + 1

def main():
    print("\n" + "="*50)
    print("LAB 4 - TASK 1: TRILATERATION")
    print("="*50)
    
    bot = HamBot(lidar_enabled=True, camera_enabled=True)
    cam = bot.camera
    time.sleep(1.5)
    
    print("-"*50 + "\n")
    
    detections = {}
    
    try:
        for color_name in COLOR_CONFIGS.keys():
            result = scan_for_landmark(bot, cam, color_name)
            if result:
                detections[color_name] = result
            time.sleep(0.5)
        
        if len(detections) < 3:
            print(f"\n✗ {len(detections)}/4\n")
            return
        
        print(f"\n✓ {len(detections)}/4\n")
        
        position = trilaterate(detections)
        if not position:
            print("✗ Failed\n")
            return
        
        x, y = position
        cell = position_to_cell(x, y)
        
        print("="*50)
        print(f"Position: ({x:.2f}, {y:.2f}), Cell: {cell}")
        print("="*50 + "\n")
        
        for color in ['blue', 'red', 'yellow', 'green']:
            lx, ly = LANDMARKS[color]
            calc = math.sqrt((x-lx)**2 + (y-ly)**2) * SCALE
            if color in detections:
                meas = detections[color][1]
                print(f"  {color:6s}: calc={calc:.3f}m, meas={meas:.3f}m, err={abs(calc-meas):.3f}m")
        print()
    
    except KeyboardInterrupt:
        print("\n✗ Interrupted\n")
    except Exception as e:
        print(f"\n✗ {e}\n")
        import traceback
        traceback.print_exc()
    finally:
        bot.stop_motors()
        cam.stop()

if __name__ == "__main__":
    main()