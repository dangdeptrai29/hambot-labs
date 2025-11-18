# color_test_rotating.py
import time
from robot_systems.robot import HamBot

COLORS = {
    'red':    (216, 0, 105),
    'green':  (37, 217, 103),
    'blue':   (73, 188, 233),
    'yellow': (233, 147, 33)
}

bot = HamBot(lidar_enabled=True, camera_enabled=True)
cam = bot.camera
time.sleep(1)

MIN_AREA = 200
MAX_AREA = 5000

for color_name, rgb in COLORS.items():
    print(f"\n{color_name.upper()} - rotating 360°")
    cam.set_target_colors([rgb], tolerance=0.18)
    time.sleep(0.3)
    
    bot.set_left_motor_speed(-12)
    bot.set_right_motor_speed(12)
    
    start = bot.get_heading() or 0.0
    angle = 0.0
    last = start
    areas_seen = []
    
    while angle < 360:
        landmarks = cam.find_landmarks(min_area=MIN_AREA)
        
        if landmarks:
            for lm in landmarks:
                area = lm.width * lm.height
                status = "✓" if MIN_AREA < area < MAX_AREA else "✗"
                print(f"  {status} {area} ({lm.width}x{lm.height})")
                areas_seen.append(area)
        
        curr = bot.get_heading()
        if curr:
            delta = curr - last
            if delta > 180: delta -= 360
            elif delta < -180: delta += 360
            angle += abs(delta)
            last = curr
        
        time.sleep(0.1)
    
    bot.stop_motors()
    
    if areas_seen:
        print(f"  Min: {min(areas_seen)}, Max: {max(areas_seen)}, Avg: {sum(areas_seen)//len(areas_seen)}")
    else:
        print(f"  None detected")
    
    time.sleep(0.5)

cam.stop()