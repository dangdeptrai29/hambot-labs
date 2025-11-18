# HamBot API Reference - Lab 4

## Initialization
```python
from robot_systems.robot import HamBot
bot = HamBot(lidar_enabled=True, camera_enabled=True)
time.sleep(1.0)  # Allow sensors to initialize
```

## Camera
- **Access**: `cam = bot.camera`
- **Configure**: `cam.set_target_colors([rgb_tuple], tolerance=0.25)`
  - Tolerance: 0.20-0.35 typical
- **Detect**: `landmarks = cam.find_landmarks(min_area=300)`
  - Returns list with properties: `.x`, `.y`, `.width`, `.height` (pixels)
- **Resolution**: 640×480, center = 320
- **Cleanup**: `cam.stop()`

## LIDAR
- **Read**: `scan = bot.get_range_image()` → 360 distances in **millimeters**
  - Returns `-1` if not ready
- **Indices**: 0=back, 90=left, 180=front, 270=right
- **Convert**: `distance_m = scan[idx] * 0.001`

## Motors
- **Control**: `bot.set_left_motor_speed(rpm)`, `bot.set_right_motor_speed(rpm)`
- **Range**: -75 to +75 RPM
- **Stop**: `bot.stop_motors()`
- **Rotate CCW**: left=-N, right=+N

## IMU
- **Heading**: `heading = bot.get_heading()` → 0-360°
  - 90° = initial forward direction
  - Relative to start orientation

## Rotation Tracking
```python
last_heading = bot.get_heading()
angle_traveled = 0.0

# In loop:
current = bot.get_heading()
delta = current - last_heading
if delta > 180: delta -= 360
elif delta < -180: delta += 360
angle_traveled += abs(delta)
last_heading = current
```

## Physical Specs
- Wheel radius: 0.045 m
- Track width: 0.184 m
- Max speed: 0.81 m/s

## Cleanup
```python
try:
    # main logic
except KeyboardInterrupt:
    pass
finally:
    bot.stop_motors()
    cam.stop()
```