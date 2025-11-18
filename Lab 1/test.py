from robot_systems.robot import HamBot
import time

bot = HamBot(lidar_enabled=False, camera_enabled=False)

# Move forward for 2 seconds
bot.set_left_motor_speed(-50)   # left motor reversed
bot.set_right_motor_speed(50)   # right motor forward
time.sleep(2)

# Stop the motors
bot.stop_motors()