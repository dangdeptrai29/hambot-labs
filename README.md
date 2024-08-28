# HamBot: A Python Library for Robot Control

HamBot is a Python library designed to control a robot equipped with various sensors, including IMU, Lidar, and Camera. The library is built to simplify robot programming, providing easy-to-use interfaces for sensor data acquisition and motor control.

## Features
- **Motor Control** using the Raspberry Pi Build HAT.
- **IMU Integration** with the BNO055 sensor for orientation and acceleration data.
- **Lidar Integration** with the RPLidar for distance measurements.
- **Camera Integration** with the Raspberry Pi Camera Board for image processing and landmark detection.

## Getting Started

### Prerequisites

Before you begin, ensure you have met the following requirements:

- A Raspberry Pi running Raspberry Pi OS.
- Python 3.6 or higher.
- Access to the Raspberry Pi GPIO pins for motor control.
- The following hardware components:
  - **IMU Sensor**: BNO055 (Adafruit CircuitPython library)
  - **Lidar Sensor**: RPLidar (Adafruit CircuitPython library)
  - **Camera**: Raspberry Pi Camera Board v2 - 8 Megapixels
  - **Motor Controller**: Raspberry Pi Build HAT

### Installation

1. **Clone the Repository**

   Clone this repository to your local machine:

   ```bash
   git clone https://github.com/yourusername/robot_systems.git
   cd robot_systems
   ```

2. **Set Up a Virtual Environment**

   It's recommended to use a virtual environment to manage dependencies:

   ```bash
   python3 -m venv --system-sit-packages hambot_env
   source hambot_env/bin/activate
   ```

3. **Install Dependencies**

   Install the required Python libraries:

   ```bash
   pip install -e .
   ```

   This command installs the `robot_systems` package and its dependencies.

### Hardware Components

#### 1. **IMU Sensor (BNO055)**

- **Description**: The BNO055 is an advanced 9-axis sensor that provides absolute orientation, acceleration, and magnetic field strength. It simplifies sensor integration by fusing sensor data into a stable output.
- **Library Documentation**: [Adafruit CircuitPython BNO055](https://docs.circuitpython.org/projects/bno055/en/latest/)

#### 2. **Lidar Sensor (RPLidar)**

- **Description**: The RPLidar is a low-cost Lidar sensor capable of scanning 360° surroundings and producing a 2D range image. It's commonly used in robotics for mapping and navigation.
- **Library Documentation**: [Adafruit CircuitPython RPLidar](https://docs.circuitpython.org/projects/rplidar/en/latest/)

#### 3. **Camera (Raspberry Pi Camera Board v2)**

- **Description**: The Raspberry Pi Camera Board v2 is an 8-megapixel camera capable of taking high-definition still photographs and videos. It's suitable for computer vision tasks in robotics.
- **Library Documentation**: [Picamera2 Documentation](https://datasheets.raspberrypi.com/camera/picamera2-manual.pdf)

#### 4. **Motor Controller (Build HAT)**

- **Description**: The Build HAT by Raspberry Pi is a motor controller that allows easy interfacing with LEGO motors and sensors, making it ideal for educational robotics projects.
- **Library Documentation**: [Build HAT Documentation](https://buildhat.readthedocs.io/en/latest/)

### Usage Examples

Below are some examples of how to use the `HamBot` class to control the robot and interact with the sensors.

#### 1. **Initialize the HamBot**

```python
from robot_systems.robot import HamBot

# Initialize the HamBot
robot = HamBot()
```

#### 2. **Get IMU Heading**

```python
# Get the current heading from the IMU
heading = robot.get_heading()
print(f"Robot heading: {heading}°")
```

#### 3. **Get Lidar Range Image**

```python
# Get the current range image from the Lidar
range_image = robot.get_range_image()
print(f"Range image: {range_image[:10]}")  # Print the first 10 values
```

#### 4. **Capture an Image with the Camera**

```python
# Capture an image and process it
image = robot.camera.get_image()
robot.camera.find_landmarks((255, 0, 0))  # Find red landmarks
```

#### 5. **Detect Olive Green and White Landmarks with the Camera**

```python
# Detect landmarks of olive green and white colors
olive_green_hsv = (70, 153, 102)
white_hsv = (0, 0, 255)

landmarks = robot.camera.find_landmarks([olive_green_hsv, white_hsv], tolerance=0.05, area_threshold=500)

for landmark in landmarks:
    print(f"Landmark at ({landmark.center_x}, {landmark.center_y}) with size {landmark.width}x{landmark.height}")
```

#### 6. **Control Motors**

```python
# Set motor speeds
robot.set_left_motor_speed(50)  # Set left motor to 50 RPM
robot.set_right_motor_speed(50)  # Set right motor to 50 RPM

# Run motors for 2 seconds
robot.run_motors_for_seconds(2)

# Stop the motors
robot.stop_motors()
```

#### 7. **Disconnect the Robot**

```python
# Properly disconnect the robot
robot.disconnect_robot()
```

### Contributing

Contributions are welcome! Please submit a pull request or open an issue to discuss any changes or improvements.

### License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

### Acknowledgments

Special thanks to the developers of the libraries and hardware components used in this project.
