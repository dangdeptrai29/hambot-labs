import board
import adafruit_bno055

class IMU:
    def __init__(self):
        self.i2c = board.I2C()  # Uses board.SCL and board.SDA
        self.sensor = adafruit_bno055.BNO055_I2C(self.i2c)

    def get_temperature(self):
        """
        Get the temperature of the IMU sensor.

        Returns:
            float: The temperature in degrees Celsius.
        """
        return self.sensor.temperature

    def get_euler_angles(self):
        """
        Get the orientation of the IMU as Euler angles.

        Returns:
            tuple: A tuple containing (heading, roll, pitch) in degrees.
        """
        return self.sensor.euler

    def get_quaternion(self):
        """
        Get the orientation of the IMU as a quaternion.

        Returns:
            tuple: A tuple containing (x, y, z, w) components of the quaternion.
        """
        return self.sensor.quaternion

    def get_linear_acceleration(self):
        """
        Get the linear acceleration from the IMU.

        Returns:
            tuple: A tuple containing (x, y, z) acceleration in m/s^2.
        """
        return self.sensor.linear_acceleration

    def get_gravity(self):
        """
        Get the gravity vector from the IMU.

        Returns:
            tuple: A tuple containing (x, y, z) gravity vector in m/s^2.
        """
        return self.sensor.gravity

    def get_gyroscope(self):
        """
        Get the gyroscope data from the IMU.

        Returns:
            tuple: A tuple containing (x, y, z) angular velocity in radians/s.
        """
        return self.sensor.gyro

    def get_magnetometer(self):
        """
        Get the magnetometer data from the IMU.

        Returns:
            tuple: A tuple containing (x, y, z) magnetic field in microteslas.
        """
        return self.sensor.magnetic

    def get_acceleration(self):
        """
        Get the raw acceleration data from the IMU.

        Returns:
            tuple: A tuple containing (x, y, z) acceleration in m/s^2.
        """
        return self.sensor.acceleration

    def get_calibration_status(self):
        """
        Get the calibration status of the IMU.

        Returns:
            tuple: A tuple containing the calibration status for the system, gyroscope, accelerometer, and magnetometer.
        """
        return self.sensor.calibration_status

    def get_forward_acceleration(self):
        """
        Get the forward acceleration based on the IMU orientation.

        Returns:
            float: The acceleration along the z-axis (forward direction) in m/s^2.
        """
        _, _, z_accel = self.get_linear_acceleration()
        return z_accel

    def get_lateral_acceleration(self):
        """
        Get the lateral acceleration based on the IMU orientation.

        Returns:
            float: The acceleration along the y-axis (side-to-side direction) in m/s^2.
        """
        _, y_accel, _ = self.get_linear_acceleration()
        return y_accel

    def get_vertical_acceleration(self):
        """
        Get the vertical acceleration based on the IMU orientation.

        Returns:
            float: The acceleration along the x-axis (downward direction) in m/s^2.
        """
        x_accel, _, _ = self.get_linear_acceleration()
        return x_accel

    def get_heading(self):
        """
        Get the robot's heading in degrees from East, assuming the robot starts facing North.

        Returns:
            float: The heading in degrees from East (0° to 360°).
        """
        heading, _, _ = self.get_euler_angles()  # Retrieve the yaw angle (heading)
        heading_from_east = (-heading + 90) % 360  # Adjust to be relative to East
        return heading_from_east
