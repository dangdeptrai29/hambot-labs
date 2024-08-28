from picamera2 import Picamera2
from landmark import Landmark
import numpy as np
import threading
import time
import cv2
import os

os.environ['LIBCAMERA_LOG_LEVELS'] = '*:ERROR'

class Camera:
    def __init__(self, fps=5):
        self.picam2 = Picamera2()
        self.picam2.start_preview()
        self.picam2.configure(self.picam2.create_preview_configuration(main={"size": (640, 480)}))
        self.picam2.start()
        self.fps = fps
        self.image = None
        self.running = True
        self.camera_thread = threading.Thread(target=self._capture_images)
        self.camera_thread.start()

    def _capture_images(self):
        while self.running:
            self.image = self.picam2.capture_array()
            time.sleep(1 / self.fps)

    def get_image(self):
        """
        Returns the latest image captured by the camera as a numpy array.
        The array is in the format (height, width, RGB).
        """
        return self.image

    def find_landmarks(self, hsv_values, tolerance=0.05, area_threshold=500):
        """
        Processes the latest image to find landmarks of specific HSV colors.
        Returns a list of Landmark objects, each representing a detected object.

        Args:
            hsv_values (tuple or list of tuples): A tuple containing the HSV values to detect,
                                                  or a list of such tuples.
            tolerance (float): Tolerance for color matching in HSV space. Default is 0.05 (5%).
            area_threshold (int): Minimum area of contours to consider as landmarks. Default is 500 pixels.

        Returns:
            list: A list of Landmark objects, each containing the center coordinates, width,
                  and height of the bounding box in pixels.
        """
        if self.image is None:
            print("No image captured yet.")
            return []

        # Convert the image to HSV color space
        hsv_image = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)

        # Convert tolerance from percentage to absolute range for each HSV channel
        tolerance_h = int(180 * tolerance)  # Hue range is 0-180
        tolerance_sv = int(255 * tolerance)  # Saturation and Value range is 0-255

        if isinstance(hsv_values, tuple):
            hsv_values = [hsv_values]  # Convert to list if a single value is provided

        mask_total = np.zeros(hsv_image.shape[:2], dtype=np.uint8)

        for hsv_value in hsv_values:
            lower_bound = np.array(
                [hsv_value[0] - tolerance_h, hsv_value[1] - tolerance_sv, hsv_value[2] - tolerance_sv])
            upper_bound = np.array(
                [hsv_value[0] + tolerance_h, hsv_value[1] + tolerance_sv, hsv_value[2] + tolerance_sv])

            # Ensure the bounds are within valid HSV ranges
            lower_bound = np.clip(lower_bound, [0, 0, 0], [180, 255, 255])
            upper_bound = np.clip(upper_bound, [0, 0, 0], [180, 255, 255])

            # Create a mask for the current color
            mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
            mask_total = cv2.bitwise_or(mask_total, mask)

        # Find contours based on the combined mask
        contours, _ = cv2.findContours(mask_total, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        landmarks = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > area_threshold:
                x, y, w, h = cv2.boundingRect(contour)
                center_x = x + w // 2
                center_y = y + h // 2
                landmarks.append(Landmark(center_x, center_y, w, h))

        return landmarks

    def stop_camera(self):
        """
        Stops the camera and the image capturing thread.
        """
        self.running = False
        self.camera_thread.join()
        self.picam2.stop_preview()
        self.picam2.close()
