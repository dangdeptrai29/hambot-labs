from picamera2 import Picamera2
import numpy as np
import threading
import time
import cv2
import os

os.environ['LIBCAMERA_LOG_LEVELS'] = '*:ERROR'


class Landmark:
    def __init__(self, x, y, width, height, r, g, b):
        """Represents a detected landmark in the camera frame."""
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.r = r
        self.g = g
        self.b = b

    def __repr__(self):
        return f"Landmark(x={self.x}, y={self.y}, w={self.width}, h={self.height}, R={self.r}, G={self.g}, B={self.b})"


class Camera:
    def __init__(self, fps=5):
        """
        Initializes the camera and starts capturing images in a separate thread.
        Default FPS is 5 to balance performance and responsiveness.
        """
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(main={"size": (640, 480), "format": "RGB888"})
        self.picam2.configure(config)
        self.picam2.start()

        # Store image width and height
        self.width, self.height = config["main"]["size"]

        self.fps = fps
        self.image = None
        self.running = True
        self.landmark_colors = []  # List of colors to detect
        self.tolerance = 0.05  # Default tolerance (5%)

        # Start image capturing thread
        self.camera_thread = threading.Thread(target=self._capture_images)
        self.camera_thread.start()

    def _capture_images(self):
        """Continuously captures frames at the defined FPS."""
        while self.running:
            self.image = self.picam2.capture_array()
            time.sleep(1 / self.fps)

    def get_image(self):
        """Returns the latest image captured by the camera as a numpy array (height, width, RGB)."""
        return self.image

    def set_landmark_colors(self, colors, tolerance=0.05):
        """
        Updates the list of colors to be detected as landmarks.

        Args:
            colors (list of tuples): List of (R, G, B) tuples to detect.
            tolerance (float): Tolerance for color matching (0 to 1). Default is 5%.
        """
        self.landmark_colors = colors if isinstance(colors, list) else [colors]
        self.tolerance = max(0, min(1, tolerance))  # Ensure tolerance is within [0,1]

    def find_landmarks(self, area_threshold=500):
        """
        Detects regions in the image matching the stored landmark colors.

        Args:
            area_threshold (int): Minimum area of a detected region to be considered a landmark.

        Returns:
            list: A list of Landmark objects representing detected objects.
        """
        if self.image is None:

            return []

        if not self.landmark_colors:

            return []

        # Convert tolerance to absolute range for RGB
        tolerance_val = int(255 * self.tolerance)

        # Create an empty mask for the detected colors
        mask_total = np.zeros((self.height, self.width), dtype=np.uint8)

        detected_landmarks = []

        for rgb_value in self.landmark_colors:
            lower_bound = np.array([
                max(0, rgb_value[0] - tolerance_val),
                max(0, rgb_value[1] - tolerance_val),
                max(0, rgb_value[2] - tolerance_val)
            ])
            upper_bound = np.array([
                min(255, rgb_value[0] + tolerance_val),
                min(255, rgb_value[1] + tolerance_val),
                min(255, rgb_value[2] + tolerance_val)
            ])

            # Create mask for current color range
            mask = cv2.inRange(self.image, lower_bound, upper_bound)
            mask_total = cv2.bitwise_or(mask_total, mask)

        # Find contours in the combined mask
        contours, _ = cv2.findContours(mask_total, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > area_threshold:
                x, y, w, h = cv2.boundingRect(contour)
                center_x = x + w // 2
                center_y = y + h // 2

                # Get the color from the image at the detected landmark center
                detected_color = self.image[center_y, center_x]
                detected_landmarks.append(Landmark(center_x, center_y, w, h,
                                                   int(detected_color[0]),
                                                   int(detected_color[1]),
                                                   int(detected_color[2])))

        return detected_landmarks

    def stop_camera(self):
        """Stops the camera and the image capturing thread."""
        self.running = False
        self.camera_thread.join()
        self.picam2.close()
