import threading
import time
import numpy as np
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera

class Camera:
    def __init__(self, resolution=(640, 480), fps=5):
        """
        Initialize the camera, set resolution, and start the capture thread.

        Args:
            resolution (tuple): The resolution of the camera (width, height).
            fps (int): The frames per second for the camera capture.
        """
        self.camera = PiCamera()
        self.camera.resolution = resolution
        self.camera.framerate = fps
        self.rawCapture = PiRGBArray(self.camera, size=resolution)
        self.lock = threading.Lock()
        self.running = True
        self.frame = None
        self.capture_thread = threading.Thread(target=self._capture_frames)
        self.capture_thread.start()

    def _capture_frames(self):
        """
        Continuously capture frames from the camera in a separate thread.
        """
        for frame in self.camera.capture_continuous(self.rawCapture, format="rgb", use_video_port=True):
            with self.lock:
                self.frame = frame.array
            self.rawCapture.truncate(0)  # Clear the stream for the next frame
            time.sleep(1 / self.camera.framerate)  # Sleep to maintain FPS

            if not self.running:
                break

    def find_landmarks(self, rgb_value, tolerance=40):
        """
        Process the current frame to find landmarks matching the specified RGB value.

        Args:
            rgb_value (tuple): The RGB color value to detect (R, G, B).
            tolerance (int): The tolerance for color matching.

        Returns:
            numpy array: The processed frame with bounding boxes drawn around the detected landmarks.
        """
        with self.lock:
            if self.frame is None:
                return None

            frame = self.frame.copy()

        # Convert the frame to HSV for better color detection
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        rgb_hsv = cv2.cvtColor(np.uint8([[rgb_value]]), cv2.COLOR_RGB2HSV)[0][0]

        # Define range of color in HSV
        lower_bound = np.array([max(0, rgb_hsv[0] - tolerance), 100, 100])
        upper_bound = np.array([min(179, rgb_hsv[0] + tolerance), 255, 255])

        # Threshold the HSV image to get only the desired colors
        mask = cv2.inRange(hsv_frame, lower_bound, upper_bound)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Draw bounding boxes around detected contours
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        return frame

    def get_image(self):
        """
        Get the current frame as a numpy array.

        Returns:
            numpy array: The current frame in (height, width, RGB) format.
        """
        with self.lock:
            if self.frame is None:
                return None
            return self.frame.copy()

    def stop_camera(self):
        """
        Stop the camera capture thread and release the camera.
        """
        self.running = False
        self.capture_thread.join()
        self.camera.close()