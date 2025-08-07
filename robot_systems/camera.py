import cv2
import numpy as np
import threading
import time


class Landmark:
    def __init__(self, x, y, width, height, r, g, b):
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
    def __init__(self, fps=5, camera_index=0, resolution=(640, 480)):
        self.cap = cv2.VideoCapture(camera_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])

        if not self.cap.isOpened():
            raise RuntimeError("Could not open camera")

        self.width, self.height = resolution
        self.fps = fps
        self.image = None
        self.running = True
        self.landmark_colors = []
        self.tolerance = 0.05

        self.camera_thread = threading.Thread(target=self._capture_images)
        self.camera_thread.start()

    def _capture_images(self):
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                self.image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            time.sleep(1 / self.fps)

    def get_image(self):
        return self.image

    def set_landmark_colors(self, colors, tolerance=0.05):
        self.landmark_colors = colors if isinstance(colors, list) else [colors]
        self.tolerance = max(0, min(1, tolerance))

    def find_landmarks(self, area_threshold=500):
        if self.image is None or not self.landmark_colors:
            return []

        tolerance_val = int(255 * self.tolerance)
        mask_total = np.zeros((self.height, self.width), dtype=np.uint8)
        detected_landmarks = []

        for rgb in self.landmark_colors:
            lower = np.array([max(0, c - tolerance_val) for c in rgb])
            upper = np.array([min(255, c + tolerance_val) for c in rgb])
            mask = cv2.inRange(self.image, lower, upper)
            mask_total = cv2.bitwise_or(mask_total, mask)

        contours, _ = cv2.findContours(mask_total, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            if cv2.contourArea(contour) > area_threshold:
                x, y, w, h = cv2.boundingRect(contour)
                cx, cy = x + w // 2, y + h // 2
                color = self.image[cy, cx]
                detected_landmarks.append(Landmark(cx, cy, w, h, *color))

        return detected_landmarks

    def stop_camera(self):
        self.running = False
        self.camera_thread.join()
        self.cap.release()
