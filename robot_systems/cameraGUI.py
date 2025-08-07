import cv2
import numpy as np
import tkinter as tk
from tkinter import Scale, Button
import os


class SimpleRGBPicker:
    def __init__(self, camera_index=0):
        # Initialize OpenCV camera
        self.cap = cv2.VideoCapture(camera_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

        if not self.cap.isOpened():
            raise RuntimeError("Could not open camera")

        # Check if running in a GUI environment
        self.has_display = "DISPLAY" in os.environ

        # Create main Tkinter window
        self.root = tk.Tk()
        self.root.title("RGB Color Selector")

        # RGB sliders
        self.red_slider = Scale(self.root, from_=0, to=255, orient=tk.HORIZONTAL, label="Red")
        self.red_slider.pack()
        self.green_slider = Scale(self.root, from_=0, to=255, orient=tk.HORIZONTAL, label="Green")
        self.green_slider.pack()
        self.blue_slider = Scale(self.root, from_=0, to=255, orient=tk.HORIZONTAL, label="Blue")
        self.blue_slider.pack()

        # Tolerance slider
        self.tolerance_slider = Scale(self.root, from_=0, to=100, orient=tk.HORIZONTAL, label="Tolerance (%)")
        self.tolerance_slider.set(10)
        self.tolerance_slider.pack()

        # Quit button
        self.quit_button = Button(self.root, text="Quit", command=self.quit)
        self.quit_button.pack()

        # Mouse click coordinates
        self.click_x, self.click_y = None, None

        # Start image update loop
        self.update_image()
        self.root.protocol("WM_DELETE_WINDOW", self.quit)
        self.root.mainloop()

    def update_image(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        # Flip frame to match original behavior
        frame = cv2.flip(frame, 0)

        r = self.red_slider.get()
        g = self.green_slider.get()
        b = self.blue_slider.get()
        tolerance = self.tolerance_slider.get() / 100.0

        lower = np.array([max(0, r - int(255 * tolerance)),
                          max(0, g - int(255 * tolerance)),
                          max(0, b - int(255 * tolerance))])
        upper = np.array([min(255, r + int(255 * tolerance)),
                          min(255, g + int(255 * tolerance)),
                          min(255, b + int(255 * tolerance))])

        # Convert BGR to RGB for consistent behavior
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mask = cv2.inRange(rgb_frame, lower, upper)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            if cv2.contourArea(contour) > 100:
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        if self.click_x is not None and self.click_y is not None:
            rgb_clicked = rgb_frame[self.click_y, self.click_x]
            self.red_slider.set(int(rgb_clicked[0]))
            self.green_slider.set(int(rgb_clicked[1]))
            self.blue_slider.set(int(rgb_clicked[2]))
            self.click_x, self.click_y = None, None

        if self.has_display:
            cv2.imshow("Live Feed", frame)
            cv2.setMouseCallback("Live Feed", self.get_pixel_color)
            cv2.waitKey(1)

        self.root.after(30, self.update_image)

    def get_pixel_color(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.click_x, self.click_y = x, y

    def quit(self):
        self.cap.release()
        if self.has_display:
            cv2.destroyAllWindows()
        self.root.quit()
        self.root.destroy()


if __name__ == "__main__":
    SimpleRGBPicker()
