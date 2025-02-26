from picamera2 import Picamera2
import cv2
import numpy as np
import tkinter as tk
from tkinter import Scale, Button
import os

class SimpleRGBPicker:
    def __init__(self):
        # Initialize PiCamera2
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(main={"format": "RGB888", "size": (320, 240)})
        self.picam2.configure(config)
        self.picam2.start()

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
        self.tolerance_slider.set(10)  # Default to 10% tolerance
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
        # Capture frame
        frame = self.picam2.capture_array()
        frame = cv2.flip(frame, 0)  # Flip to correct orientation

        # Get RGB and tolerance values
        r = self.red_slider.get()
        g = self.green_slider.get()
        b = self.blue_slider.get()
        tolerance = self.tolerance_slider.get() / 100.0  # Convert to fraction

        # Define color bounds
        lower_bound = np.array([max(0, r - int(255 * tolerance)),
                                max(0, g - int(255 * tolerance)),
                                max(0, b - int(255 * tolerance))])
        upper_bound = np.array([min(255, r + int(255 * tolerance)),
                                min(255, g + int(255 * tolerance)),
                                min(255, b + int(255 * tolerance))])

        # Create mask for color detection
        mask = cv2.inRange(frame, lower_bound, upper_bound)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Draw bounding boxes
        for contour in contours:
            if cv2.contourArea(contour) > 100:  # Ignore small noise
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # If a click happened, update the RGB sliders to match the pixel color
        if self.click_x is not None and self.click_y is not None:
            clicked_color = frame[self.click_y, self.click_x]
            self.red_slider.set(int(clicked_color[0]))
            self.green_slider.set(int(clicked_color[1]))
            self.blue_slider.set(int(clicked_color[2]))
            self.click_x, self.click_y = None, None  # Reset click after updating

        # Show the image using OpenCV's GUI
        if self.has_display:
            cv2.imshow("Live Feed", frame)
            cv2.setMouseCallback("Live Feed", self.get_pixel_color)  # Attach mouse event
            cv2.waitKey(1)

        # Schedule next update
        self.root.after(30, self.update_image)

    def get_pixel_color(self, event, x, y, flags, param):
        """Store clicked pixel coordinates."""
        if event == cv2.EVENT_LBUTTONDOWN:
            self.click_x, self.click_y = x, y

    def quit(self):
        """Exit the program cleanly."""
        self.picam2.stop()
        if self.has_display:
            cv2.destroyAllWindows()
        self.root.quit()
        self.root.destroy()

