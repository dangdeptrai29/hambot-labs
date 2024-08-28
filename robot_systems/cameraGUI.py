from picamera2 import Picamera2
import cv2
import numpy as np
import tkinter as tk
from tkinter import Scale, Entry, Label, Button

class GUICamera:
    def __init__(self):
        # Initialize the Picamera2
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(main={"format": "RGB888", "size": (640, 480)})
        self.picam2.configure(config)
        self.picam2.start()

        # Create the main window
        self.root = tk.Tk()
        self.root.title("HSV Color Picker")

        # HSV sliders and tolerance input
        self.hue_slider = Scale(self.root, from_=0, to=179, orient=tk.HORIZONTAL, label="Hue")
        self.hue_slider.pack()

        self.sat_slider = Scale(self.root, from_=0, to=255, orient=tk.HORIZONTAL, label="Saturation")
        self.sat_slider.pack()

        self.val_slider = Scale(self.root, from_=0, to=255, orient=tk.HORIZONTAL, label="Value")
        self.val_slider.pack()

        self.tolerance_label = Label(self.root, text="Tolerance (0-1)")
        self.tolerance_label.pack()

        self.tolerance_entry = Entry(self.root)
        self.tolerance_entry.insert(0, "0.05")
        self.tolerance_entry.pack()

        self.quit_button = Button(self.root, text="Quit", command=self.quit)
        self.quit_button.pack()

        # Start the update loop
        self.update_image()
        self.root.protocol("WM_DELETE_WINDOW", self.quit)
        self.root.mainloop()

    def update_image(self):
        # Capture a frame from the Picamera2
        frame = self.picam2.capture_array()

        # Convert the frame to HSV color space
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

        # Get current HSV and tolerance values
        h = self.hue_slider.get()
        s = self.sat_slider.get()
        v = self.val_slider.get()
        tolerance = float(self.tolerance_entry.get())

        # Define the lower and upper bounds for color detection
        lower_bound = np.array([h - int(179 * tolerance), s - int(255 * tolerance), v - int(255 * tolerance)])
        upper_bound = np.array([h + int(179 * tolerance), s + int(255 * tolerance), v + int(255 * tolerance)])
        lower_bound = np.clip(lower_bound, [0, 0, 0], [179, 255, 255])
        upper_bound = np.clip(upper_bound, [0, 0, 0], [179, 255, 255])

        # Create a mask based on the current HSV values and tolerance
        mask = cv2.inRange(hsv_frame, lower_bound, upper_bound)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Set an area threshold to filter out small contours
        area_threshold = 500  # Adjust this value based on your needs

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > area_threshold:
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Convert the image from RGB to BGR for OpenCV compatibility
        bgr_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        # Convert the image to PhotoImage format for Tkinter
        img = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, (640, 480))  # Resize to fit Tkinter window
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGBA)
        img = np.array(img)
        img = cv2.flip(img, 1)  # Optional: Flip image if needed

        # Display the image in the GUI
        self.photo = tk.PhotoImage(width=img.shape[1], height=img.shape[0], data=cv2.imencode('.png', img)[1].tobytes())
        if not hasattr(self, 'label'):
            self.label = tk.Label(self.root, image=self.photo)
            self.label.pack()
        else:
            self.label.config(image=self.photo)
            self.label.image = self.photo

        # Schedule the next update
        self.root.after(10, self.update_image)

    def quit(self):
        # Stop the camera and close the GUI
        self.picam2.stop()
        self.root.quit()
        self.root.destroy()

# To use the GUI camera, create an instance of GUICamera
gui_camera = GUICamera()
