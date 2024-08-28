class Landmark:
    def __init__(self, center_x, center_y, width, height):
        """
        Initializes a Landmark object.

        Args:
            center_x (int): The x-coordinate of the landmark's center.
            center_y (int): The y-coordinate of the landmark's center.
            width (int): The width of the bounding box in pixels.
            height (int): The height of the bounding box in pixels.
        """
        self.center_x = center_x
        self.center_y = center_y
        self.width = width
        self.height = height

    def __repr__(self):
        return f"Landmark(center=({self.center_x}, {self.center_y}), width={self.width}, height={self.height})"
