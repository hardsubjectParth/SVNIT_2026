import math

class VisionBridge:
    def __init__(self, horizontal_fov=62.2, vertical_fov=48.8):
        # Default FOV for Raspberry Pi Camera Module 2 (IMX219)
        self.hfov = math.radians(horizontal_fov)
        self.vfov = math.radians(vertical_fov)
        
        # Camera Resolution (must match your capture settings)
        self.img_w = 640
        self.img_h = 480

    def get_ground_offsets(self, px_x, px_y, altitude):
        """
        Converts pixel offsets (px_x, px_y) to NED offsets (dn, de) in meters.
        dn = North offset (meters)
        de = East offset (meters)
        """
        # Calculate meters per pixel at the current altitude
        # Use simple trigonometry based on FOV and Altitude
        meters_per_px_w = (2 * altitude * math.tan(self.hfov / 2)) / self.img_w
        meters_per_px_h = (2 * altitude * math.tan(self.vfov / 2)) / self.img_h

        # Convert pixel offset to meters
        # Note: If camera is mounted facing forward, North/East depends on drone heading.
        # Assuming camera 'Up' is Drone 'Forward' (North)
        dn = -px_y * meters_per_px_h  # Inverse because pixels increase downward
        de = px_x * meters_per_px_w
        
        return dn, de