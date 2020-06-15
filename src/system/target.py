from src.geometry import Point3d
import math

class Target:
    def __init__(self, init_coords: Point3d,  radius: float):
        self.location: Point3d = init_coords
        self.radius = radius

    def rotate(self, center: Point3d, angle):
        self.location = self.location - center
        x = self.location.x * math.cos(angle) - self.location.z * math.sin(angle)
        z = self.location.x * math.sin(angle) + self.location.z * math.cos(angle)
        self.location.x = x
        self.location.z = z
        self.location = self.location + center