from src.geometry import Point3d

class Target:
    def __init__(self, init_coords: Point3d,  radius: float):
        self.location: Point3d = init_coords
        self.radius = radius