from src.geometry import *

class Mirror:
    def __init__(self, triangle: Triangle):
        self.static_point = triangle.point_a
        self.triangle = triangle


    def get_points_list(self):
        return [self.triangle.point_a, self.triangle.point_b, self.triangle.point_c]

