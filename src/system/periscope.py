from src.geometry import *
from src.system.mirror import Mirror
from src.system.target import Target

class MirrorLocation(Enum):
    UP = 1
    DOWN = 2
    THREE = 3
    FOUR = 4

class Periscope:

    EPS_ANGLE_DELTA = 0.008

    def __init__(self, config):
        self.laser: Ray = Ray(config['start_laser_location'], config['start_laser_direction'])
        points3_down_tr = config['down_triangle']
        self.mirror_down = Mirror(Triangle(points3_down_tr[0], points3_down_tr[1], points3_down_tr[2]))
        points3_up_tr = config['up_triangle']
        self.mirror_up = Mirror(Triangle(points3_up_tr[0], points3_up_tr[1], points3_up_tr[2]))
        points3_tr_3 = config['triangle_3']
        self.mirror_3 = Mirror(Triangle(points3_tr_3[0], points3_tr_3[1], points3_tr_3[2]))
        points3_tr_4 = config['triangle_4']
        self.mirror_4 = Mirror(Triangle(points3_tr_4[0], points3_tr_4[1], points3_tr_4[2]))

        self.target: Target = Target

    def set_target(self, target: Target):
        self.target = target

    def ray_to_aim(self) -> Ray:
        return self.laser.reflect_plane(self.mirror_down.triangle).reflect_plane(self.mirror_up.triangle)\
            .reflect_plane(self.mirror_3.triangle).reflect_plane(self.mirror_4.triangle)
