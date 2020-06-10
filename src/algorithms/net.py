from src.system.periscope import Periscope, Angle
from keras.models import load_model
import numpy
from src.geometry import Point3d
from src.system.periscope import MirrorLocation
import multiprocessing as mp

def add_point_to_list(str_list: [], p: Point3d):
    str_list.append(p.x)
    str_list.append(p.y)
    str_list.append(p.z)


class NeuralNetAlgorithm:
    # def __init__(self):
    #     self.model = load_model('./neuralnet/my_model.h5')
    #
    # # for all in one process implementation
    # def step(self, periscope: Periscope, up, down):
    #     input_list = [periscope.target.location.y, periscope.target.location.z]
    #
    #     angles =  self.model.predict(numpy.array([input_list]))
    #     periscope.mirror_down.triangle = down.rotate_plane(angles[0][0], Angle.YAW)
    #     periscope.mirror_down.triangle = periscope.mirror_down.triangle.rotate_plane(angles[0][1], Angle.PITCH)
    #     periscope.mirror_down.triangle = periscope.mirror_down.triangle.rotate_plane(angles[0][2], Angle.ROLL)
    #
    #     periscope.mirror_up.triangle = up.rotate_plane(angles[0][3], Angle.YAW)
    #     periscope.mirror_up.triangle = periscope.mirror_up.triangle.rotate_plane(angles[0][4], Angle.PITCH)
    #     periscope.mirror_up.triangle = periscope.mirror_up.triangle.rotate_plane(angles[0][5], Angle.ROLL)
    #
    @staticmethod
    def run(self_queue: mp.Queue, arr, periscope: Periscope, plane_loc: MirrorLocation, model):
        down = periscope.mirror_down.triangle
        up = periscope.mirror_up.triangle
        while True:
            periscope.target = self_queue.get()

            angles = model.predict(numpy.array([[periscope.target.location.y, periscope.target.location.z]]))
            periscope.mirror_down.triangle = down.rotate_plane(angles[0][0], Angle.YAW)
            periscope.mirror_down.triangle = periscope.mirror_down.triangle.rotate_plane(angles[0][1], Angle.PITCH)
            periscope.mirror_down.triangle = periscope.mirror_down.triangle.rotate_plane(angles[0][2], Angle.ROLL)

            periscope.mirror_up.triangle = up.rotate_plane(angles[0][3], Angle.YAW)
            periscope.mirror_up.triangle = periscope.mirror_up.triangle.rotate_plane(angles[0][4], Angle.PITCH)
            periscope.mirror_up.triangle = periscope.mirror_up.triangle.rotate_plane(angles[0][5], Angle.ROLL)

            self_plane = periscope.mirror_down.triangle
            if plane_loc == MirrorLocation.UP:
                self_plane = periscope.mirror_up.triangle

            arr[0] = self_plane.point_b.x
            arr[1] = self_plane.point_b.y
            arr[2] = self_plane.point_b.z
            arr[3] = self_plane.point_c.x
            arr[4] = self_plane.point_c.y
            arr[5] = self_plane.point_c.z
