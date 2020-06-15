import json
import copy
from src.geometry import Point3d, Vector, Triangle
import math
from pathlib import Path


def get_project_root() -> Path:
    """Returns project root folder."""
    return Path(__file__).parent.parent


def mirror(pivot: Vector, center: Vector, norm: Vector):
    base = pivot - center
    norm.normalize()
    a = copy.deepcopy(base)
    a.rotate(Triangle.get_rotate_matrix(norm, 2 / 3 * math.pi))
    a = center + a
    b = copy.deepcopy(base)
    b.rotate(Triangle.get_rotate_matrix(norm, -2 / 3 * math.pi))
    b = center + b
    return [pivot.get_point(), a.get_point(), b.get_point()]


def parse(model: str) -> {}:
    with open(str(get_project_root()) + '/configs/' + model + '_input.json', 'r') as input_file:
        data = input_file.read()

    configs = json.loads(data)
    return {
        'start_laser_location': Point3d(configs['laser_loc'][0], configs['laser_loc'][1], configs['laser_loc'][2]),
        'start_laser_direction': Vector(Point3d(configs['laser_dir'][0], configs['laser_dir'][1], configs['laser_dir'][2])),
        'down_triangle': mirror(
            Vector(Point3d(configs['down_tr'][0][0], configs['down_tr'][0][1], configs['down_tr'][0][2])),
            Vector(Point3d(configs['down_tr'][1][0], configs['down_tr'][1][1], configs['down_tr'][1][2])),
            Vector(Point3d(configs['down_tr'][2][0], configs['down_tr'][2][1], configs['down_tr'][2][2]))),
        'up_triangle': mirror(
            Vector(Point3d(configs['up_triangle'][0][0], configs['up_triangle'][0][1], configs['up_triangle'][0][2])),
            Vector(Point3d(configs['up_triangle'][1][0], configs['up_triangle'][1][1], configs['up_triangle'][1][2])),
            Vector(Point3d(configs['up_triangle'][2][0], configs['up_triangle'][2][1], configs['up_triangle'][2][2]))),
        'triangle_3': mirror(
            Vector(Point3d(configs['triangle_3'][0][0], configs['triangle_3'][0][1], configs['triangle_3'][0][2])),
            Vector(Point3d(configs['triangle_3'][1][0], configs['triangle_3'][1][1], configs['triangle_3'][1][2])),
            Vector(Point3d(configs['triangle_3'][2][0], configs['triangle_3'][2][1], configs['triangle_3'][2][2]))),
        'triangle_4': mirror(
            Vector(Point3d(configs['triangle_4'][0][0], configs['triangle_4'][0][1], configs['triangle_4'][0][2])),
            Vector(Point3d(configs['triangle_4'][1][0], configs['triangle_4'][1][1], configs['triangle_4'][1][2])),
            Vector(Point3d(configs['triangle_4'][2][0], configs['triangle_4'][2][1], configs['triangle_4'][2][2]))),
        "target_radius": configs["target_radius"]
    }
