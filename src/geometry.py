from math import cos, sin, fabs, asin
from enum import Enum

class Angle(Enum):
    YAW = 1
    PITCH = 2
    ROLL = 3


class Point2d:
    def __init__(self, x: float = 0, y: float = 0):
        self.x: float = x
        self.y: float = y


class Vector2d:
    def __init__(self, point_end: Point2d, point_start: Point2d = Point2d(0, 0)):
        self.x = point_end.x - point_start.x
        self.y = point_end.y - point_start.y

    def pseudo_scalar_prod(self, other: 'Vector2d') -> float:
        return self.x * other.y - other.x * self.y

class Point3d:
    def __init__(self, x: float = 0, y: float = 0, z: float = 0):
        self.x: float = x
        self.y = y
        self.z = z

    def __add__(self, other) -> 'Point3d':
        return Point3d(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other) -> 'Point3d':
        return Point3d(self.x - other.x, self.y - other.y, self.z - other.z)

    def __truediv__(self, constant) -> 'Point3d':
        return Point3d(self.x / constant, self.y / constant, self.z / constant)

    def distance_to_plane(self, plane: 'Triangle') -> float:
        v = Vector(plane.point_a, self)  # V = A - X
        d = plane.n.scalar_prod(v)  # d = N * V
        return d

    def distance_to_point(self, other: 'Point3d') -> float:
       return ((self.x - other.x) ** 2 + (self.y - other.y) ** 2 + (self.z - other.z) ** 2) ** 0.5

    def distance_to_line(self, point1: 'Point3d', point2: 'Point3d'):
        square = fabs(Vector(point2, point1).vector_prod(Vector(self, point1)).length())
        return square / Vector(point2, point1).length()

    def get_point(self):
        return self.x, self.y, self.z

class Vector:
    def __init__(self, point_end: Point3d, point_start: Point3d = Point3d(0, 0, 0)):
        self.x = point_end.x - point_start.x
        self.y = point_end.y - point_start.y
        self.z = point_end.z - point_start.z

    def normalize(self):
        norm = (self.x ** 2 + self.y ** 2 + self.z ** 2) ** 0.5
        self.x /= norm
        self.y /= norm
        self.z /= norm

    def scalar_prod(self, b: 'Vector') -> float:
        return self.x * b.x + self.y * b.y + self.z * b.z

    def vector_prod(self, b: 'Vector') -> 'Vector':
        # right triple vectors
        return Vector(Point3d(
                self.y * b.z - self.z * b.y,
                self.z * b.x - self.x * b.z,
                self.x * b.y - self.y * b.x,
            )
        )
        pass

    def get_point(self) -> Point3d:
        return Point3d(self.x, self.y, self.z)

    def __add__(self, other) -> 'Vector':
        return Vector(Point3d(self.x + other.x, self.y + other.y, self.z + other.z))

    def __sub__(self, other) -> 'Vector':
        return Vector(Point3d(self.x - other.x, self.y - other.y, self.z - other.z))

    def __mul__(self, constant: float) -> 'Vector':
        return Vector(Point3d(self.x * constant, self.y * constant, self.z * constant))

    def __truediv__(self, constant: float) -> 'Vector':
        return Vector(Point3d(self.x / constant, self.y / constant, self.z / constant))

    def projection(self, axe: str):
        projections = {
            'x': self.x,
            'y': self.y,
            'z': self.z
        }
        return projections[axe]

    def length(self) -> float:
        return (self.x ** 2 + self.y ** 2 + self.z ** 2) ** 0.5

    def rotate(self, m: []):
        x = self.x * m[0][0] +  self.y * m[0][1] + self.z * m[0][2]
        y = self.x * m[1][0] + self.y * m[1][1] + self.z * m[1][2]
        z = self.x * m[2][0] +  self.y * m[2][1] + self.z * m[2][2]

        self.x = x
        self.y = y
        self.z = z

class Plane:  # A * x + B * y + C * z + d = 0
    def __init__(self, a: float, b: float, c: float, d: float):
        self.a = a
        self.b = b
        self.c = c
        self.d = d

    # @staticmethod
    # def create_plane_from_parallel_plane_point(plane: 'Plane', point: Point3d):

    def point_projection(self, m: Point3d) -> Point3d:
        t = -(self.a * m.x + self.b * m.y + self.c * m.z + self.d) / (self.a ** 2 + self.b ** 2 + self.c ** 2)
        return Point3d(self.a * t + m.x, self.b * t + m.y, self.c * t + m.z)

class Triangle(Plane):
    def __init__(self, point_a, point_b, point_c):
        self.point_a: Point3d = point_a
        self.point_b: Point3d = point_b
        self.point_c: Point3d = point_c

        self.n: Vector
        self.update_normal()

        d = -(self.point_a.x * self.n.x + self.point_a.y * self.n.y + self.point_a.z * self.n.z)
        super().__init__(self.n.x, self.n.y, self.n.z, d)

    def update_normal(self):
        n = Vector(self.point_b, self.point_a).vector_prod(Vector(self.point_c, self.point_a))
        n.normalize()
        self.n = n

    def rotate_plane(self, angle, angle_name: Angle):
        v_ab = Vector(self.point_b, self.point_a)
        v_ac = Vector(self.point_c, self.point_a)

        x = v_ab + v_ac
        y = v_ab.vector_prod(x)

        if angle_name == Angle.YAW:
            y.normalize()
            m = self.get_rotate_matrix(y, angle)
        elif angle_name == Angle.PITCH:
            z = x.vector_prod(y)
            z.normalize()
            m = self.get_rotate_matrix(z, angle)
        else:  # angle_name == Angle.ROLL:
            x.normalize()
            m = self.get_rotate_matrix(x, angle)

        v_ab.rotate(m)
        v_ac.rotate(m)

        point_b = self.point_a + v_ab
        point_c = self.point_a + v_ac

        return Triangle(self.point_a, point_b, point_c)

    @staticmethod
    def get_rotate_matrix(unit_vector: Vector, a):
        x, y, z = unit_vector.x, unit_vector.y, unit_vector.z
        matrix = [
            [cos(a) + (1 - cos(a)) * x ** 2, (1 - cos(a)) * x * y - sin(a) * z, (1 - cos(a)) * x * z + sin(a) * y],
            [(1 - cos(a)) * y * x + sin(a) * z, cos(a) + (1 - cos(a)) * y ** 2, (1 - cos(a)) * y * z - sin(a) * x],
            [(1 - cos(a)) * z * x - sin(a) * y, (1 - cos(a)) * z * y + sin(a) * x, cos(a) + (1 - cos(a)) * z ** 2],
        ]
        return matrix

    def get_points(self):
        return (
            (self.point_a.x, self.point_a.y, self.point_a.z),
            (self.point_b.x, self.point_b.y, self.point_b.z),
            (self.point_c.x, self.point_c.y, self.point_c.z),
        )

    def get_axe(self, axe_str: str) -> Vector:
        v_ab = Vector(self.point_b, self.point_a)
        v_ac = Vector(self.point_c, self.point_a)

        x = v_ab + v_ac
        y = v_ab.vector_prod(x)

        if axe_str == 'x':
            x.normalize()
            return x
        elif axe_str == 'y':
            y.normalize()
            return y
        else:  # axe_str == 'z':
            z = x.vector_prod(y)
            z.normalize()
            return z
class Ray:
    def __init__(self, location: Point3d,  direction: Vector):
        self.startPos: Point3d = location
        direction.normalize()
        self.dir: Vector = direction
        self.unit_length_dist_point = location + direction

    def intersect_plane(self, plane: Triangle) -> Point3d:
        # normal to plane: N = (B - A) x (C - A)
        n = Vector(plane.point_b, plane.point_a).vector_prod(Vector(plane.point_c, plane.point_a))
        n.normalize()
        v = Vector(plane.point_a, self.startPos)  # V = A - X
        # distance from X to plane
        d = n.scalar_prod(v)  # d = N * V
        # dir projection to N
        e = n.scalar_prod(self.dir) + 0.0001  #  e = N * (Y - X)

        if e == 0:
            raise ValueError()
            #return self.startPos

        return self.startPos + (self.dir * d / e)

    def reflect_plane(self, plane: Triangle) -> 'Ray':
        o = self.intersect_plane(plane)
        # za = Vector(o, self.startPos)
        m = self.startPos + Vector(o, self.startPos) * 2

        n = Vector(plane.point_b, plane.point_a).vector_prod(Vector(plane.point_c, plane.point_a))
        n.normalize()
        v = Vector(plane.point_a, self.startPos)  # V = A - X
        d = n.scalar_prod(v)  # d = N * V

        m_symm = m - n * 2 * d  # symmetrical point

        return Ray(o, Vector(m_symm - o))


def get_euler_angle_from_point_plane(plane: Triangle, new_point: Point3d, plane_point: Point3d) -> float:
    vertical_proj = fabs(Vector(new_point, plane_point).projection('y'))
    distance_to_rotation_p = new_point.distance_to_point(plane.point_a)
    return asin(vertical_proj/distance_to_rotation_p)