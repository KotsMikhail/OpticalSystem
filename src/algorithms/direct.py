import copy

from src.system.periscope import *
import multiprocessing as mp


class DirectAlgorithm:
    @staticmethod
    def final_ray_target_diff(laser: Ray, periscope: Periscope, target: Point3d) -> float:
        ray_to_target = laser.reflect_plane(periscope.mirror_down.triangle).reflect_plane(periscope.mirror_up.triangle). \
            reflect_plane(periscope.mirror_3.triangle).reflect_plane(periscope.mirror_4.triangle)
        return target.distance_to_line(ray_to_target.startPos, ray_to_target.startPos + ray_to_target.dir)

    @staticmethod
    def __rotate_plane_in_best_angle(
            periscope: Periscope,
            mirror_loc: MirrorLocation,
            angle_name: Angle,
            step: int
    ):
        if mirror_loc == MirrorLocation.FOUR:
            angle = Periscope.EPS_ANGLE_DELTA / (2 ** step)
            input_diff = DirectAlgorithm.final_ray_target_diff(periscope.laser, periscope, periscope.target.location)

            plane_angle_plus: Triangle = periscope.mirror_4.triangle.rotate_plane(angle, angle_name)
            plane_angle_minus: Triangle = periscope.mirror_4.triangle.rotate_plane(-angle, angle_name)
            periscope_plus = copy.deepcopy(periscope)
            periscope_plus.mirror_4 = Mirror(plane_angle_plus)
            periscope_minus = copy.deepcopy(periscope)
            periscope_minus.mirror_4 = Mirror(plane_angle_minus)

            diff_angle_plus = DirectAlgorithm.final_ray_target_diff(periscope.laser, periscope_plus,
                                                                    periscope.target.location)
            diff_angle_minus = DirectAlgorithm.final_ray_target_diff(periscope.laser, periscope_minus,
                                                                     periscope.target.location)

            if diff_angle_plus < diff_angle_minus:
                diff = diff_angle_plus
                angle_sign = 1
                plane_angle_step = plane_angle_plus
            else:
                diff = diff_angle_minus
                angle_sign = -1
                plane_angle_step = plane_angle_minus

            prev_diff = input_diff
            angle_step = 1
            while diff < prev_diff:
                angle_step += 1
                new_plane_angle_step: Triangle = periscope.mirror_4.triangle. \
                    rotate_plane(angle * angle_step * angle_sign, angle_name)
                prev_diff = diff
                periscope_new = copy.deepcopy(periscope)
                periscope_new.mirror_4 = Mirror(new_plane_angle_step)
                diff = DirectAlgorithm.final_ray_target_diff(periscope.laser, periscope_new, periscope.target.location)
                plane_angle_step = new_plane_angle_step
            return plane_angle_step

    # if point (on ray and plane) is in triangle
    @staticmethod
    def __check_rotate_relevant(ray: Ray, plane: Triangle) -> bool:
        point_plane_intersect: Point3d = ray.intersect_plane(plane)
        xz_a = Point2d(plane.point_a.x, plane.point_a.z)
        xz_b = Point2d(plane.point_b.x, plane.point_b.z)
        xz_c = Point2d(plane.point_c.x, plane.point_c.z)
        xz_k = Point2d(point_plane_intersect.x, point_plane_intersect.z)

        is_relevant = True
        is_relevant *= DirectAlgorithm.__on_one_side_of_the_plane(Vector2d(xz_b, xz_a), Vector2d(xz_c, xz_a),
                                                                  Vector2d(xz_k, xz_a))
        is_relevant *= DirectAlgorithm.__on_one_side_of_the_plane(Vector2d(xz_c, xz_a), Vector2d(xz_b, xz_a),
                                                                  Vector2d(xz_k, xz_a))
        is_relevant *= DirectAlgorithm.__on_one_side_of_the_plane(Vector2d(xz_b, xz_c), Vector2d(xz_a, xz_c),
                                                                  Vector2d(xz_k, xz_c))
        return is_relevant

    @staticmethod
    def __on_one_side_of_the_plane(v_plane: Vector2d, v2: Vector2d, vk: Vector2d) -> bool:
        pseudo_scalar_v_plane_vk = v_plane.pseudo_scalar_prod(vk)
        pseudo_scalar_v_plane_v2 = v_plane.pseudo_scalar_prod(v2)

        if pseudo_scalar_v_plane_vk * pseudo_scalar_v_plane_v2 > 0:
            return True

        return False

    @staticmethod
    def correct_one_plane(periscope: Periscope, mirror_loc: MirrorLocation, angle: Angle, step: int):
        new_plane = DirectAlgorithm.__rotate_plane_in_best_angle(periscope, mirror_loc, angle, step)
        if new_plane is None:
            return

        if mirror_loc == MirrorLocation.UP:
            periscope.mirror_up.triangle = new_plane
        elif mirror_loc == MirrorLocation.DOWN:
            periscope.mirror_down.triangle = new_plane
        elif mirror_loc == MirrorLocation.THREE:
            periscope.mirror_3.triangle = new_plane
        elif mirror_loc == MirrorLocation.FOUR:
            periscope.mirror_4.triangle = new_plane

    # implementation for 1 process program
    @staticmethod
    def correct_planes(periscope: Periscope, iteration: int = 0):
        diff = DirectAlgorithm.final_ray_target_diff(periscope.laser, periscope.mirror_down.triangle,
                                                     periscope.mirror_up.triangle, periscope.target.location)

        first_loc_plane = MirrorLocation.UP
        second_loc_plane = MirrorLocation.DOWN
        if iteration % 2 == 0:
            first_loc_plane = MirrorLocation.DOWN
            second_loc_plane = MirrorLocation.UP

        step = 0
        while diff > periscope.target.radius / 2 and step < 10:
            DirectAlgorithm.correct_one_plane(periscope, first_loc_plane, Angle.ROLL, step)
            DirectAlgorithm.correct_one_plane(periscope, first_loc_plane, Angle.PITCH, step)

            DirectAlgorithm.correct_one_plane(periscope, second_loc_plane, Angle.ROLL, step)
            DirectAlgorithm.correct_one_plane(periscope, second_loc_plane, Angle.PITCH, step)

            diff = DirectAlgorithm.final_ray_target_diff(periscope.laser, periscope.mirror_down.triangle,
                                                         periscope.mirror_up.triangle, periscope.target.location)
            step += 1

    @staticmethod
    def plane_direct_process(self_queue: mp.Queue, arr, periscope: Periscope, plane_loc: MirrorLocation):
        while True:
            if plane_loc == MirrorLocation.FOUR:
                periscope.target = self_queue.get()

                diff = DirectAlgorithm.final_ray_target_diff(periscope.laser, periscope, periscope.target.location)
                step = 0
                while diff > periscope.target.radius / 2 and step < 10:
                    DirectAlgorithm.correct_one_plane(periscope, MirrorLocation.FOUR, Angle.ROLL, step)
                    DirectAlgorithm.correct_one_plane(periscope, MirrorLocation.FOUR, Angle.PITCH, step)

                    diff = DirectAlgorithm.final_ray_target_diff(periscope.laser, periscope, periscope.target.location)
                    step += 1

                arr[0] = periscope.mirror_4.triangle.point_b.x
                arr[1] = periscope.mirror_4.triangle.point_b.y
                arr[2] = periscope.mirror_4.triangle.point_b.z
                arr[3] = periscope.mirror_4.triangle.point_c.x
                arr[4] = periscope.mirror_4.triangle.point_c.y
                arr[5] = periscope.mirror_4.triangle.point_c.z
