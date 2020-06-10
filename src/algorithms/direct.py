import math

from src.geometry import *
from src.system.periscope import *
import multiprocessing as mp


class DirectAlgorithm:
    @staticmethod
    def final_ray_target_diff(laser: Ray, down_plane: Triangle, up_plane: Triangle, target: Point3d) -> float:
        ray_to_target = laser.reflect_plane(down_plane).reflect_plane(up_plane)
        return target.distance_to_line(ray_to_target.startPos, ray_to_target.startPos + ray_to_target.dir)

    @staticmethod
    def __rotate_plane_in_best_angle(
            periscope: Periscope,
            mirror_loc: MirrorLocation,
            angle_name: Angle,
            step: int
    ):
        angle = Periscope.EPS_ANGLE_DELTA /( 2 ** step)
        input_ray = periscope.laser.reflect_plane(periscope.mirror_down.triangle)

        if mirror_loc == MirrorLocation.UP:
            current_plane = periscope.mirror_up.triangle
        else:
            current_plane = periscope.mirror_down.triangle

        input_diff = DirectAlgorithm.final_ray_target_diff(periscope.laser, periscope.mirror_down.triangle,
                                                           periscope.mirror_up.triangle, periscope.target.location)

        plane_angle_plus: Triangle = current_plane.rotate_plane(angle, angle_name)
        plane_angle_minus: Triangle = current_plane.rotate_plane(-angle, angle_name)

        if mirror_loc == MirrorLocation.UP:
            diff_angle_plus = DirectAlgorithm.final_ray_target_diff(periscope.laser, periscope.mirror_down.triangle,
                                                                    plane_angle_plus, periscope.target.location)
            diff_angle_minus = DirectAlgorithm.final_ray_target_diff(periscope.laser, periscope.mirror_down.triangle,
                                                                     plane_angle_minus, periscope.target.location)
        else:
            diff_angle_plus = DirectAlgorithm.final_ray_target_diff(periscope.laser, plane_angle_plus,
                                                                    periscope.mirror_up.triangle, periscope.target.location)
            diff_angle_minus = DirectAlgorithm.final_ray_target_diff(periscope.laser, plane_angle_minus,
                                                                    periscope.mirror_up.triangle, periscope.target.location)

        if math.fabs(diff_angle_plus - diff_angle_minus) < 1e-5:
            return

        if diff_angle_plus < diff_angle_minus:
            diff = diff_angle_plus
            angle_sign = 1
            plane_angle_step = plane_angle_plus
        else:
            diff = diff_angle_minus
            angle_sign = -1
            plane_angle_step = plane_angle_minus

        if mirror_loc == MirrorLocation.UP:
            if not DirectAlgorithm.__check_rotate_relevant(input_ray, plane_angle_step):
                return
        else:
            ray = periscope.laser.reflect_plane(plane_angle_step)
            if not DirectAlgorithm.__check_rotate_relevant(ray, periscope.mirror_up.triangle):
                return #current_plane.rotate_plane(angle * -angle_sign, angle_name)
            if not DirectAlgorithm.__check_rotate_relevant(periscope.laser, plane_angle_step):
                return

        prev_diff = input_diff
        angle_step = 1
        while diff < prev_diff:
            angle_step += 1
            new_plane_angle_step: Triangle = current_plane.rotate_plane(angle * angle_step * angle_sign, angle_name)
            prev_diff = diff

            if mirror_loc == MirrorLocation.UP:
                if not DirectAlgorithm.__check_rotate_relevant(input_ray, new_plane_angle_step):
                    return
            else:
                ray = periscope.laser.reflect_plane(new_plane_angle_step)
                if not DirectAlgorithm.__check_rotate_relevant(ray, periscope.mirror_up.triangle):
                    return #current_plane.rotate_plane(angle * -angle_sign, angle_name)
                if not DirectAlgorithm.__check_rotate_relevant(periscope.laser, new_plane_angle_step):
                    return


            plane_angle_step = new_plane_angle_step

            if mirror_loc == MirrorLocation.UP:
                diff = DirectAlgorithm.final_ray_target_diff(periscope.laser, periscope.mirror_down.triangle,
                                                             plane_angle_step, periscope.target.location)
            else:
                diff = DirectAlgorithm.final_ray_target_diff(periscope.laser, plane_angle_step,
                                                             periscope.mirror_up.triangle, periscope.target.location)

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
        iteration = 0

        while True:
            periscope.target = self_queue.get()

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

            self_plane = periscope.mirror_down.triangle
            if plane_loc == MirrorLocation.UP:
                self_plane = periscope.mirror_up.triangle

            arr[0] = self_plane.point_b.x
            arr[1] = self_plane.point_b.y
            arr[2] = self_plane.point_b.z
            arr[3] = self_plane.point_c.x
            arr[4] = self_plane.point_c.y
            arr[5] = self_plane.point_c.z
            iteration += 1