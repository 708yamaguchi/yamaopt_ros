#!/usr/bin/env python
import argparse
import math
import numpy as np
from skrobot.coordinates.math import rotation_matrix
from yamaopt.solver import KinematicSolver, SolverConfig
from yamaopt.polygon_constraint import polygon_to_trans_constraint
from yamaopt.visualizer import VisManager
import rospkg


class SensorPlacement(object):
    def __init__(self, config_path, use_base):
        self.config = SolverConfig.from_config_path(
            config_path, use_base=use_base)
        self.kinsol = KinematicSolver(self.config)

    def solve(self, visualize):
        # Polygons and target pos
        # TODO: call this function in service call
        polygon1 = np.array([[1.0, -0.5, -0.5], [1.0, 0.5, -0.5], [1.0, 0.5, 0.5], [1.0, -0.5, 0.5]]) + np.array([0, 0, 1.0])
        polygon2 = polygon1.dot(rotation_matrix(math.pi / 2.0, [0, 0, 1.0]).T)
        polygon3 = polygon1.dot(rotation_matrix(-math.pi / 2.0, [0, 0, 1.0]).T)
        polygons = [polygon1, polygon2, polygon3]
        target_obj_pos = np.array([-0.1, 0.7, 0.3])

        # Solve
        q_init = -np.ones(len(self.kinsol.control_joint_ids)) * 0.4
        sol, target_polygon = self.kinsol.solve_multiple(
            q_init, polygons, target_obj_pos, d_hover=d_hover)
        assert sol.success
        if visualize:
            # visualize
            vm = VisManager(self.config)
            vm.add_polygon_list(polygons)
            vm.add_target(target_obj_pos)
            vm.set_angle_vector(sol.x)
            vm.show_while()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-robot', type=str, default='pr2', help='robot name')
    parser.add_argument('-hover', type=float, default='0.05', help='hover distance')
    parser.add_argument('--visualize', action='store_true', help='visualize')
    parser.add_argument('--use_base', action='store_true', help='with base')
    args = parser.parse_args()
    robot_name = args.robot
    visualize = args.visualize
    use_base = args.use_base
    d_hover = args.hover

    r = rospkg.RosPack()
    if robot_name == 'fetch':
        config_path = r.get_path('yamaopt_ros') + "/config/fetch_conf.yaml"
    elif robot_name == 'pr2':
        config_path = r.get_path('yamaopt_ros') + "/config/pr2_rarm_conf.yaml"
    else:
        raise Exception()

    sp = SensorPlacement(config_path, use_base)
    sp.solve(visualize)
