#!/usr/bin/env python

import numpy as np
import os

import skrobot
from skrobot.interfaces.ros import tf_utils
from skrobot.planner.utils import set_robot_config
from yamaopt.solver import KinematicSolver, SolverConfig
from yamaopt.visualizer import VisManager

import rospkg
import rospy
from std_msgs.msg import Float32, String
from yamaopt_ros.srv import SensorPlacement, SensorPlacementResponse


class CalcSensorPlacement(object):
    def __init__(self):
        # Get rosparam
        robot_name = rospy.get_param('~robot_name', 'pr2')
        use_base = rospy.get_param('~use_base', True)
        self.d_hover = rospy.get_param('~d_hover', 0.05)
        self.visualize = rospy.get_param('~visualize', False)
        # Optimization config
        r = rospkg.RosPack()
        if robot_name == 'fetch':
            config_path = r.get_path('yamaopt_ros') + "/config/fetch_conf.yaml"
        elif robot_name == 'pr2':
            config_path = r.get_path('yamaopt_ros') + "/config/pr2_conf.yaml"
        else:
            raise Exception()
        self.config = SolverConfig.from_config_path(
            config_path, use_base=use_base)
        self.kinsol = KinematicSolver(self.config)
        # Robot config
        self.robot = skrobot.model.RobotModel()
        urdf_path = os.path.expanduser(self.config.urdf_path)
        self.robot.load_urdf_file(urdf_path)
        # Advertise Service
        rospy.Service('sensor_placement', SensorPlacement, self.solve)

    def polygon_array_to_polygon_list(self, polygon_array):
        """
        Args:
            polygon_array (jsk_recognition_msgs/PolygonArray)

        Returns:
            polygon_list (list of numpy array 3d points):
        """
        polygon_list = []
        for polygon_stamped in polygon_array.polygons:
            polygon_msg = polygon_stamped.polygon
            polygon = None
            for point in polygon_msg.points:
                np_point = np.array([point.x, point.y, point.z])
                if polygon is None:
                    polygon = np.array([np_point])
                else:
                    polygon = np.append(polygon, [np_point], axis=0)
            polygon = np.array(polygon)
            polygon_list.append(polygon)
        return polygon_list

    def solve(self, req):
        rospy.loginfo("Calculate sensor placement.")
        # Polygons and target pos
        polygons = self.polygon_array_to_polygon_list(req.polygon_array)
        target_pos = req.target_point
        target_pos = np.array([target_pos.x, target_pos.y, target_pos.z])
        # Solve optimization
        q_init = -np.ones(len(self.kinsol.control_joint_ids)) * 0.4
        sol, target_polygon = self.kinsol.solve_multiple(
            q_init, polygons, target_pos, d_hover=self.d_hover)
        assert sol.success
        rospy.loginfo('Calculation finished successfully')
        # Return rosservice response
        joints = [self.robot.__dict__[name]
                  for name in self.config.control_joint_names]
        set_robot_config(
            self.robot, joints, sol.x, with_base=self.config.use_base)
        res = SensorPlacementResponse()
        res.joint_names = [String(j.name) for j in joints]
        if self.config.use_base is True:
            av = sol.x[:-3]
        else:
            av = sol.x
        res.angle_vector = [Float32(i) for i in av]
        res.base_pose = tf_utils.coords_to_geometry_pose(sp.robot.coords())
        # Visualize in scikit-robot
        if self.visualize:
            vm = VisManager(self.config)
            vm.add_polygon_list(polygons)
            vm.add_target(target_pos)
            vm.set_angle_vector(sol.x)
            vm.show_while()
        return res


if __name__ == '__main__':
    rospy.init_node('sensor_placement')
    sp = CalcSensorPlacement()
    rospy.spin()
