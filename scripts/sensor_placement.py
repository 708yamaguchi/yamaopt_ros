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
from std_msgs.msg import Bool, Float32, String
from yamaopt_ros.srv import SensorPlacement
from yamaopt_ros.srv import SensorPlacementRequest, SensorPlacementResponse


class CalcSensorPlacement(object):
    def __init__(self):
        # Get rosparam
        robot_name = rospy.get_param('~robot_name', 'pr2')
        use_base = rospy.get_param('~use_base', True)
        self.d_hover = rospy.get_param('~d_hover', 0.05)
        self.joint_limit_margin = rospy.get_param('~joint_limit_margin', 5.0)
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
        urdf_path = os.path.expanduser(self.config.urdf_path)
        self.robot = skrobot.model.RobotModel()
        self.robot.load_urdf_file(urdf_path)
        self.robot_orig = skrobot.model.RobotModel()  # to visualize orig pos
        self.robot_orig.load_urdf_file(urdf_path)
        # Advertise Service
        rospy.Service('sensor_placement', SensorPlacement, self.solve)
        # Publish service request and response for rosbag record
        # Tricky because the message types are for rosservice. But it works.
        self.request_pub = rospy.Publisher(
            '/sensor_placement/request', SensorPlacementRequest, queue_size=10)
        self.response_pub = rospy.Publisher(
            '/sensor_placement/response', SensorPlacementResponse,
            queue_size=10)

    def set_angle_vector(self, robot, angle_vector, target_joints):
        current_av = robot.angle_vector()
        for av, tj in zip(angle_vector, target_joints):
            for i, j in enumerate(robot.joint_list):
                if tj.data == j.name:
                    current_av[i] = av.data
        robot.angle_vector(current_av)

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

    def coefficients_to_coefficient_list(self, coefficients):
        """
        Args:
            coefficients (jsk_recognition_msgs/ModelCoefficientsArray)

        Returns:
            coefficient_list (list of numpy array 3d points):
        """
        coefficient_list = []
        for coefficient in coefficients.coefficients:
            coefficient_list.append(coefficient.values)
        return np.array(coefficient_list)

    def solve(self, req):
        rospy.loginfo("Calculate sensor placement.")
        # Polygons and target pos
        polygons = self.polygon_array_to_polygon_list(req.polygon_array)
        coefficients = self.coefficients_to_coefficient_list(req.coefficients)
        movable_polygon = None  # if movable_polygon is not given, set None
        movable_points = req.movable_polygon.polygon.points
        if len(movable_points) != 0:
            movable_polygon = np.array(
                [[point.x, point.y, point.z] for point in movable_points])
        normals = coefficients[:, :3] * -1
        target_obj_pos = req.target_point
        target_obj_pos = np.array(
            [target_obj_pos.x, target_obj_pos.y, target_obj_pos.z])
        # Solve optimization
        q_init = -np.ones(len(self.kinsol.control_joint_ids)) * 0.4
        sol = self.kinsol.solve_multiple(
            q_init, polygons, target_obj_pos, normals=normals,
            movable_polygon=movable_polygon,
            d_hover=self.d_hover, joint_limit_margin=self.joint_limit_margin)
        success = sol.success
        if not success:
            rospy.logerr('Sensor placement SQP optimization failed.')
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
        res.success = Bool(data=success)
        # Publish request and response as rostopic for rosbag record
        self.request_pub.publish(req)
        self.response_pub.publish(res)
        # Visualize in scikit-robot
        if self.visualize:
            vm = VisManager(self.config)
            vm.add_target(target_obj_pos)
            # Reflect current PR2's angle vector
            self.set_angle_vector(
                vm.robot, req.angle_vector, req.joint_names)
            # Reflect sensor placement angle vector
            vm.reflect_solver_result(
                sol, polygons, movable_polygon=movable_polygon,
                normals=normals, show_polygon_axis=True)
            vm.show_while()
        return res


if __name__ == '__main__':
    rospy.init_node('sensor_placement')
    sp = CalcSensorPlacement()
    rospy.spin()
