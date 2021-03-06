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
        arm = rospy.get_param('~arm', 'right')
        use_base = rospy.get_param('~use_base', True)
        self.d_hover = rospy.get_param('~d_hover', 0.05)
        self.polygon_shrink = rospy.get_param('~polygon_shrink', 0.03)
        self.joint_limit_margin = rospy.get_param(
            '~joint_limit_margin', (5 / 180.0) * np.pi)
        sensor_type = rospy.get_param('~sensor_type', "none")
        self.visualize = rospy.get_param('~visualize', False)
        self.pub_service = rospy.get_param('~pub_service', True)
        # Optimization config
        r = rospkg.RosPack()
        if robot_name == 'fetch':
            config_path = r.get_path('yamaopt_ros') + "/config/fetch_conf.yaml"
        elif robot_name == 'pr2':
            if arm == 'right':
                config_path = r.get_path(
                    'yamaopt_ros') + "/config/pr2_rarm_conf.yaml"
            elif arm == 'left':
                config_path = r.get_path(
                    'yamaopt_ros') + "/config/pr2_larm_conf.yaml"
            else:
                rospy.logerr("'~arm' param is right or left.")
        else:
            raise Exception()
        # Change optframe_xyz_from_ef based on /sensor_type topic.
        if sensor_type == "none":
            rospy.logwarn('Wait for /sensor_type topic to come ...')
            msg = rospy.wait_for_message('/sensor_type', String, timeout=None)
            sensor_type = msg.data
        # Here, self.config_x(self.config_z) is for align_axis_name == 'x(z)'
        if sensor_type in ['thermography']:
            optframe_xyz_from_ef = [0.0, 0.0, 1.0]
        elif sensor_type in ['ai_camera']:
            optframe_xyz_from_ef = [0.0, 0.0, 1.0]
        elif sensor_type in ['microphone'] or sensor_type in ['co2_sensor']:
            optframe_xyz_from_ef = [0.0, 0.0, 0.0]
        else:
            optframe_xyz_from_ef = [0.0, 0.0, 0.0]
        self.config_x = SolverConfig.from_config_path(
            config_path, use_base=use_base,
            optframe_xyz_from_ef=optframe_xyz_from_ef)
        self.config_z = SolverConfig.from_config_path(
            config_path, use_base=use_base,
            optframe_xyz_from_ef=[-1 * optframe_xyz_from_ef[2],
                                  optframe_xyz_from_ef[1],
                                  optframe_xyz_from_ef[0]])
        self.kinsol_x = KinematicSolver(self.config_x)
        self.kinsol_z = KinematicSolver(self.config_z)
        # Robot config
        urdf_path = os.path.expanduser(self.config_x.urdf_path)
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

    def set_angle_vector_for_skrobot(self, robot, angle_vector, target_joints):
        """
        Args:
            angle_vector: std_msgs/Float32[]
            target_joints: std_msgs/String[]
        """
        current_av = robot.angle_vector()
        for av, tj in zip(angle_vector, target_joints):
            for i, j in enumerate(robot.joint_list):
                if tj.data == j.name:
                    current_av[i] = av.data
        robot.angle_vector(current_av)

    def set_angle_vector(self, kinsol, angle_vector, target_joints):
        """
        Args:
            angle_vector: std_msgs/Float32[]
            target_joints: std_msgs/String[]
        """
        joint_names = map(lambda x: x.data, target_joints)
        joint_ids = self.kinsol_x.kin.get_joint_ids(joint_names)
        angle_vectors = map(lambda x: x.data, angle_vector)
        kinsol.kin.set_joint_angles(joint_ids, angle_vectors)

    def clamp_angle_vector(self, joint_list, angle_vector, target_joints):
        """
        Args:
            joint_list: List of scikit-robot's RotationalJoint and LinearJoint
            angle_vector: Input angle vector (std_msgs/Float32[])
            target_joints: Joint names
                           according to angle_vector (std_msgs/String[])

        Returns:
            ret: Clamed angle vecor (std_msgs/Float32[])
        """
        ret = []
        for av, tj in zip(angle_vector, target_joints):
            for j in joint_list:
                if tj.data == j.name:
                    if j.max_angle == float('inf'):
                        clamped_angle = av.data % (2 * np.pi)
                    else:
                        clamped_angle = av.data
                    ret.append(Float32(clamped_angle))
        return ret

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
        # Set current PR2 angle vector to solver
        min_cost = np.inf
        min_sol = None
        min_config = None
        min_axis = None
        success = False
        for align_axis_name, config, kinsol in zip(
                ['x', 'z'],
                [self.config_x, self.config_z],
                [self.kinsol_x, self.kinsol_z]):
            q_init = -np.ones(len(kinsol.control_joint_ids)) * 0.4
            self.set_angle_vector(kinsol, req.angle_vector, req.joint_names)
            sol = kinsol.solve_multiple(
                q_init, polygons, target_obj_pos, normals=normals,
                movable_polygon=movable_polygon,
                align_axis_name=align_axis_name,
                d_hover=self.d_hover,
                polygon_shrink=self.polygon_shrink,
                joint_limit_margin=self.joint_limit_margin)
            success = sol.success
            if success and sol.fun < min_cost:
                min_cost = sol.fun
                min_sol = sol
                min_config = config
                min_axis = align_axis_name
        if success:
            rospy.loginfo('Calculation finished successfully')
            rospy.loginfo('align axis: {}'.format(min_axis))
        else:
            rospy.logerr('Sensor placement SQP optimization failed.')
        # Return rosservice response
        joints = [self.robot.__dict__[name]
                  for name in min_config.control_joint_names]
        set_robot_config(
            self.robot, joints, min_sol.x, with_base=min_config.use_base)
        res = SensorPlacementResponse()
        res.joint_names = [String(j.name) for j in joints]
        if min_config.use_base is True:
            av = min_sol.x[:-3]
        else:
            av = min_sol.x
        # Clamp angle vector between -2*pi and 2*pi
        res.angle_vector = self.clamp_angle_vector(
            self.robot.joint_list, [Float32(i) for i in av], res.joint_names)
        res.base_pose = tf_utils.coords_to_geometry_pose(sp.robot.coords())
        res.axis = String(data=min_axis)
        res.success = Bool(data=success)
        # Publish request and response as rostopic for rosbag record
        if self.pub_service:
            self.request_pub.publish(req)
            self.response_pub.publish(res)
        # Visualize in scikit-robot
        if self.visualize:
            vm = VisManager(min_config)
            vm.add_target(target_obj_pos)
            # Reflect current PR2's angle vector
            self.set_angle_vector_for_skrobot(
                vm.robot, req.angle_vector, req.joint_names)
            # Reflect sensor placement angle vector
            vm.reflect_solver_result(
                min_sol, polygons, movable_polygon=movable_polygon,
                normals=normals, show_polygon_axis=True)
            vm.show_while()
        return res


if __name__ == '__main__':
    rospy.init_node('sensor_placement')
    sp = CalcSensorPlacement()
    rospy.spin()
