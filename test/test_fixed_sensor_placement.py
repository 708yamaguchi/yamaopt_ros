#!/usr/bin/env python

import math
import numpy as np
from skrobot.coordinates.math import rotation_matrix
import rospy
from geometry_msgs.msg import Point, Point32, PolygonStamped
from jsk_recognition_msgs.msg import PolygonArray
from yamaopt_ros.srv import SensorPlacement, SensorPlacementRequest


def polygon_list_to_polygon_array(polygon_list):
    """
    Args:
        polygon_list (list of numpy array 3d points):

    Returns:
        polygon_array (jsk_recognition_msgs/PolygonArray)
    """
    polygon_array = PolygonArray()
    for polygon in polygon_list:
        points = []
        for point in polygon:
            p = Point32()
            p.x = point[0]
            p.y = point[1]
            p.z = point[2]
            points.append(p)
        polygon_msg = PolygonStamped()
        polygon_msg.polygon.points = points
        polygon_array.polygons.append(polygon_msg)
    return polygon_array


if __name__ == '__main__':

    rospy.init_node('test_sensor_placement')

    polygon1 = np.array(
        [[1.0, -0.5, -0.5],
         [1.0, 0.5, -0.5],
         [1.0, 0.5, 0.5],
         [1.0, -0.5, 0.5]])
    + np.array([0, 0, 1.0])
    polygon2 = polygon1.dot(rotation_matrix(math.pi / 2.0, [0, 0, 1.0]).T)
    polygon3 = polygon1.dot(rotation_matrix(-math.pi / 2.0, [0, 0, 1.0]).T)
    polygon_list = [polygon1, polygon2, polygon3]
    polygon_array = polygon_list_to_polygon_array(polygon_list)

    target_obj_pos = np.array([-0.1, 0.7, 0.3])
    target_point = Point()
    target_point.x = target_obj_pos[0]
    target_point.y = target_obj_pos[1]
    target_point.z = target_obj_pos[2]

    service_name = 'sensor_placement'
    rospy.wait_for_service(service_name)
    sensor_placement = rospy.ServiceProxy(service_name, SensorPlacement)
    req = SensorPlacementRequest()
    req.target_point = target_point
    req.polygon_array = polygon_array
    res = sensor_placement(req)
    rospy.loginfo("Get sensor placement optimization result:")
    rospy.loginfo(res)
