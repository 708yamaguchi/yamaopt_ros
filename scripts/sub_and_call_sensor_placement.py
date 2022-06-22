#!/usr/bin/env python

import rospy
from yamaopt_ros.srv import SensorPlacement, SensorPlacementRequest


class SubAndCallSensorPlacement(object):
    def __init__(self):
        self.service_name = 'sensor_placement'
        rospy.Subscriber(
            self.service_name + '/request',
            SensorPlacementRequest,
            self.callback)
        rospy.wait_for_service(self.service_name)
        rospy.loginfo('Find service: {}'.format(self.service_name))

    def callback(self, msg):
        srv = rospy.ServiceProxy(self.service_name, SensorPlacement)
        srv(msg)
        exit()


if __name__ == '__main__':
    rospy.init_node('sub_and_call_sensor_placement')
    sc = SubAndCallSensorPlacement()
    rospy.spin()
