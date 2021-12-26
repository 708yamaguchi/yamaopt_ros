#!/usr/bin/env python

import rospy
from jsk_recognition_msgs.msg import PolygonArray
from yamaopt_ros.srv import AccumulatePolygons, AccumulatePolygonsResponse


class AccumPolygons:
    def __init__(self):
        self.sub = rospy.Subscriber("~input", PolygonArray, self.callback)
        # Advertise Service
        rospy.Service(
            '~accum_polygons', AccumulatePolygons, self.accum_polygons)
        rospy.Service(
            '~clear_polygons', AccumulatePolygons, self.clear_polygons)
        self.current_polygons = None
        self.accum_polygons = []
        self.max_polygon_len = rospy.get_param('max_polygon_len', 100)

    def callback(self, msg):
        self.current_polygons = msg

    def accum_polygons(self, req):
        if self.current_polygons is None:
            polygons = []
        else:
            polygons = self.current_polygons.polygons
        rospy.loginfo("accumulating {} polygons".format(len(polygons)))
        # Extend polygons using subscribed topic
        self.accum_polygons.extend(polygons)
        # Extract last self.max_polygon_len polygons
        if len(self.accum_polygons) > self.max_polygon_len:
            self.accum_polygons = self.accum_polygons[
                len(self.accum_polygons)-self.max_polygon_len:]
        # Create response for rosservice
        res = AccumulatePolygonsResponse()
        res_poly = PolygonArray()
        res_poly.header.stamp = rospy.Time.now()
        if self.current_polygons is not None:
            res_poly.header.frame_id = self.current_polygons.header.frame_id
        res_poly.polygons = self.accum_polygons
        res.polygon_array = res_poly
        return res

    def clear_polygons(self, req):
        self.accum_polygons = []
        return AccumulatePolygonsResponse()


if __name__ == '__main__':
    rospy.init_node('accum_polygons', anonymous=True)
    acc = AccumPolygons()
    rospy.spin()
