#!/usr/bin/env python

from threading import Lock
import rospy
import message_filters
from jsk_recognition_msgs.msg import PolygonArray
from jsk_recognition_msgs.msg import ModelCoefficientsArray
from yamaopt_ros.srv import AccumulatePolygons, AccumulatePolygonsResponse


class AccumPolygons:
    def __init__(self):
        self.lock = Lock()
        # Subscribe polygon and coef with time synchronize
        polygon_sub = message_filters.Subscriber("~input", PolygonArray)
        coef_sub = message_filters.Subscriber(
            "~input_coefficients", ModelCoefficientsArray)
        ts = message_filters.TimeSynchronizer([polygon_sub, coef_sub], 10)
        ts.registerCallback(self.callback)
        # Advertise Service
        rospy.Service(
            '~accum_polygons', AccumulatePolygons, self.accum_polygons)
        rospy.Service(
            '~clear_polygons', AccumulatePolygons, self.clear_polygons)
        self.current_polygons = None
        self.accum_polygons = []
        self.accum_coefs = []
        self.max_polygon_len = rospy.get_param('max_polygon_len', 100)

    def callback(self, polygon, coef):
        self.current_polygons = {'polygon': polygon, 'coef': coef}

    def accum_polygons(self, req):
        with self.lock:
            if self.current_polygons is None:
                polygons = []
                coefs = []
            else:
                polygons = self.current_polygons['polygon'].polygons
                coefs = self.current_polygons['coef'].coefficients
            rospy.loginfo("accumulating {} polygons".format(len(polygons)))
            # Extend polygons using subscribed topic
            self.accum_polygons.extend(polygons)
            self.accum_coefs.extend(coefs)
            # Extract last self.max_polygon_len polygons
            for accums in self.accum_polygons, self.accum_coefs:
                if len(accums) > self.max_polygon_len:
                    accums = accums[len(accums)-self.max_polygon_len:]
            # Create response for rosservice
            res = AccumulatePolygonsResponse()
            now = rospy.Time.now()
            if self.current_polygons is not None:
                poly_frame_id = self.current_polygons[
                    'polygon'].header.frame_id
                coef_frame_id = self.current_polygons[
                    'coef'].header.frame_id
            # polygon
            res_poly = PolygonArray()
            res_poly.header.stamp = now
            res_poly.header.frame_id = poly_frame_id
            res_poly.polygons = self.accum_polygons
            res.polygon_array = res_poly
            # coef
            res_coef = ModelCoefficientsArray()
            res_coef.header.stamp = now
            res_coef.header.frame_id = coef_frame_id
            res_coef.coefficients = self.accum_coefs
            res.coefficients = res_coef
            return res

    def clear_polygons(self, req):
        self.accum_polygons = []
        return AccumulatePolygonsResponse()


if __name__ == '__main__':
    rospy.init_node('accum_polygons', anonymous=True)
    acc = AccumPolygons()
    rospy.spin()
