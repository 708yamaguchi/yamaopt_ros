#!/usr/bin/env python

import rospy
from jsk_recognition_msgs.msg import PolygonArray


class AccumPolygons:
    def __init__(self):
        self.sub = rospy.Subscriber("~input", PolygonArray, self.callback)
        self.pub = rospy.Publisher("~output", PolygonArray, queue_size=10)
        self.accum_polygons = []
        self.max_polygon_len = rospy.get_param('max_polygon_len', 100)

    def callback(self, msg):
        rospy.loginfo("accumulating {} polygons".format(len(msg.polygons)))
        # Extend polygons using subscribed topic
        self.accum_polygons.extend(msg.polygons)
        # Extract last self.max_polygon_len polygons
        if len(self.accum_polygons) > self.max_polygon_len:
            self.accum_polygons = self.accum_polygons[
                len(self.accum_polygons)-self.max_polygon_len:]
        # Publish accum polygons
        self.publish_accum_polygons()

    def clear_accum_polygons(self):
        self.accum_polygons = []

    def publish_accum_polygons(self):
        if len(self.accum_polygons) > 0:
            pub_msg = PolygonArray()
            pub_msg.header.stamp = rospy.Time.now()
            pub_msg.header.frame_id = self.accum_polygons[0].header.frame_id
            pub_msg.polygons = self.accum_polygons
            self.pub.publish(pub_msg)


if __name__ == '__main__':
    rospy.init_node('accum_polygons', anonymous=True)
    acc = AccumPolygons()
    rospy.spin()
