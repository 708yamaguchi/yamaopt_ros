#!/usr/bin/env python

import rospy
import pickle
from jsk_recognition_msgs.msg import PolygonArray


class Accumulator:
    def __init__(self):
        sub_topic = "/organized_multi_plane_segmentation/output_polygon"
        pub_topic = "/accum_polygon"
        self.sub = rospy.Subscriber(sub_topic, PolygonArray, self.callback)
        self.pub = rospy.Publisher(pub_topic, PolygonArray, queue_size=10)
        self.accum_polygons = []
        self.max_polygon_len = 1000
        self.dump_filename = "accum_polygons.pickle"

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

    def dump_pickle(self):
        rospy.loginfo("dumping {0} polygons to {1}".format(
            len(self.accum_polygons), self.dump_filename))
        with open(self.dump_filename, 'wb') as f:
            pickle.dump(self.accum_polygons, f)

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
    rospy.init_node('listener', anonymous=True, disable_signals=True)
    acc = Accumulator()
    try:
        rospy.loginfo("start node")
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()
    except KeyboardInterrupt:
        acc.dump_pickle()
