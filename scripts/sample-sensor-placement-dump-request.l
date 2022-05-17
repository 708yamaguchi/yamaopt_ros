#!/usr/bin/env roseus

(ros::load-ros-manifest "yamaopt_ros")
(ros::roseus "sensor_placement_client")

(load "package://pr2eus/pr2.l")
(load "package://yamaopt_ros/test/test-sensor-placement-common.l")
(load "package://yamaopt_ros/data/req-global.l")

(ros::wait-for-service "sensor_placement")
(setq res (ros::service-call "sensor_placement" req-global))
(setq res-angle-vector (calc-res-angle-vector res))
(setq res-base-pose (calc-res-base-pose res))

(pr2)
(print res-angle-vector)
;; Note that angle-vector except rarm is not reflected
(send *pr2* :rarm :angle-vector res-angle-vector)
(objects (list *pr2*))
