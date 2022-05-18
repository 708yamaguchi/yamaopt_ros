#!/usr/bin/env roseus

(ros::load-ros-manifest "yamaopt_ros")
(ros::roseus "sensor_placement_client")

(load "package://pr2eus/pr2.l")
(load "package://yamaopt_ros/test/test-sensor-placement-common.l")
(load "package://yamaopt_ros/data/req-global.l")

(ros::wait-for-service "sensor_placement")

(let* ((res (ros::service-call "sensor_placement" req-global))
       (res-angle-vector (calc-res-angle-vector res))
       (res-base-pose (calc-res-base-pose res))
       arm-name target-arm)
  (pr2)
  (print res-angle-vector)
  (setq arm-name (ros::get-param "~arm" "right"))
  (cond ((equal arm-name "right") (setq target-arm :rarm))
        ((equal arm-name "left") (setq target-arm :larm)))
  ;; Note that angle-vector except rarm is not reflected
  (send *pr2* target-arm :angle-vector res-angle-vector)
  (objects (list *pr2*))
  )
