#!/usr/bin/env roseus

(load "package://pr2eus/pr2-interface.l")
(ros::load-ros-manifest "yamaopt_ros")
(ros::load-ros-manifest "jsk_recognition_msgs")

(pr2-init)

;; Wait for accum_polygons/output to be published
(unix:sleep 1)

(let ((req (instance yamaopt_ros::SensorPlacementRequest :init))
      (accum-polygons
       (one-shot-subscribe "accum_polygons/output"
                           jsk_recognition_msgs::PolygonArray))
      (target-point (instance geometry_msgs::Point :init))
      res)
  (send target-point :x 3)
  (send target-point :y 0)
  (send target-point :z 1)
  (send req :target_point target-point)
  (send req :polygon_array accum-polygons)
  (print target-point)
  (print accum-polygons)
  (setq res (ros::service-call "sensor_placement" req))

  (print (send res :joint_names))
  (print (send res :angle_vector))
  (print (send res :base_pose))
  )
