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
      res res-joint-names res-angle-vector res-base-pose)
  ;; Send request
  (send target-point :x 3)
  (send target-point :y 0)
  (send target-point :z 1)
  (send req :target_point target-point)
  (send req :polygon_array accum-polygons)
  ;; Receive response
  (setq res (ros::service-call "sensor_placement" req))
  (setq res-joint-names
        (mapcar #'(lambda (x) (send x :data)) (send res :joint_names)))
  (setq res-angle-vector
        (concatenate float-vector
                     (mapcar #'(lambda (x) (rad2deg (send x :data))) (send res :angle_vector))))
  (setq res-base-pose (ros::tf-pose->coords (send res :base_pose)))
  ;; Move PR2
  (send *pr2* :rarm :angle-vector res-angle-vector)
  (send *pr2* :move-to res-base-pose)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :move-to res-base-pose)
  (objects (list *pr2* (make-coords :pos (ros::tf-point->pos target-point))))
  )
