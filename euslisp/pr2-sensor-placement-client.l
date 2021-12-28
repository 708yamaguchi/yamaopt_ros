#!/usr/bin/env roseus

(load "package://pr2eus/pr2-interface.l")
(ros::load-ros-manifest "yamaopt_ros")
(ros::load-ros-manifest "jsk_recognition_msgs")

(pr2-init)

(let ((req (instance yamaopt_ros::SensorPlacementRequest :init))
      (target-point (instance geometry_msgs::Point :init))
      (accum-polygons (instance jsk_recognition_msgs::PolygonArray :init))
      res res-joint-names res-angle-vector res-base-pose)
  ;; Clear accumlated polygons
  (ros::service-call "/accum_polygons/clear_polygons"
                     (instance yamaopt_ros::AccumulatePolygonsRequest :init))
  ;; Get accumlated polygon
  (dotimes (i 5)
    (unix:sleep 2)
    (setq res
          (ros::service-call "/accum_polygons/accum_polygons"
                             (instance yamaopt_ros::AccumulatePolygonsRequest :init)))
    (send accum-polygons :header (send (send res :polygon_array) :header))
    (send accum-polygons :polygons
          (append (send accum-polygons :polygons)
                  (send (send res :polygon_array) :polygons))))
  ;; Send sensor placement request
  (send target-point :x 1.0)
  (send target-point :y 0.3)
  (send target-point :z 1.0)
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
