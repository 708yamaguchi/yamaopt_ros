#!/usr/bin/env roseus

(require :unittest "lib/llib/unittest.l")
(load "package://fetcheus/fetch-interface.l")
(ros::load-ros-manifest "yamaopt_ros")
(ros::load-ros-manifest "jsk_recognition_msgs")

(init-unit-test)

(fetch-init)

(deftest test-fetch-sensor-placement
  (let ((req (instance yamaopt_ros::SensorPlacementRequest :init))
        (target-point (instance geometry_msgs::Point :init))
        (accum-polygons (instance jsk_recognition_msgs::PolygonArray :init))
        (accum-coefs (instance jsk_recognition_msgs::ModelCoefficientsArray :init))
        (movable-polygon (instance geometry_msgs::PolygonStamped :init))
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
                    (send (send res :polygon_array) :polygons)))
      (send accum-coefs :header (send (send res :polygon_array) :header))
      (send accum-coefs :coefficients
            (append (send accum-coefs :coefficients)
                    (send (send res :coefficients) :coefficients))))
    ;; Set movable area of Fetch
    (send movable-polygon :polygon
          (instance geometry_msgs::Polygon :init :points
                    (list (instance geometry_msgs::Point32 :init :x 4.0 :y 1.0 :z 0.0)
                          (instance geometry_msgs::Point32 :init :x -1.0 :y 1.0 :z 0.0)
                          (instance geometry_msgs::Point32 :init :x -1.0 :y -1.0 :z 0.0)
                          (instance geometry_msgs::Point32 :init :x 4.0 :y -1.0 :z 0.0))))
    ;; Send sensor placement request
    (send target-point :x 1.0)
    (send target-point :y 0.3)
    (send target-point :z 1.0)
    (send req :target_point target-point)
    (send req :polygon_array accum-polygons)
    (send req :coefficients accum-coefs)
    (send req :movable_polygon movable-polygon)
    ;; Receive response
    (setq res (ros::service-call "sensor_placement" req))
    (setq res-joint-names
          (mapcar #'(lambda (x) (send x :data)) (send res :joint_names)))
    (setq res-angle-vector
          (concatenate float-vector
                       (mapcar #'(lambda (x) (rad2deg (send x :data))) (send res :angle_vector))))
    (setq res-base-pose (ros::tf-pose->coords (send res :base_pose)))
    ;; Move PR2
    (send *fetch* :rarm :angle-vector res-angle-vector)
    (send *fetch* :move-to res-base-pose)
    (send *ri* :angle-vector (send *fetch* :angle-vector))
    (send *ri* :move-to res-base-pose)
    (objects (list *fetch* (make-coords :pos (ros::tf-point->pos target-point))))
    ;; Assertion for test
    (assert (send (send res :success) :data))
    ))

(run-all-tests)
(exit)
