#!/usr/bin/env roseus

(require :unittest "lib/llib/unittest.l")
(load "package://yamaopt_ros/test/test-sensor-placement-common.l")
(load "package://pr2eus/pr2-interface.l")

(init-unit-test)

(pr2-init)

(deftest test-pr2-sensor-placement
  (let ((req (instance yamaopt_ros::SensorPlacementRequest :init))
        tmp accum-polygons accum-coefs movable-polygon target-point
        res res-joint-names res-angle-vector res-base-pose)
    ;; calculate request information for /sensor_placement service
    (clear-accumulated-polygons)
    (setq tmp (get-accumulated-polygons))
    (setq accum-polygons (car tmp))
    (setq accum-coefs (cadr tmp))
    (setq movable-polygon
          (movable-area->msg (list (list 1.0 1.0 0)
                                   (list -1.0 1.0 0)
                                   (list -1.0 -1.0 0)
                                   (list 1.0 -1.0 0))))
    (setq target-point (target-point->msg (list 1.0 0.3 1.0)))
    ;; set request variable
    (send req :polygon_array accum-polygons)
    (send req :coefficients accum-coefs)
    (send req :movable_polygon movable-polygon)
    (send req :target_point target-point)
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
    ;; Assertion for test
    (assert (send (send res :success) :data))
    )
)

(run-all-tests)
(exit)
