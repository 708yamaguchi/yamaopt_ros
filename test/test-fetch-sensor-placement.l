#!/usr/bin/env roseus

(require :unittest "lib/llib/unittest.l")
(load "package://yamaopt_ros/test/test-sensor-placement-common.l")
(load "package://fetcheus/fetch-interface.l")

(init-unit-test)

(fetch-init)

(deftest test-fetch-sensor-placement
  (let ((req (instance yamaopt_ros::SensorPlacementRequest :init))
        tmp accum-polygons accum-coefs movable-polygon target-point
        res res-joint-names res-angle-vector res-base-pose)
    ;; calculate request information for /sensor_placement service
    (clear-accumulated-polygons)
    (setq tmp (get-accumulated-polygons))
    (setq accum-polygons (car tmp))
    (setq accum-coefs (cadr tmp))
    (setq movable-polygon
          (movable-area->msg (list (list 4.0 1.0 0)
                                   (list -1.0 1.0 0)
                                   (list -1.0 -1.0 0)
                                   (list 4.0 -1.0 0))))
    (setq target-point (target-point->msg (list 2.0 -0.8 1.0)))
    ;; set request variable
    (send req :polygon_array accum-polygons)
    (send req :coefficients accum-coefs)
    (send req :movable_polygon movable-polygon)
    (send req :target_point target-point)
    ;; Receive response
    (setq res (ros::service-call "sensor_placement" req))
    (setq res-angle-vector (calc-res-angle-vector res))
    (setq res-base-pose (calc-res-base-pose res))
    ;; Move fetch
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
