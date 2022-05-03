#!/usr/bin/env roseus

(load "package://yamaopt_ros/test/test-sensor-placement-common.l")

(ros::roseus "sensor_placement_client")

(defun main()
  (let ((req (instance yamaopt_ros::SensorPlacementRequest :init))
        tmp accum-polygons accum-coefs movable-polygon target-point
        res res-angle-vector res-base-pose)
    ;; calculate request information for /sensor_placement service
    (clear-accumulated-polygons)
    (setq tmp (get-accumulated-polygons))
    (setq accum-polygons (car tmp))
    (setq accum-coefs (cadr tmp))
    (setq movable-polygon
          (movable-area->msg (list (list 1.5 1.5 0)
                                   (list -1.5 1.5 0)
                                   (list -1.5 -1.5 0)
                                   (list 1.5 -1.5 0))))
    (setq target-point (target-point->msg (list 1.0 0.3 1.0)))
    ;; set request variable
    (send req :polygon_array accum-polygons)
    (send req :coefficients accum-coefs)
    (send req :movable_polygon movable-polygon)
    (send req :target_point target-point)
    ;; Receive response (angle vector and base pose)
    (setq res (ros::service-call "sensor_placement" req))
    (setq res-angle-vector (calc-res-angle-vector res))
    (setq res-base-pose (calc-res-base-pose res))
)
