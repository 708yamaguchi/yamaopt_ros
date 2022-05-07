(ros::load-ros-manifest "yamaopt_ros")
(load "package://yamaopt_ros/test/test-sensor-placement-common.l")
(load "package://yamaopt_ros/scripts/pub-robot-plane-pr2.l")

(ros::roseus "sensor_placement_client")

(pr2-init)

(defun main()
  ;; Accum polygon and coef
  (clear-accumulated-polygons)
  (let ((neck-p-y (list (list 40 0) (list -40 0) (list 0 20) (list 0 -20)))
        accum-res)
    (dolist (p-y neck-p-y)
      (send *pr2* :head :neck-p :joint-angle (car p-y))
      (send *pr2* :head :neck-y :joint-angle (cadr p-y))
      (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
      (send *ri* :wait-interpolation)
      ;; Add both robot body's plane and vision plane
      ;; TODO: This program works without errors, but there are following problems:
      ;; With this method, low frequency polygon sometimes does not accumulated
      ;; and calc time is long because the number of polygon is big.
      ;; So, use different topic namefor robot body plane and vision plane
      (dotimes (i 5)
        (setq accum-res
              (ros::service-call "/accum_polygons/accum_polygons"
                                 (instance yamaopt_ros::AccumulatePolygonsRequest :init)))
        (unix:usleep (* 300 1000)
        )))
    ;; Set /sensor_placement request and response
    (let ((req (instance yamaopt_ros::SensorPlacementRequest :init))
          (movable-polygon (movable-area->msg (list (list 1.5 1.5 0)
                                                    (list -1.5 1.5 0)
                                                    (list -1.5 -1.5 0)
                                                    (list 1.5 -1.5 0))))
          (target-point (target-point->msg (list 1.0 0.3 1.0)))
          res res-angle-vector res-base-pose)
      (send req :polygon_array (send accum-res :polygon_array))
      (print (send accum-res :polygon_array))
      (send req :coefficients (send accum-res :coefficients))
      (print (send accum-res :coefficients))
      (send req :movable_polygon movable-polygon)
      (send req :target_point target-point)
      ;; Service call /sensor_placement
      (setq res (ros::service-call "sensor_placement" req))
      (setq res-angle-vector (calc-res-angle-vector res))
      (setq res-base-pose (calc-res-base-pose res))
    )))
