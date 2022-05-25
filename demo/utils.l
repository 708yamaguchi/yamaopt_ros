#!/usr/bin/env roseus

;; Curry demo. Need the following branch.
;; https://github.com/708yamaguchi/jsk_demos/tree/kanazawa-bthesis-demo
(load "package://jsk_2021_10_soup_from_boil/euslisp/motion/arrangement-at-arrange.l")
(load "package://jsk_2021_10_soup_from_boil/euslisp/motion/ih-at-arrange.l")
(load "package://jsk_2021_10_soup_from_boil/euslisp/motion/tool-at-arrange.l")

;; yamaopt_ros
(ros::load-ros-manifest "yamaopt_ros")
(load "package://yamaopt_ros/test/test-sensor-placement-common.l")
(load "package://yamaopt_ros/scripts/pub-robot-plane-pr2.l")

;; Init node
(ros::roseus "sensor_placement_client")
(pr2-init)

(defun search-sensor-placement (target-arm)
  ;; Accum polygon and coef
  (let ((neck-p-y (list (list 40 0) (list -40 0) (list 0 20) (list 0 -20)))
        (polygon-array (instance jsk_recognition_msgs::PolygonArray :init))
        (coef-array (instance jsk_recognition_msgs::ModelCoefficientsArray :init))
        accum-res-vision accum-res-robot)
    ;; Get vision planes
    (clear-accumulated-polygons "accum_polygons_vision")
    (dolist (p-y neck-p-y)
      (send *pr2* :head :neck-p :joint-angle (car p-y))
      (send *pr2* :head :neck-y :joint-angle (cadr p-y))
      (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
      (send *ri* :wait-interpolation)
      ;; Add both robot body's plane and vision plane
      (dotimes (i 3)
        (setq accum-res-vision
              (ros::service-call "/accum_polygons_vision/accum_polygons"
                                 (instance yamaopt_ros::AccumulatePolygonsRequest :init)))
        (unix:usleep (* 300 1000)))
      )
    ;; Get robot body planes after looking at target point
    (send *pr2* :head :neck-p :joint-angle 30)
    (send *pr2* :head :neck-y :joint-angle 0)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    (unix:usleep (* 300 1000)) ;; wait for robot body planes are published
    (clear-accumulated-polygons "accum_polygons_robot")
    (setq accum-res-body
          (ros::service-call "/accum_polygons_robot/accum_polygons"
                             (instance yamaopt_ros::AccumulatePolygonsRequest :init)))
    ;; Append vision plane and robot body plane
    (send polygon-array :polygons
          (append (send accum-res-vision :polygon_array :polygons)
                  (send accum-res-body :polygon_array :polygons)))
    (send polygon-array :header (send accum-res-body :polygon_array :header))
    (send coef-array :coefficients
          (append (send accum-res-vision :coefficients :coefficients)
                  (send accum-res-body :coefficients :coefficients)))
    (send coef-array :header (send accum-res-body :coefficients :header))
    ;; Set /sensor_placement request and response
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector :wait-until-update t))
    (let ((req (instance yamaopt_ros::SensorPlacementRequest :init))
          (movable-polygon (movable-area->msg
                            ;; (list (list 1.5 1.5 0)
                            ;;       (list -1.5 1.5 0)
                            ;;       (list -1.5 -1.5 0)
                            ;;       (list 1.5 -1.5 0))
                            nil ;; Do not use movable polygon
                            ))
          (target-point (target-point->msg (list 0.8 0.3 1.0)))
          (arm-left (concatenate cons (send *pr2* :larm :angle-vector)))
          (arm-right (concatenate cons (send *pr2* :rarm :angle-vector)))
          (torso (list (send *pr2* :torso :waist-z :joint-angle)))
          (neck (list (send *pr2* :head :neck-y :joint-angle)
                      (send *pr2* :head :neck-p :joint-angle)))
          req-angle-vector req-joint-names
          res res-angle-vector res-base-pose)
      (send req :polygon_array polygon-array)
      (send req :coefficients coef-array)
      (send req :movable_polygon movable-polygon)
      (send req :target_point target-point)
      (setq req-angle-vector
            (append
             (mapcar #'(lambda (x) (instance std_msgs::Float32 :init :data (deg2rad x)))
                     arm-left)
             (mapcar #'(lambda (x) (instance std_msgs::Float32 :init :data (deg2rad x)))
                     arm-right)
             (mapcar #'(lambda (x) (instance std_msgs::Float32 :init :data (/ x 1000.0)))
                     torso)
             (mapcar #'(lambda (x) (instance std_msgs::Float32 :init :data (deg2rad x)))
                     neck)))
      (send req :angle_vector req-angle-vector)
      (setq req-joint-names
            (append (mapcar #'(lambda (x) (send x :name)) (send *pr2* :larm :joint-list))
                    (mapcar #'(lambda (x) (send x :name)) (send *pr2* :rarm :joint-list))
                    (mapcar #'(lambda (x) (send x :name)) (send *pr2* :torso :joint-list))
                    (mapcar #'(lambda (x) (send x :name)) (send *pr2* :head :joint-list))))
      (setq req-joint-names
            (mapcar #'(lambda (x) (instance std_msgs::String :init :data x)) req-joint-names))
      (send req :joint_names req-joint-names)
      ;; Save request for debugging without real PR2
      (setq req-global req)
      (dump-loadable-structure "req-global.l" req-global)
      ;; Service call /sensor_placement
      (setq res (ros::service-call "sensor_placement" req))
      (setq res-angle-vector (calc-res-angle-vector res))
      (send *pr2* target-arm :angle-vector res-angle-vector)
      (ros::ros-info (format nil "~A angle-vector: ~A" target-arm res-angle-vector))
      (objects (list *pr2*))
      (setq res-base-pose (calc-res-base-pose res))
      (send *pr2* :angle-vector)
      )))

(defun place-sensor ()
  (let (arm-name target-arm av-ret)
    ;; Get target arm
    (setq arm-name (ros::get-param "~arm" "right"))
    (cond ((equal arm-name "right") (setq target-arm :rarm))
          ((equal arm-name "left") (setq target-arm :larm)))
    ;; Move robot
    (send *pr2* :reset-manip-pose)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
    (send *ri* :wait-interpolation)
    (send *pr2* :angle-vector (search-sensor-placement target-arm))
    (send *pr2* :angle-vector)
    (send *pr2* target-arm :move-end-pos #f(-50 0 0) :local :use-torso nil)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
    (send *ri* :wait-interpolation)
    ;; Push module to plane
    (let (av avs tms)
      (dotimes (i 10)
        (send *pr2* target-arm :move-end-pos #f(10 0 0) :local)
        (setq av (send *pr2* :angle-vector))
        (setq avs (append avs (list av)))
        (setq tms (append tms (list 400))))
      (send *ri* :angle-vector-sequence avs tms))
    (cancel-motion-when-touch :arm target-arm :direction :x :threshold -1.5)
    ;; Push again
    (let (av avs tms)
      (dotimes (i 2)
        (send *pr2* target-arm :move-end-pos #f(10 0 0) :local)
        (setq av (send *pr2* :angle-vector))
        (setq avs (append avs (list av)))
        (setq tms (append tms (list 400))))
      (send *ri* :angle-vector-sequence avs tms))
    (send *ri* :wait-interpolation)
    ;; Release module
    (setq av-ret (send *ri* :state :potentio-vector))
    (send *ri* :stop-grasp target-arm :wait t)
    (send *pr2* target-arm :move-end-pos #f(-100 0 0) :local :use-torso nil)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    ;; Return PR2 angle-vector when touch
    av-ret)
  )
