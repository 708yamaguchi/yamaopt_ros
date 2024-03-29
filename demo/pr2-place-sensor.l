#!/usr/bin/env roseus

;; Need the following branch.
;; https://github.com/708yamaguchi/jsk_demos/tree/kanazawa-bthesis-demo
;; Load (cancel-motion-when-touch)
(load "package://jsk_2021_10_soup_from_boil/euslisp/util/wrench.l")

;; yamaopt_ros
(ros::load-ros-manifest "yamaopt_ros")
(load "package://pr2eus/pr2-interface.l")
(load "package://yamaopt_ros/test/test-sensor-placement-common.l")
(load "package://yamaopt_ros/demo/switch-module-grasp.l")

;; Init node
(ros::roseus "sensor_placement_client")
(pr2-init)

(defun search-sensor-placement (&optional (target-arm :rarm) (demo "soup_from_boil") (use-robot-planes-arg t))
  ;; Accum polygon and coef
  (let ((polygon-array (instance jsk_recognition_msgs::PolygonArray :init))
        (coef-array (instance jsk_recognition_msgs::ModelCoefficientsArray :init))
        (use-vision-planes (ros::get-param "~use_vision_planes" t))
        (use-robot-planes (ros::get-param "~use_robot_planes" use-robot-planes-arg))
        (accum-res-vision (instance yamaopt_ros::AccumulatePolygonsResponse :init))
        (accum-res-body (instance yamaopt_ros::AccumulatePolygonsResponse :init))
        neck-p-y opt-target-point)
    ;; Change params based on the demo
    (cond
     ((equal demo "soup_from_boil")
      (setq neck-p-y (list (list 40 0) (list -40 0) (list 0 20) (list 0 -20)))
      (setq opt-target-point (list 0.8 0.1 1.1)))
     ((equal demo "pr2_broom")
      (setq neck-p-y (list (list 30 -7.7453)))
      (setq opt-target-point (list 1.1 0.5 0.9))))
    ;; Get vision planes
    (when use-vision-planes
      (clear-accumulated-polygons "accum_polygons_vision")
      (dolist (p-y neck-p-y)
        (send *pr2* :head :neck-p :joint-angle (car p-y))
        (send *pr2* :head :neck-y :joint-angle (cadr p-y))
        (send *ri* :angle-vector (send *pr2* :angle-vector) 1000 :head-controller)
        (send *ri* :wait-interpolation)
        ;; Add both robot body's plane and vision plane
        (dotimes (i 3)
          (setq accum-res-vision
                (ros::service-call "/accum_polygons_vision/accum_polygons"
                                   (instance yamaopt_ros::AccumulatePolygonsRequest :init)))
          (unix:usleep (* 300 1000)))))
    ;; Get robot body planes after looking at target point
    (when use-robot-planes
      (send *pr2* :angle-vector (send *ri* :state :potentio-vector :wait-until-update t))
      (send *pr2* :head :neck-p :joint-angle 30)
      (send *pr2* :head :neck-y :joint-angle 0)
      (send *ri* :angle-vector (send *pr2* :angle-vector) 1000 :head-controller)
      (send *ri* :wait-interpolation)
      (unix:usleep (* 300 1000)) ;; wait for robot body planes are published
      (clear-accumulated-polygons "accum_polygons_robot")
      (setq accum-res-body
            (ros::service-call "/accum_polygons_robot/accum_polygons"
                               (instance yamaopt_ros::AccumulatePolygonsRequest :init))))
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
          (target-point (target-point->msg opt-target-point))
          (arm-left (concatenate cons (send *pr2* :larm :angle-vector)))
          (arm-right (concatenate cons (send *pr2* :rarm :angle-vector)))
          (torso (list (send *pr2* :torso :waist-z :joint-angle)))
          (neck (list (send *pr2* :head :neck-y :joint-angle)
                      (send *pr2* :head :neck-p :joint-angle)))
          req-angle-vector req-joint-names res-axis
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
      (setq res-axis (send (send res :axis) :data))
      (ros::ros-info (format nil "axis: ~A target-arm: ~A angle-vector: ~A" res-axis target-arm res-angle-vector))
      (objects (list *pr2*))
      (setq res-base-pose (calc-res-base-pose res))
      (list (send *pr2* :angle-vector) res-axis)
      )))

;; The default value of pre-pose is :reset-manip-pose
(defun place-sensor (&optional (arm "right")
                               (prepose #f(300.0 75.0 50.0 110.0 -110.0 -20.0 -10.0 -10.0 -75.0 50.0 -110.0 -110.0 20.0 -10.0 -10.0 0.0 50.0))
                               (demo "soup_from_boil"))
  (let (arm-name target-arm av-ret sensor-placement-res)
    ;; Get target arm
    (setq arm-name (ros::get-param "~arm" arm))
    (cond ((equal arm-name "right") (setq target-arm :rarm))
          ((equal arm-name "left") (setq target-arm :larm)))
    ;; Search sensor placement
    (send *pr2* :angle-vector prepose)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
    (send *ri* :wait-interpolation)
    (let (use-robot-plane)
      (cond ((equal demo "soup_from_boil") (setq use-robot-plane t))
            ((equal demo "pr2_broom") (setq use-robot-plane nil)))
      (setq sensor-placement-res (search-sensor-placement target-arm demo use-robot-plane)))

    ;; If needed, switch module grasping
    (when (equal (cadr sensor-placement-res) "z")
      (switch-module-grasp))

    ;; Place sensor pre-pose
    (send *pr2* :angle-vector prepose)
    (let (av1 av2 flip) ;; av1 is intermediate pose
      (send *pr2* :larm :angle-vector #f(44.0768 -1.45561 80.3999 -114.4 -8.36035 -48.7027 -1.59719))
      (setq av1 (send *pr2* :angle-vector))
      (send *pr2* :angle-vector (car sensor-placement-res))
      ;; If PR2 hand coords is wrong, flip. (yamaopt's bug)

      (let ((wc (send *pr2* :copy-worldcoords))
            (lwc (send *pr2* :larm :end-coords :copy-worldcoords))
            z-axis-z)
        ;; temporary move *pr2* to (make-coords)
        (send *pr2* :move-to (make-coords) :world)
        (send lwc :locate #f(0 0 0) :world)
        (setq z-axis-z (elt (send lwc :transform-vector #f(0 0 1)) 2))
        (when (< z-axis-z 0)
          (setq flip t))
        (send *pr2* :move-to wc :world))

      (setq z-axis (send (send *pr2* :larm :end-coords :copy-worldcoords) :z-axis))
      (send *pr2* target-arm :move-end-pos #f(-100 0 0) :local :use-torso nil)
      (when flip
        (ros::ros-error "flip is t")
        (send *pr2* target-arm :wrist-r :joint-angle (- (send *pr2* target-arm :wrist-r :joint-angle) 180)))
      (setq av2 (send *pr2* :angle-vector))
      (send *pr2* :angle-vector (car sensor-placement-res))
      (send *pr2* target-arm :move-end-pos #f(-20 0 0) :local :use-torso nil) ;;
      (when flip
        (ros::ros-error "flip is t")
        (send *pr2* target-arm :wrist-r :joint-angle (- (send *pr2* target-arm :wrist-r :joint-angle) 180)))
      (setq av3 (send *pr2* :angle-vector))
      (send *ri* :angle-vector-sequence (list av1 av2 av3) (list 4000 2000 2000))
      (send *ri* :wait-interpolation))

    ;; Push module to plane
    (let (axis move touch-thre)
      (cond ((equal (cadr sensor-placement-res) "x")
             (setq axis :x)
             (setq move #f(10 0 0))
             (setq touch-thre -1.5))
            ((equal (cadr sensor-placement-res) "z")
             (setq axis :z)
             (setq move #f(0 0 -10))
             (setq touch-thre 1.5)))
      ;; Pre-pose before placing module
      ;; Note that we use d_hover (default: 0.05), so pre-pose might not be needed.
      ;; (send *pr2* target-arm :move-end-pos
      ;;       (scale -5 move) :local :use-torso nil)
      ;; (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
      ;; (send *ri* :wait-interpolation)
      ;; Push module to plane
      ;; (let (av avs tms)
      ;;   (dotimes (i 10)
      ;;     (send *pr2* target-arm :move-end-pos move :local)
      ;;     (setq av (send *pr2* :angle-vector))
      ;;     (setq avs (append avs (list av)))
      ;;     (setq tms (append tms (list 400))))
      ;;   (send *ri* :angle-vector-sequence avs tms))
      ;; (cancel-motion-when-touch :arm target-arm :direction axis :threshold touch-thre)

      ;; (send *pr2* :larm :move-end-rot -2 :y)
      ;; (send *pr2* :larm :move-end-rot -2 :y)

      ;; ;; Push again
      (let (av avs tms)
        (dotimes (i 4)
          (send *pr2* target-arm :move-end-pos move :local)
          (setq av (send *pr2* :angle-vector))
          (setq avs (append avs (list av)))
          (setq tms (append tms (list 400))))
        (send *ri* :angle-vector-sequence avs tms))
      (send *ri* :wait-interpolation)
      ;; head up
      ;; (send *pr2* :head :neck-p :joint-angle
      ;;       (+ -10 (send *pr2* :head :neck-p :joint-angle)))
      ;; (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
      ;; (send *ri* :wait-interpolation)
      ;; guri-guri
      ;; (let (av-orig av1 av2)
      ;;   (setq av-orig (send *pr2* :angle-vector))
      ;;   (send *pr2* target-arm :wrist-r :joint-angle
      ;;         (+ 7.5 (send *pr2* target-arm :wrist-r :joint-angle)))
      ;;   (setq av1 (send *pr2* :angle-vector))
      ;;   (send *pr2* target-arm :wrist-r :joint-angle
      ;;         (+ -15 (send *pr2* target-arm :wrist-r :joint-angle)))
      ;;   (setq av2 (send *pr2* :angle-vector))
      ;;   (send *ri* :angle-vector-sequence (list av1 av2 av-orig) (list 2000 2000 2000))
      ;;   (send *ri* :wait-interpolation))

      ;; Release module
      (setq av-ret (send *ri* :state :potentio-vector))
      (send *ri* :stop-grasp target-arm :wait t)
      (unix:sleep 1)
      (send *pr2* target-arm :move-end-pos #f(-100 0 0) :local :use-torso nil)
      ;; (send *pr2* target-arm :move-end-pos
      ;;       (scale -5 move) :local :use-torso nil)
      (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
      (send *ri* :wait-interpolation))

    ;; Return to prepose
    (cond
     ;; Move to reset-manip-pose with avoiding collision with kitchen
     ((equal demo "soup_from_boil")
      (let (av1 av2)
        (send *pr2* :angle-vector prepose)
        (send *pr2* :head :neck-p :joint-angle 30)
        (send *pr2* :larm :move-end-pos #f(-100 50 0) :world :use-torso nil)
        (setq av1 (send *pr2* :angle-vector))
        (send *pr2* :angle-vector prepose)
        (send *pr2* :head :neck-p :joint-angle 30)
        (setq av2 (send *pr2* :angle-vector))
        (send *ri* :angle-vector-sequence (list av1 av2) (list 4000 1000))
        (send *ri* :wait-interpolation)))
     ((equal demo "pr2_broom")
      (send *pr2* :angle-vector prepose)
      (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
      (send *ri* :wait-interpolation)))

    ;; Return PR2 angle-vector when touch
    av-ret)
  )
