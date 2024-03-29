#!/usr/bin/env roseus

;; yamaopt_ros
(ros::load-ros-manifest "yamaopt_ros")
(load "package://fetcheus/fetch-interface.l")
(load "package://yamaopt_ros/test/test-sensor-placement-common.l")

;; Init node
(ros::roseus "sensor_placement_client")
(fetch-init)

(defun search-sensor-placement (target-arm &optional (use-robot-planes-arg t))
  ;; Accum polygon and coef
  (let ((neck-p-y (list (list 20 0) (list -20 0) (list 0 20) (list 0 -20)))
        (polygon-array (instance jsk_recognition_msgs::PolygonArray :init))
        (coef-array (instance jsk_recognition_msgs::ModelCoefficientsArray :init))
        (use-vision-planes (ros::get-param "~use_vision_planes" t))
        (use-robot-planes (ros::get-param "~use_robot_planes" t))
        (accum-res-vision (instance yamaopt_ros::AccumulatePolygonsResponse :init))
        (accum-res-body (instance yamaopt_ros::AccumulatePolygonsResponse :init)))
    ;; Get vision planes
    (when use-vision-planes
      (clear-accumulated-polygons "accum_polygons_vision")
      (dolist (p-y neck-p-y)
        (send *fetch* :head :neck-p :joint-angle (car p-y))
        (send *fetch* :head :neck-y :joint-angle (cadr p-y))
        (send *ri* :angle-vector-raw (send *fetch* :angle-vector) 1000 :head-controller)
        (send *ri* :wait-interpolation)
        ;; Add both robot body's plane and vision plane
        (dotimes (i 3)
          (setq accum-res-vision
                (ros::service-call "/accum_polygons_vision/accum_polygons"
                                   (instance yamaopt_ros::AccumulatePolygonsRequest :init)))
          (unix:usleep (* 300 1000)))))
    ;; Get robot body planes after looking at target point
    (when use-robot-planes
      (send *fetch* :angle-vector (send *ri* :state :potentio-vector :wait-until-update t))
      (send *fetch* :head :neck-p :joint-angle 30)
      (send *fetch* :head :neck-y :joint-angle 0)
      (send *ri* :angle-vector (send *fetch* :angle-vector) 1000)
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
    (send *fetch* :angle-vector (send *ri* :state :potentio-vector :wait-until-update t))
    (let ((req (instance yamaopt_ros::SensorPlacementRequest :init))
          (movable-polygon (movable-area->msg
                            ;; (list (list 1.5 1.5 0)
                            ;;       (list -1.5 1.5 0)
                            ;;       (list -1.5 -1.5 0)
                            ;;       (list 1.5 -1.5 0))
                            nil ;; Do not use movable polygon
                            ))
          (target-point (target-point->msg (list 0.8 0.1 0.9)))
          ;; (arm-left (concatenate cons (send *fetch* :larm :angle-vector)))
          (arm-right (concatenate cons (send *fetch* :rarm :angle-vector)))
          (torso (list (send *fetch* :torso :waist-z :joint-angle)))
          (neck (list (send *fetch* :head :neck-y :joint-angle)
                      (send *fetch* :head :neck-p :joint-angle)))
          req-angle-vector req-joint-names res-axis
          res res-angle-vector res-base-pose)
      (send req :polygon_array polygon-array)
      (send req :coefficients coef-array)
      (send req :movable_polygon movable-polygon)
      (send req :target_point target-point)
      (setq req-angle-vector
            (append
             ;; (mapcar #'(lambda (x) (instance std_msgs::Float32 :init :data (deg2rad x)))
             ;;         arm-left)
             (mapcar #'(lambda (x) (instance std_msgs::Float32 :init :data (deg2rad x)))
                     arm-right)
             (mapcar #'(lambda (x) (instance std_msgs::Float32 :init :data (/ x 1000.0)))
                     torso)
             (mapcar #'(lambda (x) (instance std_msgs::Float32 :init :data (deg2rad x)))
                     neck)))
      (send req :angle_vector req-angle-vector)
      (setq req-joint-names
            (append ;; (mapcar #'(lambda (x) (send x :name)) (send *fetch* :larm :joint-list))
                    (mapcar #'(lambda (x) (send x :name)) (send *fetch* :rarm :joint-list))
                    (mapcar #'(lambda (x) (send x :name)) (send *fetch* :torso :joint-list))
                    (mapcar #'(lambda (x) (send x :name)) (send *fetch* :head :joint-list))))
      (setq req-joint-names
            (mapcar #'(lambda (x) (instance std_msgs::String :init :data x)) req-joint-names))
      (send req :joint_names req-joint-names)
      ;; Save request for debugging without real PR2
      (setq req-global req)
      (dump-loadable-structure "req-global.l" req-global)
      ;; Service call /sensor_placement
      (setq res (ros::service-call "sensor_placement" req))
      (setq res-angle-vector (calc-res-angle-vector res))
      (send *fetch* target-arm :angle-vector res-angle-vector)
      (setq res-axis (send (send res :axis) :data))
      (ros::ros-info (format nil "axis: ~A target-arm: ~A angle-vector: ~A" res-axis target-arm res-angle-vector))
      (objects (list *fetch*))
      (setq res-base-pose (calc-res-base-pose res))
      (list (send *fetch* :angle-vector) res-axis)
      )))

;; TODO: we need to add information about handling axis (x or z)
(defun place-sensor (&optional (arm "right") (use-robot-planes-arg t))
  (let (arm-name target-arm av-ret sensor-placement-res)
    ;; Get target arm
    (setq arm-name (ros::get-param "~arm" arm))
    (cond ((equal arm-name "right") (setq target-arm :rarm))
          ((equal arm-name "left") (setq target-arm :larm)))
    ;; Search sensor placement
    ;; (send *fetch* :reset-pose)
    ;; (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
    ;; (send *ri* :wait-interpolation)
    (setq sensor-placement-res (search-sensor-placement target-arm))

    ;; If needed, switch module grasping
    (when (equal (cadr sensor-placement-res) "z")
      (switch-module-grasp))

    ;; Push module to plane
    (let (axis move)
      (cond ((equal (cadr sensor-placement-res) "x")
             (setq axis :x)
             (setq move #f(10 0 0)))
            ((equal (cadr sensor-placement-res) "z")
             (setq axis :z)
             (setq move #f(0 0 10))))
      ;; Place sensor pre-pose
      (let (av1 av2)
        (send *fetch* :angle-vector (send *ri* :state :potentio-vector :wait-until-update t))
        (send *fetch* :rarm :angle-vector #f(-68.3597 8.60077 -97.8784 -120.274 5.86314 -38.3702 -23.4518))
        (setq av1 (send *fetch* :angle-vector))
        (send *fetch* :angle-vector (car sensor-placement-res))
        (send *fetch* target-arm :move-end-pos (scale -15 move) :local)
        (setq av2 (send *fetch* :angle-vector))
        (send *ri* :angle-vector-sequence
              (list av1 av2) (list 3000 3000))
        (unix:sleep 6))

        ;; Push module to plane
        (send *fetch* target-arm :move-end-pos (scale 20 move) :local)
        (send *ri* :angle-vector (send *fetch* :angle-vector) 3000)
        (send *ri* :wait-interpolation)

        ;; Release module
        (send *ri* :stop-grasp :wait t)
        (send *fetch* target-arm :move-end-pos
              (scale -15 move) :local :use-torso nil
              :rotation-axis nil)
        (send *ri* :angle-vector (send *fetch* :angle-vector) 2000)
        (send *ri* :wait-interpolation)

        ;; Reset pose
        (send *fetch* :rarm :angle-vector
              #f(-30.4756 -25.263 308.359 -101.81 41.2738 0.0 337.082))
        (send *ri* :angle-vector (send *fetch* :angle-vector) 2000)
        (send *ri* :wait-interpolation)
        (set-arm :mirror 5000))
    ;; Return PR2 angle-vector when touch
    av-ret)
  )

  ;; :mirror means mirror pose of :reset-pose
(defun set-arm (&optional (pose :normal) (tm 15000))
  (cond ((eq pose :normal) (send *fetch* :reset-pose))
        ((eq pose :mirror) (send *fetch* :angle-vector #f(20.0 -75.6304 80.2141 -157.644 -98.5487 0.0 -95.111 0.0 0.0 0.0))))
  (send *ri* :angle-vector (send *fetch* :angle-vector) tm)
  (send *ri* :wait-interpolation))
