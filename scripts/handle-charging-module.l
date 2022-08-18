#!/usr/bin/env roseus

;; You can use the following launch file to detect green charging mark
;; roslaunch yamaopt_ros charging_module_detector.launch

(load "package://fetcheus/fetch-interface.l")
;; ;; This takes long time at first time
(load "package://yamaopt_ros/scripts/simple-navigation.l")
(ros::load-ros-manifest "jsk_recognition_msgs")
(fetch-init)

;; Global variable to save angle vectors to reach for charging module
(setq *av1* #f(50.0 -40.5203 25.6296 126.55 121.381 -64.757 65.4506 -28.088 81.0 54.0))
(setq *av2* #f(50.0 -18.6104 64.4238 107.364 128.973 -126.189 43.7173 -14.9165 81.0 54.0))
(setq *av3* #f(50.0 -13.226 86.975 124.06 126.589 -118.286 56.5119 -42.0648 81.0 54.0))

;; This function assumes to be passed to (sort) function
;; args should be jsk_recognition_msgs::Boundingbox
(defun box-sort-x (box-msg1 box-msg2)
  (> (send box-msg1 :pose :position :x)
     (send box-msg2 :pose :position :x)))

(defun get-charger-boxes ()
  (format t "Searching for charger boxes~%")
  (let ((box-array
         (one-shot-subscribe "/l515_head/depth_registered/boxes"
                             jsk_recognition_msgs::BoundingBoxArray))
        box-list)
    (format t "Find ~A charger boxes~%" (length (send box-array :boxes)))
    ;; Note that sort is destructive
    ;; The first element of box-list has the biggest x position value
    (setq box-list (sort (send box-array :boxes) #'box-sort-x))
    box-list))

(defun look-at-modules (&optional (tm 3000))
  (send *ri* :angle-vector
        (send *fetch* :angle-vector #f(269 76 86.5 -12 92 0.1 93 0.0 75.5 60))
        tm)
  (send *ri* :wait-interpolation))

(defun is-module-charging (module-name)
  (send (one-shot-subscribe (format nil "~A/is_charging" module-name)
                            std_msgs::Bool)
        :data))

;; Before this function, do not forget to call (look-at-module)
;; Smaller index corresponds to the module, which is places at the bigger x position
(defun grasp-module (index)
  (send *ri* :stop-grasp :wait t)
  (let* ((target-box (elt (get-charger-boxes) index))
         (target-pos (ros::tf-point->pos (send target-box :pose :position)))
         (offset-world #f(-30 0 0)) ;; This value changes when camera is calibrated
         (offset-local #f(-55 0 10)) ;; This value is fixed based on the fetch's hand shape
         av1 av2 av3)
    ;; intermidiate pose
    ;; Note that IK is unstable
    (setq av1
          (send *fetch* :angle-vector
                #f(50 -40.5203 25.6296 126.55 121.381 -64.757 65.4506 -28.088 81 54)))
    (setq av2 #f(50.0 -18.6104 64.4238 107.364 128.973 -126.189 43.7173 -14.9165 81.0 54.0))
    (send *fetch* :inverse-kinematics
          (make-coords :pos target-pos :rpy (float-vector pi/2 0 0))
          :use-torso nil)
    (setq av3 (send *fetch* :rarm :move-end-pos offset-world :world :use-torso nil))
    (setq av3 (send *fetch* :rarm :move-end-pos offset-local :local :use-torso nil))
    (setq *av1* av1)
    (setq *av2* av2)
    (setq *av3* av3)
    ;; debug
    (format t "av1: ~A~%" av1)
    (format t "av2: ~A~%" av2)
    (format t "av3: ~A~%" av3)
    ;; Move robot
    (send *ri* :angle-vector-sequence
          (list av1 av2 av3)
          (list 5000 5000 5000)
          )
    (unix:sleep 15)
    ;; Grasp
    (send *ri* :start-grasp :wait t)
    (unix:sleep 1)
    ;; Reverse movement
    (send *ri* :angle-vector-sequence
          (list av2 av1 (send *fetch* :reset-pose))
          (list 5000 5000 5000)
          )
    (unix:sleep 15)
    nil
  ))

(defun charge-module ()
  (send *ri* :angle-vector-sequence
        (list *av1* *av2* *av3*)
        (list 7000 5000 5000)
        )
  (unix:sleep 17)

  ;; check the module is charged
  (let ((current-av *av3*)
        (move-list (list #f(0 20 0) #f(0 -20 0) #f(20 0 0) #f(-20 0 0))))
    (dolist (move move-list)
      (unix:sleep 3) ;; wait for is_charging topics to come
      (when (is-module-charging "/enviii")
        (return))
      (send *fetch* :angle-vector current-av)
      (send *ri* :angle-vector
            (send *fetch* :rarm :move-end-pos move :local :use-torso nil)
            2000)
      (send *ri* :wait-interpolation)
      ))
  (send *ri* :stop-grasp :wait t)

  ;; Reverse movement
  (send *ri* :angle-vector-sequence
        (list *av2* *av1* (send *fetch* :reset-pose))
        (list 5000 5000 5000)
        )
  (unix:sleep 15)
)

(defun pre-task ()
  (look-at-modules)
  (unix:sleep 3) ;; Wait for chaging module recognition
  (grasp-module 0) ;; depending on the task
  (undock)
  (go-to-spot "/eng2/7f/room73B2-sink-front0" :relative-pos #f(400 -500 0) :undock-rotate t)
  )

(defun post-task ()
  (auto-dock)
  (charge-module)
  )
