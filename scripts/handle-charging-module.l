#!/usr/bin/env roseus

;; (ros::roseus "charging_module_detector")
(load "package://fetcheus/fetch-interface.l")
(ros::load-ros-manifest "jsk_recognition_msgs")

;; This function assumes to be passed to (sort) function
;; args should be jsk_recognition_msgs::Boundingbox
(defun box-sort-x (box-msg1 box-msg2)
  (> (send box-msg1 :pose :position :x)
     (send box-msg2 :pose :position :x)))

(defun get-charger-boxes ()
  (let ((box-array
         (one-shot-subscribe "/l515_head/depth_registered/boxes"
                             jsk_recognition_msgs::BoundingBoxArray))
        box-list)
    ;; Note that sort is destructive
    ;; The first element of box-list has the biggest x position value
    (setq box-list (sort (send box-array :boxes) #'box-sort-x))
    box-list))

(defun look-at-modules (&optional (tm 3000))
  (send *ri* :angle-vector
        (send *fetch* :angle-vector #f(50 0 83.923 -180 109.231 0 77.206 0 81 54))
        tm)
  (send *ri* :wait-interpolation))

;; Before this function, do not forget to call (look-at-module)
;; Smaller index corresponds to the module, which is places at the bigger x position
(defun grasp-module (index)
  (let* ((target-box (elt (get-charger-boxes) index))
         (target-pos (ros::tf-point->pos (send target-box :pose :position)))
         (offset-world #f(-30 -30 10)) ;; This value changes when camera is calibrated
         av1 av2 av3)
  ;; intermidiate pose
  (setq av1
        (send *fetch* :angle-vector
              #f(50 -40.5203 25.6296 126.55 121.381 -64.757 65.4506 -28.088 81 54)))
  (send *fetch* :inverse-kinematics
                  (make-coords :pos target-pos :rpy (float-vector pi/2 0 0))
                  :use-torso nil)
  (setq av3 (send *fetch* :rarm :move-end-pos offset-world :world :use-torso nil))
  (setq av2 (send *fetch* :rarm :move-end-pos #f(-50 0 50) :local :use-torso nil))
  ;; debug
  (print av1)
  (print av2)
  (print av3)
  ;; Move robot
  (send *ri* :angle-vector-sequence
        (list av1 av2 av3)
        (list 5000 5000 5000)
        )
  (unix:sleep 15)
  ;; Grasp
  ;; Reverse movement
  nil
  ))
