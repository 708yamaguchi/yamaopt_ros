#!/usr/bin/env roseus

;; (load "package://pr2eus/pr2-interface.l")
(load "package://pr2eus/pr2.l")
(ros::load-ros-manifest "jsk_recognition_msgs")

;; (pr2-init)
(pr2)
(ros::roseus "pub_broom_box_array")

(setq *broom-box-array* "/broom_box_array")
(ros::advertise *broom-box-array* jsk_recognition_msgs::BoundingBoxArray 1)

(defun cube->bounding-box (cube)
  (let ((pose (ros::coords->tf-pose (send cube :copy-worldcoords)))
        (dim-x (* (x-of-cube cube) 0.001))
        (dim-y (* (y-of-cube cube) 0.001))
        (dim-z (* (z-of-cube cube) 0.001))
        (bb (instance jsk_recognition_msgs::BoundingBox :init)))
    (send bb :header :frame_id "base_footprint")
    (send bb :pose pose)
    (send bb :dimensions :x dim-x)
    (send bb :dimensions :y dim-y)
    (send bb :dimensions :z dim-z)
    bb))

(defun broom-box-array ()
  ;; (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  ;; copied from *look-at-broom* in wireless_sensing/eusilsp/pr2-broom.l
  (send *pr2* :angle-vector
        #f(50.0 60.0 74.0 70.0 -120.0 20.0 -30.0 180.0 -34.0166 46.3037 -60.0949 -99.0876 200.996 -21.3085 -93.6422 -9.70605 47.2676))
  (send *pr2* :move-to (make-coords) :world)
  (let ((box-up (make-cube 200 200 800))
        (box-down (make-cube 200 200 300))
        (rarm-coords (send *pr2* :rarm :end-coords :copy-worldcoords))
        (bb-array (instance jsk_recognition_msgs::BoundingBoxArray :init)))
    (send box-up :transform rarm-coords)
    (send box-up :translate #f(0 0 450) :local)
    (send box-down :transform rarm-coords)
    (send box-down :translate #f(0 0 -180) :local)
    (send bb-array :boxes
          (list (cube->bounding-box box-up) (cube->bounding-box box-down)))
    (send bb-array :header :stamp (ros::time-now))
    (send bb-array :header :frame_id "base_footprint")
    bb-array))

(defun main ()
  (ros::rate 5)
  (while (ros::ok)
    (ros::publish *broom-box-array* (broom-box-array))
    (ros::spin-once)
    (ros::sleep)))

(main)
