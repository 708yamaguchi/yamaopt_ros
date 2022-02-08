#!/usr/bin/env roseus

;; TODO: get real robot angle vector

;; (load "package://fetcheus/fetch-interface.l")
(load "package://fetcheus/fetch-utils.l")

;; (fetch-init)
(fetch)
(ros::roseus "hoge")
;; (send *fetch* :angle-vector (send *ri* :state :potentio-vector :wait-until-update t))

(ros::load-ros-manifest "geometry_msgs")
(ros::load-ros-manifest "pcl_msgs")
(ros::load-ros-manifest "jsk_recognition_msgs")

;; Publish topic which has the same name as plane polygon topic from pointcloud
(defvar *polygon-topic-name* "/organized_multi_plane_segmentation/output_polygon")
(defvar *coef-topic-name* "/organized_multi_plane_segmentation/output_coefficients")
(ros::advertise *polygon-topic-name* jsk_recognition_msgs::PolygonArray)
(ros::advertise *coef-topic-name* jsk_recognition_msgs::ModelCoefficientsArray)

(defun float-vector->point (f)
  (instance geometry_msgs::Point32 :init
            :x (/ (elt f 0) 1000.0)
            :y (/ (elt f 1) 1000.0)
            :z (/ (elt f 2) 1000.0)))

(defun point->float-vector (p)
  (float-vector (send p :x) (send p :y) (send p :z)))

;; Convert euslisp cube to ROS PolygonStamped
(defun cube->polygon-stamped (cube)
  (send cube :worldcoords) ;; make sure that cube vertices are moved
  (let ((polygon-stamped (instance geometry_msgs::PolygonStamped :init))
        (polygon (instance geometry_msgs::Polygon :init))
        (stamp (ros::time-now))
        (frame-id "base_link")
        (vertices (send cube :vertices))
        points)
  ;; Assume that the cube is 5mm thickness
  (cond
   ((eq 5.0 (x-of-cube cube))
    (setq points
          (list (float-vector->point (elt vertices 0))
                (float-vector->point (elt vertices 1))
                (float-vector->point (elt vertices 3))
                (float-vector->point (elt vertices 2)))))
   ((eq 5.0 (y-of-cube cube))
    (setq points
          (list (float-vector->point (elt vertices 0))
                (float-vector->point (elt vertices 1))
                (float-vector->point (elt vertices 7))
                (float-vector->point (elt vertices 4)))))
   ((eq 5.0 (z-of-cube cube))
    (setq points
          (list (float-vector->point (elt vertices 1))
                (float-vector->point (elt vertices 3))
                (float-vector->point (elt vertices 6))
                (float-vector->point (elt vertices 7))))))
  (send polygon :points points)
  (send polygon-stamped :header :stamp stamp)
  (send polygon-stamped :header :frame_id frame-id)
  (send polygon-stamped :polygon polygon)
  polygon-stamped))

;; Convert coef vector to ROS ModelcoefficientsArray
(defun coef->coef-msg (coef)
  (let ((coef-msg (instance pcl_msgs::ModelCoefficients :init)))
    (send coef-msg :values coef)
    coef-msg))

(defun polygon-stamped->coef (polygon-stamped)
  (let* ((points (send (send polygon-stamped :polygon) :points))
         (vec0 (point->float-vector (elt points 0)))
         (vec1 (point->float-vector (elt points 1)))
         (vec2 (point->float-vector (elt points 2))))
    (triangle-normal vec0 vec1 vec2)))

(defun fetch-link-coords (coords-name)
  (let ((links (send *fetch* :links)))
    (dolist (l links)
      (when (equal (send l :name) coords-name)
        (return-from fetch-link-coords (send l :copy-worldcoords)))))
  (return-from fetch-link-coords nil))

(defun fetch-attachable-cubes ()
  (let (cubes head-tilt head-left head-right torso-left torso-right torso-back base-top
        coefs head-tilt-coef head-left-coef head-right-coef torso-left-coef torso-right-coef torso-back-coef base-top-coef)
    ;; head-tilt
    (setq head-tilt (make-cube 80 240 5))
    (send head-tilt :transform
          (send (fetch-link-coords "head_tilt_link") :translate #f(0 0 60))
          :world)
    (send head-tilt :copy-worldcoords)
    (setq head-tilt-coef (float-vector 0 0 1 (* -1 (elt (send head-tilt :worldpos) 2))))
    ;; head-left
    (setq head-left (make-cube 180 5 80))
    (send head-left :transform
          (send (fetch-link-coords "head_pan_link") :translate #f(70 -145 50))
          :world)
    (send head-left :copy-worldcoords)
    (setq head-left-coef (float-vector 0 1 0 (* -1 (elt (send head-left :worldpos) 1))))
    ;; head-right
    (setq head-right (make-cube 180 5 80))
    (send head-right :transform
          (send (fetch-link-coords "head_pan_link") :translate #f(70 145 50))
          :world)
    (setq head-right-coef (float-vector 0 -1 0 (elt (send head-right :worldpos) 1)))
    ;; torso-left
    (setq torso-left (make-cube 100 5 480))
    (send torso-left :transform
          (send (fetch-link-coords "torso_lift_link") :translate #f(-50 -180 260))
          :world)
    (setq torso-left-coef (float-vector 0 1 0 (* -1 (elt (send torso-left :worldpos) 1))))
    ;; torso-right
    (setq torso-right (make-cube 100 5 480))
    (send torso-right :transform
          (send (fetch-link-coords "torso_lift_link") :translate #f(-50 180 260))
          :world)
    (setq torso-right-coef (float-vector 0 -1 0 (elt (send torso-right :worldpos) 1)))
    ;; torso-back
    (setq torso-back (make-cube 5 200 500))
    (send torso-back :transform
          (send (fetch-link-coords "torso_lift_link") :translate #f(-190 0 260))
          :world)
    (setq torso-back-coef (float-vector -1 0 0 (elt (send torso-back :worldpos) 0)))
    ;; base-top
    (setq base-top (make-cube 130 200 5))
    (send base-top :transform
          (send (fetch-link-coords "base_link") :translate #f(200 0 360))
          :world)
    (setq base-top-coef (float-vector 0 0 1 (* -1 (elt (send base-top :worldpos) 0))))
    ;; set color to cubes and visualize
    (setq cubes (list head-tilt head-left head-right torso-left torso-right torso-back base-top))
    (setq coefs (list head-tilt-coef head-left-coef head-right-coef torso-left-coef torso-right-coef torso-back-coef base-top-coef))
    (dolist (p cubes)
      (send p :set-color #f(0.8 0 0)))
    (objects (append (list *fetch*) cubes))
    (list cubes coefs)
    ))

(defun pub-robot-plane ()
  (let* ((polygon-array (instance jsk_recognition_msgs::PolygonArray :init))
         (cubes-and-coefs (fetch-attachable-cubes))
         (cubes (car cubes-and-coefs))
         (polygons (mapcar #'cube->polygon-stamped cubes))
         (coefs (cadr cubes-and-coefs))
         (coef-array (instance jsk_recognition_msgs::ModelCoefficientsArray :init))
         (frame-id (send (car polygons) :header :frame_id))
         (stamp (send (car polygons) :header :stamp)))
    (send polygon-array :polygons polygons)
    (send polygon-array :header :frame_id frame-id)
    (send polygon-array :header :stamp stamp)
    (ros::publish *polygon-topic-name* polygon-array)
    (send coef-array :coefficients (mapcar #'coef->coef-msg coefs))
    (send coef-array :header :frame_id frame-id)
    (send coef-array :header :stamp stamp)
    (ros::publish *coef-topic-name* coef-array)
    ))

(defun main ()
  ;; TODO: set appropriate rate
  (ros::rate 10)
  (do-until-key
   (pub-robot-plane)
   (ros::spin-once)
   (ros::sleep)))

(main)
