#!/usr/bin/env roseus

(ros::load-ros-manifest "geometry_msgs")
(ros::load-ros-manifest "pcl_msgs")
(ros::load-ros-manifest "jsk_recognition_msgs")

(ros::roseus "pub_robot_plane")

;; Publish topic which has the same name as plane polygon topic from pointcloud
(defvar *polygon-topic-name* "/polygon_array_transformer/output_polygons")
(defvar *coef-topic-name* "/polygon_array_transformer/output_coefficients")
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

(defun robot-link-coords (robot coords-name)
  (let ((links (send robot :links)))
    (dolist (l links)
      (when (equal (send l :name) coords-name)
        (return-from robot-link-coords (send l :copy-worldcoords)))))
  (return-from robot-link-coords nil))

(defun robot-attachable-cubes (robot)
  (cond
   ((and (boundp '*fetch*) (eq robot *fetch*))
    (send *fetch* :angle-vector (send *ri* :state :potentio-vector :wait-until-update t))
    (fetch-attachable-cubes))
   ((and (boundp '*pr2*) (eq robot *pr2*))
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector :wait-until-update t))
    (pr2-attachable-cubes))
   ))

(defun polygon-and-coef-msg (robot)
  (let* ((polygon-array (instance jsk_recognition_msgs::PolygonArray :init))
         (cubes-and-coefs (robot-attachable-cubes robot))
         (cubes (car cubes-and-coefs))
         (polygons (mapcar #'cube->polygon-stamped cubes))
         (coefs (cadr cubes-and-coefs))
         (coef-array (instance jsk_recognition_msgs::ModelCoefficientsArray :init))
         (frame-id (send (car polygons) :header :frame_id))
         (stamp (send (car polygons) :header :stamp)))
    (send polygon-array :polygons polygons)
    (send polygon-array :header :frame_id frame-id)
    (send polygon-array :header :stamp stamp)
    (send coef-array :coefficients (mapcar #'coef->coef-msg coefs))
    (send coef-array :header :frame_id frame-id)
    (send coef-array :header :stamp stamp)
    (list polygon-array coef-array)
    ))

(defun pub-robot-plane (robot)
  (let* ((polygon-and-coef (polygon-and-coef-msg robot))
         (polygon-array (car polygon-and-coef))
         (coef-array (cadr polygon-and-coef)))
    (ros::publish *polygon-topic-name* polygon-array)
    (ros::publish *coef-topic-name* coef-array)))
