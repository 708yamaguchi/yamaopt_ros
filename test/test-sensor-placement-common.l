(ros::load-ros-manifest "yamaopt_ros")
(ros::load-ros-manifest "jsk_recognition_msgs")

(defun clear-accumulated-polygons ()
  (ros::service-call "/accum_polygons/clear_polygons"
                     (instance yamaopt_ros::AccumulatePolygonsRequest :init)))

(defun get-accumulated-polygons ()
  (let ((accum-polygons (instance jsk_recognition_msgs::PolygonArray :init))
        (accum-coefs (instance jsk_recognition_msgs::ModelCoefficientsArray :init))
        res)
  (dotimes (i 5)
    (unix:sleep 2)
    (setq res
          (ros::service-call "/accum_polygons/accum_polygons"
                             (instance yamaopt_ros::AccumulatePolygonsRequest :init)))
    (send accum-polygons :header (send (send res :polygon_array) :header))
    (send accum-polygons :polygons
          (append (send accum-polygons :polygons)
                  (send (send res :polygon_array) :polygons)))
    (send accum-coefs :header (send (send res :polygon_array) :header))
    (send accum-coefs :coefficients
          (append (send accum-coefs :coefficients)
                  (send (send res :coefficients) :coefficients))))
  (list accum-polygons accum-coefs)))

(defun movable-area->msg (movable-points-list)
  "
input: list of points indicating movable polygon. For example,
(list (x1 y1 z1) (x2 y2 z2) (x3 y3 z3) ...)
"
  (let ((movable-polygon (instance geometry_msgs::PolygonStamped :init)))
    (send movable-polygon :polygon
          (instance geometry_msgs::Polygon :init :points
                    (mapcar #'(lambda (point)
                                (instance geometry_msgs::Point32 :init
                                          :x (elt point 0)
                                          :y (elt point 1)
                                          :z (elt point 2)))
                            movable-points-list)))
    movable-polygon))

(defun target-point->msg (point)
  (instance geometry_msgs::Point :init
            :x (elt point 0) :y (elt point 1) :z (elt point 2)))

(defun calc-res-angle-vector (res)
  (concatenate float-vector
               (mapcar #'(lambda (x) (rad2deg (send x :data)))
                       (send res :angle_vector))))

(defun calc-res-base-pose (res)
  (ros::tf-pose->coords (send res :base_pose)))
