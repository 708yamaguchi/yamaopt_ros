#!/usr/bin/env roseus

(load "package://yamaopt_ros/scripts/pub-robot-plane-common.l")
(load "package://pr2eus/pr2-interface.l")

(setq *viewer* t)

(pr2-init)
(send *pr2* :angle-vector (send *ri* :state :potentio-vector :wait-until-update t))

(defun pr2-attachable-cubes ()
  (let (cubes head-tilt head-front head-left head-right
              head-left-below head-right-below
              torso-left torso-right
              arm-left-1 arm-left-2 arm-right-1 arm-right-2
              torso-back-1 torso-back-2 base-top-left base-top-right
        coefs head-tilt-coef head-front-coef head-left-coef head-right-coef
              head-left-below-coef head-right-below-coef
              torso-left-coef torso-right-coef
              arm-left-1-coef arm-left-2-coef arm-right-1-coef arm-right-2-coef
              torso-back-1-coef torso-back-2-coef base-top-left-coef base-top-right-coef)
    ;; head-tilt
    (setq head-tilt (make-cube 200 240 5))
    (send head-tilt :transform
          (send (robot-link-coords *pr2* "head_tilt_link") :translate #f(-50 0 160))
          :local)
    (send head-tilt :worldcoords)
    (let ((normal (send (elt (send head-tilt :faces) 1) :normal)) ;; 1 is -z axis face
          (pos (send head-tilt :worldpos)))
      (setq head-tilt-coef
            (float-vector (elt normal 0) (elt normal 1) (elt normal 2)
                          (* -1 (v. normal pos)))))
    ;; head-front
    (setq head-front (make-cube 5 170 30))
    (send head-front :transform
          (send (robot-link-coords *pr2* "head_tilt_link") :translate #f(80 0 70))
          :local)
    (send head-front :worldcoords)
    (let ((normal (send (elt (send head-front :faces) 3) :normal)) ;; 3 is x axis face
          (pos (send head-front :worldpos)))
      (setq head-front-coef
            (float-vector (elt normal 0) (elt normal 1) (elt normal 2)
                          (* -1 (v. normal pos)))))
    ;; head-left
    (setq head-left (make-cube 150 5 50))
    (send head-left :transform
          (send (robot-link-coords *pr2* "head_tilt_link") :translate #f(-70 -152 115))
          :local)
    (send head-left :worldcoords)
    (let ((normal (send (elt (send head-left :faces) 4) :normal)) ;; 4 is -y axis face
          (pos (send head-left :worldpos)))
      (setq head-left-coef
            (float-vector (elt normal 0) (elt normal 1) (elt normal 2)
                          (* -1 (v. normal pos)))))
    ;; head-right
    (setq head-right (make-cube 150 5 50))
    (send head-right :transform
          (send (robot-link-coords *pr2* "head_tilt_link") :translate #f(-70 152 115))
          :local)
    (send head-right :worldcoords)
    (let ((normal (send (elt (send head-right :faces) 2) :normal)) ;; 2 is y axis face
          (pos (send head-right :worldpos)))
      (setq head-right-coef
            (float-vector (elt normal 0) (elt normal 1) (elt normal 2)
                          (* -1 (v. normal pos)))))
    ;; head-left-below
    (setq head-left-below (make-cube 80 5 80))
    (send head-left-below :transform
          (send (robot-link-coords *pr2* "head_pan_link") :translate #f(50 -160 0))
          :local)
    (send head-left-below :worldcoords)
    (let ((normal (send (elt (send head-left-below :faces) 4) :normal)) ;; 4 is -y axis face
          (pos (send head-left-below :worldpos)))
      (setq head-left-below-coef
            (float-vector (elt normal 0) (elt normal 1) (elt normal 2)
                          (* -1 (v. normal pos)))))
    ;; head-right-below
    (setq head-right-below (make-cube 80 5 80))
    (send head-right-below :transform
          (send (robot-link-coords *pr2* "head_pan_link") :translate #f(50 160 0))
          :local)
    (send head-right-below :worldcoords)
    (let ((normal (send (elt (send head-right-below :faces) 2) :normal)) ;; 2 is -y axis face
          (pos (send head-right-below :worldpos)))
      (setq head-right-below-coef
            (float-vector (elt normal 0) (elt normal 1) (elt normal 2)
                          (* -1 (v. normal pos)))))
    ;; arm-left-1
    (setq arm-left-1 (make-cube 180 5 180))
    (send arm-left-1 :transform
          (send (robot-link-coords *pr2* "l_shoulder_pan_link") :translate #f(50 -130 0))
          :world)
    (send arm-left-1 :worldcoords)
    (let ((normal (send (elt (send arm-left-1 :faces) 4) :normal)) ;; 4 is -y axis face
          (pos (send arm-left-1 :worldpos)))
      (setq arm-left-1-coef
            (float-vector (elt normal 0) (elt normal 1) (elt normal 2)
                          (* -1 (v. normal pos)))))
    ;; arm-left-2
    (setq arm-left-2 (make-cube 180 5 180))
    (send arm-left-2 :transform
          (send (robot-link-coords *pr2* "l_shoulder_pan_link") :translate #f(50 130 0))
          :world)
    (send arm-left-2 :worldcoords)
    (let ((normal (send (elt (send arm-left-2 :faces) 2) :normal)) ;; 2 is y axis face
          (pos (send arm-left-2 :worldpos)))
      (setq arm-left-2-coef
            (float-vector (elt normal 0) (elt normal 1) (elt normal 2)
                          (* -1 (v. normal pos)))))
    ;; arm-right-1
    (setq arm-right-1 (make-cube 180 5 180))
    (send arm-right-1 :transform
          (send (robot-link-coords *pr2* "r_shoulder_pan_link") :translate #f(50 130 0))
          :world)
    (send arm-right-1 :worldcoords)
    (let ((normal (send (elt (send arm-right-1 :faces) 2) :normal)) ;; 2 is y axis face
          (pos (send arm-right-1 :worldpos)))
      (setq arm-right-1-coef
            (float-vector (elt normal 0) (elt normal 1) (elt normal 2)
                          (* -1 (v. normal pos)))))
    ;; arm-right-2
    (setq arm-right-2 (make-cube 180 5 180))
    (send arm-right-2 :transform
          (send (robot-link-coords *pr2* "r_shoulder_pan_link") :translate #f(50 -130 0))
          :world)
    (send arm-right-2 :worldcoords)
    (let ((normal (send (elt (send arm-right-2 :faces) 4) :normal)) ;; 4 is -y axis face
          (pos (send arm-right-2 :worldpos)))
      (setq arm-right-2-coef
            (float-vector (elt normal 0) (elt normal 1) (elt normal 2)
                          (* -1 (v. normal pos)))))
    ;; torso-left
    (setq torso-left (make-cube 120 5 120))
    (send torso-left :transform
          (send (robot-link-coords *pr2* "torso_lift_link") :translate #f(-10 -210 240))
          :world)
    (setq torso-left-coef (float-vector 0 -1 0 (* 1 (elt (send torso-left :worldpos) 1))))
    ;; torso-right
    (setq torso-right (make-cube 120 5 120))
    (send torso-right :transform
          (send (robot-link-coords *pr2* "torso_lift_link") :translate #f(-10 260 240))
          :world)
    (setq torso-right-coef (float-vector 0 1 0 (* -1 (elt (send torso-right :worldpos) 1))))
    ;; torso-back 1
    (setq torso-back-1 (make-cube 5 300 260))
    (send torso-back-1 :transform
          (send (robot-link-coords *pr2* "torso_lift_link") :translate #f(-300 0 100))
          :world)
    (setq torso-back-1-coef (float-vector -1 0 0 (elt (send torso-back-1 :worldpos) 0)))
    ;; torso-back 2
    (setq torso-back-2 (make-cube 5 340 400))
    (send torso-back-2 :transform
          (send (robot-link-coords *pr2* "base_footprint") :translate #f(-335 0 500))
          :world)
    (setq torso-back-2-coef (float-vector -1 0 0 (elt (send torso-back-2 :worldpos) 0)))
    ;; base-top left
    (setq base-top-left (make-cube 200 200 5))
    (send base-top-left :transform
          (send (robot-link-coords *pr2* "base_footprint") :translate #f(180 -200 280))
          :world)
    (setq base-top-left-coef (float-vector 0 0 1 (* -1 (elt (send base-top-left :worldpos) 0))))
    ;; base-top right
    (setq base-top-right (make-cube 200 200 5))
    (send base-top-right :transform
          (send (robot-link-coords *pr2* "base_footprint") :translate #f(180 200 280))
          :world)
    (setq base-top-right-coef (float-vector 0 0 1 (* -1 (elt (send base-top-right :worldpos) 0))))
    ;; set color to cubes and visualize
    ;; Do not publish arm planes because arm planes moves when robot arm moves
    ;; (difficult to calculate sensor placement)
    (setq cubes (list head-tilt ;; head-front
                      head-left head-right
                      head-left-below head-right-below
                      ;; arm-left-1 arm-left-2 arm-right-1 arm-right-2
                      torso-left torso-right torso-back-1 torso-back-2
                      base-top-left base-top-right
                      ))
    (setq coefs (list head-tilt-coef ;; head-front-coef
                      head-left-coef head-right-coef
                      head-left-below-coef head-right-below-coef
                      ;; arm-left-1-coef arm-left-2-coef arm-right-1-coef arm-right-2-coef
                      torso-left-coef torso-right-coef torso-back-1-coef torso-back-2-coef
                      base-top-left-coef base-top-right-coef
                      ))
    (dolist (p cubes)
      (send p :set-color #f(0.8 0 0)))
    (when *viewer*
      (objects (append (list *pr2*) cubes)))
    (list cubes coefs)
    ))

(defun main (&optional (gui t))
  (setq *viewer* gui)
  (ros::rate 10)
  (do-until-key
   (pub-robot-plane *pr2*)
   (ros::spin-once)
   (ros::sleep)
   (when *viewer*
     (x::window-main-one))
   ))

;; (main)
