#!/usr/bin/env roseus

(load "package://yamaopt_ros/scripts/pub-robot-plane-common.l")
(load "package://pr2eus/pr2-interface.l")

(setq *viewer* t)

(pr2-init)
(send *pr2* :angle-vector (send *ri* :state :potentio-vector :wait-until-update t))

(defun pr2-attachable-cubes ()
  (let (cubes head-tilt head-left head-right torso-left torso-right
              arm-left-1 arm-left-2 arm-right-1 arm-right-2
              torso-back-1 torso-back-2 base-top-left base-top-right
        coefs head-tilt-coef head-left-coef head-right-coef torso-left-coef torso-right-coef
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
    ;; head-left
    (setq head-left (make-cube 200 5 60))
    (send head-left :transform
          (send (robot-link-coords *pr2* "head_tilt_link") :translate #f(-50 -155 110))
          :local)
    (send head-left :worldcoords)
    (let ((normal (send (elt (send head-left :faces) 4) :normal)) ;; 4 is -y axis face
          (pos (send head-left :worldpos)))
      (setq head-left-coef
            (float-vector (elt normal 0) (elt normal 1) (elt normal 2)
                          (* -1 (v. normal pos)))))
    ;; head-right
    (setq head-right (make-cube 200 5 60))
    (send head-right :transform
          (send (robot-link-coords *pr2* "head_tilt_link") :translate #f(-50 155 110))
          :local)
    (send head-right :worldcoords)
    (let ((normal (send (elt (send head-right :faces) 2) :normal)) ;; 2 is y axis face
          (pos (send head-right :worldpos)))
      (setq head-right-coef
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
    (setq torso-left (make-cube 140 5 140))
    (send torso-left :transform
          (send (robot-link-coords *pr2* "torso_lift_link") :translate #f(-10 -210 230))
          :world)
    (setq torso-left-coef (float-vector 0 -1 0 (* 1 (elt (send torso-left :worldpos) 1))))
    ;; torso-right
    (setq torso-right (make-cube 140 5 140))
    (send torso-right :transform
          (send (robot-link-coords *pr2* "torso_lift_link") :translate #f(-10 210 230))
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
    (setq cubes (list head-tilt head-left head-right
                      arm-left-1 arm-left-2 arm-right-1 arm-right-2
                      torso-left torso-right torso-back-1 torso-back-2
                      base-top-left base-top-right))
    (setq coefs (list head-tilt-coef head-left-coef head-right-coef
                      arm-left-1-coef arm-left-2-coef arm-right-1-coef arm-right-2-coef
                      torso-left-coef torso-right-coef torso-back-1-coef torso-back-2-coef
                      base-top-left-coef base-top-right-coef))
    (dolist (p cubes)
      (send p :set-color #f(0.8 0 0)))
    (when *viewer*
      (objects (append (list *pr2*) cubes)))
    (list cubes coefs)
    ))

(defun main (&optional (gui t))
  (setq *viewer* gui)
  ;; TODO: set appropriate rate
  (ros::rate 1)
  (do-until-key
   (pub-robot-plane *pr2*)
   (ros::spin-once)
   (ros::sleep)
   (when *viewer*
     (x::window-main-one))
   ))

;; (main)
