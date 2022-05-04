#!/usr/bin/env roseus

;; TODO: get real robot angle vector

(load "package://yamaopt_ros/scripts/pub-robot-plane-common.l")

(load "package://fetcheus/fetch-interface.l")
;; (load "package://fetcheus/fetch-utils.l")

(fetch-init)
;; (fetch)
(send *fetch* :angle-vector (send *ri* :state :potentio-vector :wait-until-update t))

(defun fetch-attachable-cubes ()
  (let (cubes head-tilt head-left head-right torso-left torso-right torso-back base-top
        coefs head-tilt-coef head-left-coef head-right-coef torso-left-coef torso-right-coef torso-back-coef base-top-coef)
    ;; head-tilt
    (setq head-tilt (make-cube 80 240 5))
    (send head-tilt :transform
          (send (robot-link-coords *fetch* "head_tilt_link") :translate #f(0 0 60))
          :world)
    (send head-tilt :copy-worldcoords)
    (setq head-tilt-coef (float-vector 0 0 1 (* -1 (elt (send head-tilt :worldpos) 2))))
    ;; head-left
    (setq head-left (make-cube 180 5 80))
    (send head-left :transform
          (send (robot-link-coords *fetch* "head_pan_link") :translate #f(70 -145 50))
          :world)
    (send head-left :copy-worldcoords)
    (setq head-left-coef (float-vector 0 -1 0 (* 1 (elt (send head-left :worldpos) 1))))
    ;; head-right
    (setq head-right (make-cube 180 5 80))
    (send head-right :transform
          (send (robot-link-coords *fetch* "head_pan_link") :translate #f(70 145 50))
          :world)
    (setq head-right-coef (float-vector 0 1 0 (* -1 (elt (send head-right :worldpos) 1))))
    ;; torso-left
    (setq torso-left (make-cube 100 5 480))
    (send torso-left :transform
          (send (robot-link-coords *fetch* "torso_lift_link") :translate #f(-50 -180 260))
          :world)
    (setq torso-left-coef (float-vector 0 -1 0 (* 1 (elt (send torso-left :worldpos) 1))))
    ;; torso-right
    (setq torso-right (make-cube 100 5 480))
    (send torso-right :transform
          (send (robot-link-coords *fetch* "torso_lift_link") :translate #f(-50 180 260))
          :world)
    (setq torso-right-coef (float-vector 0 1 0 (* -1 (elt (send torso-right :worldpos) 1))))
    ;; torso-back
    (setq torso-back (make-cube 5 200 500))
    (send torso-back :transform
          (send (robot-link-coords *fetch* "torso_lift_link") :translate #f(-190 0 260))
          :world)
    (setq torso-back-coef (float-vector -1 0 0 (elt (send torso-back :worldpos) 0)))
    ;; base-top
    (setq base-top (make-cube 130 200 5))
    (send base-top :transform
          (send (robot-link-coords *fetch* "base_link") :translate #f(200 0 360))
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

(defun main ()
  ;; TODO: set appropriate rate
  (ros::rate 10)
  (do-until-key
   (pub-robot-plane *fetch*)
   (ros::spin-once)
   (ros::sleep)
   (x::window-main-one)))

(main)
