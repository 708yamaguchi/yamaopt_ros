#!/usr/bin/env roseus

(load "package://yamaopt_ros/scripts/pub-robot-plane-common.l")
(load "package://fetcheus/fetch-interface.l")

(setq *create-viewer* t)

(fetch-init)
(send *fetch* :angle-vector (send *ri* :state :potentio-vector :wait-until-update t))

(defun fetch-attachable-cubes ()
  (let (cubes head-tilt head-left head-right
              torso-left torso-right torso-back base-top
              hand-top hand-bottom
        coefs head-tilt-coef head-left-coef head-right-coef
              torso-left-coef torso-right-coef torso-back-coef base-top-coef
              hand-top-coef hand-bottom-coef)
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
    ;; hand-top
    (setq hand-top (make-cube 100 100 5))
    (send hand-top :transform
          (send (robot-link-coords *fetch* "wrist_roll_link") :translate #f(80 0 45))
          :world)
    (send hand-top :worldcoords)
    (let ((normal (send (elt (send hand-top :faces) 1) :normal)) ;; 1 is -z axis face
          (pos (send hand-top :worldpos)))
      (setq hand-top-coef
            (float-vector (elt normal 0) (elt normal 1) (elt normal 2)
                          (* -1 (v. normal pos)))))
    ;; hand-bottom
    (setq hand-bottom (make-cube 100 100 5))
    (send hand-bottom :transform
          (send (robot-link-coords *fetch* "wrist_roll_link") :translate #f(80 0 -45))
          :world)
    (send hand-bottom :worldcoords)
    (let ((normal (send (elt (send hand-bottom :faces) 0) :normal)) ;; 0 is z axis face
          (pos (send hand-bottom :worldpos)))
      (setq hand-bottom-coef
            (float-vector (elt normal 0) (elt normal 1) (elt normal 2)
                          (* -1 (v. normal pos)))))

    ;; set color to cubes and visualize
    (setq cubes (list head-tilt head-left head-right
                      torso-left torso-right torso-back base-top
                      hand-top hand-bottom
                      ))
    (setq coefs (list head-tilt-coef head-left-coef head-right-coef
                      torso-left-coef torso-right-coef torso-back-coef base-top-coef
                      hand-top-coef hand-bottom-coef
                      ))
    (dolist (p cubes)
      (send p :set-color #f(0.8 0 0)))
    (when *create-viewer*
      (objects (append (list *fetch*) cubes)))
    (list cubes coefs)
    ))

(defun main (&optional (gui t))
  (let ((angle-vector (ros::get-param "~angle_vector" nil)))
    (setq angle-vector
          (if angle-vector
            (concatenate float-vector angle-vector)
            nil))
    (setq *create-viewer* gui)
    (ros::rate 10)
    (do-until-key
     (pub-robot-plane *fetch* angle-vector)
     (ros::spin-once)
     (ros::sleep)
     (when *create-viewer*
       (x::window-main-one)))
    ))

;; (main)
