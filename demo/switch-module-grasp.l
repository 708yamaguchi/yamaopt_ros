;; Need the following branch.
;; https://github.com/708yamaguchi/jsk_demos/tree/kanazawa-bthesis-demo
(load "package://jsk_2021_10_soup_from_boil/euslisp/util/wrench.l")

(defun put-pose ()
  (send *pr2* :reset-manip-pose)
  (send *pr2* :larm :angle-vector #f(10.401 -8.26124 74.7656 -91.5736 -278.973 -104.833 6.55036))
  (send *pr2* :larm :inverse-kinematics (make-coords :pos #f(473 -30 1050) :rpy (float-vector -pi/2 pi/2 pi)) :use-torso nil)
  (send *pr2* :rarm :angle-vector #f(-23.9263 43.9289 -58.9061 -109.335 -84.8253 -45.7187 504.2))
  (send *pr2* :rarm :inverse-kinematics (make-coords :pos #f(470 58 990) :rpy (float-vector pi/2 0 0)) :use-torso nil)
  )

(defun regrasp-pose ()
  (put-pose)
  (send *pr2* :larm :inverse-kinematics (make-coords :pos #f(473 -10 1060) :rpy (float-vector -pi/2 0 0)))
  )

(defun switch-module-grasp ()
  ;; Initial pose
  (send *ri* :angle-vector (send *pr2* :reset-manip-pose) 5000)
  (send *ri* :wait-interpolation)

  ;; Move larm
  (put-pose)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000 :larm-controller)
  (send *ri* :wait-interpolation)

  ;; Move rarm
  (put-pose)
  (let (av1 av2)
    (send *pr2* :rarm :move-end-pos #f(0 0 -100) :world :use-torso nil)
    (setq av1 (send *pr2* :angle-vector))
    (put-pose)
    (setq av2 (send *pr2* :angle-vector))
    (send *ri* :angle-vector-sequence (list av1 av2) (list 2000 1000) :rarm-controller))
  (send *ri* :wait-interpolation)

  ;; Place sensor by stopping grasping
  (send *ri* :stop-grasp :larm :wait t)

  ;; Move larm up
  (put-pose)
  (send *pr2* :larm :move-end-pos #f(0 0 50) :world :use-torso nil)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000 :larm-controller)
  (send *ri* :wait-interpolation)

  ;; Move pre-regrasp pose
  (regrasp-pose)
  (let (av1 av2)
    (send *pr2* :rarm :move-end-pos #f(0 0 -200) :world :use-torso nil)
    (setq av1 (send *pr2* :angle-vector))
    (regrasp-pose)
    (send *pr2* :rarm :move-end-pos #f(0 0 -50) :world :use-torso nil)
    (setq av2 (send *pr2* :angle-vector))
    (send *ri* :angle-vector-sequence (list av1 av2) (list 2000 1000)))
  (send *ri* :wait-interpolation)

  ;; Detect rarm position by larm
  (let (av avs tms)
    (dotimes (i 10)
      (send *pr2* :larm :move-end-pos #f(0 0 -10) :world)
      (setq av (send *pr2* :angle-vector))
      (setq avs (append avs (list av)))
      (setq tms (append tms (list 400))))
    (send *ri* :angle-vector-sequence avs tms))
  (cancel-motion-when-touch :arm :larm :direction :z :threshold 2)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector :wait-until-update t))
  (send *pr2* :larm :move-end-pos #f(0 0 15) :world :use-torso nil)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)

  ;; Grasp sensor
  (send *ri* :start-grasp :larm :wait t)

  ;; Reset rarm to :reset-manip-pose
  (regrasp-pose)
  (let (av1 av2)
    (send *pr2* :rarm :move-end-pos #f(0 0 -100) :world :use-torso nil)
    (send *pr2* :rarm :move-end-rot 30 :x)
    (setq av1 (send *pr2* :angle-vector))
    (regrasp-pose)
    (send *pr2* :rarm :move-end-pos #f(0 0 -200) :world :use-torso nil)
    (setq av1 (send *pr2* :angle-vector))
    (send *pr2* :reset-manip-pose)
    (setq av2 (send *pr2* :angle-vector))
    (send *ri* :angle-vector-sequence (list av1 av2) (list 5000 3000) :rarm-controller))
  (send *ri* :wait-interpolation)

  ;; Reset larm to :reset-mani-pose
  (send *pr2* :reset-manip-pose)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
)
