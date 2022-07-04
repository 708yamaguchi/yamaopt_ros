;; Copied from https://github.com/708yamaguchi/wireless_sensing/blob/master/euslisp/pr2-broom.l

;; Usage
;; roslaunch wireless_sensing xxx.launch (Please launch rosserial and color recognition)

(load "package://pr2eus/pr2-interface.l")
(load "package://yamaopt_ros/demo/pr2-place-sensor.l")
(ros::load-ros-manifest "jsk_recognition_msgs")

(pr2-init)

(setq *demo-init-pose* #f(52.643 52.9929 30.3742 46.8558 -98.3742 -1365.32 -57.8779 -144.105 -50.7516 11.813 -86.1603 -103.351 60.413 -5.72958 12.3549 4.7767 34.776))

;; (send *pr2* :larm :inverse-kinematics (make-coords :pos #f(600 90 820) :rpy (float-vector (/ -pi 4) 0 pi/2)) :use-torso nil)
;; (send *pr2* :rarm :inverse-kinematics (make-coords :pos #f(630 -20 920) :rpy (float-vector (/ pi 4) 0 0)) :use-torso nil)
(setq *attach-module-pre-pose*
      #f(52.654 36.7816 40.2475 66.9702 -101.449 -1665.7 -24.9766 265.422 -27.9717 21.9281 -68.518 -98.7688 211.195 -28.6571 -144.966 -1.59565 37.8384))

(setq *look-at-broom-pose*
      #f(52.8778 59.8438 28.7232 68.7124 -113.311 -1346.38 -94.1234 -228.505 -27.8964 30.9635 -37.3093 -97.6652 266.923 -45.9292 -165.62 -7.7453 40.2))

;; For small broom
;; TODO: Modify rarm end-coords
;; (send *pr2* :larm :inverse-kinematics (make-coords :pos #f(474 0 1130) :rpy (float-vector (/ -pi 4) 0 0)) :use-torso nil)
;; (send *pr2* :rarm :inverse-kinematics (make-coords :pos #f(474 0 750) :rpy (float-vector (/ pi 4) 0 0)) :use-torso nil)
(setq *grasp-broom-with-both-arm*
      #f(52.689 32.67 -20.2203 68.2928 -117.196 -1296.52 -46.2547 -257.956 -36.4624 35.2632 -100.402 -115.652 153.344 -35.8672 -101.522 -0.009459 37.1023))

;; TODO: change sweep vectors for small broom
(setq *sweep-vectors*
        (list
         (send *pr2* :angle-vector #f(52.758 58.8947 42.183 52.8039 -90.881 -1283.6 -29.7862 -175.134 -75.0 50.0 -110.0 -110.0 20.0 -10.0 -10.0 50.5807 35.904))
         (send *pr2* :larm :inverse-kinematics (make-coords :pos #f(650 386 550) :rpy (float-vector 0 0 1.02)) :use-torso nil :look-at-target (send *pr2* :larm :end-coords))
         (send *pr2* :larm :inverse-kinematics (make-coords :pos #f(650 548 520) :rpy (float-vector 0 0 1.13)) :use-torso nil :look-at-target (send *pr2* :larm :end-coords))
         (send *pr2* :larm :inverse-kinematics (make-coords :pos #f(650 519 390) :rpy (float-vector 0 0 0.90)) :use-torso nil :look-at-target (send *pr2* :larm :end-coords))
         (progn (send *pr2* :angle-vector #f(52.758 58.8947 42.183 52.8039 -90.881 -1283.6 -29.7862 -175.134 -75.0 50.0 -110.0 -110.0 20.0 -10.0 -10.0 50.5807 35.904))
                (send *pr2* :larm :inverse-kinematics (make-coords :pos #f(650 389 480) :rpy (float-vector 0 0 0.716)) :use-torso nil :look-at-target (send *pr2* :larm :end-coords)))
         (send *pr2* :larm :inverse-kinematics (make-coords :pos #f(650 276 650) :rpy (float-vector 0 0 0.38)) :use-torso nil :look-at-target (send *pr2* :larm :end-coords))
         ))
(send *pr2* :angle-vector (send *ri* :state :potentio-vector))

(setq *demo-coords*
      (make-coords :pos #f(1008.151 9636.791 29998.551) :rpy #f(0.097 -0.01 -0.005) :name "world"))

(defun ask-and-wait (ask-str &optional (speak nil))
  (if speak
    (send *ri* :speak-jp ask-str)
    (format t "~A~%" ask-str))
  (do-until-key
   (unix:sleep 1)))

;; アタッチメントをホウキの根本につけて、右手に持たせてください。
;; 左手でも掴めるように、もうひとつのアタッチメントを付けてください。
(defun set-broom-fixture (&optional (speak nil))
  (let ((start-str "ホウキに2つのアタッチメントを取り付けます。")
        (rarm-str "アタッチメントをホウキの根本につけて、右手に持たせてください。右手を閉じてよければ、エンターを押してください。")
        (larm-str "左手でホウキを掴めるように、アタッチメントを取り付けてください。左手を閉じてよければ、エンターを押してください。")
        (finish-str "両手を開いてもよければ、エンターを押してください。"))
    ;; start
    (send *ri* :stop-grasp :arms)
    (if speak
      (send *ri* :speak-jp start-str)
      (format t start-str))
    (send *ri* :angle-vector
          (send *pr2* :angle-vector *grasp-broom-with-both-arm*)
          5000)
    (send *ri* :wait-interpolation)
    ;; Grasp rarm
    (ask-and-wait rarm-str speak)
    (send *ri* :start-grasp :rarm :wait t :gain 1)
    ;; Grasp larm
    (ask-and-wait larm-str speak)
    (send *ri* :start-grasp :larm :wait t :gain 1)
    ;; Release both arms
    (ask-and-wait finish-str speak)
    (send *ri* :stop-grasp :arms :wait t)
    ))

(defun check-position ()
  (let* ((now-coords (send (send *ri* :state :worldcoords) :copy-worldcoords))
         (now-pos (send now-coords :pos))
         (now-z-rot (car (car (send now-coords :rpy-angle))))
         (demo-coords (send *demo-coords* :copy-worldcoords))
         (demo-pos (send demo-coords :pos))
         (demo-z-rot (car (car (send demo-coords :rpy-angle))))
         (diff-pos (v- demo-pos now-pos))
         (diff-rot (- demo-z-rot now-z-rot)))
    (format t "diff-pos: ~A [mm]~%diff-rot: ~A[deg]~%~%" diff-pos diff-rot)
    (when (> (norm diff-pos) 50)
      (format t "PR2 position is not correct.~%")
      (return-from check-position nil))
    (when (> (abs diff-rot) 0.1)
      (format t "PR2 rotation is not correct.~%")
      (return-from check-position nil))
    (format t "PR2 position and rotation is good for demo.~%")
    t))

(defun grasp-broom-and-module-in-advance (&optional (speak nil))
  (send *ri* :stop-grasp :arms)
  (send *ri* :angle-vector
        (send *pr2* :angle-vector *demo-init-pose*)
        5000)
  (send *ri* :wait-interpolation)
  (ask-and-wait "右手でホウキの根本を掴みます。よければエンターを押してください。" speak)
  (send *ri* :start-grasp :rarm :wait t :gain 1)
  (ask-and-wait "左手でモジュールを掴みます。よければエンターを押してください。" speak)
  (send *ri* :start-grasp :larm :wait t))

(defun find-broom-handle-pos ()
  (let (broom-handle-msg broom-handle-box)
    (while (not broom-handle-box)
      (setq broom-handle-msg (one-shot-subscribe "/broom_handle/HSI_color_filter/boxes"
                                                 jsk_recognition_msgs::BoundingBoxArray))
      (setq broom-handle-box (car (send broom-handle-msg :boxes))))
    (let* ((broom-handle-pose-camera (send broom-handle-box :pose))
           (base->cam-coords
            (send *tfl* :lookup-transform "/base_footprint"
                  (send broom-handle-msg :header :frame_id) (ros::time 0)))
           (broom-handle-coords-base
            (send (send base->cam-coords :copy-worldcoords)
                  :transform (ros::tf-pose->coords broom-handle-pose-camera)))
           (broom-handle-pos (send broom-handle-coords-base :worldpos)))
      broom-handle-pos)))

;; Before this function, PR2 must grasp broom in the right hand and module in the left hand
(defun attach-module ()
  ;; Reach the module to the broom
  (send *ri* :speak-jp "ホウキの先端にカメラを取り付けます")
  (send *ri* :angle-vector
        (send *pr2* :angle-vector *look-at-broom-pose*)
        5000)
  (send *ri* :wait-interpolation)
  ;; Attach
  ;; TODO: use (search-sensor-placement :larm "pr2_broom" nil)
  ;; (send *ri* :angle-vector
  ;;       (send *pr2* :larm :move-end-pos #f(80 0 0) :local :use-torso nil)
  ;;       5000)
  ;; (send *ri* :wait-interpolation)
  ;; (send *ri* :stop-grasp :larm :wait t)
  ;; (send *ri* :angle-vector
  ;;       (send *pr2* :larm :move-end-pos #f(-80 0 0) :local :use-torso nil)
  ;;       5000)
  ;; (send *ri* :wait-interpolation)
  ;; Grasp broom by the left hand. Find broom handle by black color recognition
  (send *pr2* :angle-vector *grasp-broom-with-both-arm*)
  (send *pr2* :larm :move-end-pos #f(-200 50 0) :local :use-torso nil
        :rotation-axis :z)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (let ((pr2-hand-coords (send (send *pr2* :larm :end-coords) :copy-worldcoords)))
    (ros::ros-info "Search for broom handle...")
    (send pr2-hand-coords :locate (find-broom-handle-pos) :world)
    ;; Need tuning before each demo because handle recognition is not very stable
    ;; attachment is also not stable
    ;; (send pr2-hand-coords :translate #f(-10 0 -20) :local)
    ;; (send pr2-hand-coords :translate #f(-20 0 0) :local)
    (format t "~A~%" pr2-hand-coords)
    (send *pr2* :larm :inverse-kinematics pr2-hand-coords :use-torso nil))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  (send *ri* :start-grasp :rarm :wait t :gain 0.05)
  (send *ri* :start-grasp :larm :wait t :gain 1)
  (send *ri* :stop-grasp :rarm :wait t)
  )

;; Return t if unitv finds "obj-name" object. (e.g. (find-object "bottle"))
(defun find-object (obj-name)
  (let ((class-msg
          (one-shot-subscribe "/unitv_image/class" jsk_recognition_msgs::ClassificationResult
                              :timeout 3000))
        class-name)
    (if class-msg
      (setq class-name (car (send class-msg :label_names)))
      (return-from find-object nil))
    (equal class-name obj-name)))

;; Return t if robot finds bottle by using unitv
(defun search-bottle ()
  (send *ri* :speak-jp "ゴミを探します")
  ;; TODO: Need fix for search-avs (PR2 cannot see the bottle)
  (let ((search-avs
         (list
          (send *pr2* :angle-vector
                #f(52.7 67.2314 25.9319 72.1467 -90.7628 -1290.98 -22.4216 -219.228 -74.7828 49.0326 -113.815 -109.448 19.9973 -5.72958 -10.0461 4.7647 34.848))
          (send *pr2* :angle-vector
                #f(52.7 62.2985 28.6681 53.0207 -52.6038 -1392.21 -49.093 -108.297 -74.7828 48.7466 -114.927 -109.439 19.9973 -5.72958 -10.0461 28.4767 49.368))
          (send *pr2* :angle-vector
                #f(52.7 68.8458 15.0893 69.5882 -82.4521 -1321.89 -16.3279 -202.536 -74.7828 48.7466 -114.927 -109.439 19.9973 -5.72958 -10.0461 4.7707 34.848))
               )))
    (dolist (search-av search-avs)
      (send *pr2* :angle-vector search-av)
      (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
      (dotimes (i 3)
        (when (find-object "bottle")
          (send *ri* :speak-jp "ゴミが見つかりました。" :wait t)
          (return-from search-bottle t))
        (unix:sleep 1))))
  (send *ri* :speak-jp "ゴミは見つかりませんでした。" :wait t)
  nil)

(defun sweep ()
  (send *ri* :speak-jp "ゴミをかきだします。")
  (send *ri* :angle-vector-sequence
        *sweep-vectors*
        (list 4000 2000 2000 2000 2000 2000 2000))
  (send *ri* :wait-interpolation))

(defun main ()
  ;; Set fixture to broom before the demo
  ;; (set-broom-fixture)
  ;; Check demo position
  ;; (check-position)
  ;; Grasp broom as prepose of the demo
  ;; (grasp-broom-and-module-in-advance)

  ;; Main
  (attach-module)
  ;; TODO: Use while instead of if?
  ;; (if (search-bottle)
  ;;   ;; Sweep with broom
  ;;   (sweep))

  ;; TODO: Do (sweep) regardless of the result of (search-bottle)
  (search-bottle)
  (sweep)
  )
