;; Copied from https://github.com/708yamaguchi/wireless_sensing/blob/master/euslisp/fetch-receive-delivery.l

;; TODO: Use microphone volume instead of unitv in (receive-delivery)
;; TODO: Add (place-sensor) in (attach-module)

(load "package://fetcheus/fetch-interface.l")

(ros::load-ros-manifest "sensor_msgs")
(ros::load-ros-manifest "speech_recognition_msgs")
(ros::load-ros-manifest "jsk_recognition_msgs")

(fetch-init)

(setq *target-room* "73 A 4")
(ros::set-dynamic-reconfigure-param
 "/move_base/local_costmap/inflater" "inflation_radius" :double 0.35)

;; Fetchが「宅配便を受け取って」と言われたら、どの部屋に宅配便が来るかを聞き返す。
;; その後、モジュールをQiトランスミッタから取り外して73B2から出ていく。
(defun receive-request ()
  ;; Receive request
  (let ((speech (car (send (one-shot-subscribe "/speech_to_text_google" speech_recognition_msgs::SpeechRecognitionCandidates) :transcript))))
    (print speech)
    (when (substringp "宅配" speech)
      (send *ri* :speak-jp "どの部屋に宅配便が来ますか？" :wait t)
      (unix:sleep 1)
      (setq speech
            (car (send (one-shot-subscribe "/speech_to_text_google" speech_recognition_msgs::SpeechRecognitionCandidates) :transcript)))
      (print speech)
      ;; speech should be "73 A 4"
      (send *ri* :speak-jp (format nil "~Aに宅配便がくるのを見ておきます。" speech) :wait t)
      (setq *target-room* speech)
      ))
  ;; Remove module from Qi transmitter
  (let ((av1 (send *fetch* :angle-vector
                   #f(200 81.6916 -30.137 -83.7719 98.5518 0.0404 67.56 0.058964 0 40)))
        (av2 (send *fetch* :inverse-kinematics
                   (make-coords :pos #f(780 0 920) :rpy (float-vector pi/2 pi/2 0))
                   :use-torso nil))
        (av3 (send *fetch* :rarm :move-end-pos #f(0 0 -130) :world :use-torso nil)))
    (send *ri* :stop-grasp :wait t)
    (send *ri* :angle-vector-sequence
          (list av1 av2 av3)
          (list 5000 5000 3000))
    (unix:sleep 14)
    (send *ri* :start-grasp :wait t)
    (send *fetch* :reset-pose)
    (send *ri* :angle-vector-sequence
          (list av2 av1 (send *fetch* :reset-pose))
          (list 3000 5000 5000))
    (unix:sleep 14))
  ;; Move out from 73B2
  (send *ri* :go-pos-unsafe -1 0 90)
  (send *ri* :clear-costmap)
  (send *ri* :move-to (make-coords :pos #f(-725.195 6774.428 30000.0) :rpy #f(-3.125 6.409e-15 -7.293e-15)))
  )

;; 73B2から出てきたFetchが、指定された部屋の前にモジュールを配置する。
;; その後、73B2に戻る。
(defun attach-module ()
  ;; Move to the *target-room*
  (send *ri* :go-pos-unsafe 1 0 0)
  (send *ri* :clear-costmap)
  (cond
   ((equal *target-room* "73 A 4")
    (send *ri* :move-to (make-coords :pos #f(-1200 7513.149 30000.0) :rpy #f(3.081 6.962e-15 -3.125e-15))))
    (t
     nil))
  ;; Attach the module on the wall
  ;; 配置は10度くらい斜めに傾けて、人の顔が見えるようにしている
  (let* ((av-temp
          (send *fetch* :angle-vector #f(200.0 42.7065 -0.751278 -94.3525 82.8331 -3.53775 -46.1664 -9.7726 -0.232261 15.4101)))
         (base-scan-ranges (send (one-shot-subscribe "/base_scan" sensor_msgs::LaserScan) :ranges))
         (distance-to-wall (elt base-scan-ranges (/ (length base-scan-ranges) 2)))
         (av2 (send *fetch* :rarm :move-end-pos (float-vector (- (* distance-to-wall 1000) 725) 0 0) :world :use-torso nil))
         (av1
          (send *fetch* :rarm :move-end-pos #f(-130 0 0) :world :use-torso nil)))
    (send *ri* :start-grasp :wait t)
    (send *ri* :angle-vector-sequence-raw
          (list av1 av2)
          (list 8000 3000))
    (unix:sleep 12)
    (send *ri* :stop-grasp :wait t)
    (send *ri* :angle-vector-sequence-raw
          (list av1 (send *fetch* :reset-pose))
          (list 3000 8000))
      (unix:sleep 12))
  ;; Return to 73B2
  (send *ri* :go-pos-unsafe -0.3 0.3 180)
  (send *ri* :go-pos-unsafe 0.5 0 0)
  (send *ri* :move-to (make-coords :pos #f(1750.247 6138.067 30000.0) :rpy #f(0.045 5.607e-15 3.108e-15)))
  )

;; モジュールに人の姿が写ったら、fetchは73B2から指定された部屋の前まで行く。
;; 「宅配便ですか」と聞き、「はい」と言われたら宅配便を受け取って73B2に帰っていく。
(defun receive-delivery ()
  ;; Wait for the delivery people to come
  (while t
    (let ((unitv-class (car (send (one-shot-subscribe "/unitv_image/class" jsk_recognition_msgs::ClassificationResult) :label_names))))
      (print unitv-class)
      (if (equal unitv-class "person")
        (return)
        (unix:sleep 1))))
  (send *ri* :speak-jp "宅配の人を見つけました" :wait t)
  ;; Move to the *target-room*
  (send *ri* :go-pos-unsafe 1 0 0)
  (send *ri* :clear-costmap)
  (cond
   ((equal *target-room* "73 A 4")
    (send *ri* :move-to (make-coords :pos #f(-1160.931 7460.725 30000.0) :rpy #f(1.529 -8.895e-15 2.514e-15))))
   (t
    nil))
  ;; Receive delivery
  (unix:sleep 5) ;; Wait for "到着しました" by fetch itself
  (send *ri* :speak-jp "宅配便ですか" :wait t)
  (unix:sleep 1)
  (let ((speech (car (send (one-shot-subscribe "/speech_to_text_google" speech_recognition_msgs::SpeechRecognitionCandidates) :transcript))))
    (print speech)
    (when (substringp "はい" speech)
      (send *ri* :stop-grasp :wait t)
      (send *ri* :angle-vector (send *fetch* :angle-vector #f(100 46.4035 39.1208 -87.0678 99.233 -70.0744 71.8666 -36.086 0 -20)) 5000)
      (send *ri* :wait-interpolation)
      (send *ri* :speak-jp "荷物を受け取ります" :wait t)
      (unix:sleep 3)
      (send *ri* :start-grasp :wait t)
      (send *fetch* :reset-pose)
      (send *fetch* :torso :waist-z :joint-angle 100)
      (let ((av1 (send *fetch* :angle-vector))
            (av2 (send *fetch* :reset-pose)))
        (send *ri* :angle-vector-sequence-raw (list av1 av2) (list 4000 2000))
        (unix:sleep 6))
      ))
  ;; Return to 73B2
  (send *ri* :go-pos-unsafe -0.7 0 -90)
  (send *ri* :clear-costmap)
  (send *ri* :move-to (make-coords :pos #f(1750.247 6138.067 30000.0) :rpy #f(0.045 5.607e-15 3.108e-15)))
  )

;; タスクが終わったので、fetchは73B2から出てきて指定された部屋の前まで戻り、モジュールを回収する。
(defun retrieve-module ()
  ;; いまのfetchでは移動の精度が出ないので、モジュールを回収するところから始める。（本来は、モジュールを貼った場所はロボットが覚えているので、全く同じ位置に:move-toして:angle-vectorすれば把持できるはず）
  ;; ;; Move to the *target-room*
  ;; (send *ri* :clear-costmap)
  ;; (cond
  ;;  ((equal *target-room* "73 A 4")
  ;;   (send *ri* :move-to (make-coords :pos #f(-1200 7513.149 30000.0) :rpy #f(3.081 6.962e-15 -3.125e-15))))
  ;;   (t
  ;;    nil))

  ;; Retrieve module
  (let* ((av-temp
          (send *fetch* :angle-vector #f(200.0 42.7065 -0.751278 -94.3525 82.8331 -3.53775 -46.1664 -9.7726 -0.232261 15.4101)))
         (base-scan-ranges (send (one-shot-subscribe "/base_scan" sensor_msgs::LaserScan) :ranges))
         (distance-to-wall (elt base-scan-ranges (/ (length base-scan-ranges) 2)))
         (av2 (send *fetch* :rarm :move-end-pos (float-vector (- (* distance-to-wall 1000) 725) 0 0) :world :use-torso nil))
         (av1
          (send *fetch* :rarm :move-end-pos #f(-130 0 0) :world :use-torso nil)))
    (send *ri* :stop-grasp :wait t)
    (send *ri* :angle-vector-sequence-raw
          (list av1 av2)
          (list 8000 3000))
    (unix:sleep 12)
    (send *ri* :start-grasp :wait t :effort 100)
    (send *ri* :angle-vector-sequence-raw
          (list av1 (send *fetch* :reset-pose))
          (list 3000 8000))
    (unix:sleep 12))
  ;; Return to 73B2
  (send *ri* :go-pos-unsafe -0.7 0 -90)
  (send *ri* :clear-costmap)
  (send *ri* :move-to (make-coords :pos #f(1750.247 6138.067 30000.0) :rpy #f(0.045 5.607e-15 3.108e-15)))
  )

;; 最後に、モジュールをQiトランスミッタの上に戻す。
(defun charge-module ()
  (let ((av1 (send *fetch* :angle-vector
                   #f(200 81.6916 -30.137 -83.7719 98.5518 0.0404 67.56 0.058964 0 40)))
        (av2 (send *fetch* :inverse-kinematics
                   (make-coords :pos #f(780 0 920) :rpy (float-vector pi/2 pi/2 0))
                   :use-torso nil))
        (av3 (send *fetch* :rarm :move-end-pos #f(0 0 -130) :world :use-torso nil)))
    (send *ri* :start-grasp :wait t)
    (send *ri* :angle-vector-sequence
          (list av1 av2 av3)
          (list 5000 5000 3000))
    (unix:sleep 14)
    (send *ri* :stop-grasp :wait t)
    (send *fetch* :reset-pose)
    (send *ri* :angle-vector-sequence
          (list av2 av1 (send *fetch* :reset-pose))
          (list 3000 5000 5000))
    (unix:sleep 14)
    )
  )
