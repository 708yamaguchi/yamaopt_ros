(ros::load-ros-manifest "fetch_auto_dock_msgs")
(ros::load-ros-manifest "power_msgs")

(defparameter *dock-action* nil)
(defparameter *undock-action* nil)
(defparameter *spots* nil)
;; This takes long time at first time
(unless *spots*
  (setq *spots* (one-shot-subscribe "/spots_marker_array" visualization_msgs::MarkerArray)))

(let ((robot-name (ros::get-param "/robot/name")))
  (defparameter *dock-spot*
    (cond
      ((equal robot-name "fetch15")
       "/eng2/7f/room73B2-fetch-dock-front")
      ((equal robot-name "fetch1075")
       "/eng2/7f/room73B2-fetch-dock2-front")
      (t nil))))

(defun get-spot-coords (name)
  (unless *spots*
    (setq *spots* (one-shot-subscribe "/spots_marker_array" visualization_msgs::MarkerArray)))
  (let ((spot-coords nil) (frame-id nil))
    (dolist (x (send *spots* :markers))
      (if (equal (send x :text) name)
        (progn
          (setq spot-coords (send x :pose))
          (setq frame-id (send (send x :header) :frame_id)))))
    (send (send spot-coords :position) :z 0)
    (setq spot-coords (ros::tf-pose->coords spot-coords))
    (cons spot-coords frame-id)))


(defun simple-dock (&key (use-pose t))
  (unless *dock-action*
    (setq *dock-action*
          (instance ros::simple-action-client :init
                    "/dock" fetch_auto_dock_msgs::DockAction)))
  (unless (send *dock-action* :wait-for-server 5)
    (ros::ros-error "/dock action server is not started")
    (return-from dock nil))
  (let ((dock-action-goal (instance fetch_auto_dock_msgs::DockActionGoal :init)))
    (when use-pose
      (let* ((timestamp (ros::time-now))
             (cret (get-spot-coords *dock-spot*))
             (frame-to-dock (car cret))
             (frame-id (cdr cret))
             (lret (send *tfl* :wait-for-transform "base_link" frame-id timestamp 5))
             (base-to-frame (send *tfl* :lookup-transform "base_link" frame-id timestamp))
             (goal-pose (ros::coords->tf-pose (send frame-to-dock :transform base-to-frame :world)))
             (pose-msg (instance geometry_msgs::PoseStamped :init)))
        (send pose-msg :header :stamp timestamp)
        (send pose-msg :header :frame_id "base_link")
        (send pose-msg :pose goal-pose)
        (send dock-action-goal :goal :dock_pose pose-msg)))
    (send *dock-action* :send-goal dock-action-goal)
    (unless (send *dock-action* :wait-for-result :timeout 60)
      (send *dock-action* :cancel-all-goals)
      (ros::ros-error "No result returned from /dock action server")
      (return-from simple-dock nil))
    (send (send *dock-action* :get-result) :docked)))


(defun dock ()
  ;; look down
  (unless (boundp '*ri*)
    (require :fetch-interface "package://fetcheus/fetch-interface.l")
    (fetch-init))
  (send *fetch* :angle-vector (send *ri* :state :potentio-vector))
  (send *fetch* :head :look-at (float-vector 800 0 500))
  (send *ri* :angle-vector (send *fetch* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  (simple-dock))


(defun undock ()
  (unless *undock-action*
    (setq *undock-action*
          (instance ros::simple-action-client :init
                    "/undock" fetch_auto_dock_msgs::UndockAction)))
  (unless (send *undock-action* :wait-for-server 5)
    (ros::ros-error "/undock action server is not started")
    (return-from undock nil))
  (send *undock-action* :send-goal
        (instance fetch_auto_dock_msgs::UndockActionGoal :init))
  (unless (send *undock-action* :wait-for-result :timeout 60)
    (ros::ros-error "No result returned from /undock action server")
    (return-from undock nil))
  (send (send *undock-action* :get-result) :undocked))

(defun get-battery-charging-state (&key (timeout 1500))
   (let* ((msg (one-shot-subscribe "/battery_state" power_msgs::batterystate :timeout timeout))
          (is-charging (if msg (send msg :is_charging))))
     ;; You may fail to subscribe /battery_state
     ;; because of message md5 difference between melodic and indigo.
     (if (not msg) (return-from get-battery-charging-state nil))
     (if is-charging :charging :discharging)))

(defun go-to-spot (name &key (relative-pos nil) (relative-rot nil) (undock-rotate nil) (clear-costmap t))
  "Move the robot to the spot defined in /spots_marker_array topic. The reason for using relative-pos and relative-rot instead of relative-coords is that it is easier to understand if relative-pos and relative-rot are specified in the different coords (specifically, world (map) and local (spot) coords respectively). For detail, see https://github.com/jsk-ros-pkg/jsk_robot/pull/1458#pullrequestreview-1039654868
Args:
- name : The name of the spot defined in /spots_marker_array topic
- relative-pos : The robot moves to a position that is distant from the spot by relative-pos[mm]. relative-pos is specified in the world (map) coords.
- relative-rot : The robot moves to a position rotated by relative-pos[deg] from the spot. relative-rot is specified in the local (spot) coords.
- undock-rotate : If t, the robot rotates 180 degrees after undock before moving to the spot.
- clear-costmap : If t, clear costmap for obstacle avoidance before the robot moves.
"

  ;; undock if fetch is docking
  (unless (boundp '*ri*)
    (require :fetch-interface "package://fetcheus/fetch-interface.l")
    (fetch-init))
  (if clear-costmap (send *ri* :clear-costmap))
  (let ((undock-success nil))
    (block go-to-spot-undock
      (dotimes (i 3)
        (let* ((battery-charging-state (get-battery-charging-state)))
          ;; You may fail to subscribe /battery_state
          ;; because of message md5 difference between melodic and indigo.
          (unless battery-charging-state
            (ros::ros-warn "Failed to subscribe /battery_state")
            (ros::ros-warn "Skip undocking, so please make sure that Fetch is already undocked.")
            (return-from go-to-spot-undock t))
          (if (equal battery-charging-state :charging)
            (progn
              (setq undock-success (auto-undock :n-trial 3))
              ;; rotate after undock
              (if (and undock-success undock-rotate)
                (send *ri* :go-pos-unsafe 0 0 180)))
            (return-from go-to-spot-undock t))
          (if (not undock-success)
            (progn
              (ros::ros-error "Failed to undock")
            (send *ri* :speak "I failed to undock.")))))
      (if (not undock-success) (return-from go-to-spot nil))))
  ;; go to spot
  (let* ((ret (get-spot-coords name))
         (goal-pose (car ret))
         (frame-id (cdr ret)))
    (when relative-pos
      (setq goal-pose (send goal-pose :translate relative-pos :world)))
    (when relative-rot
      (setq goal-pose (send goal-pose :rotate relative-rot :z :local)))
    (send *ri* :move-to goal-pose :frame-id frame-id)))

(defun auto-dock (&key (n-trial 1) (clear-costmap t))
  (let ((success nil))
    (dotimes (i n-trial)
      (when (go-to-spot *dock-spot*
                        :relative-pos #f(-800 0 0)
                        :clear-costmap clear-costmap)
        (ros::ros-info "arrived at the dock.")
        (setq success (dock))
        (when success (return-from auto-dock success))))
    success))


(defun auto-undock (&key (n-trial 1))
  (let ((success nil))
    (unless (boundp '*ri*)
      (require :fetch-interface "package://fetcheus/fetch-interface.l")
      (fetch-init))
    (dotimes (i n-trial)
      (ros::ros-info "trying to do undock.")
      (setq success (undock))
      (when success (return-from auto-undock success)))
    (if (not success)
      (let ((enable-request (instance power_msgs::BreakerCommandRequest :init :enable t))
            (disable-request (instance power_msgs::BreakerCommandRequest :init :enable nil))
            (breaker-service-name "base_breaker"))
        (ros::wait-for-service breaker-service-name 5)
        (ros::service-call breaker-service-name disable-request)
        (ros::ros-error "Diable base breaker")
        (send *ri* :speak "I disable and enable base breaker.")
        (unix::sleep 5)
        (ros::service-call breaker-service-name enable-request)
        (ros::ros-error "Enable base breaker")))
    success))
