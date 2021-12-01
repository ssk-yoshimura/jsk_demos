;; robotの初期化
(when (not (boundp '*dxl-armed-turtlebot*))
  (load "package://dxl_armed_turtlebot/euslisp/dxl-armed-turtlebot-interface.l")
  (dxl-armed-turtlebot-init))

(send *dxl-armed-turtlebot* :reset-pose)
(send *ri* :angle-vector (send *dxl-armed-turtlebot* :angle-vector))

(send *ri* :wait-interpolation)
(send *ri* :stop-grasp)

(ros::roseus-add-msgs "std_msgs")
(ros::roseus "listener" :anonymous t)

(setq target (make-coords))
(objects (list *dxl-armed-turtlebot* target))

(setq bx 0)
(setq y 0)

 (defun cb(msg)
   ;; (format t "~A ~%" (send msg :data))
   (setq bx (elt (send msg :data) 0))
   (setq bz (elt (send msg :data) 2))
   (setq y (+ (* bx 2) -600))
   (setq x (+ bz -100))
   (setq z 300)
   (send target :locate (float-vector x y z) :world)
   (objects (list *dxl-armed-turtlebot* target))
   (format t "~A~%" (send target :pos))
 )

 
 (ros::subscribe "ball" std_msgs::Int16MultiArray #'cb)

 (do-until-key
  (x::window-main-one)
  (ros::sleep)
  (ros::spin-once)
  )

(send *dxl-armed-turtlebot* :inverse-kinematics target
      :translation-axis t
      :rotation-axis nil
      :revert-if-fall t
      :debug-view nil
      )

(send *ri* :angle-vector (send *dxl-armed-turtlebot* :angle-vector) 2000)

(send *ri* :wait-interpolation)

(send *ri* :start-grasp)

(unix:sleep 5)
