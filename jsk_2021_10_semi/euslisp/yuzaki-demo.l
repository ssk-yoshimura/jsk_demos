;memo
;spotのworldは基本姿勢の時を基準に与えられる

(when (not (boundp '*spotkinova*))
  (load "package://spotkinovaeus/spotkinova.l")
  (spotkinova :type :gen3_lite_gen3_lite_2f))
(objects (list *spotkinova*))

(setq *paper-msg* (instance geometry_msgs::PoseArray :init))
(setq *paper-get-flag* nil)

;;;;;;;;;; ↓基本となる関数たち ;;;;;;;;;;
;ロボットからの位置posの場所までアームだけで動く
(defun arm-move-from-robot (x y z)
  ;(make-coords :pos #f(0 0 0) :rpy #f(0 0 0) )
  (setq arm-pose (send *spotkinova* :copy-worldcoords))
  (setq arm-pose (send arm-pose :translate (float-vector x y z) :world))
  (objects arm-pose)
  (send *spotkinova* :head :inverse-kinematics arm-pose :rotation-axis nil)
)

(defun arm-move-from-robot-rotx (x y z roty rotz)
  ;(make-coords :pos #f(0 0 0) :rpy #f(0 0 0) )
  (format t "[arm-move-from-robot-rotx]x:~A y:~A z:~A roty:~A rotz:~A" x y z roty rotz)
  (setq arm-pose (send *spotkinova* :copy-worldcoords))
  (setq arm-pose (send arm-pose :translate (float-vector x y z) :world))
  (setq arm-pose (send arm-pose :rotate rotz :z :world))
  (setq arm-pose (send arm-pose :rotate roty :y :world))
  (objects arm-pose)
  (send *spotkinova* :head :inverse-kinematics arm-pose :rotation-axis :x)
)

(defun arm-move-from-robot-roty (x y z rotx rotz)
  ;(make-coords :pos #f(0 0 0) :rpy #f(0 0 0) )
  (format t "[arm-move-from-robot-roty]x:~A y:~A z:~A rotx:~A rotz:~A" x y z rotx rotz)
  (setq arm-pose (send *spotkinova* :copy-worldcoords))
  (setq arm-pose (send arm-pose :translate (float-vector x y z) :world))
  (setq arm-pose (send arm-pose :rotate rotx :x :world))
  (setq arm-pose (send arm-pose :rotate rotz :z :world))
  (objects arm-pose)
  (send *spotkinova* :head :inverse-kinematics arm-pose :rotation-axis :y)
)

(defun arm-move-from-robot-rotz (x y z rotx roty)
  ;(make-coords :pos #f(0 0 0) :rpy #f(0 0 0) )
  (format t "[arm-move-from-robot-rotz]x:~A y:~A z:~A rotx:~A roty:~A" x y z rotx roty)
  (setq arm-pose (send *spotkinova* :copy-worldcoords))
  (setq arm-pose (send arm-pose :translate (float-vector x y z) :world))
  (setq arm-pose (send arm-pose :rotate rotx :x :world))
  (setq arm-pose (send arm-pose :rotate roty :y :world))
  (objects arm-pose)
  (send *spotkinova* :head :inverse-kinematics arm-pose :rotation-axis :z)
)

;ロボットからの位置posの場所まで体全体で動く
(defun full-move-from-robot (pos)
  (let ((arm-pose (make-coords :pos #f(0 0 0) :rpy #f(0 0 0)))) (
    send *spotkinova* :fullbody-inverse-kinematics
      (send arm-pose :translate pos :world)
      :root-link-virtual-joint-weight #f(0.0 0.0 0.1 0.1 0.5 0.5)
  ))
)

(defun zikki-init ()
  (load "package://spotkinovaeus/spotkinova-interface.l")
  (spotkinova-init)
)

(defun body-updown (height)
  (send *spotkinova* :body-inverse-kinematics
        (make-coords :pos (float-vector 0 0 height) :rpy (float-vector 0 0 0)))
)

(defun hand-catch ()
  (send *spotkinova* :start-grasp)
  (when  (boundp '*ri*)
    (unix:sleep 5)
    (format t "zikki daze~%")
    (send *ri* :start-grasp)
    (send *ri* :wait-interpolation)
  )
)

(defun hand-release ()
  (send *spotkinova* :stop-grasp)
  (when  (boundp '*ri*)
    (format t "zikki daze~%")
    (send *ri* :stop-grasp)
    (send *ri* :wait-interpolation)
  )
)

(defun zikki-send-angle (angle)
  (when  (boundp '*ri*)
    (format t "zikki daze~%")
    (send *ri* :angle-vector angle :fast)
    (send *ri* :wait-interpolation)
  )
)

(defun zikki-send-angle-sequence (angles)
  (when  (boundp '*ri*)
    (format t "zikki daze~%")
    (send *ri* :angle-vector-sequence angles)
    (send *ri* :wait-interpolation)
  )
)
;;;;;;;;;; ↑基本となる関数たち ;;;;;;;;;;

;;;;;;;;;; ↓お遊び ;;;;;;;;;;
(defun catch-ball-pre ()
  ;(arm-move-from-robot 600 0 500)
  (send *spotkinova* :reset-pose)
  (when  (boundp '*ri*)
    (format t "zikki daze~%")
    (send *ri* :angle-vector (send *spotkinova* :angle-vector) 10000)
    (send *ri* :wait-interpolation)
  )
)

(defun catch-ball ()
  (hand-release)
  (hand-catch)
)

(defun store-ball ()
  (arm-move-from-robot 0 -300 800)
  (when  (boundp '*ri*)
    (format t "zikki daze~%")
    (send *ri* :angle-vector (send *spotkinova* :angle-vector) 15000)
    (send *ri* :wait-interpolation)
  )
  (arm-move-from-robot -100 0 800)
  (when  (boundp '*ri*)
    (format t "zikki daze~%")
    (send *ri* :angle-vector (send *spotkinova* :angle-vector) 10000)
    (send *ri* :wait-interpolation)
  )
  (hand-release)
)
;;;;;;;;;; ↑お遊び ;;;;;;;;;;

;;;;;;;;;; ↓レポート運び(変更中のため動かない) ;;;;;;;;;;
(defun posture-for-camera ()
  (body-updown -100)

  ;カメラが見やすい絶妙な角度
  (send *spotkinova* :kinova_joint_1 :joint-angle -50)
  (send *spotkinova* :kinova_joint_2 :joint-angle 0)
  (send *spotkinova* :kinova_joint_3 :joint-angle 30)
  (send *spotkinova* :kinova_joint_4 :joint-angle 120)
  (send *spotkinova* :kinova_joint_5 :joint-angle 120)
  (send *spotkinova* :kinova_joint_6 :joint-angle 70)

  (when  (boundp '*ri*)
    (format t "zikki daze~%")
    (send *ri* :body-pose (send *spotkinova* :copy-worldcoords))
    (send *ri* :wait-interpolation)
    (send *ri* :angle-vector (send *spotkinova* :angle-vector) :fast)
    (send *ri* :wait-interpolation)
  )
)

(defun wait-for-report ()
  ;(send *spotkinova* :move-to (make-coords :pos #f(3000 500 0) :rpy (float-vector pi/2 0 0)) :world)
  ;(send *spotkinova* :locate #f(0 0 550))

  (posture-for-camera)

  (send *irtviewer* :draw-objects)
)

(defun catch-report-pre-slide ()
  ;topicうけとる
  (setq *paper-get-flag* nil)

  (ros::roseus-add-msgs "geometry_msgs")
  (ros::roseus "yuzaki-demo" :anonymous t)
  (ros::subscribe "paper_finder/output/pose" geometry_msgs::PoseArray #'paperpose-cb)

  (setq topic-wait-time 0.0)
  (ros::rate 10)
  (while (ros::ok)
    ;経過時間
    (setq topic-wait-time (+ topic-wait-time 0.1))
    (format t "wait time:~A~%" topic-wait-time)
    (ros::spin-once)
    (ros::sleep)
    (if *paper-get-flag* (return))
    (if (> topic-wait-time 5.0) (return)))

  ;トピックの処理
  (if *paper-get-flag* 
    (progn
      (format t "paper found~%")
      (setq camera-to-paper (ros::tf-pose->coords (elt (send *paper-msg* :poses) 0)))
      (format t "paperfinder coords ~A~%" camera-to-paper)

      ;bodyから見た座標系に変換する
      (setq body-to-paper 
        (send (send *tfl* :lookup-transform "/body" (send *paper-msg* :header :frame_id) (ros::time 0)) 
        :transform camera-to-paper)
      )
      (format t "from body paper coords ~A~%" body-to-paper)

      ;(objects (list body-to-paper))

      ;一応開けておく
      (hand-release)

      ;必要な座標で逆運動学
      ;(send *spotkinova* :reset-pose)     
      (setq paper-pos (send body-to-paper :pos))

      ;(arm-move-from-robot-rotx (- (elt paper-pos 0) 0) (+ (elt paper-pos 1) 50) 400 1.57 -1.57)
      ;(setq slide_wait_point (send *spotkinova* :angle-vector))

      (arm-move-from-robot-rotx (- (elt paper-pos 0) 50) (+ (elt paper-pos 1) 70) (+ (elt paper-pos 2) 20) 1.57 -1.57)
      (setq slide_start_point (send *spotkinova* :angle-vector))

      (arm-move-from-robot-rotx (- (elt paper-pos 0) 200) (+ (elt paper-pos 1) 70) (+ (elt paper-pos 2) 20) 1.57 -1.57)
      (setq slide_end_point (send *spotkinova* :angle-vector))
      
      ;スライドさせる軌道
      ;(zikki-send-angle slide_wait_point)
      (zikki-send-angle slide_start_point)
      (zikki-send-angle slide_end_point)

      (posture-for-camera)

      (send *irtviewer* :draw-objects)
    )
    (progn
      (format t "paper timeout~%" topic-wait-time)
    )
  )
)

(defun catch-report ()
  ;topicうけとる
  (setq *paper-get-flag* nil)

  (ros::roseus-add-msgs "geometry_msgs")
  (ros::roseus "yuzaki-demo" :anonymous t)
  (ros::subscribe "paper_finder/output/pose" geometry_msgs::PoseArray #'paperpose-cb)

  (setq topic-wait-time 0.0)
  (ros::rate 10)
  (while (ros::ok)
    ;経過時間
    (setq topic-wait-time (+ topic-wait-time 0.1))
    (format t "wait time:~A~%" topic-wait-time)
    (ros::spin-once)
    (ros::sleep)
    (if *paper-get-flag* (return))
    (if (> topic-wait-time 5.0) (return)))

  ;トピックの処理
  (if *paper-get-flag* 
    (progn
      (format t "paper found~%")
      (setq camera-to-paper (ros::tf-pose->coords (elt (send *paper-msg* :poses) 0)))
      (format t "paperfinder coords ~A~%" camera-to-paper)

      ;bodyから見た座標系に変換する
      (setq body-to-paper 
        (send (send *tfl* :lookup-transform "/body" (send *paper-msg* :header :frame_id) (ros::time 0)) 
        :transform camera-to-paper)
      )
      (format t "from body paper coords ~A~%" body-to-paper)

      ;(objects (list body-to-paper))

      ;一応開けておく
      (hand-release)

      ;必要な座標で逆運動学
      (send *spotkinova* :reset-pose)     
      (setq paper-pos (send body-to-paper :pos))

      (arm-move-from-robot-rotx (- (elt paper-pos 0) 200) (+ (elt paper-pos 1) 80) 600 1.57 0)
      (setq catch_wait_point (send *spotkinova* :angle-vector))

      (arm-move-from-robot-rotx (- (elt paper-pos 0) 130) (+ (elt paper-pos 1) 80) (+ (elt paper-pos 2) 10) 1.57 0)
      (setq catch_point (send *spotkinova* :angle-vector))

      (arm-move-from-robot-rotx (- (elt paper-pos 0) 200) (+ (elt paper-pos 1) 80) (+ (elt paper-pos 2) 10) 1.57 0)
      (setq catch_back_point (send *spotkinova* :angle-vector))
      
      ;掴む軌道
      (zikki-send-angle catch_wait_point)
      (zikki-send-angle catch_back_point)
      (zikki-send-angle catch_point)

      ;掴む
      (hand-catch)

      ;机から離す軌道
      (zikki-send-angle catch_wait_point)

      (send *irtviewer* :draw-objects)
    )
    (progn
      (format t "paper timeout~%" topic-wait-time)
    )
  )
)

(defun store-report ()
  (send *spotkinova* :kinova_joint_1 :joint-angle 0)
  (send *spotkinova* :kinova_joint_2 :joint-angle 0)
  (send *spotkinova* :kinova_joint_3 :joint-angle 90)
  (send *spotkinova* :kinova_joint_4 :joint-angle -90)
  (send *spotkinova* :kinova_joint_5 :joint-angle 0)
  (send *spotkinova* :kinova_joint_6 :joint-angle 0)
  (zikki-send-angle (send *spotkinova* :angle-vector))

  (send *spotkinova* :kinova_joint_1 :joint-angle 0)
  (send *spotkinova* :kinova_joint_2 :joint-angle -45)
  (send *spotkinova* :kinova_joint_3 :joint-angle -90)
  (send *spotkinova* :kinova_joint_4 :joint-angle -90)
  (send *spotkinova* :kinova_joint_5 :joint-angle 90)
  (send *spotkinova* :kinova_joint_6 :joint-angle 0)
  (zikki-send-angle (send *spotkinova* :angle-vector))

  (hand-release)

  (send *spotkinova* :kinova_joint_1 :joint-angle 0)
  (send *spotkinova* :kinova_joint_2 :joint-angle -30)
  (send *spotkinova* :kinova_joint_3 :joint-angle -100)
  (send *spotkinova* :kinova_joint_4 :joint-angle -90)
  (send *spotkinova* :kinova_joint_5 :joint-angle 40)
  (send *spotkinova* :kinova_joint_6 :joint-angle 0)
  (zikki-send-angle (send *spotkinova* :angle-vector))

  (send *spotkinova* :kinova-rest-pose)
  (zikki-send-angle (send *spotkinova* :angle-vector))
)

(defun carry-report ()
  (send *spotkinova* :head :move-end-pos #f(0 0 300) :local :debug-view t :rotation-axis nil)  ;後ろ、上、左
  (unix:usleep 1000000)
  (send *spotkinova* :move-to (make-coords :pos #f(3000 0 0) :rpy (float-vector pi/2 0 0)) :world)
  (send *spotkinova* :locate #f(0 0 550))
  (send *irtviewer* :draw-objects)
  (unix:usleep 100000)
  (send *spotkinova* :move-to (make-coords :pos #f(3000 0 0) :rpy (float-vector pi 0 0)) :world)
  (send *spotkinova* :locate #f(0 0 550))
  (send *irtviewer* :draw-objects)
  (unix:usleep 100000)
  (send *spotkinova* :move-to (make-coords :pos #f(1000 0 0) :rpy (float-vector pi 0 0)) :world)
  (send *spotkinova* :locate #f(0 0 550))
  (send *irtviewer* :draw-objects)
)

(defun all-task ()
  (wait-for-report)
  (unix:usleep 3000000)
  (catch-report-pre-slide)
  (unix:usleep 3000000)
  (catch-report)
  (store-report)
  ;(carry-report)
)

;;;;;;;;;; ↑レポート運び(変更中のため動かない) ;;;;;;;;;;

;メッセージの受け取り
;; callback function
(defun paperpose-cb (msg) 
  ; (print (list 'cb length (send msg :poses)))
  (format t "[callback]paperfinder msg (~A) seq:~d length:~d~%" msg (send msg :header :seq) (length (send msg :poses)))
  (when (> (length (send msg :poses)) 0)
    (setq *paper-msg* msg)
    (setq *paper-get-flag* t)
  )
)

;メモゾーン
;send *spotkinova* :kinova_joint_1 :joint-angle kakudo
;setq hoge (ros::tf-pose->coords pose)    ;;lispのcoordsにもってくる
;norm (send hoge :pos)                    ;ノルム
;send (make-coords :pos #f(1000 0 0)) :difference-position hoge ;1000 0 0から見てのhoge
;:thre 100
;(setq msg (one-shot-subscribe "paper_finder/output/pose" geometry_msgs::PoseArray))