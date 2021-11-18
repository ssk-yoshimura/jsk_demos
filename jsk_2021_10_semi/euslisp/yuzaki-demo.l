;memo
;spotのworldは基本姿勢の時を基準に与えられる

; (when (not (boundp '*room73b2*))
;   (require "models/room73b2-scene.l")
;   (setq *room73b2* (room73b2)))

(when (not (boundp '*spotkinova*))
  (load "package://spotkinovaeus/spotkinova.l")
  (spotkinova :type :gen3_lite_gen3_lite_2f))
(objects (list *spotkinova*))
;(objects (list *room73b2* *spotkinova*))

;reportの代わりにボトル
; (setq *obj* (send *room73b2* :object "room73b2-bottle"))
; (send (car (send *obj* :handle)) :draw-on :flush t)

;;;;;;;;;; ↓基本となる関数たち ;;;;;;;;;;
;ロボットからの位置posの場所までアームだけで動く
(defun arm-move-from-robot (x y z)
  ;(make-coords :pos #f(0 0 0) :rpy #f(0 0 0) )
  (setq arm-pose (send *spotkinova* :copy-worldcoords))
  (setq arm-pose (send arm-pose :translate (float-vector x y z) :world))
  (objects arm-pose)
  (send *spotkinova* :head :inverse-kinematics arm-pose :rotation-axis nil)
)

;ロボットからの位置posの場所まで体全体で動く
(defun full-move-from-robot (pos)
  (let ((arm-pose (make-coords :pos #f(0 0 0) :rpy #f(0 0 0)))) (
    send *spotkinova* :fullbody-inverse-kinematics
      (send arm-pose :translate pos :world)
      :root-link-virtual-joint-weight #f(0.0 0.0 0.1 0.1 0.5 0.5)
  ))
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
  (send *spotkinova* :stop-grasp)
  (when  (boundp '*ri*)
    (format t "zikki daze~%")
    (send *ri* :stop-grasp)
    (send *ri* :wait-interpolation)
  )

  (send *spotkinova* :start-grasp)
  (when  (boundp '*ri*)
    (unix:sleep 5)
    (format t "zikki daze~%")
    (send *ri* :start-grasp)
    (send *ri* :wait-interpolation)
  )
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

  (send *spotkinova* :stop-grasp)
  (when  (boundp '*ri*)
    (format t "zikki daze~%")
    (send *ri* :stop-grasp)
    (send *ri* :wait-interpolation)
  )
)
;;;;;;;;;; ↑お遊び ;;;;;;;;;;

;;;;;;;;;; ↓レポート運び(変更中のため動かない) ;;;;;;;;;;
(defun wait-for-report ()
  (send *spotkinova* :move-to (make-coords :pos #f(3000 500 0) :rpy (float-vector pi/2 0 0)) :world)
  (send *spotkinova* :locate #f(0 0 550))
  (send *irtviewer* :draw-objects)
)

(defun catch-report ()
  (if (send *obj* :parent) (send (send *obj* :parent) :dissoc *obj*))
  (send *spotkinova* :reset-pose)
  ; (send *spotkinova* :fullbody-inverse-kinematics
  ;   (car (send *obj* :handle))
  ;   :root-link-virtual-joint-weight #f(0.0 0.0 0.1 0.1 0.5 0.5)
  ;   :debug-view t)

  (send *spotkinova* :stop-grasp)
  (let ((arm-pose (make-coords :pos #f(0 0 0) :rpy #f(0 0 0 ) )))
    (send *spotkinova* :fullbody-inverse-kinematics
      (send arm-pose :translate
        (float-vector  300 0 0)
        :world)
      :root-link-virtual-joint-weight #f(0.0 0.0 0.1 0.1 0.5 0.5)))
  (unix:usleep 1000000)
  (send *spotkinova* :start-grasp)
  ;(send *spotkinova* :head :end-coords :assoc *obj*)
  (send *irtviewer* :draw-objects)
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
  (catch-report)
  (carry-report)
)

(defun pass-report ()
  (send *spotkinova* :reset-pose)
  (send *spotkinova* :head :move-end-pos #f(600 -300 -500) :world :debug-view t :rotation-axis nil)  ;後ろ、上、左
  ;つかむ
  (send *spotkinova* :reset-pose)
  (send *spotkinova* :head :move-end-rot -90 :z :local :debug-view t)
  ;わたす
  (send *irtviewer* :redraw)
)

;;;;;;;;;; ↑レポート運び(変更中のため動かない) ;;;;;;;;;;
