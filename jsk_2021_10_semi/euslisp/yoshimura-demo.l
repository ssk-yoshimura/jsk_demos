;; todo ゼミ11/17
;; がぞうにんしきのやり方とかっこいいシミュレーションのやり方を教わる。


;; ファイルをもたせたい
;; ファイルを作る

(when (not (boundp '*pr2*))
  (require "package://pr2eus/pr2-utils.l")
  (setq *pr2* (pr2)))

;; 実機
(load "package://pr2eus/pr2-interface.l")
;; (pr2-init)
;; (send *ri* :speak "hello")

(setq file (make-cube 265 150 15))
(send file :set-color :green)

(setq file-rhandle (instance cascaded-coords :init :rot #2f((0 0 1) (1 0 0) (0 1 0))))
(send file-rhandle :locate #f(-100 -75 0) :world)
(send file :assoc file-rhandle)

(setq file-lhandle (instance cascaded-coords :init :rot #2f((0 0 1) (-1 0 0) (0 -1 0))))
(send file-lhandle :locate #f(-100 75 0) :world)
(send file :assoc file-lhandle)

(setq file-prepare (instance cascaded-coords :init :rot #2f((0 0 1) (-1 0 0) (0 -1 0))))
(send file-prepare :locate #f(-100 150 0) :world)
(send file :assoc file-prepare)

(setq file-lhandle-fixed (instance cascaded-coords :init :rot #2f((0 0 1) (-1 0 0) (0 -1 0))))
(send file-lhandle-fixed :locate #f(570 75 800) :world)

(send file :locate #f(700 0 800))
(send *pr2* :reset-pose)

(setq rhand (send *pr2* :rarm :end-coords))
(setq lhand (send *pr2* :larm :end-coords))

;; ファイルを机の上の位置に移動する
(defun move-file-stand ()
  (send file :locate #f(800 100 950) :world) ;; 本の位置(yが重要)
  (send file :orient -1.57 :y :world)
  (send file :rotate 1.57 :x)
  )
   
;; 使いません
(defun rarm-catchfile () 
  (send *pr2* :rarm :inverse-kinematics
	(send file-rhandle :worldcoords)
	:link-list (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent))
	:move-target (send *pr2* :rarm :end-coords)
	:rotation-axis t
	:debug-view nil)
  )

;; ファイルの前に手をもってくる
(defun get-file-on-desk-prepare ()
  (move-file-stand)
  (send *pr2* :inverse-kinematics
	file-prepare
	:move-target (send *pr2* :larm :end-coords)
	:use-torso nil
	:rotation-axis t
	:rthre (list 1 1 1)
	)
  )

;; ファイルをつかむ
(defun get-file-on-desk ()
  (move-file-stand)
  (send *pr2* :inverse-kinematics
	file-lhandle
	:move-target (send *pr2* :larm :end-coords)
	:use-torso nil
	:rotation-axis t
	:rthre (list 1 1 1)
	)
  )

;; 手を本棚から抜く
(defun get-file-return ()
  (send lhand :assoc file)
  (send *pr2* :larm :move-end-pos #f(-200 0 100) :world))

(defun get-file-return2 ()
  (send lhand :assoc file)
  (send *pr2* :larm :move-end-pos #f(-100 -100 0) :world :rotation-axis nil))

;; ファイルを開く姿勢(固定)
(defun start-open-file ()
  (pr2-ik-l file-lhandle-fixed)
  (rhand-leave)
  ;;(pr2-ik-r file-rhandle)
  )

(defun pr2-ik-both (ltarget rtarget)
  (send *pr2* :inverse-kinematics
	(list ltarget rtarget)
	:move-target (list (send *pr2* :larm :end-coords)
			   (send *pr2* :rarm :end-coords))
	:use-torso t
	:use-base nil
	)
  )

(defun pr2-ik (isL target)
  (if isL
      (send *pr2* :inverse-kinematics
	    target
	    :move-target (send *pr2* :larm :end-coords)
	    :use-torso nil
	    )
    (send *pr2* :inverse-kinematics
	  target
	  :move-target (send *pr2* :rarm :end-coords)
	  :use-torso nil
	  ))
  )

(defun pr2-ik-l (target) (pr2-ik t target))
(defun pr2-ik-r (target) (pr2-ik nil target))

;; ファイルを受け取る姿勢
(defun catch-file ()
  (pr2-ik-both (send file-lhandle :worldcoords) (send file-rhandle :worldcoords))
  )

;; 右手をはなす
(defun rhand-leave ()
  (setq rhand-target (send file-rhandle :copy-worldcoords))
  (send rhand-target :translate #f(-200 0 0) rhand-target)
  (pr2-ik-r rhand-target))

;; 左手を傾ける
(defun lhand-tilt ()
  (send lhand :assoc file)
  (setq lhand-target (send file-lhandle :copy-worldcoords))
  (send lhand-target :rotate -0.7 :z lhand-target)
  (pr2-ik-l lhand-target))

;; 右手を戻す
(defun rhand-catch-again ()
  (setq rhand-target (send rhand :copy-worldcoords))
  (send rhand-target :translate #f(250 20 0) rhand-target)
  (pr2-ik-r rhand-target))

;; 左手を離す、右手を右側にやる
(defun lhand-leave ()
  (send file :dissoc file-rhandle) ;; 右手で持つ
  (send file-rhandle :assoc file)
  (send file-rhandle :newcoords (send rhand :worldrot) (send rhand :worldpos))
  (send file-rhandle :dissoc file) (send file :assoc file-rhandle)
  (send rhand :assoc file)
  (setq rhand-target (send file-rhandle :copy-worldcoords))
  (send rhand-target :translate #f(-100 0 0) rhand-target)
  (pr2-ik-r rhand-target)
  (setq lt (send (send *pr2* :larm :end-coords) :copy-worldcoords))
  (send lt :translate #f(-25 -140 0))
  (pr2-ik-l lt)
  )

;; (send *ri* :start-grasp :gain 0.1)


;; ファイルを開く
(defun open-file ()
  (setq rhand-target (send file-rhandle :copy-worldcoords))
  (setq rmat (make-coords))
  (send rmat :orient -0.05 :z)
  (setq av-seq ())
  (dotimes (i 20)
    ;;    (send rhand-target :rotate -0.05 :z lhand-target)
    (send rhand-target :transform rmat lhand-target) 
    (setq av-seq (cons (pr2-ik-r rhand-target) av-seq))
    (send *irtviewer* :draw-objects)
    (unix:usleep 10000)
    )
  (setq av-seq (reverse av-seq))
  av-seq)

(defun open-file2 ()
  (setq rmat2 (make-coords))
  (send rmat2 :orient 0.05 :z)
  (setq av-seq ())
  (dotimes (i 15)
    (send rhand-target :transform rmat2 lhand-target)
    (setq av-seq (cons (pr2-ik-r rhand-target) av-seq))
    ;;    (pr2-ik-r rhand-target)
    (send *irtviewer* :draw-objects)
    (unix:usleep 10000)
    )
  (setq av-seq (reverse av-seq))
  av-seq
  )

(defun init-moves ()
  (send file :locate #f(700 0 800) :world))


(defun all-moves ()
  (if (send file :parent) (send (send file :parent) :dissoc file))
  ;; (send file :locate #f(500 0 800) :world)
  (send *pr2* :reset-pose)
  (send *irtviewer* :draw-objects)
  (get-file-on-desk-prepare)
  (unix:sleep 1) (send *irtviewer* :draw-objects)
  (get-file-on-desk)
  (unix:sleep 1) (send *irtviewer* :draw-objects)
  (get-file-return)
  (unix:sleep 1) (send *irtviewer* :draw-objects)
  (get-file-return2)
  (unix:sleep 1) (send *irtviewer* :draw-objects)
  (start-open-file)
  (unix:sleep 1) (send *irtviewer* :draw-objects)
  (lhand-tilt)
  (unix:sleep 1) (send *irtviewer* :draw-objects)
  (rhand-catch-again)
  (unix:sleep 1) (send *irtviewer* :draw-objects)
  (lhand-leave)
  (unix:sleep 1) (send *irtviewer* :draw-objects)
  (open-file)
  (unix:sleep 1) (send *irtviewer* :draw-objects)
  (open-file2)
  )

 (objects (list file file-rhandle file-lhandle file-lhandle-fixed file-prepare *pr2* rhand lhand))
