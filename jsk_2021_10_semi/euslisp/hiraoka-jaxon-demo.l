(when (not (boundp '*room73b2*))
  (require "models/room73b2-scene.l")
  (setq *room73b2* (room73b2)))

(when (not (boundp '*nao*))
  (require "package://naoeus/nao.l")
  (setq *nao* (NaoH25V50)))

(when (not (boundp '*fetch*))
  (require "package://fetcheus/fetch.l")
  (setq *fetch* (fetch)))

(when (not (boundp '*spotkinova*))
  (require "package://spotkinovaeus/spotkinova.l")
  (setq *spotkinova* (spotkinova)))

(when (not (boundp '*jaxon*))
  (require "package://cart_humanoid/euslisp/jaxon_red.l")
  (setq *jaxon* (JAXON_RED)))

(when (not (boundp '*pr2*))
  (require "package://pr2eus/pr2.l")
  (setq *pr2* (pr2)))

(when (not (boundp '*hrp2jsknts*))
  (require :hrp2jsknts "package://hrpsys_ros_bridge_tutorials/models/hrp2jsknts.l")
  (setq *hrp2jsknts* (instance hrp2jsknts-robot :init)))

;;hrp2
;;(require "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-interface.l ")
;;(hrp2jsknts-init)
;;(setq *jaxon* *hrp2jsknts*)

;;jaxon
;;(require "package://hrpsys_choreonoid_tutorials/euslisp/jaxon_jvrc-interface.l")
;;(jaxon_jvrc-init)

(objects (list *room73b2* *jaxon* *hrp2jsknts*))
;(objects (list *room73b2* *nao* *fetch* *pr2* *spotkinova* *jaxon* *hrp2jsknts*))

(load "hiraoka-chair.l")
(load "hiraoka-shelf.l")
(load "hiraoka-table.l")

(defun demo-prep ()
  (send *hrp2jsknts* :locate #f(1550 0 700) :worldpos)
  (send *jaxon* :locate #f(800 0 1000) :worldpos)
  (send *hrp2jsknts* :reset-pose)
  (send *jaxon* :reset-pose)
  (send *irtviewer* :draw-objects)
  )

;;使わない
(defun move-to-spot(spot)
  (let ((coods-dif (send *jaxon* :transformation (send *room73b2* :spot spot))))
    (send *jaxon* :calc-walk-pattern-from-footstep-list (send *jaxon* :go-pos-params->footstep-list (aref (send coods-dif :pos) 0) (aref (send coods-dif :pos) 1) (rad2deg (elt (car (euler-angle (send coods-dif :rot))) 2)))
	  :debug-view
	  :no-message
	  :default-step-height 120)
  (if (boundp '*ri*)
      (send *ri* :go-pos (* 0.001 (aref (send coods-dif :pos) 0)) (* 0.001 (aref (send coods-dif :pos) 1)) (rad2deg (elt (car (euler-angle (send coods-dif :rot))) 2)))
    )
  nil
  )
)

(defun move-to (*robot* x y th)  
  (send *robot* :calc-walk-pattern-from-footstep-list (send *robot* :go-pos-params->footstep-list x y th) :debug-view :no-message :default-step-height 120)
  (if (boundp '*ri*)
      (send *ri* :go-pos (* 0.001 x) (* 0.001 y) th)
    )
  nil
)
  
(defun full-ik (rarm-tgt larm-tgt rleg-tgt lleg-tgt foot-step
			 &key 
			 (robot *jaxon*)
			 (target-centroid-pos (apply #'midpoint 0.5 (send robot :legs :end-coords :worldpos)))
			 )
  ;;両手先、足先の目標。位置姿勢を維持するかどうか、指令するかどうか、足を動かすかどうか
  (let (
	(ra (if (null rarm-tgt) (make-cascoords) (if (eq t rarm-tgt) (send robot :rarm :end-coords ) rarm-tgt)))
	(la (if (null larm-tgt) (make-cascoords) (if (eq t larm-tgt) (send robot :larm :end-coords ) larm-tgt)))
	(rl (if (null rleg-tgt) (make-cascoords) (if (eq t rleg-tgt) (send robot :rleg :end-coords ) rleg-tgt)))
	(ll (if (null lleg-tgt) (make-cascoords) (if (eq t lleg-tgt) (send robot :lleg :end-coords ) lleg-tgt)))
	)
    (send robot :fullbody-inverse-kinematics (list ra la rl ll)
	  :link-list  (list (send robot :link-list (send robot :rarm :end-coords :parent))
			    (send robot :link-list (send robot :larm :end-coords :parent))
			    (send robot :link-list (send robot :rleg :end-coords :parent))
			    (send robot :link-list (send robot :lleg :end-coords :parent)))
	  :move-target (list (send robot :rarm :end-coords )
			     (send robot :larm :end-coords )
			     (send robot :rleg :end-coords )
			     (send robot :lleg :end-coords ))
	  :rotation-axis (list (not (null rarm-tgt)) (not (null larm-tgt)) (not (null rleg-tgt)) (not (null lleg-tgt)))
	  :translation-axis (list (not (null rarm-tgt)) (not (null larm-tgt)) (not (null rleg-tgt)) (not (null lleg-tgt)))
	  :debug-view
	  :no-message
	  )
    )
  (if (boundp '*ri*)
      (progn
	(if foot-step
	    (send *ri* :set-foot-steps (list (make-coords :coords (send robot :lleg :end-coords :copy-worldcoords) :name :lleg) (make-coords :coords (send robot :rleg :end-coords :copy-worldcoords) :name :rleg)));;足を動かす
	  nil
	  )
	(send *ri* :angle-vector (send robot :angle-vector) 10000)
	(send *ri* :wait-interpolation)
	)
    nil)
  
  )


;;冷蔵庫の中を探す //使わない、
(defun search-fridge()
  (move-to-spot "fridge-front-spot");冷蔵庫の前まで移動
  (full-ik t (make-cascoords :pos #f(5350 1480 1300) :rpy #f(0 0 0)) (make-cascoords :pos #f(4500 1200 40)) t t) ;冷蔵庫に手をつく
  (unix::sleep 1)
  (full-ik (make-cascoords :pos #f(5350 1480 400) :rpy #f(0 0.85 0)) t t t t)  ;冷蔵庫の取手に手をのばす
  (unix::sleep 1)
  (setq *handle* (send (send *room73b2* :object "room73b2-hitachi-fiesta-refrigerator") :link :room73b2-hitachi-fiesta-refrigerator-bodyset5))
  (send (send *handle* :parent-link) :dissoc *handle*)
  (send *jaxon* :rarm :end-coords :assoc *handle*)
  (full-ik (make-cascoords :pos #f(5050 1480 400) :rpy #f(0 0.85 0)) t t t t) ;冷蔵庫の引き出しを引く
  (unix::sleep 1)
  (send *jaxon* :head :angle-vector #f(30 0))
  (unix::sleep 1)
  (send *jaxon* :head :angle-vector #f(-90 0))
  (unix::sleep 1)
  (send *jaxon* :head :angle-vector #f(0 0))
  (full-ik (make-cascoords :pos #f(5350 1480 400) :rpy #f(0 0.85 0)) t t t t)
  (unix::sleep 1)
  (send *jaxon* :rarm :end-coords :dissoc *handle*)
  nil
  )

(defun bring-book ()
  (demo-prep)
  (chair-prep)
;  (remove-chair-hrp2) ;;hrp2 通り道の椅子をどける
;  (send *jaxon* :locate #f(2000 0 0)) ;;現実環境で実行するため
					;  (move-to *jaxon* 960 -2130 -90) ;;jaxon 本棚の前に移動
  (send *jaxon* :move-to (make-cascoords :pos #f(3760 -2130 1000) :rpy #f(-1.57 0 0)) :world)
  (take-book) ;;jaxon バインダーをとる
;  (move-to *hrp2jsknts* 0 500 -180) ;;hrp2は次のロボットのために道を開ける
;  (move-to *hrp2jsknts* -500 0 0)
  (move-to *jaxon* 0 0 0)
  (move-to *jaxon* -500 0 0);;本棚から離れる
  (move-to *jaxon* 0 0 90) 
  (set-binder)
  ;(move-to *jaxon* 0 1000 -180) ;;次のロボットに道を譲る
 )

