;; (load "nao-utils.l")
;; (nao-init)

;; まずbalancer-set（一回きり）
;; その後bik-ri-ik-pre（一回きり）
;; その後bik-ri-init（実機）
;; その後bik-ri-p（毎回）


;; 右手関係の変数の初期化など
(defun balancer-init ()
  (setq bl-target (instance cascaded-coords :init :pos #f(120 -30 40) :rot #2f((1 0 0) (0 0 -1) (0 1 0))))
  (setq bl-target-theta 0.0)
  (setq balancer (instance cascaded-coords :init :coords (send *nao* :rarm :end-coords :worldcoords)))
  (send *nao* :rarm :end-coords :assoc balancer)
  (send balancer :locate #f(50 50 -70))
  (objects (list *nao* balancer bl-target))
  )

(balancer-init)

(defun nao-ik-b (target th)
  (send *nao* :rarm :inverse-kinematics
	target
	:link-list (send *nao* :link-list (send *nao* :rarm :end-coords :parent))
	:move-target balancer
	 :thre (list th th th)
	)
  )

(defun nao-ik-l (target th)
  (send *nao* :larm :inverse-kinematics
	target
	:link-list (send *nao* :link-list (send *nao* :larm :end-coords :parent))
	:move-target (send *nao* :larm :end-coords)
	 :thre (list th th th)
	)
  )

;; 全身のセット
(defun balancer-set ()
  (balancer-init)
  (send *nao* :reset-pose)
  (setq lhand-coords (coords :coords (send *nao* :larm :end-coords :worldcoords)))
  (send *nao* :init-pose)
  (send *nao* :head :neck-p :joint-angle 15)
  (nao-ik-l lhand-coords 30)
  (nao-ik-b bl-target 30)
  (nao-ik-b bl-target 5)
  )

(defun bl-target-rotate (theta)
  (setq theta-diff (- theta bl-target-theta))
  (setq bl-target-theta theta)
  (send bl-target :rotate theta-diff :x)
  )

;; 角度変更
(defun balancer-ik (theta)
  (bl-target-rotate theta)
  (nao-ik-b bl-target 30)
  )

;; 角度変更（実機）
(defun bik-ri (theta)
  (balancer-ik theta)
  (send *ri* :angle-vector (send *nao* :angle-vector) :fast nil 0 :min-time 0.05)
  (send *ri* :wait-interpolation)
  )

;; ikを先に解いておく
(defun bik-ri-ik-pre ()
  (setq angle_num 0)
  (setq angle_num_max 6)
  (setq bik-list ())
  (push (balancer-ik -0.3) bik-list) ;; 6
  (push (balancer-ik -0.2) bik-list) ;; 5
  (push (balancer-ik -0.1) bik-list) ;; 4
  (push (balancer-ik -0.0) bik-list) ;; 3
  (push (balancer-ik 0.1) bik-list) ;; 2
  (push (balancer-ik 0.2) bik-list) ;; 1
  (push (balancer-ik 0.3) bik-list) ;; 0
  )

;; 先に解いておいたikの姿勢で実機を動かす
(defun bik-ri-p (num)
  (send *ri* :angle-vector (elt bik-list num) :fast nil 0 :min-time 0.05)
  (send *ri* :wait-interpolation)
  )

;; 先に解いておいたikの姿勢で、持つ姿勢にする
(defun bik-ri-init ()
  (send *ri* :angle-vector (elt bik-list 0)))

;; 0,1,2のいずれかが与えられたとき、0なら角度-1, 1なら角度+0, 2なら角度+1する
(defun change_angle (change)
  (setq angle_num (+ angle_num (- change 1)))
  (if (< angle_num 0) (setq angle_num 0))
  (if (> angle_num angle_num_max) (setq angle_num angle_num_max))
  (bik-ri-p angle_num)
  (/ (float angle_num) (float angle_num_max))
  )

(objects (list *nao* balancer bl-target))
