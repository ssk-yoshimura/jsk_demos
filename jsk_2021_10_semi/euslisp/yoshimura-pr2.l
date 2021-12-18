;; 実機用

(defun pr2-init-yoshimura ()
  (load "package://pr2eus/pr2-interface.l"))

(defun pr2-init-yoshimura2 ()
  (pr2-init))

(defun pr2i ()
  (pr2-init-yoshimura)
  (pr2-init-yoshimura2))

(defun pr2ri ()
  (send *ri* :angle-vector (send *pr2* :angle-vector)))

(defun pr2reset ()
  (send *pr2* :reset-pose)
  (pr2ri))

;; ファイルをつかむ
(defun grasp-file ()
  (load "yoshimura-demo.l")
  (pr2reset)
  (send *pr2* :head-neck-p :joint-angle 30)
  (pr2ri)
  (send *ri* :wait-interpolation)
  (send *ri* :stop-grasp :larm)
  (get-file-on-desk-prepare) ;; つかむ準備
  (pr2ri)
  (send *ri* :wait-interpolation)
  (get-file-on-desk) ;; つかむ
  (pr2ri)
  (send *ri* :wait-interpolation)
  (send *ri* :start-grasp :larm :gain 0.1)
  (send *ri* :wait-interpolation)
  (unix:sleep 1)
  (get-file-return) ;; 本棚から抜く
  (pr2ri)
  (send *ri* :wait-interpolation)
  (get-file-return2)
  (pr2ri)
  (send *ri* :wait-interpolation)
  (start-open-file) ;; ひらく場所へ
  (pr2ri)
  (send *ri* :wait-interpolation)
  )
  

;; 一連動作
(defun all-moves-ri ()
  
  ;; (load "yoshimura-demo.l")
  """
  (send *ri* :stop-grasp)
  (send *ri* :wait-interpolation)
  (send *pr2* :reset-pose)
  (send file :locate #f(500 0 800) :world)
  (catch-file)
  (pr2ri)
  (send *ri* :wait-interpolation)
  (send *ri* :stop-grasp)
  (send *ri* :start-grasp)
  (send *ri* :stop-grasp :rarm)
  (rhand-leave) (pr2ri)
  (send *ri* :wait-interpolation)
  """
  (grasp-file)
  (lhand-tilt) (pr2ri)
  (send *ri* :wait-interpolation)
  (send *ri* :stop-grasp :rarm)
  (unix:sleep 1)
  (send *ri* :wait-interpolation)
  (rhand-catch-again)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (send *ri* :start-grasp :rarm)
  (send *ri* :wait-interpolation)
  (send *ri* :stop-grasp :larm)
  (unix:sleep 1)
  (lhand-leave)
  (pr2ri)
  (send *ri* :wait-interpolation)
  (setq avs (open-file))
  (send *ri* :angle-vector-sequence avs (list 200))
  (send *ri* :wait-interpolation)
  (unix:sleep 1)
  (send *ri* :start-grasp :larm)
  (send *ri* :wait-interpolation)
  (setq avs (open-file2))
  (send *ri* :angle-vector-sequence avs (list 200))
  (send *ri* :wait-interpolation)
  )

;; 次回試す
(defun go-forward ()
  (send *ri* :go-pos 0.01 0 0))
