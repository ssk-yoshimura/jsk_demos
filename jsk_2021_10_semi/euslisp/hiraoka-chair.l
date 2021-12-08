(defun chair-prep ()
  (setq *chair* (send *room73b2* :object "room73b2-chair0"))
  (send *chair* :locate #f(2300 0 0) :world)
  (send *chair* :rotate 3.14 :z)
  (send *irtviewer* :draw-objects)
  )

(defun remove-chair ()
  ;;椅子の横に手をもってくる
  (if (boundp '*hrp2jsknts*)
      (send *jaxon* :arms :inverse-kinematics
	    (list (make-cascoords :pos #f(1900 -190 750) :rpy #f(0 0 0)) (make-cascoords :pos #f(1900 190 750) :rpy #f(0 0 0)))
	    :move-target (list (send *jaxon* :rarm :end-coords) (send *jaxon* :larm :end-coords))
	    :link-list (list (send *jaxon* :link-list (send *jaxon* :rarm :end-coords :parent)) (send *jaxon* :link-list (send *jaxon* :larm :end-coords :parent)))
	    )
    (progn
      (if (boundp '*ri*)
	    (send *ri* :stop-grasp) nil
	    )
      (send *jaxon* :arms :inverse-kinematics
	    (list (make-cascoords :pos #f(2000 -400 750) :rpy #f(1.57 1.57 0)) (make-cascoords :pos #f(2000 400 750) :rpy #f(-1.57 1.57 0)))
	    :move-target (list (send *jaxon* :rarm :end-coords) (send *jaxon* :larm :end-coords))
	    :link-list (list (send *jaxon* :link-list (send *jaxon* :rarm :end-coords :parent)) (send *jaxon* :link-list (send *jaxon* :larm :end-coords :parent)))
	    )
	)
  )
  (if (boundp '*ri*)
      (progn
	(send *ri* :angle-vector (send *jaxon* :angle-vector) 5000)
	(send *ri* :wait-interpolation)
	)
    nil
    )
  ;;椅子に両手をそえる。
  (if (boundp '*hrp2jsknts*)
      (progn
	(print "掴んでください")
	(read-char)
	)
      (progn
	(send *jaxon* :arms :inverse-kinematics
	      (list (make-cascoords :pos #f(2000 -250 750) :rpy #f(1.57 0.5 0)) (make-cascoords :pos #f(2000 250 750) :rpy #f(-1.57 0.5 0)))
	      :move-target (list (send *jaxon* :rarm :end-coords) (send *jaxon* :larm :end-coords))
	      :link-list (list (send *jaxon* :link-list (send *jaxon* :rarm :end-coords :parent)) (send *jaxon* :link-list (send *jaxon* :larm :end-coords :parent))))
	(if (boundp '*ri*)
	    (progn
	      (send *ri* :angle-vector (send *jaxon* :angle-vector) 5000)
	      (send *ri* :wait-interpolation)
	      )
	  nil
	  )
	)
    )
  
  (send *jaxon* :rarm :end-coords :assoc *chair*)
  (move-to 1000 0 0) ;;椅子を運ぶ
  (move-to 500 0 90)
  (move-to 300 0 0) 
  (send *jaxon* :rarm :end-coords :dissoc *chair*)

  ;;椅子から手を離す。
  (if (boundp '*hrp2jsknts*)
      (progn
	(print "離してください")
	(read-char)
	(send *jaxon* :reset-pose)
	(send *jaxon* :arms :inverse-kinematics
	      (list (make-cascoords :pos #f(3500 630 950) :rpy #f(1.57 0 0)) (make-cascoords :pos #f(2700 630 950) :rpy #f(1.57 0 0)))
	      :move-target (list (send *jaxon* :rarm :end-coords) (send *jaxon* :larm :end-coords))
	      :link-list (list (send *jaxon* :link-list (send *jaxon* :rarm :end-coords :parent)) (send *jaxon* :link-list (send *jaxon* :larm :end-coords :parent))) :debug-view t) 
	)
    (progn
      (send *jaxon* :arms :inverse-kinematics
	    (list (make-cascoords :pos #f(3500 680 750) :rpy #f(1.57 1.57 0)) (make-cascoords :pos #f(2600 680 750) :rpy #f(-1.57 1.57 0)))
	    :move-target (list (send *jaxon* :rarm :end-coords) (send *jaxon* :larm :end-coords))
	    :link-list (list (send *jaxon* :link-list (send *jaxon* :rarm :end-coords :parent)) (send *jaxon* :link-list (send *jaxon* :larm :end-coords :parent))))
      )
    )
  (if (boundp '*ri*)
      (progn
	(send *ri* :angle-vector (send *jaxon* :angle-vector) 5000)
	(send *ri* :wait-interpolation)
	)
    nil
    )
  (send *jaxon* :reset-pose)
  (if (boundp '*ri*)
      (send *ri* :angle-vector (send *jaxon* :angle-vector) 5000)
    nil
    )
  )
