(defun chair-prep ()
  (setq *chair* (send *room73b2* :object "room73b2-chair0"))
  (send *chair* :locate #f(2300 0 0) :world)
  (send *chair* :rotate 3.14 :z)
  (send *irtviewer* :draw-objects)
  )

;;jaxon(send *hrp2jsknts* :locate #f(1550 0 700) :worldpos)
(defun remove-chair-hrp2 ()
  ;;椅子の横に手をもってくる
  (send *hrp2jsknts* :arms :inverse-kinematics
	(list (make-cascoords :pos #f(1900 -190 750) :rpy #f(0 0 0)) (make-cascoords :pos #f(1900 190 750) :rpy #f(0 0 0)))
	:move-target (list (send *hrp2jsknts* :rarm :end-coords) (send *hrp2jsknts* :larm :end-coords))
	:link-list (list (send *hrp2jsknts* :link-list (send *hrp2jsknts* :rarm :end-coords :parent)) (send *hrp2jsknts* :link-list (send *hrp2jsknts* :larm :end-coords :parent)))
	)

  (if (boundp '*ri*)
      (progn
	(send *ri* :angle-vector (send *hrp2jsknts* :angle-vector) 5000)
	(send *ri* :wait-interpolation)
	)
    nil
    )
  ;;椅子に両手をそえる。
  (print "掴んでください")
  (read-char)
  
  (send *hrp2jsknts* :rarm :end-coords :assoc *chair*)
  (move-to *hrp2jsknts* 1000 0 0) ;;椅子を運ぶ
  (move-to *hrp2jsknts* 500 0 90)
  (move-to *hrp2jsknts* 300 0 0) 
  (send *hrp2jsknts* :rarm :end-coords :dissoc *chair*)

  ;;椅子から手を離す。
  (print "離してください")
  (read-char)
  (send *hrp2jsknts* :reset-pose)
  (send *hrp2jsknts* :arms :inverse-kinematics
	(list (make-cascoords :pos #f(3500 630 950) :rpy #f(1.57 0 0)) (make-cascoords :pos #f(2700 630 950) :rpy #f(1.57 0 0)))
	:move-target (list (send *hrp2jsknts* :rarm :end-coords) (send *hrp2jsknts* :larm :end-coords))
	:link-list (list (send *hrp2jsknts* :link-list (send *hrp2jsknts* :rarm :end-coords :parent)) (send *hrp2jsknts* :link-list (send *hrp2jsknts* :larm :end-coords :parent)))) 

  (if (boundp '*ri*)
      (progn
	(send *ri* :angle-vector (send *hrp2jsknts* :angle-vector) 5000)
	(send *ri* :wait-interpolation)
	)
    nil
    )
  (send *hrp2jsknts* :reset-pose)
  (if (boundp '*ri*)
      (send *ri* :angle-vector (send *hrp2jsknts* :angle-vector) 5000)
    nil
    )
  )

;;jaxon(send *jaxon* :locate #f(1550 0 1000) :worldpos)
(defun remove-chair-jaxon ()
  (move-to *jaxon* 750 0 0)
  ;;椅子の横に手をもってくる
  (send *jaxon* :arms :inverse-kinematics
	(list (make-cascoords :pos #f(2000 -400 750) :rpy #f(1.57 1.57 0)) (make-cascoords :pos #f(2000 400 750) :rpy #f(-1.57 1.57 0)))
	:move-target (list (send *jaxon* :rarm :end-coords) (send *jaxon* :larm :end-coords))
	:link-list (list (send *jaxon* :link-list (send *jaxon* :rarm :end-coords :parent)) (send *jaxon* :link-list (send *jaxon* :larm :end-coords :parent)))
	)
  
  (if (boundp '*ri*)
      (progn
	(send *ri* :angle-vector (send *jaxon* :angle-vector) 5000)
	(send *ri* :wait-interpolation)
	)
    nil
    )
  
  ;;椅子に両手をそえる。
  (send *jaxon* :arms :inverse-kinematics
	(list (make-cascoords :pos #f(2000 -180 750) :rpy #f(1.57 0.5 0)) (make-cascoords :pos #f(2000 180 750) :rpy #f(-1.57 0.5 0)))
	:move-target (list (send *jaxon* :rarm :end-coords) (send *jaxon* :larm :end-coords))
	:link-list (list (send *jaxon* :link-list (send *jaxon* :rarm :end-coords :parent)) (send *jaxon* :link-list (send *jaxon* :larm :end-coords :parent))))
  
  (if (boundp '*ri*)
      (progn
	(send *ri* :angle-vector (send *jaxon* :angle-vector) 5000)
	(send *ri* :wait-interpolation)
	)
    nil
    )
  
  (send *jaxon* :rarm :end-coords :assoc *chair*)
  (move-to *jaxon* 1000 0 0) ;;椅子を運ぶ
  (move-to *jaxon* 500 0 90)
  (move-to *jaxon* 300 0 0) 
  (send *jaxon* :rarm :end-coords :dissoc *chair*)

  ;;椅子から手を離す。
  (send *jaxon* :arms :inverse-kinematics
	(list (make-cascoords :pos #f(3500 680 750) :rpy #f(3.14 1.57 0)) (make-cascoords :pos #f(2600 680 750) :rpy #f(0 1.57 0)))
	:move-target (list (send *jaxon* :rarm :end-coords) (send *jaxon* :larm :end-coords))
	:link-list (list (send *jaxon* :link-list (send *jaxon* :rarm :end-coords :parent)) (send *jaxon* :link-list (send *jaxon* :larm :end-coords :parent))))
  
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

