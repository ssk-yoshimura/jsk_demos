(defun take-book ()
  ;;(send *jaxon* :move-to (make-cascoords :pos #f(3760 -2130 1000) :rpy #f(-1.57 0 0)) :world)
  (if (boundp '*ri*)
      ;(send *ri* :start-grasp :larm) nil
      )
  (full-ik (make-cascoords :pos #f(3250 -2300 1000) :rpy #f(-1.57 0 0)) (make-cascoords :pos #f(4120 -2280 950) :rpy #f(-1.57 0 0)) t t nil)
  (full-ik t (make-cascoords :pos #f(3750 -2700 810) :rpy #f(-1.57 0 0)) (make-cascoords :pos #f(3400.0 -2100 23) :rpy #f(-1.57 0 0)) t t);;机に左手をつく
;  (if (boundp '*ri*)
;      (send *ri* :autobalancerservice_stopautobalancer)
;      nil
;    )
  (full-ik  (make-cascoords :pos #f(3700 -2700 1580) :rpy #f(-1.57 0 0)) t t t nil);;右手を伸ばす
  (send *jaxon* :rarm :inverse-kinematics (make-cascoords :pos #f(3700 -3000 1580) :rpy #f(-1.57 0 0))
	:move-target (send *jaxon* :rarm :end-coords)) ;;右手を棚の中に
  (if (boundp '*ri*)
      (progn
	(send *ri* :angle-vector (send *jaxon* :angle-vector) 5000)
	(send *ri* :wait-interpolation)
;(send *ri* :start-grasp :rarm)
	(print "掴んでください")
	(read-char)
	)
    nil
    )
  (full-ik  (make-cascoords :pos #f(3700 -2700 1580) :rpy #f(-1.57 0 0)) t t t nil)
;  (if (boundp '*ri*)
;    (progn
;	(send *ri* :angle-vector (send *jaxon* :angle-vector) 10000)
;	(send *ri* :wait-interpolation)
;	)
;    nil
;    )
  (send *jaxon* :reset-pose)
  (send *jaxon* :move-to (make-cascoords :pos #f(3630 -2200 890) :rpy #f(-1.57 0 0)) :world)
  (send *jaxon* :rarm :inverse-kinematics (make-cascoords :pos #f(3350 -2450 1200) :rpy #f(-0.78 0 0))
	:move-target (send *jaxon* :rarm :end-coords))
  (if (boundp '*ri*)
      (progn
	(send *ri* :angle-vector (send *jaxon* :angle-vector) 5000)
	(send *ri* :wait-interpolation)
	(unix::sleep 8)
	)
    nil
    )
  (send *irtviewer* :draw-objects)
  )
