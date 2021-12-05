(defun take-book ()
  ;;(send *jaxon* :move-to (make-cascoords :pos #f(3060 -2130 1000) :rpy #f(-1.57 0 0)) :world)
  (if (boundp '*ri*)
      (send *ri* :start-grasp :larm) nil
      )
  (full-ik t (make-cascoords :pos #f(3100 -2700 780) :rpy #f(-1.57 0 0)) t t);;机に左手をつく
  (full-ik  (make-cascoords :pos #f(2700 -2900 1380) :rpy #f(-1.57 0 0)) t t t);;右手を伸ばす
  (if (boundp '*ri*)
      (send *ri* :start-grasp :rarm) nil
      )
  (full-ik  (make-cascoords :pos #f(2700 -2400 1380) :rpy #f(-1.57 0 0)) t t t);;書類を取り出す
  (send *jaxon* :reset-pose)
  (send *jaxon* :move-to (make-cascoords :pos #f(3000 -1900 970) :rpy #f(-1.57 0 0)) :world)
  (setq avs (send *jaxon* :rarm :inverse-kinematics (make-cascoords :pos #f(2700 -2400 1380) :rpy #f(-1.57 0 0)) :move-target (send *jaxon* :rarm :end-coords)))
  (if (boundp '*ri*)
      (progn
	(send *ri* :angle-vector-sequence avs)
	(send *ri* :wait-interpolation)
	)
    nil
    )
  (send *irtviewer* :draw-objects)
  )
