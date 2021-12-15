(defun set-binder ()
  ;;(send *jaxon* :move-to (make-cascoords :pos #f(3950 1600 970) :rpy #f(3.14 0 0)) :world)
  (full-ik (make-cascoords :pos #f(3300 1700 900) :rpy #f(-1.57 0.78 1.57)) nil t t nil) ;;机の上にバインダーを置く
  (if (boundp '*ri*)
      (send *ri* :stop-grasp :rarm) nil
      )
  (full-ik (make-cascoords :pos #f(3300 1900 900) :rpy #f(-1.57 0.78 1.57)) nil t t nil)
  (send *jaxon* :reset-pose)
  (send *jaxon* :move-to (make-cascoords :pos #f(4050 1550 985) :rpy #f(3.14 0 0)) :world)
  (if (boundp '*ri*)
      (progn
	(send *ri* :angle-vector (send *jaxon* :angle-vector 5000))
	(send *ri* :wait-interpolation)
	)
    nil
    )
  (send *irtviewer* :draw-objects)
  (unix::sleep 1)
)
