;;(send *jaxon* :move-to (make-cascoords :pos #f(4700 -1200 890) :rpy #f(0 0 0)) :world)
(defun set-binder ()
  (full-ik (make-cascoords :pos #f(5200 -1600 900) :rpy #f(0 0.78 -1.57)) nil t t nil) ;;机の上にバインダーを置く
  (print "離してください")
  (read-char)
  (send *jaxon* :reset-pose)
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
