;; 吉村が環境の確認用に作ったやつ
;; 動くとは限りません

(load "package://spoteus/spot-utils.l")

(spot)
(objects (list *spot*))

(dotimes (i 100)
  (send *spot* :body-inverse-kinematics
        (make-coords :pos (float-vector 0 (* 100 (cos (* pi i 0.02))) (* 100 (sin (* pi i 0.02))))))
  (unix:usleep 20000)
  (send *irtviewer* :redraw))

(dotimes (i 100)
  (send *spot* :body-inverse-kinematics
        (make-coords :pos (float-vector 0 0 (* 20 (sin (* pi i 0.02))))
                     :rpy (float-vector (* 0.2 (sin (* pi i 0.02))) (* 0.2 (sin (* pi i 0.02))) (* 0.2 (sin (* pi i 0.02))))))
  (unix:usleep 20000)
  (send *irtviewer* :redraw))

(load "package://spotkinovaeus/spotkinova.l")

(spotkinova :type :gen3_lite_gen3_lite_2f)
(objects (list *spotkinova*))

(dotimes (i 100)
  (send *spotkinova* :move-to (make-coords) :world)
  (send *spotkinova* :reset-pose)
  (let ((arm-pose (send *spotkinova* :head :end-coords :copy-worldcoords)))
    (send *spotkinova* :fullbody-inverse-kinematics
          (send arm-pose :translate
                (float-vector (* 300 (cos (* pi i 0.02))) 0 (* 300 (sin (* pi i 0.02))))
                :world)))
  (unix:usleep 20000)
  (send *irtviewer* :redraw))

;; :reset-pose unix:usleep
