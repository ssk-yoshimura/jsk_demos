#!/usr/bin/env roseus

;; actionlibクライアントのサンプルプログラム

#|
rosrun jsk_2021_10_semi cv_pose_as.py
とするとActionサーバができる

yoshimura_clientノード(yoshimura-ac.l)
-> goal 適当
<- result ファイルの横幅（距離を測るため）
サーバのノード(cv_pose_as.py)
<- 画像topic
<- pose_estimator topic
|#

(ros::roseus-add-msgs "jsk_2021_10_semi")

(ros::roseus "yoshimura_client")

(defun yoshimura-client ()
  (let (goal)
    (if (not (boundp '*c*))
	(setq *c* (instance ros::simple-action-client :init
			    "yoshimura" jsk_2021_10_semi::YoshimuraAction)))
    (warning-message 2 ";; ~A wait-for-server~%" (unix::getpid))
    (send *c* :wait-for-server)
    (setq goal (instance jsk_2021_10_semi::YoshimuraActionGoal :init))
    (send goal :goal :yoshimura_goal 11)
    (send *c* :send-goal goal)

    (warning-message 2 ";; ~A wait-for-result~%" (unix::getpid))
    (send *c* :wait-for-result)
    (warning-message 2 ";; ~A result -> ~A~%" (unix::getpid) (send (send *c* :get-result) :yoshimura_result))
    (send (send *c* :get-result) :yoshimura_result)
    ))

(yoshimura-client)
