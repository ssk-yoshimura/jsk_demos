#!/usr/bin/env roseus

;; ActionServer example

(ros::roseus-add-msgs "jsk_2021_10_semi")

(ros::roseus "yoshimura_server")

(defun balancer_cb (s g)
  (setq gc (send g :goal :yoshimura_goal_change))
  (warning-message 2 "hello~A~%" gc)
  ;; (setq ra (change_angle gc)) ;; 実機
  (setq ra 0.5) ;; デバッグ用
  (unix:sleep 1) ;; デバッグ用
  (send result :result :yoshimura_result_angle ra)
  (send s :set-succeeded result)
  )

(setq result (instance jsk_2021_10_semi::YoshimuraActionResult :init))
(send result :result :yoshimura_result_angle 0.5)

(setq *as* (instance ros::simple-action-server :init
		     "yoshimura" jsk_2021_10_semi::YoshimuraAction :execute-cb 'balancer_cb))

(ros::rate 100)

(defun yoshimura-s ()
  (do-until-key
   (send *as* :worker)
   ;; (send *as* :spin-once)
   (ros::spin-once)
   (ros::sleep)
   )
  )
