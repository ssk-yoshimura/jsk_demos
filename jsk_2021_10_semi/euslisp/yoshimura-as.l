#!/usr/bin/env roseus

;; ActionServer example

(ros::roseus-add-msgs "jsk_2021_10_semi")

(ros::roseus "yoshimura_server")

(defun balancer_cb (s g)
  (setq gc (send g :goal :yoshimura_goal_change))
  (warning-message 2 "goal_change is ~A~%" gc)
  (setq ra 0.0)
  (cond
   ((= gc -99) ;; リセット
    (progn 
      (bik-ri-init-fast)
      (unix:sleep 2)
      )
    )
   ((= gc 88) ;; 結果を言う
    (progn
      (setq goal_num (send g :goal :yoshimura_goal_num))
      (setq goal_score (send g :goal :yoshimura_goal_score))
      (send *ri* :speak (format nil "~A回目の点数は ~A点でした" goal_num goal_score))
      )
    )
   ((= gc 100) ;; 初期化
    (progn
      (send *ri* :speak "学習を始めます")
      )
    )
   ((not (member gc (list -99 88))) ;; 角度指令
    (progn
      (setq ra (change_angle gc)) ;; 実機
      (unix:usleep 400000)
      ;; (setq ra 0.5) ;; デバッグ用
      ;; (unix:sleep 1) ;; デバッグ用
      )
    )
   )
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
