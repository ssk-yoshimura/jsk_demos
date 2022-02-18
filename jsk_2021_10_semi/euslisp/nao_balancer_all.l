(load "nao-utils.l")
(nao-init)

(load "nao_balancer_pose.l")

(balancer-set)
(bik-ri-ik-pre)
(send *ri* :servo-on)
(bik-ri-init)

;; stop-grasp, start-grasp

(defun start-as ()
  (load "yoshimura-as.l")
  (yoshimura-s)
  )
