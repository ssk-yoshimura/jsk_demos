#!/usr/bin/env python
# -*- coding: utf-8 -*-   

from __future__ import print_function

import roslib
# roslib.load_manifest('my_package')
import actionlib
import sys
import rospy
import cv2
import time
import numpy as np
from std_msgs.msg import String, Int16, Int16MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from jsk_2021_10_semi.msg import *

from DQN import *

"""
うまくいったパラメータ
試行30回、30回
層[3,20,20,2]
学習係数0.5
gamma=0.3
epsilon: 0.5から線形
reward scale: 1.0
"""

class image_converter:

    def __init__(self):

        self.client = actionlib.SimpleActionClient('yoshimura', YoshimuraAction)
        self.client.wait_for_server()

        self.bridge = CvBridge()
        
        self.timer = time.time()

        self.trial_count = 0
        self.trial_count_max = 30
        
        # dqn
        self.net = dqn([3, 20, 20, 2])
        self.net.set_func([tanh(), tanh(), identify()])
        self.net.set_rate_all(0.5)
        self.epsilon = 0.5
        self.gamma = 0.3

        self.count_max = 30

        # 学習スタート
        goal = YoshimuraGoal()
        goal.yoshimura_goal_change = 100
        self.client.send_goal(goal)
        self.client.wait_for_result()

        self.reset()

    def reset(self):
        self.state_list = []
        self.action_list = []
        self.reward_list = []
        self.count = 0
        # 状態用変数
        self.state_p = 0.0
        self.state_po = 0.0
        self.state_a = 0.0
        # 行動用変数
        self.action = 0
        # 報酬用変数
        self.reward = 0.0

        self.reward_sum = 0.0

        # collect開始
        self.net.collect_init()

        # 実機の角度初期化
        goal = YoshimuraGoal()
        goal.yoshimura_goal_change = -99
        self.client.send_goal(goal)
        self.client.wait_for_result()

        # pc camera
        # rosrun usb_cam usb_cam_node
        # self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback, queue_size=1, buff_size=2**24)
        # self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image,self.callback)
        self.image_sub = rospy.Subscriber("/nao_robot/naoqi_driver/camera/bottom/image_raw", Image, self.callback, queue_size=1, buff_size=2**24)
        print("reset complete")

    def callback(self,data):

        # 1. ここでaction=角度の変化を指定、goalとして送信
        goal = YoshimuraGoal()
        goal.yoshimura_goal_change = self.action
        # print("action="+str(self.action))
        self.client.send_goal(goal)
        
        # 2. ここで画像処理、NNの処理
        img = self.bridge.imgmsg_to_cv2(data, "bgr8")

        (rows,cols,channels) = img.shape
        pv = img[10, 20]
        # print ('pv = ' + str(pv))
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        pv2 = hsv[10,20]
        # print ('hsv = ' + str(pv2))
        gray = np.zeros((rows, cols))
        gray2 = np.zeros((rows, cols))
        gray[(hsv[:,:,0] > 80) & (hsv[:,:,0] < 100) & (hsv[:,:,1] > 40)] = 255
        # gray[(hsv[:,:,0] > 40) & (hsv[:,:,0] < 80) & (hsv[:,:,1] > 40) & (hsv[:,:,2] > 40)] = 255

        gray = gray.astype(np.uint8)
        gray, contours, hierarchy = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 最大領域の輪郭を探す
        maxareaidx = 0
        maxarea = 0
        for i, cnt in enumerate(contours):
            area = cv2.contourArea(cnt)
            if area > maxarea:
                maxareaidx = i
                maxarea = area
                
        # 最大領域の重心
        cx = 0
        cy = 0
        if len(contours) > 0:
            gray2 = cv2.drawContours(gray2, [contours[maxareaidx]], 0, 100, 1)
            M = cv2.moments(contours[maxareaidx])
            if M['m00'] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])

        cv2.circle(gray2, (cx, cy), 10, 255, thickness=2)
        # print(cx)  # xが横方向
        # print(cols) # colsが横方向

        state_p_new = float(cx) / float(cols)
        # 速度の場合
        # timer_new = time.time()
        # self.state_v = (state_p_new - self.state_p) / (timer_new - self.timer)

        # まずは（位置、前の位置、角度）でやってみる。
        self.state_po = self.state_p
        self.state_p = state_p_new

        # これで状態が揃ったのでNNに処理させる
        state_for_play = np.array([self.state_p, self.state_po, self.state_a]).reshape(3,1)
        # epsilonの更新
        # self.epsilon = 0.8 * (1.0 / (float(self.trial_count) + 1.0))
        self.epsilon = 0.5 - 0.5 * float(self.trial_count) / float(self.trial_count_max)
        self.action = self.net.play(state_for_play, self.epsilon)[0]

        # NNのニューロンの値を記録（最後以外）
        if self.count < self.count_max:
            self.net.collect()

        # 情報と行動を収集
        self.state_list.append([self.state_p, self.state_po, self.state_a])
        self.action_list.append(self.action)


        # 報酬を収集
        # 報酬は現在の状態を使って計算してよい（学習時は一つ先の報酬を使用する）
        self.reward = 1.0 - 2.0 * self.state_p
        self.reward *= 1.0
        self.reward_list.append(self.reward)
        self.reward_sum += self.reward

        # print(self.reward)

        cv2.imshow("img", img)
        cv2.imshow("gray", gray2)
        cv2.waitKey(3)

        # cv2.imwrite('a.jpg', img,)


        # 3. 角度情報受け取り
        self.client.wait_for_result()
        result = self.client.get_result()
        
        # 4. 角度・画像・NNの情報を使用して状態の更新
        self.state_a = result.yoshimura_result_angle
        # print("Result %d"%(self.state_a))
        # print(self.state_p, self.state_po, self.state_a)
        print("action="+ str(self.action) + ", angle=" + str(self.state_a) + ", pre_reward=" + str(self.reward) + ", eps="+str(self.epsilon))

        # 5. 全行動が終了したら学習
        self.count += 1
        if self.count > self.count_max:
            self.learn()
            self.reset()
            
    def learn(self):
        print("learn!")
        print("trial "+str(self.trial_count) + ", reward_sum="+str(self.reward_sum))
        self.trial_count += 1
        if self.trial_count >= self.trial_count_max:
            print("finish!")
            self.__del__()
            return
        self.image_sub.unregister()
        state = np.array(self.state_list)[:-1]
        action = np.array(self.action_list)[:-1]
        reward = np.array(self.reward_list)[1:]
        x_all = self.net.collect_end()
        mask_observed_01 = np.zeros((2, action.size))
        mask_observed_01[action, np.arange(action.size)] = 1
        mask_observed = (mask_observed_01 == 1)
        state_next = np.array(self.state_list)[1:].T
        maxQ = self.net.forward_old(state_next)
        label_onedim = reward + self.gamma * maxQ
        label = x_all[-1]
        label[mask_observed] = label_onedim
        self.net.update(label)

        # 結果を喋る
        goal = YoshimuraGoal()
        goal.yoshimura_goal_change = 88
        goal.yoshimura_goal_num = self.trial_count
        goal.yoshimura_goal_score = int(self.reward_sum)
        self.client.send_goal(goal)
        self.client.wait_for_result()

        # time.sleep(3)

def main(args):
    rospy.init_node('cv_dqn', anonymous=True)
    ic = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
