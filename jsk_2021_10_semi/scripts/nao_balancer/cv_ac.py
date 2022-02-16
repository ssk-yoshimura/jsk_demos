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

# 画像topicを受け取り、緑色かどうかで2値化してグレースケール画像を作る

class image_converter:

    def __init__(self):

        self.client = actionlib.SimpleActionClient('yoshimura', YoshimuraAction)
        self.client.wait_for_server()
        
        self.image_pub = rospy.Publisher("image_topic_2",Image, queue_size=1)
        self.x_pub = rospy.Publisher("ball_x", Int16, queue_size=1)
        self.y_pub = rospy.Publisher("ball_y", Int16, queue_size=1)
        self.fileRect = [0,0,0,0]

        self.ball = rospy.Publisher("ball", Int16MultiArray, queue_size=1) 

        self.ball_x = 0
        self.ball_y = 0
        self.timer = time.time()

        self.state_p = 0.0
        self.state_po = 0.0
        self.state_a = 0.0

        self.bridge = CvBridge()
        # pc camera
        # rosrun usb_cam usb_cam_node
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)
        # self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image,self.callback)

    def callback(self,data):

        # 1. ここで角度の変化を指定、goalとして送信
        goal = YoshimuraGoal()
        goal.yoshimura_goal_change = -1
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
                       
        # 最大領域の外接矩形(いらない)
        if len(contours) > 0:
            gray2 = cv2.drawContours(gray2, [contours[maxareaidx]], 0, 100, 1)
            self.fileRect = cv2.boundingRect(contours[maxareaidx])
            x, y, w, h = self.fileRect
            cv2.rectangle(gray2, (x,y), (x+w, y+h), 255, 3)

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
        print(self.state_p, self.state_po, self.state_a)

        # おしまい


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
