#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from jsk_recognition_msgs.msg import PeoplePoseArray
import actionlib
from jsk_2021_10_semi.msg import *


# human estimatorと画像の色情報から、人が指している方向の色を入手する
# actionlibで色を送る


class image_converter:

    def __init__(self):

        self.joint_names = ["left shoulder", "right shoulder", "left elbow", "right elbow", "right wrist"]
        self.joint_size = len(self.joint_names)
        self.joint_x = np.ones(self.joint_size)
        self.joint_y = np.ones(self.joint_size)
        self.fileRect = [0]*4 # ファイルの隅
        # 直線パラメータ
        self.a = 1
        self.b = 1
        self.c = 1

        self.goalVal = 0

        self.color = 0
        
        self.image_pub = rospy.Publisher("image_topic_2",Image, queue_size=1)

        self.bridge = CvBridge()
        # self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)
        # self.pose_sub = rospy.Subscriber("/edgetpu_human_pose_estimator/output/poses", PeoplePoseArray, self.callback_poses)

        # self.image_sub = rospy.Subscriber("/kinect_head/rgb/image_color", Image,self.callback)
        # self.pose_sub = rospy.Subscriber("/my_human_detector/output/poses", PeoplePoseArray, self.callback_poses)
        self.server = actionlib.SimpleActionServer('yoshimura', YoshimuraAction, self.execute, False)
        self.server.start()
    
    # ActionServerの関数
    def execute(self, goal):
        print ("goal is" + str(goal.yoshimura_goal))
        rate = rospy.Rate(10)
        self.image_sub = rospy.Subscriber("/kinect_head/rgb/image_color", Image,self.callback)
        self.pose_sub = rospy.Subscriber("/my_human_detector/output/poses", PeoplePoseArray, self.callback_poses)
        # self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)
        # self.pose_sub = rospy.Subscriber("/edgetpu_human_pose_estimator/output/poses", PeoplePoseArray, self.callback_poses)
        while self.goalVal < 30:
            rate.sleep()
            if rospy.is_shutdown():
                break
        self.image_sub.unregister()
        self.pose_sub.unregister()
        cv2.destroyAllWindows()
        result = self.server.get_default_result()
        result.yoshimura_result = self.color #hsvのhの値をresultとする
        print ("result is " + str(result.yoshimura_result))
        self.goalVal = 0
        self.filew = []
        self.server.set_succeeded(result)

    # 画像処理のコールバック関数
    def callback(self,data):
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        (rows,cols,channels) = img.shape

        # 緑色を閾値にして2値化
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        gray = np.zeros((rows, cols))
        # gray[(hsv[:,:,0] > 80) & (hsv[:,:,0] < 100) & (hsv[:,:,1] > 40)] = 255 # 緑

        gray[(hsv[:,:,0] > 60) & (hsv[:,:,0] < 100) & (hsv[:,:,1] > 20)] = 255 # 緑
        gray[(hsv[:,:,0] > 100) & (hsv[:,:,0] < 115) & (hsv[:,:,1] > 80)] = 255 # 青
        gray[(hsv[:,:,0] >= 20) & (hsv[:,:,0] <= 40) & (hsv[:,:,1] > 80)] = 255 # 黄

        # gray[(hsv[:,:,0] > 175) & (hsv[:,:,0] < 180) & (hsv[:,:,1] > 40)] = 255 # 赤
        # 赤は誤認識しやすいので、青にする

        # 輪郭検出
        gray2 = np.zeros((rows, cols))
        gray = gray.astype(np.uint8)
        gray, contours, hierarchy = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnt_sorted = sorted(contours, key=lambda x : cv2.contourArea(x), reverse=True)
        cnt_sorted = [cs for cs in cnt_sorted if cv2.contourArea(cs) > 200] # 小さな輪郭は消す
        
        # print(len(contours))

        # 最大領域の輪郭を探す
        maxareaidx = 0
        maxarea = 0
        """
        for i, cnt in enumerate(contours):
            area = cv2.contourArea(cnt)
            if area > maxarea:
kk               maxareaidx = i
               maxarea = area
        """
        go_flag = False

        # 直線パラメータを取得
        x1 = self.joint_x[3]
        y1 = self.joint_y[3]
        x2 = self.joint_x[4]
        y2 = self.joint_y[4]

        #  矢印の描写
        cv2.arrowedLine(gray2, (int(x1),int(y1)), (int(x2),int(y2)), 255, 2, tipLength=0.3)

        a = y2-y1
        b = x1-x2
        c = y1*x2-x1*y2

        # 直線に最も近い重心を探す関数
        def get_nearest(cnt):
            M = cv2.moments(cnt)
            if M['m00'] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
            else:
                cx = 1
                cy = 1
            dtc = abs(a*cx+b*cy+c) / np.sqrt(a*a + b*b)
            if (x2-x1)*(cx-x1) + (y2-y1)*(cy-y1) < 0:
                dtc = dtc + 500
            return dtc

        h_color = 0

        # 直線に最も近い輪郭を探す
        if len(cnt_sorted) > 0:
            cnt_nearest = min(cnt_sorted, key=get_nearest)
            x,y,w,h = cv2.boundingRect(cnt_nearest)
            cv2.rectangle(gray2, (x,y), (x+w, y+h), 255, 6)
            M = cv2.moments(cnt_nearest)
            if M['m00'] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
            else:
                cx = 1
                cy = 1
            cv2.circle(gray2, (cx, cy), 7, 255, thickness=5)
            # 直線に最も近い輪郭の色
            # print(hsv[cy,cx])
            x,y = cnt_nearest[0][0]
            h_color = hsv[cy,cx][0]
            print(y,x)
            # print(h_color)

        # 色が変わらないかどうかで判別
        print(abs(int(self.color) - int(h_color)))
        if abs(int(self.color) - int(h_color)) < 20:
            self.goalVal += 1
        else:
            self.goalVal = 0

        self.color = h_color

        print("goalVal is " + str(self.goalVal) + ", h is " + str(h_color))

        # ソート順に描く (コメントアウトすると速くなります)
        for i in range(min(5, len(cnt_sorted))):
            gray2 = cv2.drawContours(gray2, [cnt_sorted[i]], 0, 50, 1)
            x,y,w,h = cv2.boundingRect(cnt_sorted[i])
            # cv2.rectangle(gray2, (x,y), (x+w, y+h), 255, 3)
            # 重心も描く
            M = cv2.moments(cnt_sorted[i])
            if M['m00'] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
            else:
                cx = 1
                cy = 1
            cv2.circle(gray2, (cx, cy), 5, 255, thickness=1)

        # 全部描く
        """
        for i in range(len(contours)):
            gray2 = cv2.drawContours(gray2, [contours[i]], 0, 50, 1)
            x,y,w,h = cv2.boundingRect(contours[i])
            cv2.rectangle(gray2, (x,y), (x+w, y+h), 255, 3)
        """
               
        # 最大領域の外接矩形
        '''
        if len(contours) > 0:
            gray2 = cv2.drawContours(gray2, [contours[maxareaidx]], 0, 100, 1)
            self.fileRect = cv2.boundingRect(contours[maxareaidx])
            x, y, w, h = self.fileRect
            # cv2.rectangle(gray2, (x,y), (x+w, y+h), 255, 3)

            # 発進してよいかの判別
            """
            f0 = max(self.joint_y[0], self.joint_y[1]) < y
            f1 = y < min(self.joint_y[2], self.joint_y[3])
            f2 = self.joint_x[3] < x
            f3 = x+w < self.joint_x[2]
            f4 = min(self.joint_y[2], self.joint_y[3]) < y+h
            f5 = abs(self.joint_x[0]+self.joint_x[1]-x-x-w) < 50
            go_flag = f0 & f1 & f2 & f3 & f4 & f5
            # print (abs(self.joint_x[0]+self.joint_x[1]-x-x-w))
            """
            go_flag = False

            goc = [255, 255, 255] if go_flag else [0, 0, 255]
            cv2.rectangle(img, (x,y), (x+w, y+h), goc, 3)
        '''
                        
        # 人間の関節描画
        for i in range(self.joint_size):
            cv2.circle(gray2, (int(self.joint_x[i]), int(self.joint_y[i])), 10, 255, thickness=3)
            cv2.circle(img, (int(self.joint_x[i]), int(self.joint_y[i])), 10, (255, 255, 255), thickness=3)
            
        cv2.imshow("img", img)
        cv2.imshow("gray2", gray2)
        cv2.waitKey(3)

        # cv2.imwrite('a.jpg', img)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
        except CvBridgeError as e:
            print(e)

    # pose_estimatorのコールバック関数
    def callback_poses(self, data):
        if len(data.poses) == 0:
            return
        for i, v in enumerate(self.joint_names):
            if v in data.poses[0].limb_names:
                idx = data.poses[0].limb_names.index(v)
                self.joint_x[i] = data.poses[0].poses[idx].position.x
                self.joint_y[i] = data.poses[0].poses[idx].position.y
                # print(v + ' ' + str(self.joint_x[i]) + ' ' + str(self.joint_y[i]))
        
def main(args):
    rospy.init_node('image_converter', anonymous=True)
    ic = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
