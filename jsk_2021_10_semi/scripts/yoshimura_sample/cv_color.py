#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String, Int16, Int16MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# 画像topicを受け取り、緑色かどうかで2値化してグレースケール画像を作る

class image_converter:

    def __init__(self):
        self.image_pub = rospy.Publisher("image_topic_2",Image, queue_size=1)
        self.x_pub = rospy.Publisher("ball_x", Int16, queue_size=1)
        self.y_pub = rospy.Publisher("ball_y", Int16, queue_size=1)

        self.ball = rospy.Publisher("ball", Int16MultiArray, queue_size=1) 

        self.ball_x = 0
        self.ball_y = 0

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image,self.callback)
        self.depth_sub = rospy.Subscriber("/camera/depth_registered/image_raw", Image, self.callback_depth)

    def callback_depth(self, data):
        try:
            
            depth = self.bridge.imgmsg_to_cv2(data, '32FC1')
        except CvBridgeError as e:
            print(e)
            return

        xc = self.ball_x
        yc = self.ball_y

        print(xc, yc, depth[yc,xc])

        cv2.circle(depth, (xc,yc),10,0,thickness=3)

        cv2.imshow("depth", depth)
        cv2.waitKey(3)

        ball_list = [xc, yc, depth[yc,xc]]
        array_forPublish = Int16MultiArray(data=ball_list)

        self.ball.publish(array_forPublish)

    def callback(self,data):
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        (rows,cols,channels) = img.shape
        pv = img[10, 20]
        # print ('pv = ' + str(pv))
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        pv2 = hsv[10,20]
        # print ('hsv = ' + str(pv2))
        gray = np.zeros((rows, cols))
        gray2 = np.zeros((rows, cols))
        # gray[(hsv[:,:,0] > 80) & (hsv[:,:,0] < 100) & (hsv[:,:,1] > 40)] = 255
        gray[(hsv[:,:,0] > 40) & (hsv[:,:,0] < 80) & (hsv[:,:,1] > 40) & (hsv[:,:,2] > 40)] = 255

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
                       
        # 最大領域の外接矩形
        if len(contours) > 0:
            gray2 = cv2.drawContours(gray2, [contours[maxareaidx]], 0, 100, 1)
            self.fileRect = cv2.boundingRect(contours[maxareaidx])
            x, y, w, h = self.fileRect
            cv2.rectangle(gray2, (x,y), (x+w, y+h), 255, 3)

        cv2.imshow("img", img)
        cv2.imshow("gray", gray2)
        cv2.waitKey(3)

        cv2.imwrite('a.jpg', img,)

        xc = int(self.fileRect[0] + self.fileRect[2]/2)
        yc = int(self.fileRect[1] + self.fileRect[3]/2)
        self.ball_x = xc
        self.ball_y = yc
        self.x_pub.publish(xc)
        self.y_pub.publish(yc)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
        except CvBridgeError as e:
            print(e)

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
