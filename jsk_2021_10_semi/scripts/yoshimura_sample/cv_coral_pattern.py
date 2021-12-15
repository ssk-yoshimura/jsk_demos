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

# 画像topicとcoralのpose estimatorのtopicをそれぞれ受け取る
# 緑色かどうかで2値化した画像に4つの関節を描画

# 姿勢情報をcsvに保存する
# 相対位置を考えたほうがよさそう

# kinectのdepthcloudがrosbagでもうまく表示できて、使えそう
# 使えたら、自分で座標変換しなくてよさそう

class image_converter:

    def __init__(self):

        self.joint_names = ["left shoulder", "right shoulder", "left elbow", "right elbow",
                             "right wrist"]
        self.nose_pos = [0]*2
        self.joint_size = len(self.joint_names)
        self.joint_x = np.zeros(self.joint_size)
        self.joint_y = np.zeros(self.joint_size)
        
        self.image_pub = rospy.Publisher("image_topic_2",Image, queue_size=1)

        self.skip_data = 100
        self.train_data_size = 100
        self.joint_data = np.zeros((self.train_data_size, self.joint_size*2))
        self.data_count = 0

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)
        self.pose_sub = rospy.Subscriber("/edgetpu_human_pose_estimator/output/poses", PeoplePoseArray, self.callback_poses)

    def callback(self,data):
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        (rows,cols,channels) = img.shape
        
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        gray = np.zeros((rows, cols))
        gray[(hsv[:,:,0] > 80) & (hsv[:,:,0] < 100) & (hsv[:,:,1] > 40)] = 255

        for i in range(5):
            cv2.circle(gray, (int(self.joint_x[i]), int(self.joint_y[i])), 10, 255, thickness=3)
        # cv2.imshow("img", img)
        # cv2.imshow("gray", gray)
        cv2.waitKey(3)

        # cv2.imwrite('a.jpg', img)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
        except CvBridgeError as e:
            print(e)

    def callback_poses(self, data):
        if len(data.poses) == 0:
            return
        for i, v in enumerate(self.joint_names):
            if v in data.poses[0].limb_names:
                idx = data.poses[0].limb_names.index(v)
                self.joint_x[i] = data.poses[0].poses[idx].position.x
                self.joint_y[i] = data.poses[0].poses[idx].position.y
                # print(v + ' ' + str(self.joint_x[i]) + ' ' + str(self.joint_y[i]))

        if "nose" in data.poses[0].limb_names:
            idx = data.poses[0].limb_names.index("nose")
            self.nose_pos[0] = data.poses[0].poses[idx].position.x
            self.nose_pos[1] = data.poses[0].poses[idx].position.y
            print(self.nose_pos)

        # データの保存
        if self.skip_data > 0:
            self.skip_data -= 1
            return
        elif self.skip_data == 0:
            print("start recording")
            self.skip_data -= 1
            
        if self.data_count < self.train_data_size:
            self.joint_data[self.data_count, :self.joint_size] = self.joint_x - self.nose_pos[0]
            self.joint_data[self.data_count, -self.joint_size:] = self.joint_y - self.nose_pos[1]
            self.data_count += 1
            
        if self.data_count == self.train_data_size:
            print("finish recording")
            np.savetxt('testpose.csv', self.joint_data)
            self.data_count += 1

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
