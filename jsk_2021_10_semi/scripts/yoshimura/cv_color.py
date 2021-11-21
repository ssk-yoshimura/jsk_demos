#!/usr/bin/env python
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

class image_converter:

    def __init__(self):
        self.image_pub = rospy.Publisher("image_topic_2",Image, queue_size=1)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)

    def callback(self,data):
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        (rows,cols,channels) = img.shape

        '''
        if cols > 60 and rows > 60 :
            cv2.circle(cv_image, (50,50), 10, 255)
        '''

        # 2
        
        pv = img[10, 20]
        print ('pv = ' + str(pv))
        # c2 = img[:,:,2]
        # c2 = np.where(c2<50, c2, 0)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        pv2 = hsv[10,20]
        print ('hsv = ' + str(pv2))
        # gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = np.zeros((rows, cols))
        # gray = np.where(hsv[:,:,0]>0, hsv[:,:,0], 0)
        # gray = hsv[:,:,0]
        gray[(hsv[:,:,0] > 80) & (hsv[:,:,0] < 100) & (hsv[:,:,1] > 40)] = 255
        # gray[(hsv[:,:,0] > 200)] = 255

        cv2.imshow("img", img)
        cv2.imshow("gray", gray)
        cv2.waitKey(3)

        cv2.imwrite('a.jpg', img,)

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
