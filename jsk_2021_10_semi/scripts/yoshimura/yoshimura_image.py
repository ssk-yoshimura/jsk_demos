import numpy as np
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

# img = False

def callback(msg):
    try:
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(msg, "bgr8")
        rospy.loginfo(img)

    except Exception, err:
        rospy.logerr(err)

if __name__ == '__main__':
    rospy.init_node('get_image', log_level=rospy.DEBUG)
    rospy.Subscriber('/usb_cam/image_raw', Image, callback)
    while True:
        if not img:
            continue
        cv2.imshow("Img", img)
        c = cv2.waitKey(2)
            if c == 27:
                break
    capture.release()
    cv2.destroyAllWindows()
