#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from jsk_2021_10_semi.msg import *

import time

# ActionClientのサンプルプログラム

def yoshimura_send_goal(cl):
    goal = YoshimuraGoal()
    goal.yoshimura_goal = 99
    print "Requesting yoshimura_goal %d"%(goal.yoshimura_goal)
    cl.send_goal(goal)
    # time.sleep(5) # 重い処理
    cl.wait_for_result()
    result = cl.get_result()
    print "Result %d"%(result.yoshimura_result)
    

if __name__ == '__main__':
    rospy.init_node('yoshimura_python_client')
    client = actionlib.SimpleActionClient('yoshimura', YoshimuraAction)
    client.wait_for_server()

    for i in range(10):
        yoshimura_send_goal(client)
