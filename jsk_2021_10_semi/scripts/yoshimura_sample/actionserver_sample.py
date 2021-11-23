#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from jsk_2021_10_semi.msg import *

# ActionServerの動作確認プログラム

class TargetCoordsServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('yoshimura', YoshimuraAction, self.execute, False)
        self.server.start()

    def execute(self, goal):
        print "goal is %d"%(goal.yoshimura_goal)
        result = self.server.get_default_result()
        result.yoshimura_result = 100
        print "result is %d"%(result.yoshimura_result)
        self.server.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node('yoshimura_cv')
    server = TargetCoordsServer()
    rospy.spin()
