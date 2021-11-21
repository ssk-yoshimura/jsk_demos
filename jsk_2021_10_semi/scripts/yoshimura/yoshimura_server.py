#! /usr/bin/env python
import rospy
import actionlib
from jsk_2021_10_semi.msg import *

# ActionClient

if __name__ == '__main__':
    rospy.init_node('do_dishes_client')
    client = actionlib.SimpleActionClient('yoshimura', YoshimuraAction)
    client.wait_for_server()

    goal = YoshimuraGoal()
    goal.yoshimura_goal = 1
    print "Requesting yoshimura_goal %d"%(goal.yoshimura_goal)
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))

    result = client.get_result()
    print "Result %d"%(result.yoshimura_result)
