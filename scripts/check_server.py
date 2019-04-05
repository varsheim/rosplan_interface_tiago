#! /usr/bin/env python

import rospy
import actionlib

from rosplan_interface_tiago.msg import CheckAction

class CheckServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('check', CheckAction, self.execute, False)
        self.server.start()
        print "SERVER: CHECK: started\n"

    def execute(self, goal):
        # Do lots of awesome groundbreaking robot stuff here
        print "SERVER: CHECK: execute - i will sleep now\n"
        rospy.sleep(2)
        print "SERVER: CHECK: i have finished sleeping - sending 'succeeded'\n"

        self.server.set_succeeded()


if __name__ == '__main__':
    rospy.init_node('check_server')
    server = CheckServer()
    rospy.spin()