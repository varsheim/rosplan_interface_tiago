#! /usr/bin/env python

import rospy
import actionlib

from rosplan_interface_tiago.msg import CheckAction

class CheckServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('check', CheckAction, self.execute, False)
        self.server.start()
        print "CHECK server started?"

    def execute(self, goal):
        # Do lots of awesome groundbreaking robot stuff here
        for i in range(5):
            print "check {}".format(i)
            rospy.sleep(1)

        self.server.set_succeeded()


if __name__ == '__main__':
    rospy.init_node('check_server')
    server = CheckServer()
    rospy.spin()