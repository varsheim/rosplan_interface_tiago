#! /usr/bin/env python

import rospy
import actionlib

from rosplan_interface_tiago.msg import GoAction

class GoServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('go', GoAction, self.execute, False)
        self.server.start()
        print "server started?"

    def execute(self, goal):
        # Do lots of awesome groundbreaking robot stuff here
        for i in range(5):
            print "go {}".format(i)
            rospy.sleep(1)

        self.server.set_succeeded()


if __name__ == '__main__':
    rospy.init_node('go_server')
    server = GoServer()
    rospy.spin()