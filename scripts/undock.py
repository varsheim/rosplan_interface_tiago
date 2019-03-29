#! /usr/bin/env python

import rospy
import actionlib

from rosplan_interface_tiago.msg import UndockAction

class UndockServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('undock', UndockAction, self.execute, False)
        self.server.start()
        print "server started?"

    def execute(self, goal):
        # Do lots of awesome groundbreaking robot stuff here
        rospy.sleep(4)
        self.server.set_succeeded()


if __name__ == '__main__':
    rospy.init_node('undock_server')
    server = UndockServer()
    print "server created"
    rospy.spin()