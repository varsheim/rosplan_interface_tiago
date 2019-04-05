#! /usr/bin/env python

import rospy
import actionlib

from rosplan_interface_tiago.msg import GoAction

class GoServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('go', GoAction, self.execute, False)
        self.server.start()
        print "SERVER: GO: started"

    def execute(self, goal):
        # Do lots of awesome groundbreaking robot stuff here
        print "SERVER: GO: execute - i will sleep now\n"
        rospy.sleep(2)
        print "SERVER: GO: i have finished sleeping - sending 'succeeded'\n"

        self.server.set_succeeded()


if __name__ == '__main__':
    rospy.init_node('go_server')
    server = GoServer()
    rospy.spin()