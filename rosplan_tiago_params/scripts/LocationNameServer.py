#!/usr/bin/env python

import json
from rospkg import rospack
from rosplan_tiago_params.srv import *
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion

config_path = '/home/luke/tiago_rosplan_ws/src/rosplan_interface_tiago/rosplan_tiago_params/config/location.json'


def get_pose_by_name(name, config):
    with open(config_path) as json_data:
        d = json.load(json_data)
        json_data.close()
        print d

    point = Point(1, 1, 1)
    quat = Quaternion(0, 0, 0, 1)

    pose = Pose(point, quat)

    return pose;


def get_location(req):
    pose = get_pose_by_name("a", "b")

    return GetLocationResponse(pose)


def add_two_ints_server():
    rospy.init_node('location_name_server')
    s = rospy.Service('location_name_server', GetLocation, get_location)
    print "Ready to get location by its name (plan value)."
    rospy.spin()


if __name__ == "__main__":
    add_two_ints_server()