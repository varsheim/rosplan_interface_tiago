#!/usr/bin/env python

import json
import rospkg
from rosplan_tiago_params.srv import *
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import *

CONFIG_RELATIVE_PATH = '/config/location.json'
PKG_NAME = 'rosplan_tiago_params'


def get_pose_by_name(name, config):
    is_location_name_found = False
    x, y, theta = 0, 0, 0

    with open(config, 'r') as json_data:
        locations = json.load(json_data)
        json_data.close()

    for location in locations:
        if location["value"] == name.location:
            is_location_name_found = True
            try:
                # mathing location name
                x = location["pose2D"]["x"]
                y = location["pose2D"]["y"]
                theta = location["pose2D"]["theta"]
            except KeyError:
                # no matching key
                rospy.logerr("Error getting location values")
                break

    if is_location_name_found == False:
        rospy.logerr("No matching location name, returning zeros")

    point = Point(x, y, 0)
    quat_arr = quaternion_from_euler(0, 0, theta)
    quat = Quaternion(quat_arr[0], quat_arr[1], quat_arr[2], quat_arr[3])
    pose = Pose(point, quat)

    return pose


def get_location(req):
    rospack = rospkg.RosPack()
    rospack_path = rospack.get_path(PKG_NAME)
    pose = get_pose_by_name(req, rospack_path + CONFIG_RELATIVE_PATH)

    # pose = get_pose_by_name("a", "b")
    return GetLocationResponse(pose)


def location_name_server():
    rospy.init_node('location_name_service')
    s = rospy.Service('location_name_service', GetLocation, get_location)
    print "Ready to get location by its name (plan value)."
    rospy.spin()


if __name__ == "__main__":
    location_name_server()