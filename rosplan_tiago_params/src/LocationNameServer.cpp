#include "ros/ros.h"
#include "rosplan_tiago_params/GetLocation.h"

bool get_location(rosplan_tiago_params::GetLocation::Request &req,
         	  rosplan_tiago_params::GetLocation::Response &res)
{
    // Get location from file here


    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "location_name_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("location_name_server", get_location);
    ROS_INFO("Ready to get location by its name (plan value).");
    ros::spin();

    return 0;
}
