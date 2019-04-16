#include <fstream>
#include "ros/ros.h"
#include "ros/package.h"
#include "rosplan_tiago_params/GetLocation.h"
#include "yaml-cpp/yaml.h"
#include "geometry_msgs/Pose.h"
#include "tf/LinearMath/Matrix3x3.h"

static YAML::Node config;

geometry_msgs::Pose get_pose_by_name(std::string name, YAML::Node config) {
	geometry_msgs::Pose pose;
	pose.position.x = config["pose2D"]["x"].as<std::float_t>();
	pose.position.y = config["pose2D"]["y"].as<std::float_t>();
	pose.position.z = 0;
	auto theta = config["pose2D"]["theta"].as<std::float_t>();

	tf::Quaternion quat;
	tf::Matrix3x3 obs_mat;
	obs_mat.setEulerYPR(theta, 0, 0);
	obs_mat.getRotation(quat);

	pose.orientation.w = quat.getW();
	pose.orientation.x = quat.getX();
	pose.orientation.y = quat.getY();
	pose.orientation.z = quat.getZ();

	return pose;
}

bool get_location(rosplan_tiago_params::GetLocation::Request &req,
         	  rosplan_tiago_params::GetLocation::Response &res)
{
    std::string location;
    location = req.location;
    res.pose = get_pose_by_name(location, config);

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "location_name_server");
    ros::NodeHandle n;

    std::string config_path = ros::package::getPath("rosplan_tiago_params") + "/config/location.yaml";
    config = YAML::LoadFile(config_path);

    ros::ServiceServer service = n.advertiseService("location_name_server", get_location);
    ROS_INFO("Ready to get location by its name (plan value).");
    ros::spin();



	std::ofstream fout("config/location.yaml");
	fout << config;

    return 0;
}
