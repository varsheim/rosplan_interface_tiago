#include <ros/ros.h>
#include <vector>
#include <boost/thread/thread.hpp>

#include "rosplan_action_interface/RPActionInterface.h"
#include <actionlib/client/simple_action_client.h>
#include <rosplan_tiago_hazard_detection/GoAction.h>
#include <geometry_msgs/Pose.h>
#include <rosplan_tiago_params/GetLocation.h>

#ifndef ROSPLAN_INTERFACE_TIAGO_RPGO_H
#define ROSPLAN_INTERFACE_TIAGO_RPGO_H

#define ACTION_ADDITION_TIME_S 2

typedef actionlib::SimpleActionClient<rosplan_tiago_hazard_detection::GoAction> Client;

namespace KCL_rosplan {

    class RPGo: public RPActionInterface
    {

    private:
        Client action_client{"go", true};
	    std::string node_name;
	    std::string node_name_pretty;
        ros::ServiceClient service_client;
        std::string current_destination;
    public:

        /* constructor */
        RPGo(ros::NodeHandle &nh);

        /* listen to and process action_dispatch topic */
        bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
    };
}
#endif
