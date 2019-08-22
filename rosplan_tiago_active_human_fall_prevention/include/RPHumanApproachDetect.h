#include <ros/ros.h>
#include <vector>
#include <boost/thread/thread.hpp>

#include "rosplan_action_interface/RPActionInterface.h"
#include <actionlib/client/simple_action_client.h>
#include <rosplan_tiago_active_human_fall_prevention/HumanApproachDetectAction.h>

#ifndef ROSPLAN_INTERFACE_TIAGO_RPHUMANAPPROACHDETECT_H
#define ROSPLAN_INTERFACE_TIAGO_RPHUMANAPPROACHDETECT_H

/**
 * This file defines an action interface created in tutorial 10.
 */

#define ACTION_ADDITION_TIME_S 2

typedef actionlib::SimpleActionClient<rosplan_tiago_active_human_fall_prevention::HumanApproachDetectAction> Client;

namespace KCL_rosplan {

    class RPHumanApproachDetect: public RPActionInterface
    {

    private:
        Client action_client{"human_approach_detect", true};

        ros::ServiceClient service_client;
        std::string current_destination;
        std::string node_name;
        std::string node_name_pretty;
        
    public:

        /* constructor */
        RPHumanApproachDetect(ros::NodeHandle &nh);

        /* listen to and process action_dispatch topic */
        bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
    };
}
#endif
