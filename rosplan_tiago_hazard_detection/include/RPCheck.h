//
// Created by robot on 3/29/19.
//
#include <ros/ros.h>
#include <vector>
#include <boost/thread/thread.hpp>

#include "rosplan_action_interface/RPActionInterface.h"
#include <actionlib/client/simple_action_client.h>
#include <rosplan_tiago_hazard_detection/CheckAction.h>


#ifndef ROSPLAN_INTERFACE_TIAGO_RPCHECK_H
#define ROSPLAN_INTERFACE_TIAGO_RPCHECK_H

typedef actionlib::SimpleActionClient<rosplan_tiago_hazard_detection::CheckAction> Client;

namespace KCL_rosplan {

    class RPCheck: public RPActionInterface
    {

    private:
        Client client{"check", true};
    public:

        /* constructor */
        RPCheck(ros::NodeHandle &nh);

        /* listen to and process action_dispatch topic */
        bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
    };
}

#endif //ROSPLAN_INTERFACE_TIAGO_RPCHECK_H
