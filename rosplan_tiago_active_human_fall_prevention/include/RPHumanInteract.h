#include <ros/ros.h>
#include <vector>
#include <boost/thread/thread.hpp>

#include "rosplan_action_interface/RPActionInterface.h"
#include <actionlib/client/simple_action_client.h>
#include <rosplan_tiago_scenarios_msgs/HumanInteractAction.h>


#ifndef ROSPLAN_INTERFACE_TIAGO_RPHUMANINTERACT_H
#define ROSPLAN_INTERFACE_TIAGO_RPHUMANINTERACT_H

#define ACTION_ADDITION_TIME_S 2

typedef actionlib::SimpleActionClient<rosplan_tiago_scenarios_msgs::HumanInteractAction> Client;

namespace KCL_rosplan {

    class RPHumanInteract: public RPActionInterface
    {

    private:
        Client action_client{"human_interact", true};
        ros::ServiceClient service_client;
        std::string human_name;
        std::string node_name;
        std::string node_name_pretty;
    public:

        /* constructor */
        RPHumanInteract(ros::NodeHandle &nh);

        /* listen to and process action_dispatch topic */
        bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
    };
}

#endif //ROSPLAN_INTERFACE_TIAGO_RPHUMANINTERACT_H
