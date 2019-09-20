#include <ros/ros.h>
#include <vector>
#include <boost/thread/thread.hpp>

#include "rosplan_action_interface/RPActionInterface.h"
#include <actionlib/client/simple_action_client.h>
#include <rosplan_tiago_transportation_attendant/GetLoadAction.h>
#include <rosplan_tiago_params/GetHuman.h>
#include <people_msgs/Person.h>


#ifndef ROSPLAN_INTERFACE_TIAGO_RPGETLOAD_H
#define ROSPLAN_INTERFACE_TIAGO_RPGETLOAD_H

#define ACTION_ADDITION_TIME_S 2

typedef actionlib::SimpleActionClient<rosplan_tiago_transportation_attendant::GetLoadAction> Client;

namespace KCL_rosplan {

    class RPGetLoad: public RPActionInterface
    {

    private:
        Client action_client{"get_load", true};
        ros::ServiceClient service_client_human;
        std::string current_human_name;
        people_msgs::Person human;
        std::string node_name;
        std::string node_name_pretty;
    public:

        /* constructor */
        RPGetLoad(ros::NodeHandle &nh);

        /* listen to and process action_dispatch topic */
        bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
    };
}

#endif //ROSPLAN_INTERFACE_TIAGO_RPGETLOAD_H
