#include <ros/ros.h>
#include <vector>
#include <boost/thread/thread.hpp>

#include "rosplan_action_interface/RPActionInterface.h"
#include <actionlib/client/simple_action_client.h>
#include <rosplan_tiago_scenarios_msgs/GoWithAttendanceAction.h>
#include <rosplan_tiago_core_msgs/GetLocation.h>
#include <rosplan_tiago_core_msgs/GetHuman.h>
#include <people_msgs/Person.h>


#ifndef ROSPLAN_INTERFACE_TIAGO_RPGOWITHATTENDANCE_H
#define ROSPLAN_INTERFACE_TIAGO_RPGOWITHATTENDANCE_H

#define ACTION_ADDITION_TIME_S 2

typedef actionlib::SimpleActionClient<rosplan_tiago_scenarios_msgs::GoWithAttendanceAction> Client;

namespace KCL_rosplan {

    class RPGoWithAttendance: public RPActionInterface
    {

    private:
        Client action_client{"go_with_attendance", true};
        ros::ServiceClient service_client_location, service_client_human;
        people_msgs::Person human;
        std::string current_destination;
	    std::string current_human_name;
        std::string node_name;
        std::string node_name_pretty;
    public:

        /* constructor */
        RPGoWithAttendance(ros::NodeHandle &nh);

        /* listen to and process action_dispatch topic */
        bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
    };
}

#endif //ROSPLAN_INTERFACE_TIAGO_RPGOWITHATTENDANCE_H
