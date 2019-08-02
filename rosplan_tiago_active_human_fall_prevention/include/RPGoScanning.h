#include <ros/ros.h>
#include <vector>
#include <boost/thread/thread.hpp>

#include "rosplan_action_interface/RPActionInterface.h"
#include <actionlib/client/simple_action_client.h>
#include <rosplan_tiago_active_human_fall_prevention/GoScanningAction.h>


#ifndef ROSPLAN_INTERFACE_TIAGO_RPGOSCANNING_H
#define ROSPLAN_INTERFACE_TIAGO_RPGOSCANNING_H

typedef actionlib::SimpleActionClient<rosplan_tiago_active_human_fall_prevention::GoScanningAction> Client;

namespace KCL_rosplan {

    class RPGoScanning: public RPActionInterface
    {

    private:
        Client action_client{"go_scanning", true};
    public:

        /* constructor */
        RPGoScanning(ros::NodeHandle &nh);

        /* listen to and process action_dispatch topic */
        bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
    };
}

#endif //ROSPLAN_INTERFACE_TIAGO_RPGOSCANNING_H
