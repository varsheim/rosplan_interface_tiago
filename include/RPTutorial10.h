#include <ros/ros.h>
#include <vector>
#include <boost/thread/thread.hpp>

#include "rosplan_action_interface/RPActionInterface.h"
#include <actionlib/client/simple_action_client.h>
#include <rosplan_interface_tiago/UndockAction.h>

#ifndef KCL_tutorial_10
#define KCL_tutorial_10

/**
 * This file defines an action interface created in tutorial 10.
 */

typedef actionlib::SimpleActionClient<rosplan_interface_tiago::UndockAction> Client;

namespace KCL_rosplan {

    class RPTutorialInterface: public RPActionInterface
    {

    private:

    public:

        /* constructor */
        RPTutorialInterface(ros::NodeHandle &nh);

        /* listen to and process action_dispatch topic */
        bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
    };
}
#endif