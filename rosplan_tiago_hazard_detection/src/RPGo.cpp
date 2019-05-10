#include "RPGo.h"


/* The implementation of RPTutorial.h */
namespace KCL_rosplan {

    /* constructor */
    RPGo::RPGo(ros::NodeHandle &nh) {
        // Create service client for getting location params
        service_client = nh.serviceClient<rosplan_tiago_params::GetLocation>("location_name_service");
    }

    /* action dispatch callback */
    bool RPGo::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

        // The action implementation goes here.
        // Get action parameters
        auto action_parameters = msg.get()->parameters;

        // log available parameters
        for (auto it = begin (action_parameters); it != end (action_parameters); ++it) {
            if (strcmp(it->key.c_str(), "destination") == 0) {
                current_destination = it->value.c_str();
            }
        }

        // Get the actual values by calling the service
        // Fill the srv message first
        rosplan_tiago_params::GetLocation srv;
        srv.request.location = current_destination;

        if (service_client.call(srv)) {
            ROS_INFO("Got matching location params of %s.", current_destination.c_str());
        }
        else {
            ROS_ERROR("Failed to call service location_name_service");
            return false;
        }

        ROS_INFO("CLIENT: GO: Waiting for sever");
        action_client.waitForServer();
        rosplan_tiago_hazard_detection::GoGoal goal;

        // Fill in goal here
        goal.pose = srv.response.pose;

        ROS_INFO("CLIENT: GO: I will send goal now");
        action_client.sendGoal(goal);

        ROS_INFO("CLIENT: GO: I will wait for result now");
        action_client.waitForResult(ros::Duration(10.0));

        ROS_INFO("CLIENT: GO: I received result and it is:");
        if (action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            // complete the action
            ROS_INFO("KCL: (%s) GO Action completing.", msg->name.c_str());

            return true;
        }
        else {
            ROS_INFO("CLIENT: GO: Current State: %s\n", action_client.getState().toString().c_str());

            return false;
        }

    }
} // close namespace

/*-------------*/
/* Main method */
/*-------------*/

int main(int argc, char **argv) {
    ros::init(argc, argv, "rosplan_go_action_client", ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");

    // create action client here?

    // create PDDL action subscriber
    KCL_rosplan::RPGo rpti(nh);

    rpti.runActionInterface();

    return 0;
}
