#include "RPHumanInteract.h"



/* The implementation of RPTutorial.h */
namespace KCL_rosplan {

    /* constructor */
    RPHumanInteract::RPHumanInteract(ros::NodeHandle &nh) {
        // perform setup
    }

    /* action dispatch callback */
    bool RPHumanInteract::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

        // The action implementation goes here.
        // Get action parameters
        auto action_parameters = msg.get()->parameters;

        // log available parameters
        for (auto it = begin (action_parameters); it != end (action_parameters); ++it) {
            ROS_INFO("%s <----> %s", it->key.c_str(), it->value.c_str());
        }

        // Get the actual values by calling the service

        ROS_INFO(msg.get()->parameters.back().value.c_str());

//        action_client.waitForServer();
//        rosplan_tiago_active_human_fall_prevention::HumanInteractGoal goal;
//
//        // Fill in goal here
//        goal.blind_goal = 500;
//
//        action_client.sendGoal(goal);
//
//        action_client.waitForResult(ros::Duration(10.0));
//
//        if (action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
//
//        }

	    ros::Duration(3).sleep();

        // complete the action
        ROS_INFO("KCL: (%s) HUMANINTERACT Action completing.\n", msg->name.c_str());
        return true;


    }
} // close namespace

/*-------------*/
/* Main method */
/*-------------*/

int main(int argc, char **argv) {

    ros::init(argc, argv, "rosplan_human_interact_action_client", ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");

    // Create service client here
//    ros::ServiceClient client = nh.serviceClient<>("location_srv")


    // create PDDL action subscriber
    KCL_rosplan::RPHumanInteract rpti(nh);

    rpti.runActionInterface();

    return 0;
}
