#include "RPHumanApproachDetect.h"


/* The implementation of RPTutorial.h */
namespace KCL_rosplan {

    /* constructor */
    RPHumanApproachDetect::RPHumanApproachDetect(ros::NodeHandle &nh) {
        // perform setup
    }

    /* action dispatch callback */
    bool RPHumanApproachDetect::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

        // The action implementation goes here.
        // Get action parameters
        auto action_parameters = msg.get()->parameters;

        // log available parameters
        for (auto it = begin (action_parameters); it != end (action_parameters); ++it) {
            ROS_INFO("%s <----> %s", it->key.c_str(), it->value.c_str());
        }

        // Get the actual values by calling the service

        ROS_INFO(msg.get()->parameters.back().value.c_str());

        action_client.waitForServer();
        rosplan_tiago_active_human_fall_prevention::HumanApproachDetectGoal goal;

        // Fill in goal here
        goal.blind_goal = 500;

        action_client.sendGoal(goal);

        action_client.waitForResult(ros::Duration(60.0));

        if (action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {

        }

        // complete the action
        ROS_INFO("KCL: (%s) HUMANAPPROACHDETECT Action completing.\n", msg->name.c_str());
        return true;


    }
} // close namespace

/*-------------*/
/* Main method */
/*-------------*/

int main(int argc, char **argv) {
    ros::init(argc, argv, "rosplan_human_approach_detect_action_client", ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");

    // create action client here?

    // create PDDL action subscriber
    KCL_rosplan::RPHumanApproachDetect rpti(nh);

    rpti.runActionInterface();

    return 0;
}
