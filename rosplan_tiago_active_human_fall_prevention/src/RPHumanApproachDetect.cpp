#include "RPHumanApproachDetect.h"


/* The implementation of RPTutorial.h */
namespace KCL_rosplan {

    /* constructor */
    RPHumanApproachDetect::RPHumanApproachDetect(ros::NodeHandle &nh) {
        // perform setup
        node_name = ros::this_node::getName();
        node_name_pretty = '(' + node_name + ')';
    }

    /* action dispatch callback */
    bool RPHumanApproachDetect::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
        // The action implementation goes here.
        // Get action parameters
        auto action_parameters = msg.get()->parameters;
        auto action_duration_s = msg.get()->duration;
        auto action_real_duration_s = action_duration_s + ACTION_ADDITION_TIME_S;


        action_client.waitForServer();
	    rosplan_tiago_scenarios_msgs::HumanApproachDetectGoal goal;

        // Fill in goal here
        goal.dummy_goal = 500;

        action_client.sendGoal(goal);
        action_client.waitForResult(ros::Duration(action_real_duration_s));

        if (action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {

        }

        // complete the action
        ROS_INFO("%s: HUMANAPPROACHDETECT Action completing", node_name_pretty.c_str());
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
