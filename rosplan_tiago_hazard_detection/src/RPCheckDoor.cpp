#include "RPCheckDoor.h"

namespace KCL_rosplan {

    /* constructor */
    RPCheckDoor::RPCheckDoor(ros::NodeHandle &nh) {
        node_name = ros::this_node::getName();
        node_name_pretty = '(' + node_name + ')';
    }

    /* action dispatch callback */
    bool RPCheckDoor::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

        // The action implementation goes here.
        // Get action parameters
        auto action_parameters = msg.get()->parameters;
        auto action_duration_s = msg.get()->duration;
        auto action_real_duration_s = action_duration_s + ACTION_ADDITION_TIME_S;

        // log available parameters
        ROS_INFO("%s: Duration is: %f", node_name_pretty.c_str(), action_duration_s);
        for (auto it = begin (action_parameters); it != end (action_parameters); ++it) {
            ROS_INFO("%s: %s <----> %s", node_name_pretty.c_str(), it->key.c_str(), it->value.c_str());
        }

        client.waitForServer();

        // Fill in goal here
        rosplan_tiago_hazard_detection::CheckDoorGoal goal;
        goal.sensor = "robot";
        client.sendGoal(goal);
        client.waitForResult(ros::Duration(action_real_duration_s));

        if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {

        }

        // complete the action
        ROS_INFO("%s: CHECKDOOR Action completing", node_name_pretty.c_str());
        return true;
    }
} // close namespace

/*-------------*/
/* Main method */
/*-------------*/

int main(int argc, char **argv) {

    ros::init(argc, argv, "rosplan_check_door_action_client", ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");

    // create PDDL action subscriber
    KCL_rosplan::RPCheckDoor rpti(nh);

    rpti.runActionInterface();

    return 0;
}
