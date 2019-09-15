#include "RPGetLoad.h"

namespace KCL_rosplan {

    /* constructor */
    RPGetLoad::RPGetLoad(ros::NodeHandle &nh) {
        // perform setup
        node_name = ros::this_node::getName();
        node_name_pretty = '(' + node_name + ')';
    }

    /* action dispatch callback */
    bool RPGetLoad::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
        // The action implementation goes here.
        // Get action parameters
        auto action_parameters = msg.get()->parameters;
        auto action_duration_s = msg.get()->duration;
        auto action_real_duration_s = action_duration_s + ACTION_ADDITION_TIME_S;

        // log available parameters
        for (auto it = begin (action_parameters); it != end (action_parameters); ++it) {
            // "destination" is defined in pddl domain as param name
            if (strcmp(it->key.c_str(), "human") == 0) {
                human_name = it->value.c_str();
            }
        }

        action_client.waitForServer();
        rosplan_tiago_transportation_attendant::GetLoadGoal goal;

        // Fill in goal here
        goal.human = human_name;
        action_client.sendGoal(goal);
        action_client.waitForResult(ros::Duration(action_real_duration_s));
        if (action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {

        }

        // complete the action
        ROS_INFO("%s: GETLOAD Action completing", node_name_pretty.c_str());
        return true;
    }
} // close namespace

/*-------------*/
/* Main method */
/*-------------*/

int main(int argc, char **argv) {

    ros::init(argc, argv, "rosplan_get_load_action_client", ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");

    // create PDDL action subscriber
    KCL_rosplan::RPGetLoad rpti(nh);

    rpti.runActionInterface();

    return 0;
}
