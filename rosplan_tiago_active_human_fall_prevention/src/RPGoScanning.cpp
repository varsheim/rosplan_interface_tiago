#include "RPGoScanning.h"



/* The implementation of RPTutorial.h */
namespace KCL_rosplan {

    /* constructor */
    RPGoScanning::RPGoScanning(ros::NodeHandle &nh) {
	    service_client = nh.serviceClient<rosplan_tiago_core_msgs::GetLocation>("/location_name_service");
        node_name = ros::this_node::getName();
        node_name_pretty = '(' + node_name + ')';
    }

    /* action dispatch callback */
    bool RPGoScanning::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
	    // The action implementation goes here.
	    // Get action parameters
	    auto action_parameters = msg.get()->parameters;
	    auto action_duration_s = msg.get()->duration;
	    auto action_real_duration_s = action_duration_s + ACTION_ADDITION_TIME_S;

	    for (auto it = begin (action_parameters); it != end (action_parameters); ++it) {
		    // "destination" is defined in pddl domain as param name
		    if (strcmp(it->key.c_str(), "destination") == 0) {
			    current_destination = it->value.c_str();
		    }
	    }

	    // Get the actual values by calling the service
	    // Fill the srv message first
	    rosplan_tiago_core_msgs::GetLocation srv;
	    srv.request.location = current_destination;

	    if (service_client.call(srv)) {
		    ROS_INFO("%s: Got location params of %s", node_name_pretty.c_str(), current_destination.c_str());
	    }
	    else {
		    ROS_ERROR("%s: Failed to call service location_name_service", node_name_pretty.c_str());
		    return false;
	    }

        action_client.waitForServer();
	    rosplan_tiago_scenarios_msgs::GoScanningGoal goal;

        // Fill in goal here
        goal.pose = srv.response.pose;
        action_client.sendGoal(goal);
        action_client.waitForResult(ros::Duration(action_real_duration_s));

        if (action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
			// here should be return true;
			// else should be return false; - action preemted or failed
        }

        // complete the action
        ROS_INFO("%s: GOSCANNING Action completing", node_name_pretty.c_str());
        return true;
    }
} // close namespace

/*-------------*/
/* Main method */
/*-------------*/

int main(int argc, char **argv) {

    ros::init(argc, argv, "rosplan_go_scanning_action_client", ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");

    // Create service client here
//    ros::ServiceClient client = nh.serviceClient<>("location_srv")


    // create PDDL action subscriber
    KCL_rosplan::RPGoScanning rpti(nh);

    rpti.runActionInterface();

    return 0;
}
