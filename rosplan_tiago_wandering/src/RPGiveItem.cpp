#include "RPGiveItem.h"

/* The implementation of RPTutorial.h */
namespace KCL_rosplan {

	/* constructor */
	RPGiveItem::RPGiveItem(ros::NodeHandle &nh) {
//		// Create service client for getting location params
//		service_client = nh.serviceClient<rosplan_tiago_params::GetLocation>("/location_name_service");
	}

	/* action dispatch callback */
	bool RPGiveItem::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		// The action implementation goes here.
		// Get action parameters
		auto action_parameters = msg.get()->parameters;

		for (auto it = begin (action_parameters); it != end (action_parameters); ++it) {
			if (strcmp(it->key.c_str(), "item") == 0) {
				action_param_item = it->value.c_str();
			}
		}

//		rosplan_tiago_params::GetLocation srv;
//		srv.request.location = current_destination;

//		if (service_client.call(srv)) {
//			ROS_INFO("Got matching location params of %s.", current_destination.c_str());
//		}
//		else {
//			ROS_ERROR("Failed to call service location_name_service");
//			return false;
//		}

		action_client.waitForServer();
		rosplan_tiago_wandering::GiveItemGoal goal;

		// Fill in goal here
		goal.item = action_param_item;
		action_client.sendGoal(goal);
		action_client.waitForResult(ros::Duration(30.0));


		if (action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
			// complete the action
			ROS_INFO("KCL: (%s) GIVE_ITEM Action completing.", msg->name.c_str());

			return true;
		}
		else {
			ROS_INFO("CLIENT: GIVE_ITEM: Current State: %s\n", action_client.getState().toString().c_str());

			return false;
		}
	}
} // close namespace

/*-------------*/
/* Main method */
/*-------------*/

int main(int argc, char **argv) {
	ros::init(argc, argv, "rosplan_give_item_action_client", ros::init_options::AnonymousName);
	ros::NodeHandle nh("~");

	// create PDDL action subscriber
	KCL_rosplan::RPGiveItem rpti(nh);

	rpti.runActionInterface();

	return 0;
}
