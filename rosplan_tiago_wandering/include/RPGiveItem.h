#include <ros/ros.h>
#include <vector>
#include <boost/thread/thread.hpp>

#include "rosplan_action_interface/RPActionInterface.h"
#include <actionlib/client/simple_action_client.h>
#include <rosplan_tiago_scenarios_msgs/GiveItemAction.h>
#include <geometry_msgs/Pose.h>
#include <rosplan_tiago_core_msgs/GetLocation.h>

#ifndef ROSPLAN_TIAGO_WANDERING_RPGIVEITEM_H
#define ROSPLAN_TIAGO_WANDERING_RPGIVEITEM_H

/**
 * This file defines an action interface created in tutorial 10.
 */

typedef actionlib::SimpleActionClient<rosplan_tiago_scenarios_msgs::GiveItemAction> Client;

namespace KCL_rosplan {

	class RPGiveItem: public RPActionInterface
	{

	private:
		Client action_client{"give_item", true};

		ros::ServiceClient service_client;
		std::string action_param_item;
	public:

		/* constructor */
		RPGiveItem(ros::NodeHandle &nh);

		/* listen to and process action_dispatch topic */
		bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	};
}

#endif //ROSPLAN_TIAGO_WANDERING_RPGIVEITEM_H
