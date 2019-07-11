#include <ros/ros.h>
#include <vector>
#include <boost/thread/thread.hpp>

#include "rosplan_action_interface/RPActionInterface.h"
#include <actionlib/client/simple_action_client.h>
#include <rosplan_tiago_wandering/GetItemAction.h>
#include <geometry_msgs/Pose.h>
#include <rosplan_tiago_params/GetLocation.h>

#ifndef ROSPLAN_TIAGO_WANDERING_RPGETITEM_H
#define ROSPLAN_TIAGO_WANDERING_RPGETITEM_H

/**
 * This file defines an action interface created in tutorial 10.
 */

typedef actionlib::SimpleActionClient<rosplan_tiago_wandering::GetItemAction> Client;

namespace KCL_rosplan {

	class RPGetItem: public RPActionInterface
	{

	private:
		Client action_client{"get_item", true};

		ros::ServiceClient service_client;
		std::string action_param_item;
	public:

		/* constructor */
		RPGetItem(ros::NodeHandle &nh);

		/* listen to and process action_dispatch topic */
		bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	};
}

#endif //ROSPLAN_TIAGO_WANDERING_RPGETITEM_H
