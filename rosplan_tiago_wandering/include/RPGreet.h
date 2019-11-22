#include <ros/ros.h>
#include <vector>
#include <boost/thread/thread.hpp>

#include "rosplan_action_interface/RPActionInterface.h"
#include <actionlib/client/simple_action_client.h>
#include <rosplan_tiago_scenarios_msgs/GreetAction.h>
#include <geometry_msgs/Pose.h>
#include <rosplan_tiago_core_msgs/GetLocation.h>

#ifndef ROSPLAN_TIAGO_WANDERING_RPGREET_H
#define ROSPLAN_TIAGO_WANDERING_RPGREET_H

/**
 * This file defines an action interface created in tutorial 10.
 */

#define ACTION_ADDITION_TIME_S 2

typedef actionlib::SimpleActionClient<rosplan_tiago_scenarios_msgs::GreetAction> Client;

namespace KCL_rosplan {

	class RPGreet: public RPActionInterface
	{

	private:
		Client action_client{"greet", true};

		ros::ServiceClient service_client;
		std::string action_param_greet_scenario;
	public:

		/* constructor */
		RPGreet(ros::NodeHandle &nh);

		/* listen to and process action_dispatch topic */
		bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	};
}

#endif //ROSPLAN_TIAGO_WANDERING_RPGREET_H
