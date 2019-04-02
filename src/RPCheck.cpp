//
// Created by robot on 3/29/19.
//

#include "RPCheck.h"



/* The implementation of RPTutorial.h */
namespace KCL_rosplan {

    /* constructor */
    RPCheck::RPCheck(ros::NodeHandle &nh) {
        // perform setup
    }

    /* action dispatch callback */
    bool RPCheck::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

        // The action implementation goes here.

//        for (uint8_t i = 0; i < 10; i++) {
//            ROS_INFO("KCL: (%d) % TUTORIAL Action in progress.", (i * 10));
//            boost::this_thread::sleep(boost::posix_time::seconds(1));
//        }

        Client client("check", true); // true -> don't need ros::spin()
        client.waitForServer();
        rosplan_interface_tiago::CheckGoal goal;

        // Fill in goal here
        goal.blind_goal = 500;

        client.sendGoal(goal);
        client.waitForResult(ros::Duration(5.0));
        if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Yay! Checked the place\n");
        ROS_INFO("Current State: %s\n", client.getState().toString().c_str());

        // complete the action
        ROS_INFO("KCL: (%s) CHECK Action completing.\n", msg->name.c_str());
        return true;


    }
} // close namespace

/*-------------*/
/* Main method */
/*-------------*/

int main(int argc, char **argv) {

    ros::init(argc, argv, "rosplan_check_action_client", ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");

    // create action client here?

    // create PDDL action subscriber
    KCL_rosplan::RPCheck rpti(nh);

    rpti.runActionInterface();

    return 0;
}