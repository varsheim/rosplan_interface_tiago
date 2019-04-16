#include "RPGo.h"



/* The implementation of RPTutorial.h */
namespace KCL_rosplan {

    /* constructor */
    RPGo::RPGo(ros::NodeHandle &nh) {
        // perform setup
    }

    /* action dispatch callback */
    bool RPGo::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

        // The action implementation goes here.

        //Client client("go", true); // true -> don't need ros::spin()
        ROS_INFO("CLIENT: GO: Waiting for sever");
        client.waitForServer();
        rosplan_tiago_hazard_detection::GoGoal goal;

        // Fill in goal here
        goal.blind_goal = 500;

        ROS_INFO("CLIENT: GO: I will send goal now");
        client.sendGoal(goal);

        ROS_INFO("CLIENT: GO: I will wait for result now");
        client.waitForResult(ros::Duration(10.0));

        ROS_INFO("CLIENT: GO: I received result and it is:");
        if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            //ROS_INFO("CLIENT: GO ");
        }

        ROS_INFO("CLIENT: GO: Current State: %s\n", client.getState().toString().c_str());

        // complete the action
        ROS_INFO("KCL: (%s) GO Action completing.", msg->name.c_str());
        return true;


    }
} // close namespace

/*-------------*/
/* Main method */
/*-------------*/

int main(int argc, char **argv) {

    ros::init(argc, argv, "rosplan_go_action_client", ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");

    // create action client here?

    // create PDDL action subscriber
    KCL_rosplan::RPGo rpti(nh);

    rpti.runActionInterface();

    return 0;
}
