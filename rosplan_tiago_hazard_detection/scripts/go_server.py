#!/usr/bin/env python
import rospy
import smach
import smach_ros
import actionlib

from geometry_msgs.msg import Pose
from rosplan_tiago_hazard_detection.msg import GoAction
from rosplan_tiago_hazard_detection.msg import GoActionGoal
from rosplan_tiago_hazard_detection.msg import GoActionFeedback
from rosplan_tiago_hazard_detection.msg import GoActionResult

from move_base_msgs.msg import *


class Initialize(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['ok', 'preemption', 'error'],
                             input_keys=['init_goal'],
                             output_keys=['init_feedback',
                                          'init_goal_pose',
                                          'init_params'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: {}'.format(self.__class__.__name__))

        # take goal, check values, split into pose and params, check if params are okay
        # now the server receives only pose
        pose = userdata.init_goal.pose

        userdata.init_goal_pose = pose

        rospy.sleep(1)
        return 'ok'


class Navigate(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['ok', 'preemption', 'error'],
                             input_keys=['nav_goal_pose',
                                         'nav_params'],
                             output_keys=['nav_feedback',
                                          'nav_actual_pose'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: {}'.format(self.__class__.__name__))

        # robot movement here - using Tiago move_base
        pose = userdata.nav_goal_pose

        goal = MoveBaseGoal()

        goal.target_pose.pose = pose
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()

        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()


        # start moving
        client.send_goal(goal)

        print pose

        rospy.sleep(20)
        return 'ok'


class Finalize(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['ok', 'preemption', 'error'],
                             input_keys=['final_actual_pose'],
                             output_keys=['final_feedback',
                                          'final_result'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: {}'.format(self.__class__.__name__))

        # prepare some feedback & result
        action_feedback = GoActionFeedback()
        action_feedback.feedback.percent_complete = 100
        userdata.final_feedback = action_feedback.feedback

        action_result = GoActionResult()
        action_result.result.is_undocked = 1
        userdata.final_result = action_result.result

        # calc pose initial distance

        # calc current distance in a loop

        # robot approach check may be put here, some distances calculated
        rospy.sleep(2)
        return 'ok'


class GoServer(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,
                                    outcomes=['PREEMPTED',
                                              'FAILED',
                                              'APPROACHED'],
                                    input_keys=['sm_goal'],
                                    output_keys=['sm_feedback',
                                                 'sm_result'])

        # Fill the machine here
        with self:
            smach.StateMachine.add('Initialize',
                                   Initialize(),
                                   transitions={'ok': 'Navigate',
                                                'preemption': 'PREEMPTED',
                                                'error': 'FAILED'},
                                   remapping={'init_goal': 'sm_goal',
                                              'init_feedback': 'sm_feedback',
                                              'init_goal_pose': 'sm_goal_pose',
                                              'init_params': 'sm_params'})

            smach.StateMachine.add('Navigate',
                                   Navigate(),
                                   transitions={'ok': 'Finalize',
                                                'preemption': 'PREEMPTED',
                                                'error': 'FAILED'},
                                   remapping={'nav_goal_pose': 'sm_goal_pose',
                                              'nav_params': 'sm_params',
                                              'nav_feedback': 'sm_feedback',
                                              'nav_actual_pose': 'sm_actual_pose'})

            smach.StateMachine.add('Finalize',
                                   Finalize(),
                                   transitions={'preemption': 'PREEMPTED',
                                                'error': 'FAILED',
                                                'ok': 'APPROACHED'},
                                   remapping={'final_actual_pose': 'sm_actual_pose',
                                              'final_feedback': 'sm_feedback',
                                              'final_result': 'sm_result'})


def main():
    rospy.init_node('go_server')

    # Construct state machine
    sm = GoServer()

    # # Smach viewer
    sis = smach_ros.IntrospectionServer('go_server', sm, '/SM_GO_SERVER')
    sis.start()

    # Construct action server wrapper
    asw = smach_ros.ActionServerWrapper(
        server_name='go',
        action_spec=GoAction,
        wrapped_container=sm,
        succeeded_outcomes=['APPROACHED'],
        aborted_outcomes=['FAILED'],
        preempted_outcomes=['PREEMPTED'],
        goal_key='sm_goal',
        feedback_key='sm_feedback',
        result_key='sm_result')

    # Run the server in a background thread
    asw.run_server()
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
