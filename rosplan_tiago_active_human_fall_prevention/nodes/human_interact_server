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

# states from http://docs.ros.org/diamondback/api/actionlib_msgs/html/msg/GoalStatus.html
PENDING = 0  # The goal has yet to be processed by the action server
ACTIVE = 1  # The goal is currently being processed by the action server
PREEMPTED = 2  # The goal received a cancel request after it started executing
# and has since completed its execution (Terminal State)
SUCCEEDED = 3  # The goal was achieved successfully by the action server (Terminal State)
ABORTED = 4  # The goal was aborted during execution by the action server due
#    to some failure (Terminal State)
REJECTED = 5  # The goal was rejected by the action server without being processed,
#    because the goal was unattainable or invalid (Terminal State)
PREEMPTING = 6  # The goal received a cancel request after it started executing
#    and has not yet completed execution
RECALLING = 7  # The goal received a cancel request before it started executing,
#    but the action server has not yet confirmed that the goal is canceled
RECALLED = 8  # The goal received a cancel request before it started executing
#    and was successfully cancelled (Terminal State)
LOST = 9  # An action client can determine that a goal is LOST. This should not be
#    sent over the wire by an action server


NAVIGATION_MAX_TIME_S = 30

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
        self.current_pose = Pose()
        self.is_feedback_received = False
        self.move_base_status = PENDING
        self.is_goal_achieved = False

        smach.State.__init__(self,
                             outcomes=['ok', 'preemption', 'error'],
                             input_keys=['nav_goal_pose',
                                         'nav_params'],
                             output_keys=['nav_feedback',
                                          'nav_actual_pose',
                                          'nav_result'])

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
        client.send_goal(goal, self.move_base_done_cb, self.move_base_active_cb, self.move_base_feedback_cb)

        action_feedback = GoActionFeedback()
        action_result = GoActionResult()
        action_result.result.is_goal_accomplished = False
        userdata.nav_result = action_result.result

        start_time = rospy.Time.now()
        print 'IS GOAL ACHIEVED: {}'.format(self.is_goal_achieved)

        self.is_goal_achieved = False
        while self.is_goal_achieved == False:
            action_feedback.feedback.current_pose = self.current_pose

            userdata.nav_feedback = action_feedback.feedback
            userdata.nav_actual_pose = self.current_pose

            end_time = rospy.Time.now()
            loop_time = end_time - start_time
            loop_time_s = loop_time.secs

            if loop_time_s > NAVIGATION_MAX_TIME_S:
                # break the loop, end with error state
                rospy.logwarn('State: Navigation took too much time, returning error')
                return 'error'

            rospy.sleep(0.05)

        # Here check move_base DONE status
        if self.move_base_status == SUCCEEDED:
            return 'ok'

        if (self.move_base_status == PREEMPTED) or (
                self.move_base_status == PREEMPTING) or (
                self.move_base_status == RECALLING) or (
                self.move_base_status == RECALLED):
            return 'preemption'

        if (self.move_base_status == ABORTED) or (
                self.move_base_status == REJECTED) or (
                self.move_base_status == LOST) or (
                self.move_base_status == PENDING) or (
                self.move_base_status == ACTIVE):
            return 'error'

        # should not enter here
        return 'error'

    def move_base_feedback_cb(self, feedback):
        self.current_pose = feedback.base_position.pose
        self.is_feedback_received = True

    def move_base_done_cb(self, status, result):
        self.is_goal_achieved = True
        self.move_base_status = status

    def move_base_active_cb(self):
        return


class Finalize(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['ok', 'preemption', 'error'],
                             input_keys=['final_actual_pose'],
                             output_keys=['final_result'])


    def execute(self, userdata):
        rospy.loginfo('Executing state: {}'.format(self.__class__.__name__))

        # prepare some feedback & result
        action_result = GoActionResult()
        action_result.result.is_goal_accomplished = True

        userdata.final_result = action_result.result

        rospy.sleep(1)
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
                                              'nav_actual_pose': 'sm_actual_pose',
                                              'nav_result': 'sm_result'})

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
