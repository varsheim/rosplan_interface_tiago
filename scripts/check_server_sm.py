#!/usr/bin/env python
import roslib; roslib.load_manifest('rosplan_interface_tiago')
import rospy
import smach
import smach_ros

from rosplan_interface_tiago.msg import CheckAction
from rosplan_interface_tiago.msg import CheckActionGoal
from rosplan_interface_tiago.msg import CheckActionFeedback
from rosplan_interface_tiago.msg import CheckActionResult

# define state Foo
class Foo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'],
                             input_keys=['goal'],
                             output_keys=['feedback',
                                          'result'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state FOO')

        if self.counter < 3:
            self.counter += 1
            # print "{}".format(userdata.goal)
            return 'outcome1'
        else:
            feedback = CheckActionFeedback()
            feedback.feedback.percent_complete = 5
            userdata.feedback = feedback

            action_result = CheckActionResult()
            action_result.result.is_undocked = 5
            # userdata.result = action_result

            return 'outcome2'


# define state Bar
class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome2'],
                             input_keys=['goal'],
                             output_keys=['feedback',
                                          'result'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
        return 'outcome2'


class CheckServer(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,
                                    outcomes=['OVERALL_SUCCESS',
                                              'OVERALL_FAILED',
                                              'PREEMPTED'],
                                    input_keys=['goal'],
                                    output_keys=['feedback',
                                                 'result'])

        with self:
            # Add the actionlib states
            smach.StateMachine.add('Foo',
                                   Foo(),
                                   transitions={'outcome1' : 'Bar',
                                                'outcome2' : 'OVERALL_SUCCESS'})
            smach.StateMachine.add('Bar',
                                   Bar(),
                                   transitions={'outcome2' : 'Foo'})


def main():
    rospy.init_node('check_server')

    # Construct state machine
    sm = CheckServer()

    # # Smach viewer
    # sis = smach_ros.IntrospectionServer('check_server_smach_viewer', sm, '/CHECK_SERVER_SMACH_VIEWER')
    # sis.start()

    # Construct action server wrapper
    asw = smach_ros.ActionServerWrapper(
        server_name='check',
        action_spec=CheckAction,
        wrapped_container=sm,
        succeeded_outcomes=['OVERALL_SUCCESS'],
        aborted_outcomes=['OVERALL_FAILED'],
        preempted_outcomes=['PREEMPTED'],
        goal_key='goal',
        feedback_key='feedback',
        result_key='result')

    # Run the server in a background thread
    asw.run_server()
    rospy.spin()


if __name__ == '__main__':
    main()
