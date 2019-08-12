import rospy

from rosplan_dispatch_msgs.srv import *
from std_srvs.srv import *


class ROSPlanControl:
    srv_class_request_dict = {
        Empty: EmptyRequest(),
        DispatchService: DispatchServiceRequest()
    }

    srv_class_response_dict = {
        Empty: EmptyResponse(),
        DispatchService: DispatchServiceResponse()
    }

    def __init__(self,
                 problem_generation_service_name = 'rosplan_problem_interface/problem_generation_server',
                 planning_service_name = 'rosplan_planner_interface/planning_server',
                 parse_plan_service_name = 'rosplan_parsing_interface/parse_plan',
                 dispatch_service_name = 'rosplan_plan_dispatcher/dispatch_plan',
                 stop_dispatch_service_name = 'rosplan_plan_dispatcher/cancel_dispatch'):

        self.problem_generation_service_name = problem_generation_service_name
        self.planning_service_name = planning_service_name
        self.parse_plan_service_name = parse_plan_service_name
        self.dispatch_service_name = dispatch_service_name
        self.stop_dispatch_service_name = stop_dispatch_service_name

    def __call_service(self, srv_name, srv_class, delay_after_call_ms):
        class_name = self.__class__.__name__
        rospy.loginfo('{}: Waiting for service: {}'.format(class_name, srv_name))
        rospy.wait_for_service(srv_name)

        rospy.loginfo('{}: Preparing the message'.format(class_name))
        request = self.srv_class_request_dict[srv_class]

        try:
            rospy.loginfo('{}: Calling Planning Service'.format(class_name))
            proxy = rospy.ServiceProxy(srv_name, srv_class)
            update_response = proxy(request)
            rospy.loginfo('{}: Response is: '.format(class_name, update_response))
        except rospy.ServiceException, e:
            rospy.loginfo('{}: Service call failed: {}'.format(class_name, e))

        rospy.sleep(rospy.Duration(nsecs=delay_after_call_ms * 1000))

    def generate_problem(self):
        delay_after_ms = 1000
        self.__call_service(self.problem_generation_service_name, Empty, delay_after_ms)

    def planning(self):
        delay_after_ms = 1000
        self.__call_service(self.planning_service_name, Empty, delay_after_ms)

    def parse_plan(self):
        delay_after_ms = 1000
        self.__call_service(self.parse_plan_service_name, Empty, delay_after_ms)

    def dispatch_plan(self):
        delay_after_ms = 1000
        self.__call_service(self.dispatch_service_name, DispatchService, delay_after_ms)

    def stop_dispatch_plan(self):
        delay_after_ms = 5000  # maybe it can be less?
        self.__call_service(self.stop_dispatch_service_name, Empty, delay_after_ms)

    def replan_immediately_routine(self):
        self.stop_dispatch_plan()
        self.generate_problem()
        self.planning()
        self.parse_plan()
        self.dispatch_plan()
