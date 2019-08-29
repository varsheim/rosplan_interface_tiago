import rospy

from rosplan_dispatch_msgs.srv import *
from std_srvs.srv import *
from rosplan_tiago_common.msg import ROSPlanService


class ROSPlanSysControlClient:
    srv_class_string_dict = {
        Empty: 'Empty',
        DispatchService: 'DispatchService'
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

        # initialize publisher for setting services
        self.pub = rospy.Publisher("rosplan_sys_control/services", ROSPlanService, queue_size=10)
        while self.pub.get_num_connections() == 0:
            pass

    def __call_service(self, srv_name, srv_class, delay_after_call_ms):
        new_msg = ROSPlanService()
        new_msg.service_name = srv_name
        new_msg.service_class = self.srv_class_string_dict[srv_class]
        node_name = "(" + rospy.get_name() + ")"

        rospy.loginfo("{} Publishing message to call service ({}).".format(node_name, srv_name))
        self.pub.publish(new_msg)

        rospy.loginfo("{} Sleeping for {}ms.".format(node_name, delay_after_call_ms))
        rospy.sleep(rospy.Duration(nsecs=delay_after_call_ms * 1000000))

    def generate_problem(self):
        delay_after_ms = 200
        self.__call_service(self.problem_generation_service_name, Empty, delay_after_ms)

    def planning(self):
        delay_after_ms = 200
        self.__call_service(self.planning_service_name, Empty, delay_after_ms)

    def parse_plan(self):
        delay_after_ms = 200
        self.__call_service(self.parse_plan_service_name, Empty, delay_after_ms)

    def dispatch_plan(self):
        delay_after_ms = 200
        self.__call_service(self.dispatch_service_name, DispatchService, delay_after_ms)

    def stop_dispatch_plan(self):
        delay_after_ms = 1000  # maybe it can be less?
        self.__call_service(self.stop_dispatch_service_name, Empty, delay_after_ms)

    def replan_immediately_routine(self):
        self.stop_dispatch_plan()
        self.generate_problem()
        self.planning()
        self.parse_plan()
        self.dispatch_plan()
