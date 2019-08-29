import rospy
from collections import OrderedDict

from rosplan_knowledge_msgs.srv import *
from rosplan_knowledge_msgs.msg import *


class TiagoKBUpdate:
    knowledge_update_types = ['KNOWLEDGE', 'GOAL', 'METRIC']
    knowledge_update_types_dict = {
        'ADD_KNOWLEDGE': KnowledgeUpdateServiceArrayRequest.ADD_KNOWLEDGE,
        'REMOVE_KNOWLEDGE': KnowledgeUpdateServiceArrayRequest.REMOVE_KNOWLEDGE,
        'ADD_GOAL': KnowledgeUpdateServiceArrayRequest.ADD_GOAL,
        'REMOVE_GOAL': KnowledgeUpdateServiceArrayRequest.REMOVE_GOAL,
        'ADD_METRIC': KnowledgeUpdateServiceArrayRequest.ADD_METRIC,
        'REMOVE_METRIC': KnowledgeUpdateServiceArrayRequest.REMOVE_METRIC
    }

    def __init__(self,
                 update_service_name = 'rosplan_knowledge_base/update',
                 update_array_service_name = 'rosplan_knowledge_base/update_array'):

        self.update_service_name = update_service_name
        self.update_array_service_name = update_array_service_name

    def __add_remove_item(self, attribute, values, should_add, is_negation, knowledge_update_type):
        if knowledge_update_type not in self.knowledge_update_types:
            print "{}: Wrong knowledge type provided".format(self.__class__.__name__)
            return -1
        k_type_add = "ADD_" + knowledge_update_type
        k_type_remove = "REMOVE_" + knowledge_update_type

        print "{}: Waiting for service ".format(self.__class__.__name__) + self.update_array_service_name
        rospy.wait_for_service(self.update_array_service_name)

        print "{}: Preparing the message".format(self.__class__.__name__)
        items = KnowledgeUpdateServiceArrayRequest()

        temp_item_update_type = []
        for i in xrange(max(1, len(values))):
            # set new knowledge item values before appending
            temp_item_knowledge = KnowledgeUpdateServiceRequest().knowledge
            temp_item_knowledge.knowledge_type = KnowledgeItem.FACT
            temp_item_knowledge.attribute_name = attribute
            temp_item_knowledge.is_negative = is_negation
            if len(values) > 0:
                value = values[i]
                for key, value in value.iteritems():
                    temp_item_knowledge.values.append(diagnostic_msgs.msg.KeyValue(key, value))

            items.knowledge.append(temp_item_knowledge)
            temp_item_update_type.append(self.knowledge_update_types_dict[k_type_add if should_add else k_type_remove])

        # print items.knowledge
        items.update_type = temp_item_update_type

        try:
            print "{}: Calling Service".format(self.__class__.__name__)
            update_proxy = rospy.ServiceProxy(self.update_array_service_name, KnowledgeUpdateServiceArray)
            update_response = update_proxy(items)
            print "{}: Response is: {}".format(self.__class__.__name__, update_response.success)
        except rospy.ServiceException, e:
            print "{}: Service call failed: {}".format(self.__class__.__name__, e)

    def add_remove_knowledge(self, attribute, values, should_add=True, is_negation=False):
        knowledge_update_type = 'KNOWLEDGE'
        self.__add_remove_item(attribute, values, should_add, is_negation, knowledge_update_type)

    def add_remove_goals(self, attribute, values, should_add=True, is_negation=False):
        knowledge_update_type = 'GOAL'
        self.__add_remove_item(attribute, values, should_add, is_negation, knowledge_update_type)


class TiagoKBQuery:
    def __init__(self,
                 query_state_service_name='rosplan_knowledge_base/query_state',
                 is_silent=False):

        self.query_state_service_name = query_state_service_name
        self.is_silent = is_silent

    def __query_state(self, query):
        try:
            if not self.is_silent:
                print "{}: Calling Service".format(self.__class__.__name__)
            query_proxy = rospy.ServiceProxy(self.query_state_service_name, KnowledgeQueryService)
            query_response = query_proxy(query)
            if not self.is_silent:
                print "{}: Response is: {}".format(self.__class__.__name__, query_response.results)
            return query_response.results
        except rospy.ServiceException, e:
            if not self.is_silent:
                print "{}: Service call failed: {}".format(self.__class__.__name__, e)

    def query_instance(self):
        query = []

        # this must be filled
        # knowledge_type
        # string instance_type
        # string instance_name

        knowledge_type = KnowledgeItem.INSTANCE
        return self.__query_state()

    def query_fact(self, attribute, values, is_negation=False):
        '''
        Query KB for FACTs

        :param str attribute: KnowledgeItem attribute
        :param OrderedDict[] values: Elements matching information of attribute in pddl domain
        :param bool is_negation: Fact is negated or not
        :return: FACTs presence in Knowledge Base
        :rtype: bool[]
        '''

        query = []

        # this must be filled
        # knowledge_type
        # string attribute_name
        # diagnostic_msgs/KeyValue[] values

        for i in xrange(max(1, len(values))):
            item = KnowledgeItem()
            item.knowledge_type = KnowledgeItem.FACT
            item.attribute_name = attribute
            item.is_negative = is_negation
            if len(values) > 0:
                value = values[i]
                for key, value in value.iteritems():
                    item.values.append(diagnostic_msgs.msg.KeyValue(key, value))
            query.append(item)

        return self.__query_state(query)

    def query_function(self):
        query = []

        # this must be filled
        # knowledge_type
        # function_value

        knowledge_type = KnowledgeItem.FUNCTION
        return self.__query_state()

    def query_expression(self):
        query = []

        # this must be filled
        # knowledge_type
        # optimization
        # expr

        knowledge_type = KnowledgeItem.EXPRESSION
        return self.__query_state()

    def query_expression(self):
        query = []

        # this must be filled
        # knowledge_type
        # ineq

        knowledge_type = KnowledgeItem.INEQUALITY
        return self.__query_state()