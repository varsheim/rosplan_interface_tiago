import rospy
from collections import OrderedDict

from rosplan_knowledge_msgs.srv import *
from rosplan_knowledge_msgs.msg import *


class TiagoKB:
    knowledge_types = ['KNOWLEDGE', 'GOAL', 'METRIC']
    knowledge_types_dict = {
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

    def __add_remove_item(self, attribute, values, should_add, is_negation, knowledge_type):
        if knowledge_type not in self.knowledge_types:
            print "TIAGO_KB: Wrong knowledge type provided"
            return -1
        k_type_add = "ADD_" + knowledge_type
        k_type_remove = "REMOVE_" + knowledge_type

        print "TIAGO_KB: Waiting for service " + self.update_array_service_name
        rospy.wait_for_service(self.update_array_service_name)

        print "TIAGO_KB: Preparing the message"
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
            temp_item_update_type.append(self.knowledge_types_dict[k_type_add if should_add else k_type_remove])

        # print items.knowledge
        items.update_type = temp_item_update_type

        try:
            print "TIAGO_KB: Calling Service"
            update_proxy = rospy.ServiceProxy(self.update_array_service_name, KnowledgeUpdateServiceArray)
            update_response = update_proxy(items)
            print "TIAGO_KB: Response is: ", update_response.success
        except rospy.ServiceException, e:
            print "TIAGO_KB: Service call failed: %s"%e

    def add_remove_knowledge(self, attribute, values, should_add=True, is_negation=False):
        knowledge_type = 'KNOWLEDGE'
        self.__add_remove_item(attribute, values, should_add, is_negation, knowledge_type)

    def add_remove_goals(self, attribute, values, should_add=True, is_negation=False):
        knowledge_type = 'GOAL'
        self.__add_remove_item(attribute, values, should_add, is_negation, knowledge_type)