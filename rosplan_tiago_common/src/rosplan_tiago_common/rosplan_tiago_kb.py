import rospy

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

    def add_remove_item(self, attribute, keys, list_key_value, add, negation, k_type):
        if k_type in self.knowledge_types:
            k_type_add = "ADD_{}".format(k_type)
            k_type_remove = "REMOVE_{}".format(k_type)
        else:
            print "Wrong knowledge type provided"
            return

        print "Waiting for service {}".format(self.update_array_service_name)
        rospy.wait_for_service(self.update_array_service_name)

        print "Preparing the message"
        items = KnowledgeUpdateServiceArrayRequest()
        temp_item_update_type = []

        temp_item_knowledge = KnowledgeUpdateServiceRequest().knowledge
        temp_item_knowledge.knowledge_type = KnowledgeItem.FACT
        temp_item_knowledge.attribute_name = attribute
        temp_item_knowledge.is_negative = negation

        # handle attributes without keys
        if len(list_key_value) > 0:
            for dict in list_key_value:

                # clear knowledge item values before appending
                temp_item_knowledge.values = []
                for key in keys:
                    temp_item_knowledge.values.append(diagnostic_msgs.msg.KeyValue(key, dict[key]))

                items.knowledge.append(temp_item_knowledge)
                print items.knowledge
                print "\n\n\n"
                items.update_type = temp_item_update_type

        else:
            if add == True:
                temp_item_update_type.append(self.knowledge_types_dict[k_type_add])
            else:
                temp_item_update_type.append(self.knowledge_types_dict[k_type_remove])

            items.knowledge.append(temp_item_knowledge)
            items.update_type = temp_item_update_type

        try:
            print items
            print "Calling Service"
            update_proxy = rospy.ServiceProxy(self.update_array_service_name, KnowledgeUpdateServiceArray)
            update_response = update_proxy(items)
            print "Response is: ", update_response.success
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def add_remove_knowledge(self, attribute, keys, list_key_value, add=True, negation=False):
        knowledge_type = 'KNOWLEDGE'
        self.add_remove_item(attribute, keys, list_key_value, add, negation, knowledge_type)

    def add_remove_goals(self, attribute, keys, list_key_value, add=True, negation=False):
        knowledge_type = 'GOAL'
        self.add_remove_item(attribute, keys, list_key_value, add, negation, knowledge_type)