#!/usr/bin/env python
import rospy
import mongodb_store.message_store

from komodo_blockworld.msg import BlockPos

from diagnostic_msgs.msg import KeyValue
from rosplan_dispatch_msgs.msg import ActionDispatch
from rosplan_dispatch_msgs.msg import ActionFeedback
from rosplan_knowledge_msgs.msg import KnowledgeItem
from rosplan_knowledge_msgs.srv import KnowledgeUpdateServiceRequest
from rosplan_knowledge_msgs.srv import KnowledgeUpdateServiceResponse
from rosplan_knowledge_msgs.srv import KnowledgeUpdateService


class KomodoArmComp:
    message_store = None
    action_client = None
    action_feedback_pub = None
    update_knowledge_client = None
    clear_costmaps_client = None
    dispatch_subscriber = None
    debug_level = 1

    # TODO: Add timeout checks after long actions like database queries or arm movements

    def __init__(self):
        fname = "{}::{}".format(self.__class__.__name__, self.__init__.__name__)

        log_level = rospy.INFO
        if (self.debug_level > 0):
            log_level = rospy.DEBUG
            
        rospy.init_node("KomodoArmComp", anonymous=False, log_level=log_level)

        self.message_store = mongodb_store.message_store.MessageStoreProxy()
        self.update_knowledge_client = rospy.ServiceProxy("/kcl_rosplan/update_knowledge_base", KnowledgeUpdateService)

        self.dispatch_subscriber = rospy.Subscriber("/kcl_rosplan/action_dispatch", ActionDispatch,
                                               self.dispatch_callback, queue_size=1000)

        rospy.logdebug("{}: Arm Component initialized".format(fname))

    def dispatch_callback(self, action_dispatch_msg):
        fname = "{}::{}".format(self.__class__.__name__, self.dispatch_callback.__name__)

        rospy.logdebug("{}: Received action {}".format(fname, action_dispatch_msg.name))

        if (action_dispatch_msg.name == "pick_up"):
            self.handle_pick_up(action_dispatch_msg)
        elif (action_dispatch_msg.name == "put_down"):
            self.handle_put_down(action_dispatch_msg)
        else:
            rospy.logdebug("{}: Ignoring message".format(fname))

    def handle_pick_up(self, pick_up_msg):
        fname = "{}::{}".format(self.__class__.__name__, self.handle_pick_up.__name__)

        rospy.loginfo("{}: Handling pick up action".format(fname))

        (block_name, from_block_name) = self.get_parameter_values(pick_up_msg.parameters, "block", "from_block")

        if (block_name is None or from_block_name is None):
            rospy.logerr("{}: Failed to find block parameters in pick up message".format(fname))
            return

        query_result = self.message_store.query_named(block_name, BlockPos)
        if (query_result is None):
            rospy.logfatal("{}: Failed to query DB. Aborting...".format(fname))

        else:
            if (len(query_result) != 1):
                rospy.logerr("{}: Found {} blocks named {}. Aborting...".format(fname, len(query_result), block_name))
                return

            target_block_pos = query_result[0]

            actionFeedback = ActionFeedback()
            actionFeedback.action_id = pick_up_msg.action_id
            actionFeedback.status = "action enabled"
            self.action_feedback_pub.publish(actionFeedback)

            # TODO: Move arm

            # TODO: Grab block

            # TODO: Rise arm

            self.apply_effects_to_KMS(block_name, from_block_name, pick_up_msg.name)

    def handle_put_down(self, put_down_msg):
        fname = "{}::{}".format(self.__class__.__name__, self.handle_put_down.__name__)

        rospy.loginfo("{}: Handling put down action".format(fname))

        (block_name, on_block_name) = self.get_parameter_values(put_down_msg.parameters, "block", "on_block")

        if (block_name is None or on_block_name is None):
            rospy.logerr("{}: Failed to find block parameters in put down message".format(fname))
            return

        query_result = self.message_store.query_named(on_block_name, BlockPos)
        if (query_result is None):
            rospy.logfatal("{}: Failed to query DB. Aborting...".format(fname))

        else:
            if (len(query_result) != 1):
                rospy.logerr("{}: Found {} blocks named {}. Aborting...".format(fname, len(query_result), block_name))
                return

            block_pos = query_result[0]

            actionFeedback = ActionFeedback()
            actionFeedback.action_id = put_down_msg.action_id
            actionFeedback.status = "action enabled"
            self.action_feedback_pub.publish(actionFeedback)

            # TODO: Move arm

            # TODO: Release block

            # TODO: Rise arm

            self.apply_effects_to_KMS(block_name, on_block_name, put_down_msg.name)

    def get_parameter_values(self, parameters, first_param, second_param):
        block_name = None
        from_block_name = None
        for param_idx in xrange(0, len(parameters)):
            if (parameters[param_idx].key == first_param):
                block_name = parameters[param_idx].value
            if (parameters[param_idx].key == second_param):
                from_block_name = parameters[param_idx].value
        return block_name, from_block_name

    def apply_effects_to_KMS(self, block_name, other_block_name, action):

        emptyhand_update_type = KnowledgeUpdateServiceRequest.REMOVE_KNOWLEDGE if \
                                (action == "pick_up") else KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE

        onblock_update_type = KnowledgeUpdateServiceRequest.REMOVE_KNOWLEDGE if \
                                (action == "pick_up") else KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE

        clear_update_type = KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE if \
                                (action == "pick_up") else KnowledgeUpdateServiceRequest.REMOVE_KNOWLEDGE

        inhand_update_type = KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE if \
                                (action == "pick_up") else KnowledgeUpdateServiceRequest.REMOVE_KNOWLEDGE

        # Remove emptyhand predicate
        update_knowledge_request = KnowledgeUpdateServiceRequest()
        update_knowledge_request.knowledge.knowledge_type = KnowledgeItem.FACT
        update_knowledge_request.update_type = emptyhand_update_type
        update_knowledge_request.knowledge.attribute_name = "emptyhand"
        self.update_knowledge_client.call(update_knowledge_request)

        # Remove (on ?block ?from_block) predicate
        update_knowledge_request = KnowledgeUpdateServiceRequest()
        update_knowledge_request.knowledge.knowledge_type = KnowledgeItem.FACT
        update_knowledge_request.update_type = onblock_update_type
        update_knowledge_request.knowledge.attribute_name = "on"
        pair = KeyValue()
        pair.key = "block"
        pair.value = block_name
        update_knowledge_request.knowledge.values.append(pair)
        pair = KeyValue()
        pair.key = "on_block"
        pair.value = other_block_name
        update_knowledge_request.knowledge.values.append(pair)
        self.update_knowledge_client.call(update_knowledge_request)

        # Add (clear ?from_block) predicate
        update_knowledge_request = KnowledgeUpdateServiceRequest()
        update_knowledge_request.knowledge.knowledge_type = KnowledgeItem.FACT
        update_knowledge_request.update_type = clear_update_type
        update_knowledge_request.knowledge.attribute_name = "clear"
        pair = KeyValue()
        pair.key = "?block"
        pair.value = other_block_name
        update_knowledge_request.knowledge.values.append(pair)
        self.update_knowledge_client.call(update_knowledge_request)

        # Add (inhand ?block) predicate
        update_knowledge_request = KnowledgeUpdateServiceRequest()
        update_knowledge_request.knowledge.knowledge_type = KnowledgeItem.FACT
        update_knowledge_request.update_type = inhand_update_type
        update_knowledge_request.knowledge.attribute_name = "inhand"
        pair = KeyValue()
        pair.key = "?block"
        pair.value = block_name
        update_knowledge_request.knowledge.values.append(pair)
        self.update_knowledge_client.call(update_knowledge_request)


if __name__ == '__main__':
    try:
        move_base = KomodoArmComp()

        rospy.spin()

    except rospy.ROSInterruptException, e:
        pass

