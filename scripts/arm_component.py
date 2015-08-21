#!/usr/bin/env python
import rospy
import mongodb_store.message_store
import math
import sys

from komodo_blockworld.msg import BlockPos
from std_msgs.msg import Float64
from diagnostic_msgs.msg import KeyValue
from rosplan_dispatch_msgs.msg import ActionDispatch
from rosplan_dispatch_msgs.msg import ActionFeedback
from rosplan_knowledge_msgs.msg import KnowledgeItem

from rosplan_knowledge_msgs.srv import GetInstanceServiceRequest
from rosplan_knowledge_msgs.srv import GetInstanceServiceResponse
from rosplan_knowledge_msgs.srv import GetInstanceService

from rosplan_knowledge_msgs.srv import GetAttributeServiceRequest
from rosplan_knowledge_msgs.srv import GetAttributeServiceResponse
from rosplan_knowledge_msgs.srv import GetAttributeService

from rosplan_knowledge_msgs.srv import KnowledgeUpdateServiceRequest
from rosplan_knowledge_msgs.srv import KnowledgeUpdateServiceResponse
from rosplan_knowledge_msgs.srv import KnowledgeUpdateService

# Distances in cm, angles in rad
default_values = {
    "base_rotation_center_offset": 0.1,
    "raised_shoulder_angle": 1.0,
    "max_base_angle": 0.45,
    "min_base_angle": -0.45,
    "shoulder_max_angle": math.pi / 2,
    "elbow_min_angle": math.pi / 2,
    "wrist_max_angle": math.pi / 2,
    "block_height": 4.0,
    "arm_length": 50.0,
    "left_finger_released_angle": -1.0,
    "right_finger_released_angle": 1.0,
    "init_left_finger_angle": -0.15,
    "init_right_finger_angle": 0.15,
    "static_elbow_offset": 0.05,
    "finger_level_offset_factor": 0.1
}

current_wrist_pos = None

class KomodoPicknPlaceComp:
    message_store = None
    action_feedback_pub = None
    attribute_query_client = None
    instance_query_client = None
    update_knowledge_client = None
    dispatch_subscriber = None
    debug_level = 0
    ready = False
    table_positions_count = -1


    # TODO: Add timeout checks after long actions like database queries or arm movements

    def parse_conf_file(self, conf_file):
        fname = "{}::{}".format(self.__class__.__name__, self.parse_conf_file.__name__)

        configs = {}

        for line in conf_file:
            if (line.lstrip()[0] == "#"):
                continue
            line_parts = line.lstrip().split(":")
            if ((line_parts[0].find(" ") != -1) or (line_parts[0].find("#"))):
                rospy.logerr("{}: Parameter {} contains space or hash. Can't parse configuration file (using defaults)".
                             format(fname, line_parts[0]))
            if ((line_parts[1].find(" ") != -1) or (line_parts[1].find("#"))):
                rospy.logerr("{}: value {} contains space or hash. Can't parse configuration file (using defaults)".
                             format(fname, line_parts[1]))
            configs[line_parts[0]] = float(line_parts[1])

        for config in configs:
            default_values[config[0]] = config[1]

    def update_block_pos(self, block, level, table_pos):
        block_pos = BlockPos()
        block_pos.tablePos = table_pos
        block_pos.level = level
        query_response = self.message_store.update_named(block, block_pos, upsert=True)
        return query_response

    def init_db_block_positions(self):
        fname = "{}::{}".format(self.__class__.__name__, self.init_db_block_positions.__name__)

        prob_blocks = self.instance_query_client.call("block_t")
        on_predicates = self.attribute_query_client.call("on")
        tables_blocks = []
        rospy.logdebug("{}: Got {} blocks and {} \"on\" predicates from KMS".format(fname, len(prob_blocks.instances),
                                                                                    len(on_predicates.attributes)))

        # Update position of "inhand" block (if such exists)
        inhand_predicates = self.attribute_query_client.call("inhand")
        for inhand_pred in inhand_predicates.attributes:
            block_name = inhand_pred.values[0].value
            query_response = self.update_block_pos(block_name, -1, -1)
            if (query_response.success is False):
                rospy.logfatal("{}: Failed to update block {} position to {} {}. Will not Initialize!".
                               format(fname, block_name, -1, -1))
                return False
            else:
                rospy.loginfo("{}: Put block {} in hand (Pos {} {})".
                              format(fname, block_name, -1, -1))
            prob_blocks.instances.remove(block_name)

        # Looking for tables (block which are not "on" anything) and updating the database
        current_table_pos = 1
        for block in prob_blocks.instances:
            found_on_pred_for_block = False
            for on_predicate in on_predicates.attributes:
                for keyValue in on_predicate.values:
                    if ((keyValue.value == block) and (keyValue.key == "block")):
                        found_on_pred_for_block = True
                        break
                if (found_on_pred_for_block is True):
                    break
            if not found_on_pred_for_block:
                query_response = self.update_block_pos(block, 0, current_table_pos)
                if (query_response.success is False):
                    rospy.logfatal("{}: Failed to update block {} position to {} {}. Will not Initialize!".
                                   format(fname, block, 0, current_table_pos))
                    return False
                else:
                    rospy.loginfo("{}: Positioned table block {} at level {}, tablePos {}".
                                  format(fname, block, 0, current_table_pos))

                current_table_pos += 1
                tables_blocks.append(block)
        prob_blocks.instances = [block for block in prob_blocks.instances if block not in tables_blocks]

        # Updating other blocks (not tables) positions in the database
        found_blocks = 0
        while len(prob_blocks.instances) > found_blocks:
            for block in prob_blocks.instances:
                for on_predicate in on_predicates.attributes:
                    block_name = None
                    on_block_name = None
                    for keyValue in on_predicate.values:
                        if ((keyValue.value == block) and (keyValue.key == "block")):
                            block_name = block
                        if (keyValue.key == "on_block"):
                            on_block_name = keyValue.value
                    if ((block_name is not None) and (on_block_name is not None)):
                        query_result = self.message_store.query_named(on_block_name, "komodo_blockworld/BlockPos",
                                                                      False)
                        if (query_result is None):
                            rospy.logfatal("{}: Failed to query DB. Aborting...".format(fname))
                            return False
                        if (len(query_result) > 1):
                            rospy.logerr("{}: Found {} blocks named {}. Aborting...".format(fname, len(query_result),
                                                                                            on_block_name))
                            return False
                        if (len(query_result) == 1):
                            level = query_result[0][0].level + 1
                            tablePos = query_result[0][0].tablePos
                            query_response = self.update_block_pos(block_name, level, tablePos)
                            if (query_response.success is False):
                                rospy.logfatal("{}: Failed to update block {} position to {} {}. Will not Initialize!".
                                               format(fname, block_name, level, tablePos))
                                return False
                            else:
                                rospy.loginfo("{}: Positioned block {} at level {}, tablePos {}".
                                              format(fname, block, level, tablePos))
                            found_blocks += 1
                        break

    def __init__(self, conf_file=None):
        fname = "{}::{}".format(self.__class__.__name__, self.__init__.__name__)

        self.message_store = mongodb_store.message_store.MessageStoreProxy()
        self.update_knowledge_client = rospy.ServiceProxy("/kcl_rosplan/update_knowledge_base", KnowledgeUpdateService)
        self.attribute_query_client = rospy.ServiceProxy("/kcl_rosplan/get_current_knowledge", GetAttributeService)
        self.instance_query_client = rospy.ServiceProxy("/kcl_rosplan/get_current_instances", GetInstanceService)

        self.dispatch_subscriber = rospy.Subscriber("/kcl_rosplan/action_dispatch", ActionDispatch,
                                                    self.dispatch_callback, queue_size=1000)
        self.action_feedback_pub = rospy.Publisher("/kcl_rosplan/action_feedback", ActionFeedback, queue_size=10)

        if (conf_file is not None):
            self.parse_conf_file(conf_file)

        self.rise_arm()
        rospy.sleep(1.5)

        if (self.init_db_block_positions() is False):
            return

        self.ready = True
        rospy.logdebug("{}: Pick&Place Component initialized".format(fname))

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

        if (not self.ready):
            rospy.loginfo("{}: Pick&Place Component not ready to handle commands".format(fname))
            return

        rospy.loginfo("{}: Handling pick up action".format(fname))

        (block_name, from_block_name) = self.get_parameter_values(pick_up_msg.parameters, "block", "from_block")

        if (block_name is None or from_block_name is None):
            rospy.logerr("{}: Failed to find block parameters in pick up message".format(fname))
            return

        query_result = self.message_store.query_named(block_name, "komodo_blockworld/BlockPos", False)
        if (query_result is None):
            rospy.logfatal("{}: Failed to query DB. Aborting...".format(fname))
            return

        else:

            if (len(query_result) != 1):
                rospy.logerr("{}: Found {} blocks named {}. Aborting...".format(fname, len(query_result), block_name))
                return

            block_pos = query_result[0][0]
            if (block_pos.tablePos == -1 or block_pos.level == -1):
                rospy.logfatal("{}: Block {} table position is {} and it's level is {}!".
                               format(fname, block_name, block_pos.tablePos, block_pos.level))

            actionFeedback = ActionFeedback()
            actionFeedback.action_id = pick_up_msg.action_id
            actionFeedback.status = "action enabled"
            self.action_feedback_pub.publish(actionFeedback)

            self.release_grip()
            rospy.sleep(1.5)

            self.position_arm(block_pos.tablePos, block_pos.level * 1.1)
            rospy.sleep(1.5)

            self.grip_block(block_pos.level)
            rospy.sleep(2)

            self.rise_arm()

            self.apply_effects_to_KMS(block_name, from_block_name, pick_up_msg.name)

            # Block is in hand remove it's position from the DB
            query_response = self.update_block_pos(block_name, -1, -1)
            if (query_response.success is False):
                rospy.logerr("{}: Failed to updat block {} position to {} {}".
                             format(fname, block_name, -1, -1))
                actionFeedback = ActionFeedback()
                actionFeedback.action_id = pick_up_msg.action_id
                actionFeedback.status = "action failed"
                self.action_feedback_pub.publish(actionFeedback)
            else:
                rospy.logdebug("{}: Updated block {} position to {} {}".
                               format(fname, block_name, -1, -1))
                actionFeedback = ActionFeedback()
                actionFeedback.action_id = pick_up_msg.action_id
                actionFeedback.status = "action achieved"
                self.action_feedback_pub.publish(actionFeedback)

    def handle_put_down(self, put_down_msg):
        fname = "{}::{}".format(self.__class__.__name__, self.handle_put_down.__name__)

        if (not self.ready):
            rospy.loginfo("{}: Pick&Place Component not ready to handle commands".format(fname))
            return

        rospy.loginfo("{}: Handling put down action".format(fname))

        (block_name, on_block_name) = self.get_parameter_values(put_down_msg.parameters, "block", "on_block")

        if (block_name is None or on_block_name is None):
            rospy.logerr("{}: Failed to find block parameters in put down message".format(fname))
            return

        query_result = self.message_store.query_named(on_block_name, "komodo_blockworld/BlockPos", False)
        if (query_result is None):
            rospy.logfatal("{}: Failed to query DB. Aborting...".format(fname))
            return

        else:
            if (len(query_result) != 1):
                rospy.logerr("{}: Found {} blocks named {}. Aborting...".format(fname, len(query_result), block_name))
                return

            on_block_pos = query_result[0][0]

            actionFeedback = ActionFeedback()
            actionFeedback.action_id = put_down_msg.action_id
            actionFeedback.status = "action enabled"
            self.action_feedback_pub.publish(actionFeedback)

            self.position_arm(on_block_pos.tablePos, on_block_pos.level + 1.5)
            rospy.sleep(2)

            self.release_grip()
            rospy.sleep(0.5)

            # Re-grip the block at a slightly lower position to stabilize the stack
            self.position_arm(on_block_pos.tablePos, on_block_pos.level + 1.1)
            rospy.sleep(0.2)
            self.grip_block(on_block_pos.level + 1)
            rospy.sleep(2)
            self.release_grip()
            rospy.sleep(0.2)

            self.rise_arm()

            self.apply_effects_to_KMS(block_name, on_block_name, put_down_msg.name)

            query_response = self.update_block_pos(block_name, on_block_pos.level + 1, on_block_pos.tablePos)
            if (query_response.success is False):
                rospy.logerr("{}: Failed to updat block {} position to {} {}".
                             format(fname, block_name, on_block_pos.level + 1, on_block_pos.tablePos))
                actionFeedback = ActionFeedback()
                actionFeedback.action_id = put_down_msg.action_id
                actionFeedback.status = "action failed"
                self.action_feedback_pub.publish(actionFeedback)
            else:
                rospy.logdebug("{}: Updated block {} position to {} {}".
                               format(fname, block_name, on_block_pos.level + 1, on_block_pos.tablePos))
                actionFeedback = ActionFeedback()
                actionFeedback.action_id = put_down_msg.action_id
                actionFeedback.status = "action achieved"
                self.action_feedback_pub.publish(actionFeedback)

    def position_arm(self, table_pos, height_level):
        fname = "{}::{}".format(self.__class__.__name__, self.position_arm.__name__)

        # Position static elbow (elbow 1)
        static_elbow_pub = rospy.Publisher("/komodo_1/elbow1_controller/command", Float64, queue_size=10, latch=True)
        static_elbow_msg = Float64()
        static_elbow_msg.data = default_values["static_elbow_offset"]
        rospy.loginfo("{}: Moving elbow 1 to 0rad".format(fname))
        static_elbow_pub.publish(static_elbow_msg)

        # Position Base
        base_angle = self.get_base_angle(table_pos)
        base_rot_pub = rospy.Publisher("/komodo_1/base_rotation_controller/command", Float64, queue_size=10, latch=True)
        base_angle_msg = Float64()
        base_angle_msg.data = base_angle + default_values["base_rotation_center_offset"]
        rospy.loginfo("{}: Moving base to {}rad".format(fname, base_angle))
        base_rot_pub.publish(base_angle_msg)

        # Position wrist
        wrist_angle = math.pi / 2 + base_angle
        # Rotate the wrist back 180deg since it can't make a full rotation
        if (wrist_angle > default_values["wrist_max_angle"]):
            wrist_angle -= math.pi
        wrist_rot_pub = rospy.Publisher("/komodo_1/wrist_controller/command", Float64, queue_size=10, latch=True)
        wrist_angle_msg = Float64()
        wrist_angle_msg.data = wrist_angle
        rospy.loginfo("{}: Moving wrist to {}rad".format(fname, wrist_angle))
        wrist_rot_pub.publish(wrist_angle_msg)

        # Compute shoulder values
        shoulder_angle_ratio = (height_level - 1.) * default_values["block_height"] / default_values["arm_length"]
        shoulder_angle = default_values["shoulder_max_angle"] - math.asin(shoulder_angle_ratio)

        # Position elbow 2
        elbow_angle = math.pi - shoulder_angle
        elbow_rot_pub = rospy.Publisher("/komodo_1/elbow2_controller/command", Float64, queue_size=10, latch=True)
        elbow_angle_msg = Float64()
        elbow_angle_msg.data = elbow_angle
        rospy.loginfo("{}: Moving elbow 2 to {}rad".format(fname, elbow_angle))
        elbow_rot_pub.publish(elbow_angle_msg)

        # Allow the joints to reach their target position before lowering the shoulder onto the block stack
        rospy.sleep(2.5)

        # Position shoulder
        shoulder_rot_pub = rospy.Publisher("/komodo_1/shoulder_controller/command", Float64, queue_size=10, latch=True)
        shoulder_angle_msg = Float64()
        shoulder_angle_msg.data = shoulder_angle
        rospy.loginfo("{}: Moving shoulder to {}rad".format(fname, shoulder_angle))
        shoulder_rot_pub.publish(shoulder_angle_msg)

    def get_base_angle(self, table_pos):
        fname = "{}::{}".format(self.__class__.__name__, self.get_base_angle.__name__)
        # TODO: Remove to allow actual table positions count computation
        #self.table_positions_count = 4

        if (self.table_positions_count < 0):
            rospy.logdebug("{}: Table position count not set, querying database to find it".format(fname))
            # count how many table positions are there (table positions are blocks that are not
            # "on" any other block or in hand)
            prob_blocks = self.instance_query_client.call("block_t")
            on_predicates = self.attribute_query_client.call("on")
            inhand_predicates = self.attribute_query_client.call("inhand")

            rospy.logdebug("{}: Query results: blocks count: {}, \"on\" count: {}, \"inhand\" count: {}".
                           format(fname, len(prob_blocks.instances), len(on_predicates.attributes),
                                  len(inhand_predicates.attributes)))

            self.table_positions_count = len(prob_blocks.instances) - (len(on_predicates.attributes) +
                                                                       len(inhand_predicates.attributes))

        min_rotation_angle = default_values["min_base_angle"]
        max_rotation_angle = default_values["max_base_angle"]
        base_angle = min_rotation_angle + (max_rotation_angle - min_rotation_angle) * \
                                          ((table_pos - 1.0) / (self.table_positions_count - 1.0))

        rospy.logdebug(
            "{}: Calculated base angle (before offset correction): {}. table pos = {}, table position count = {}".
            format(fname, base_angle, table_pos, self.table_positions_count))

        return base_angle

    def rise_arm(self):
        fname = "{}::{}".format(self.__class__.__name__, self.rise_arm.__name__)
        shoulder_publisher = rospy.Publisher("/komodo_1/shoulder_controller/command", Float64, queue_size=10,
                                             latch=True)
        shoulder_angle = Float64()
        shoulder_angle.data = default_values["raised_shoulder_angle"]
        rospy.logdebug("{}: Rising shoulder to {}rad".format(fname, shoulder_angle.data))
        shoulder_publisher.publish(shoulder_angle)
        rospy.sleep(0.2)

        elbow_rot_publisher = rospy.Publisher("/komodo_1/elbow2_controller/command", Float64, queue_size=10, latch=True)
        elbow_angle_msg = Float64()
        elbow_angle_msg.data = math.pi - shoulder_angle.data
        rospy.logdebug("{}: Moving elbow 2 to {}rad".format(fname, elbow_angle_msg.data))
        elbow_rot_publisher.publish(elbow_angle_msg)

        static_elbow_publisher = rospy.Publisher("/komodo_1/elbow1_controller/command", Float64, queue_size=10,
                                                 latch=True)
        static_elbow_angle_msg = Float64()
        static_elbow_angle_msg.data = default_values["static_elbow_offset"]
        rospy.logdebug("{}: Moving static elbow to {}rad".format(fname, static_elbow_angle_msg.data))
        static_elbow_publisher.publish(static_elbow_angle_msg)


    def grip_block(self, level):
        fname = "{}::{}".format(self.__class__.__name__, self.grip_block.__name__)
        # level is 1-based in the DB, but needed as 0-based for calculations
        level -= 1

        first_finger = "left" if current_wrist_pos < 0 else "right"
        second_finger = "right" if current_wrist_pos < 0 else "left"

        finger_publisher = rospy.Publisher("/komodo_1/{}_finger_controller/command".format(first_finger), Float64, queue_size=10)
        first_finger_msg = Float64()
        first_finger_msg.data = default_values["init_{}_finger_angle".format(first_finger)] + \
                               0.9 * level * default_values["finger_level_offset_factor"]
        finger_publisher.publish(first_finger_msg)
        rospy.logdebug("{}: Moving {} finger to {}rad".format(fname, first_finger, first_finger_msg.data))
        rospy.sleep(0.7 * level)

        finger_publisher = rospy.Publisher("/komodo_1/{}_finger_controller/command".format(second_finger), Float64, queue_size=10)
        second_finger_msg = Float64()
        second_finger_msg.data = default_values["init_{}_finger_angle".format(second_finger)] + \
                                level * default_values["finger_level_offset_factor"]
        finger_publisher.publish(second_finger_msg)
        rospy.logdebug("{}: Moving {} finger to {}rad".format(fname, second_finger, second_finger_msg.data))

    def release_grip(self):

        first_finger = "right" if current_wrist_pos < 0 else "left"
        second_finger = "left" if current_wrist_pos < 0 else "right"

        finger_publisher = rospy.Publisher("/komodo_1/{}_finger_controller/command".format(first_finger), Float64, queue_size=10,
                                           latch=True)
        first_finger_msg = Float64()
        first_finger_msg.data = default_values["{}_finger_released_angle".format(first_finger)]
        finger_publisher.publish(first_finger_msg)

        rospy.sleep(1)

        finger_publisher = rospy.Publisher("/komodo_1/{}_finger_controller/command".format(second_finger), Float64, queue_size=10,
                                           latch=True)
        second_finger_msg = Float64()
        second_finger_msg.data = default_values["{}_finger_released_angle".format(second_finger)]
        finger_publisher.publish(second_finger_msg)

    def get_parameter_values(self, parameters, first_param, second_param):
        block_name = None
        from_block_name = None
        for param_idx in xrange(0, len(parameters)):
            if (parameters[param_idx].key == first_param):
                block_name = parameters[param_idx].value
            if (parameters[param_idx].key == second_param):
                from_block_name = parameters[param_idx].value
        return block_name, from_block_name

    def apply_effects_to_KMS(self, block_name, other_block_name, action_name):
        fname = "{}::{}".format(self.__class__.__name__, self.apply_effects_to_KMS.__name__)

        emptyhand_update_type = KnowledgeUpdateServiceRequest.REMOVE_KNOWLEDGE if \
            (action_name == "pick_up") else KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE

        not_emptyhand_update_type = KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE if \
            (action_name == "pick_up") else KnowledgeUpdateServiceRequest.REMOVE_KNOWLEDGE

        onblock_update_type = KnowledgeUpdateServiceRequest.REMOVE_KNOWLEDGE if \
            (action_name == "pick_up") else KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE

        clear_update_type = KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE if \
            (action_name == "pick_up") else KnowledgeUpdateServiceRequest.REMOVE_KNOWLEDGE

        inhand_update_type = KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE if \
            (action_name == "pick_up") else KnowledgeUpdateServiceRequest.REMOVE_KNOWLEDGE

        # not_emptyhand predicate
        updated_knowledge = KnowledgeItem()
        updated_knowledge.knowledge_type = KnowledgeItem.FACT
        updated_knowledge.attribute_name = "not_emptyhand"
        update_response = self.update_knowledge_client(not_emptyhand_update_type, updated_knowledge)
        rospy.logdebug(
            "{}: Updated KMS with {} {}".format(fname, updated_knowledge.attribute_name, not_emptyhand_update_type))
        if (update_response.success is not True):
            rospy.logerr("{}: Could not update KMS with action effect ({} {})".
                         format(fname, not_emptyhand_update_type, "not_emptyhand"))

        # emptyhand predicate
        updated_knowledge = KnowledgeItem()
        updated_knowledge.knowledge_type = KnowledgeItem.FACT
        updated_knowledge.attribute_name = "emptyhand"
        update_response = self.update_knowledge_client(emptyhand_update_type, updated_knowledge)
        rospy.logdebug(
            "{}: Updated KMS with {} {}".format(fname, updated_knowledge.attribute_name, emptyhand_update_type))
        if (update_response.success is not True):
            rospy.logerr("{}: Could not update KMS with action effect ({} {})".
                         format(fname, emptyhand_update_type, "emptyhand"))

        # (on ?block ?from_block) predicate
        updated_knowledge = KnowledgeItem()
        updated_knowledge.knowledge_type = KnowledgeItem.FACT
        updated_knowledge.attribute_name = "on"
        pair = KeyValue()
        pair.key = "block"
        pair.value = block_name
        updated_knowledge.values.append(pair)
        pair = KeyValue()
        pair.key = "on_block"
        pair.value = other_block_name
        updated_knowledge.values.append(pair)
        update_response = self.update_knowledge_client(onblock_update_type, updated_knowledge)
        rospy.logdebug("{}: Updated KMS with {} ({}, {}) {}".
                       format(fname,
                              updated_knowledge.attribute_name,
                              updated_knowledge.values[0].value,
                              updated_knowledge.values[1].value,
                              onblock_update_type))
        if (update_response.success is not True):
            rospy.logerr("{}: Could not update KMS with action effect ({} {})".
                         format(fname, onblock_update_type, "on"))

        # (clear ?from_block) predicate
        updated_knowledge = KnowledgeItem()
        updated_knowledge.knowledge_type = KnowledgeItem.FACT
        updated_knowledge.attribute_name = "clear"
        pair = KeyValue()
        pair.key = "block"
        pair.value = other_block_name
        updated_knowledge.values.append(pair)
        update_response = self.update_knowledge_client(clear_update_type, updated_knowledge)
        rospy.logdebug("{}: Updated KMS with {} ({}) {}".
                       format(fname,
                              updated_knowledge.attribute_name,
                              updated_knowledge.values[0].value,
                              clear_update_type))
        if (update_response.success is not True):
            rospy.logerr("{}: Could not update KMS with action effect ({} {})".
                         format(fname, clear_update_type, "clear"))

        # (inhand ?block) predicate
        updated_knowledge = KnowledgeItem()
        updated_knowledge.knowledge_type = KnowledgeItem.FACT
        updated_knowledge.attribute_name = "inhand"
        pair = KeyValue()
        pair.key = "block"
        pair.value = block_name
        updated_knowledge.values.append(pair)
        update_response = self.update_knowledge_client(inhand_update_type, updated_knowledge)
        rospy.logdebug("{}: Updated KMS with {} ({}) {}".
                       format(fname,
                              updated_knowledge.attribute_name,
                              updated_knowledge.values[0].value,
                              inhand_update_type))
        if (update_response.success is not True):
            rospy.logerr("{}: Could not update KMS with action effect ({} {})".
                         format(fname, inhand_update_type, "inhand"))


def print_usage():
    print "Usage:\n\t{} <conf file path>".format(sys.argv[0].split("/")[-1])

def update_wrist_pos(wrist_pos_data):
    current_wrist_pos = wrist_pos_data.position[0]
    #Not good at all, fix it

if __name__ == '__main__':
    try:

        conf_file = None
        if (len(sys.argv) == 3):
            conf_file = open(sys.argv[1], "r")

        log_level = rospy.DEBUG

        rospy.init_node("KomodoPickNPlaceComp", anonymous=False, log_level=log_level)
        component = KomodoPicknPlaceComp(conf_file)
        component.debug_level = 1

        rospy.Subscriber("/komodo_1/wrist_controller/state", JointState, update_wrist_pos)

        rospy.spin()

    except rospy.ROSInterruptException, e:
        print e

