#!/usr/bin/env python
import rospy
import sys

from diagnostic_msgs.msg import KeyValue
from rosplan_dispatch_msgs.msg import ActionDispatch


if __name__ == '__main__':
    try:
        if (len(sys.argv) < 4):
            print "Usage:\n\t{} <pick_up/put_down> <block_1> <block_2>".format(sys.argv[0].split("/")[-1])

        rospy.init_node("mock_action_dispatch", log_level=rospy.DEBUG)

        publisher = rospy.Publisher("/kcl_rosplan/action_dispatch", ActionDispatch, queue_size=1000, latch=True)
        action = ActionDispatch()
        action.name = sys.argv[1]
        pair = KeyValue()
        pair.key = "block"
        pair.value = sys.argv[2]
        action.parameters.append(pair)
        pair = KeyValue()
        pair.key = "from_block" if (action.name == "pick_up") else "on_block"
        pair.value = sys.argv[3]
        action.parameters.append(pair)

        publisher.publish(action)

        rospy.spin()


    except rospy.ROSInterruptException, e:
        pass

