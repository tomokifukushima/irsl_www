#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Bool


def callback_right_pos(msg):
    rospy.loginfo("RIGHT pos=(%.3f, %.3f, %.3f)", msg.x, msg.y, msg.z)


def callback_left_pos(msg):
    rospy.loginfo("LEFT pos=(%.3f, %.3f, %.3f)", msg.x, msg.y, msg.z)


def callback_right_button(msg):
    rospy.loginfo("RIGHT A=%s", msg.data)


def callback_left_button(msg):
    rospy.loginfo("LEFT X=%s", msg.data)


if __name__ == "__main__":
    rospy.init_node("subscribe_webxr_controller")
    rospy.Subscriber('/webxr/controller_state/right_con/position', Point, callback_right_pos)
    rospy.Subscriber('/webxr/controller_state/left_con/position', Point, callback_left_pos)
    rospy.Subscriber('/webxr/controller_state/right_con/button_a', Bool, callback_right_button)
    rospy.Subscriber('/webxr/controller_state/left_con/button_x', Bool, callback_left_button)
    rospy.spin()