#!/usr/bin/env python
"""
Generate waypoint using joystick
"""
from enum import IntEnum

import roslib
import rospy

from sensor_msgs.msg import Joy
import std_msgs.msg as std_msgs
import geometry_msgs.msg as geometry_msgs

from JoystickPlanner import * 

# Params
CONTROLLER_HZ = 30.0
CONTROLLER_DT = 1.0/CONTROLLER_HZ

planner = JoystickPlanner(-0.5, 0.0, # Center
                          1.0, 1.0, # Offset
                          1.0, 0.5 )

if __name__ == '__main__':
    rospy.init_node('trajectory_planner')

    rospy.Subscriber('/joy', Joy, planner.sub_joy_callback)
    rospy.Subscriber('~current_pose', geometry_msgs.PoseStamped, planner.sub_vrpn_callback)
    
    pub_target = rospy.Publisher(
        '/hover_controller/target_pose', geometry_msgs.Pose, queue_size=1)
    pub_state = rospy.Publisher(
        '/hover_controller/state', std_msgs.UInt8, queue_size=1)
    
    rospy.Timer(rospy.Duration(CONTROLLER_DT),
                planner.create_pub_target_timer_callback(pub_target))
    rospy.Timer(rospy.Duration(CONTROLLER_DT),
                planner.create_pub_states_timer_callback(pub_state))
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "shutting down"
