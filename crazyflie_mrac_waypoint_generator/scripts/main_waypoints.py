#!/usr/bin/env python
"""
See WaypointsPlanner.py for details
"""
from enum import IntEnum

import roslib
import rospy

from sensor_msgs.msg import Joy
import std_msgs.msg as std_msgs
import geometry_msgs.msg as geometry_msgs

from WaypointsPlanner import * 

# Params
CONTROLLER_HZ = 50.0
CONTROLLER_DT = 1.0/CONTROLLER_HZ

# takeoff and landing
takeoff_landing_wps = []
for i in range(3):
    takeoff_landing_wps.append((-1.0, 0.0, 0.03))
    takeoff_landing_wps.append((-1.0, 0.0, 0.10))

# step response, z: 1.0 -> 0.5
steps_response = []
for i in range(3):
    steps_response.append((-1.0, 0.0, 1.0))
    steps_response.append((-1.0, 0.0, 0.5))

# const z square path
Z_SQUARE = 0.5
square_wps = [
(-1.0,  0.0, Z_SQUARE),
(-1.0, -1.0, Z_SQUARE),
( 0.0, -1.0, Z_SQUARE),
( 1.0, -1.0, Z_SQUARE),
( 1.0,  0.0, Z_SQUARE),
( 1.0,  1.0, Z_SQUARE),
( 0.0,  1.0, Z_SQUARE),
(-1.0,  1.0, Z_SQUARE),
(-1.0,  0.0, Z_SQUARE)]

# ground effect experiment
ground_effect_zStep = 0.025
ground_effect = [(-1.0, 0.0, 0.2),]
for i in range(10):
    last_z = ground_effect[i][2]
    ground_effect.append((-1.0, 0.0, last_z - ground_effect_zStep))

# mission
Z_MISSION_CURISE = 0.8
Z_MISSION_LOW = 0.08
mission_wps = [
    (-1.0,  0.0, Z_MISSION_CURISE),
    (-1.0, -1.0, Z_MISSION_CURISE),
    ( 0.0, -1.0, Z_MISSION_LOW),
    ( 1.0, -1.0, Z_MISSION_LOW),
    ( 1.0,  0.0, Z_MISSION_CURISE),
    ( 1.0,  1.0, Z_MISSION_CURISE),
    ( 0.0,  1.0, Z_MISSION_LOW),
    (-1.0,  1.0, Z_MISSION_LOW),
    (-1.0,  0.0, Z_MISSION_CURISE)
]

# Which one to use?
#planner = WaypointsPlanner(CONTROLLER_HZ, takeoff_landing_wps, 0.1, 15.0)
#planner = WaypointsPlanner(CONTROLLER_HZ, steps_response, 0.5, 15.0)
#planner = WaypointsPlanner(CONTROLLER_HZ, square_wps, 0.5, 10.0)
#planner = WaypointsPlanner(CONTROLLER_HZ, ground_effect, 0.3, 10.0)
planner = WaypointsPlanner(CONTROLLER_HZ, mission_wps, 0.3, 10.0)

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
    rospy.Timer(rospy.Duration(CONTROLLER_DT),
                planner.create_loop_callback())

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "shutting down"
