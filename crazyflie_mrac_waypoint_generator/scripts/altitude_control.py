#!/usr/bin/env python
"""
mapping joystick inputs to cmd_vel to control crazyflie
"""

import roslib
import rospy

from std_srvs.srv import Empty
import sensor_msgs.msg as sensor_msgs
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped

import tf
from jutils.jmath import * 

CONTROLLER_HZ = 40.0
CONTROLLER_DT = 1.0/CONTROLLER_HZ

MAX_ROLL_DEG = 30
MAX_PITCH_DEG = 30
MAX_YAW_DEG = 200

service_emergency = None
twist = Twist()
pose = PoseStamped()

def vrpn_callback(data):
	global pose
	pose.header.stamp = data.header.stamp

	pose.pose.position.x = data.pose.position.x
	pose.pose.position.y = data.pose.position.y
	pose.pose.position.z = data.pose.position.z

	# This is EULER ANGLES!!!
	euler = tf.transformations.euler_from_quaternion(
								(data.pose.orientation.x,
								 data.pose.orientation.y,
								 data.pose.orientation.z,
								 data.pose.orientation.w))
	pose.pose.orientation.x =  euler[1] # Roll
	pose.pose.orientation.y =  euler[0] # Pitch
	pose.pose.orientation.z = -euler[2] # Yaw
	pose.pose.orientation.w = 0.0 # always zero

	#rospy.loginfo("%.1f, %.1f, %.1f", euler[0]/3.1415*180.0, euler[1]/3.1415*180.0, euler[2]/3.1415*180.0)

def joystick_callback(data):
	global twist
	twist.linear.y  = -1*mapping(
		data.axes[0],-1, 1, -MAX_ROLL_DEG, MAX_ROLL_DEG) # roll
	twist.linear.x  = -1*mapping(
		data.axes[1],-1, 1, -MAX_ROLL_DEG, MAX_ROLL_DEG) # pitch
	twist.angular.z = -1*mapping(
		data.axes[2],-1, 1, -MAX_YAW_DEG, MAX_YAW_DEG) # yaw rate (-200 to +200 deg/sec)

	if data.axes[3] < -0.9:
		twist.linear.z = 0
	else:
		twist.linear.z  = mapping(
			data.axes[3],-1, 1, 0, 60000) # Thrust (0 to 60000)

def timer1_callback(timer):
	pub_cmd.publish(twist)

def timer2_callback(timer):
	pub_euler.publish(pose)

if __name__ == '__main__':
	rospy.init_node('control_by_vel_cmd')

	rospy.loginfo("waiting for emergency service")
	rospy.wait_for_service('/crazyflie/emergency')
	rospy.loginfo("found emergency service")
	service_emergency = rospy.ServiceProxy('/crazyflie/emergency', Empty)

	rospy.Subscriber('/joy', sensor_msgs.Joy, joystick_callback)
	rospy.Subscriber('~current_pose', PoseStamped, vrpn_callback)
	pub_cmd = rospy.Publisher('/crazyflie/cmd_vel', Twist, queue_size=1)
	pub_euler = rospy.Publisher('/crazyflie/current_pose', PoseStamped, queue_size=1)

	rospy.Timer(rospy.Duration(CONTROLLER_DT),timer1_callback)
	rospy.Timer(rospy.Duration(CONTROLLER_DT),timer2_callback)

	try:
	    rospy.spin()
	except KeyboardInterrupt:
	    print "shutting down"