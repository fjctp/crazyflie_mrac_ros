import rospy
import std_msgs.msg as std_msgs
import geometry_msgs.msg as geo_msgs
import uav_control.msg as uav_msgs

def Array2Point(array):
	return geo_msgs.Point(*array)
def Array2Vector3(array):
	return geo_msgs.Vector3(*array)

def Array2Quaternion(array):
	return geo_msgs.Quaternion(*array)

def vectors2BodyInfo(translation, rotation, velocity, twist, linear_accel, angular_accel):
	
	header = std_msgs.Header()
	header.stamp = rospy.Time.now()

	pose = geo_msgs.Pose(Array2Point(translation), Array2Quaternion(rotation))
	twist = geo_msgs.Twist(Array2Vector3(velocity), Array2Vector3(twist))
	accel = geo_msgs.Accel(Array2Vector3(linear_accel), Array2Vector3(angular_accel))

	return uav_msgs.BodyInfo(header, pose, twist, accel)
