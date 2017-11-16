from collections import deque
import geometry_msgs.msg as geo_msgs

class Differentiation:
	def __init__(self):
		maxlen = 3
		self.poseList = deque([None, None, None], maxlen);

	def __calc_vel__(self, dt, p0, p1):
		return (p1-p0)/dt

	def __calc_acc__(self, p0, p1, p2):
		return PLACEHOLDER/dt/dt

	def calculate(self, pose):
		pass
