
from math import pi as PI

def limit(value, vmin, vmax):
	return max(min(value, vmax), vmin)

def mapping(value, old_min, old_max, new_min, new_max):
	return (value-old_min)/(old_max-old_min)*(new_max-new_min)+new_min

def deg2rad(deg):
	return deg/180*PI
	
def rad2deg(rad):
	return rad*180/PI