#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate
from math import sqrt, cos, sin

# desired waypoints
## sinusoid
#ref_x = np.arange(0, 2*np.pi+np.pi/4, 2*np.pi/32)
#ref_y = np.sin(ref_x)

## y axis
#ref_x = np.arange(0, 20, 0.1)
#ref_y = np.zeros(len(ref_x))

# ramp
# ref_x = np.arange(0, 100, 0.5)
# ref_y = np.arange(0, 100, 0.5)

data = np.loadtxt('/home/turtlebot/gem_sim/src/polaris_control/src/sparse_lane_data.dat')
ref_x = data[:, 0]
ref_y = data[:, 1]
#ref_x = np.flip(ref_x)
#ref_y = np.flip(ref_y)
ref_x = ref_x - 7
ref_y = ref_y + 8
#print(ref_x, ref_y)
#plt.plot(ref_x,ref_y)
#plt.show()

# get the distance between two points
def GetDistance(Traj, State):
	dist = sqrt(pow((State[0] - Traj[0]), 2) + pow((State[1] - Traj[1]), 2))
	return dist

def TransCoordinate(Traj, State):
	# for readability
	ref_x_ = Traj[0]
	ref_y_ = Traj[1]
	cur_x_ = State[0]
	cur_y_ = State[1]
	cur_yaw_ = State[2]
	dx = ref_x_ - cur_x_
	dy = ref_y_ - cur_y_
	Traj[0] = dx * cos(cur_yaw_) + dy * sin(cur_yaw_)
	Traj[1] = dy * cos(cur_yaw_) - dx * sin(cur_yaw_)

def GetOffset(car_states):

	near_ref_states = []
	node = 0

	ref_states = list(zip(ref_x,ref_y))
	# change tuple to list
	for i in range(len(ref_states)):
		ref_states[i] = list(ref_states[i])

	# find the nearest neighbour
	for i in range(len(ref_states)):
		if GetDistance(ref_states[i], car_states) < GetDistance(ref_states[node], car_states):
			node = i

	# select 10 near points for interpolation
	if node < 4:
		near_ref_states = ref_states[len(ref_states) - (4 - node):len(ref_states)] + ref_states[0:node + 6]
		#print("start")
		#print(len(ref_states) - (4 - node))
	elif node > (len(ref_states) - 6):
		near_ref_states = ref_states[node - 4:len(ref_states)] + ref_states[0:node - len(ref_states) + 6]
		#print("end")
		#print(node - len(ref_states) + 6)
	else:
		near_ref_states = ref_states[node - 4:node + 6]
		#print("normal")

	#print(node)
	# transfer coordinate
	for i in range(len(near_ref_states)):
		TransCoordinate(near_ref_states[i], car_states)

	# spline interpolation
	near_ref_states = sorted(near_ref_states)
	near_ref_x, near_ref_y = zip(*near_ref_states)
	near_ref_x = list(near_ref_x)
	near_ref_y = list(near_ref_y)
	tck = interpolate.splrep(near_ref_x, near_ref_y, s=0)
	offset = interpolate.splev(0, tck, der=0)
	return offset