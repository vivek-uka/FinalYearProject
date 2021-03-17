#!/usr/bin/env python
import rospy
import numpy as np
from multi_robot_mpc.msg import States


rx0 = 0
rx1 = 0
rx2 = 0
rx3 = 0

state0 = [0, 0, 0]
states_x0 = []
states_y0 = []
states_psi0 = []

state1 = [0, 0, 0]
states_x1 = []
states_y1 = []
states_psi1 = []

state2 = [0, 0, 0]
states_x2 = []
states_y2 = []
states_psi2 = []

state3 = [0, 0, 0]
states_x3 = []
states_y3 = []
states_psi3 = []

def statesCallback0(data):
	global states_x0, states_y0, states_psi0, rx0, state0

	states_x0 = data.x
	states_y0 = data.y
	states_psi0 = data.psi

	state0[0] = data.x0
	state0[1] = data.y0
	state0[2] = data.psi0

	rx0 += 1

def statesCallback1(data):
	global states_x1, states_y1, states_psi1, rx1, state1

	states_x1 = data.x
	states_y1 = data.y
	states_psi1 = data.psi

	state1[0] = data.x0
	state1[1] = data.y0
	state1[2] = data.psi0
	rx1 += 1
	

def statesCallback2(data):
	global states_x2, states_y2, states_psi2, rx2, state2

	states_x2 = data.x
	states_y2 = data.y
	states_psi2 = data.psi

	state2[0] = data.x0
	state2[1] = data.y0
	state2[2] = data.psi0
	rx2 += 1
	

def statesCallback3(data):
	global states_x3, states_y3, states_psi3, rx3, state3

	states_x3 = data.x
	states_y3 = data.y
	states_psi3 = data.psi

	state3[0] = data.x0
	state3[1] = data.y0
	state3[2] = data.psi0
	rx3 += 1

def dist(x1, y1, x2, y2):
	return np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

if __name__ == '__main__':
	
	freq = 10
	rospy.init_node('my_stats', anonymous='True')	
	rospy.Subscriber('tb3_0/pre_state', States, statesCallback0)
	rospy.Subscriber('tb3_1/pre_state', States, statesCallback1)
	rospy.Subscriber('tb3_2/pre_state', States, statesCallback2)
	rospy.Subscriber('tb3_3/pre_state', States, statesCallback3)
	rate = rospy.Rate(freq)
	
	dist_01 = []
	dist_02 = []
	dist_03 = []
	dist_12 = []
	dist_13 = []
	dist_23 = []
	while not rospy.is_shutdown():

		if rx0 >= 10 and rx1 >= 10 and rx2 >= 10 and rx3 >= 10:
			dist_01.append(dist(state0[0], state0[1], state1[0], state1[1]) - 1.0)
			dist_02.append(dist(state0[0], state0[1], state2[0], state2[1]) - 1.0)
			dist_03.append(dist(state0[0], state0[1], state3[0], state3[1]) - 1.0)
			dist_12.append(dist(state1[0], state1[1], state2[0], state2[1]) - 1.0)
			dist_13.append(dist(state1[0], state1[1], state3[0], state3[1]) - np.sqrt(3))
			dist_23.append(dist(state2[0], state2[1], state3[0], state3[1]) - 1.0) 
			#print(round(dist_01[-1], 3), round(dist_02[-1], 3), round(dist_03[-1], 3), round(dist_12[-1], 3), round(dist_13[-1], 3), round(dist_23[-1], 3))
			print(round(np.mean(dist_01), 3), round(np.mean(dist_02), 3), round(np.mean(dist_03), 3), round(np.mean(dist_12), 3), round(np.mean(dist_13), 3), round(np.mean(dist_23), 3))
		rate.sleep()
	rospy.spin()

