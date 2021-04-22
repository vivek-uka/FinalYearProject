#!/usr/bin/env python
import rospy
import numpy as np
from multi_robot_mpc.msg import States
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
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

x0 = y0 = v0 = wz0 = psi0 = 0.0
goal0 = [-4.5, 2, 3*np.pi/4]
obsx = [-6, -6, -5, -5, -5.5] 
obsy = [0.5, 1.5, 1.5, 0.5, 1]

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

def odomCallback0(data):
	global x0, y0, v0, wz0, psi0
	x0 = data.pose.pose.position.x
	y0 = data.pose.pose.position.y

	vx0 = data.twist.twist.linear.x
	vy0 = data.twist.twist.linear.y
	v0 = np.sqrt(vx0*vx0 + vy0*vy0)

	wz0 = data.twist.twist.angular.z

	qx = data.pose.pose.orientation.x
	qy = data.pose.pose.orientation.y
	qz = data.pose.pose.orientation.z
	qw = data.pose.pose.orientation.w

	siny_cosp = 2 * (qw * qz + qx * qy)
	cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
	psi0 = np.arctan2(siny_cosp, cosy_cosp)


def dist(x1, y1, x2, y2):
	return np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

if __name__ == '__main__':
	
	freq = 100
	rospy.init_node('my_stats', anonymous='True')	
	rospy.Subscriber('tb3_0/pre_state', States, statesCallback0)
	rospy.Subscriber('tb3_1/pre_state', States, statesCallback1)
	rospy.Subscriber('tb3_2/pre_state', States, statesCallback2)
	rospy.Subscriber('tb3_3/pre_state', States, statesCallback3)
	rospy.Subscriber('tb3_0/odom', Odometry, odomCallback0)
	
	rate = rospy.Rate(freq)
	
	dist_01 = []
	dist_02 = []
	dist_03 = []
	dist_12 = []
	dist_13 = []
	dist_23 = []
	time = []
	

	simulation_time = []
	simulation_v = []
	simulation_psidot = []
	simulation_x_residue = []
	simulation_y_residue = []
	simulation_psi_residue = []
	iter = 0.0
	cnt = 0
	fig = plt.figure()
	ax1 = fig.add_subplot(1, 1, 1)

	while not rospy.is_shutdown():
		if rx0 >= 5:
			iter += 1
			dist_goal = np.sqrt((goal0[0] - x0) ** 2 + (goal0[1] - y0) ** 2)
			simulation_time.append(iter * 1/freq)
			simulation_v.append(v0)
			simulation_psidot.append(wz0)
			simulation_x_residue.append((abs(x0 - goal0[0])))
			simulation_y_residue.append((abs(y0 - goal0[1])))
			simulation_psi_residue.append((abs(psi0 - goal0[2])))
			
			if simulation_x_residue[-1] < 0.01 and simulation_y_residue[-1] < 0.01 and simulation_psi_residue[-1] < 0.01:
				break
			plt.clf()
			ax1 = fig.add_subplot(1, 1, 1)
			ax1.set_aspect(1)
			
			circle0 = (plt.Circle((obsx[0], obsy[0]), 0.2+0.1, color='orange', fill=False))
			circle1 = (plt.Circle((obsx[1], obsy[1]), 0.2+0.1, color='orange', fill=False))
			circle2 = (plt.Circle((obsx[2], obsy[2]), 0.2+0.1, color='orange', fill=False))
			circle3 = (plt.Circle((obsx[3], obsy[3]), 0.2+0.1, color='orange', fill=False))
			circle4 = (plt.Circle((obsx[4], obsy[4]), 0.2+0.1, color='orange', fill=False))
			circle5 = (plt.Circle((obsx[0], obsy[0]), 0.2, color='orange'))
			circle6 = (plt.Circle((obsx[1], obsy[1]), 0.2, color='orange'))
			circle7 = (plt.Circle((obsx[2], obsy[2]), 0.2, color='orange'))
			circle8 = (plt.Circle((obsx[3], obsy[3]), 0.2, color='orange'))
			circle9 = (plt.Circle((obsx[4], obsy[4]), 0.2, color='orange'))

			ax1.add_artist(circle0)
			ax1.add_artist(circle1)
			ax1.add_artist(circle2)
			ax1.add_artist(circle3)
			ax1.add_artist(circle4)
			ax1.add_artist(circle5)
			ax1.add_artist(circle6)
			ax1.add_artist(circle7)
			ax1.add_artist(circle8)
			ax1.add_artist(circle9)
				

			plt.xlim(-7, -4)
			plt.ylim(-0.0, 2.5)
			plt.scatter(x0, y0, linewidths=0.05)
			plt.scatter(goal0[0], goal0[1], linewidths=0.05, color='green')
			plt.plot(states_x0, states_y0, linestyle=':',color='red')
			plt.title('Obstacle avoidance')
			plt.draw()
			plt.pause(0.000001)
		"""if rx0 >= 10 and rx1 >= 10 and rx2 >= 10 and rx3 >= 10:
			plt.clf()
			dist_01.append(dist(state0[0], state0[1], state1[0], state1[1]) - 1.0)
			dist_02.append(dist(state0[0], state0[1], state2[0], state2[1]) - 1.0)
			dist_03.append(dist(state0[0], state0[1], state3[0], state3[1]) - 1.0)
			dist_12.append(dist(state1[0], state1[1], state2[0], state2[1]) - 1.0)
			dist_13.append(dist(state1[0], state1[1], state3[0], state3[1]) - np.sqrt(3))
			dist_23.append(dist(state2[0], state2[1], state3[0], state3[1]) - 1.0)
			#plt.plot([state0[0], state1[0], state2[0], state3[0], state0[0]], [state0[1], state1[1], state2[1], state3[1], state0[1]])
			#plt.xlim(-10, 10)
			#plt.ylim(-10, 10)
			#plt.pause(0.000001)
			#plt.draw()
			#print(round(dist_01[-1], 3), round(dist_02[-1], 3), round(dist_03[-1], 3), round(dist_12[-1], 3), round(dist_13[-1], 3), round(dist_23[-1], 3))
			#print(round(np.mean(dist_01), 3), round(np.mean(dist_02), 3), round(np.mean(dist_03), 3), round(np.mean(dist_12), 3), round(np.mean(dist_13), 3), round(np.mean(dist_23), 3))
		"""
		rate.sleep()
	v_max = 0.22
	psidot_max = 2.84

	plt.figure(5)
	plt.title('residue_goal')
	plt.plot(simulation_time, simulation_x_residue)
	plt.plot(simulation_time, simulation_y_residue)
	plt.legend(["res_x", "res_y"], loc ="upper right")

	plt.figure(2)
	plt.title('v_controls')
	plt.plot(simulation_time, simulation_v)
	plt.plot([0, simulation_time[-1]], [v_max, v_max])
	plt.plot([0, simulation_time[-1]], [-v_max, -v_max])
	plt.legend(["linear_velocity", "max_bounds", "min_bounds"], loc ="upper right")

	plt.figure(3)
	plt.title('psidot_controls')
	plt.plot(simulation_time, simulation_psidot)
	plt.plot([0, simulation_time[-1]], [psidot_max, psidot_max])
	plt.plot([0, simulation_time[-1]], [-psidot_max, -psidot_max])
	plt.legend(["angular_velocity", "max_bounds", "min_bounds"], loc ="upper right")

	plt.figure(4)
	plt.title('res_psi')
	plt.plot(simulation_time, simulation_psi_residue)
	plt.legend(["res_psi"], loc="upper right")

	plt.show()
	rospy.spin()

