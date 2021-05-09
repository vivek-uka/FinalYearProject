#!/usr/bin/env python
import rospy
import numpy as np
from multi_robot_mpc.msg import States
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from time import time

rx0 = 0
rx1 = 0
rx2 = 0
rx3 = 0

l = 0
state0 = [0, 0, 0]
init0 = [-4, -3, 1.57]
states_x0 = []
states_y0 = []
states_psi0 = []

state1 = [0, 0, 0]
init1 = [1.7, -4, 1.57]
states_x1 = []
states_y1 = []
states_psi1 = []

state2 = [0, 0, 0]
init2 = [1.7, 1.2, -1.57]
states_x2 = []
states_y2 = []
states_psi2 = []

state3 = [0, 0, 0]
init3 = [-4, 1.2, -1.57]
states_x3 = []
states_y3 = []
states_psi3 = []

v0 = wz0 = v1 = wz1 = v2 = wz2 = v3 = wz3 = 0.0
goal0 = [5.2, -2.2, 0]
goal1 = [0, 13, 0]#[5.2, -3.8, 0.0]
goal2= [1.7, 2.5, 0]
goal3 = [1.7, -4, -np.pi/4]

obsx = [-2.5, 0.34, 0.34, -1, -2.4]#[-1.7, -0.36, -1.7, -1, -0.36] #[-6, -6, -5, -5, -5.5] 
obsy = [0.5, 0.51, -2.27, -0.8, -2.2]#[-1.5, -0.36, -0.36, -0.8, -1.5] #[0.5, 1.5, 1.5, 0.5, 1]

shelfx = [-1.5, 1.5]#[4.73, 4.73, 4.73, 4.73, 4.73, 4.73, 4.4, -0.8, -1.08, -5.79]
shelfy = [8.22, 8.22]#[-8.66, -6.75, -4.84, -2.93, -1.02, 0.89, 6.67, 9.09, -0.7, -0.95]
shelfa = [1, 1]#[3.87, 3.87, 3.87, 3.87, 3.87, 3.87, 4.7, 3.31, 2.9, 2.42]
shelfb = [4.42, 4.42]#[0.85, 0.85, 0.85, 0.85, 0.85, 0.85, 7.8, 1.76, 15.87, 18] 

def statesCallback0(data):
	global states_x0, states_y0, states_psi0, rx0, state0, init0

	states_x0 = data.x 
	states_y0 = data.y
	states_psi0 = data.psi

	state0[0] = data.x0
	state0[1] = data.y0
	state0[2] = data.psi0

	rx0 += 1

def statesCallback1(data):
	global states_x1, states_y1, states_psi1, rx1, state1, l

	states_x1 = data.x
	states_y1 = data.y
	states_psi1 = data.psi

	state1[0] = data.x0
	state1[1] = data.y0
	state1[2] = data.psi0
	rx1 += 1
	l = data.l

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
	global v0, wz0

	vx0 = data.twist.twist.linear.x
	vy0 = data.twist.twist.linear.y
	v0 = np.sqrt(vx0*vx0 + vy0*vy0)
	wz0 = data.twist.twist.angular.z

def odomCallback1(data):
	global v1, wz1

	vx0 = data.twist.twist.linear.x
	vy0 = data.twist.twist.linear.y
	v1 = np.sqrt(vx0*vx0 + vy0*vy0)
	wz1 = data.twist.twist.angular.z

def odomCallback2(data):
	global v2, wz2

	vx0 = data.twist.twist.linear.x
	vy0 = data.twist.twist.linear.y
	v2 = np.sqrt(vx0*vx0 + vy0*vy0)
	wz2 = data.twist.twist.angular.z

def odomCallback3(data):
	global v3, wz3

	vx0 = data.twist.twist.linear.x
	vy0 = data.twist.twist.linear.y
	v3 = np.sqrt(vx0*vx0 + vy0*vy0)
	wz3 = data.twist.twist.angular.z

def dist(x1, y1, x2, y2):
	return np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

if __name__ == '__main__':
	
	freq = 100
	rospy.init_node('my_stats', anonymous='True')	
	rospy.Subscriber('volta_0/pre_state', States, statesCallback0)
	rospy.Subscriber('volta_1/pre_state', States, statesCallback1)
	rospy.Subscriber('volta_2/pre_state', States, statesCallback2)
	rospy.Subscriber('volta_3/pre_state', States, statesCallback3)
	rospy.Subscriber('volta_0/volta_base_controller/odom', Odometry, odomCallback0)
	rospy.Subscriber('volta_1/volta_base_controller/odom', Odometry, odomCallback1)
	rospy.Subscriber('volta_2/volta_base_controller/odom', Odometry, odomCallback2)
	rospy.Subscriber('volta_3/volta_base_controller/odom', Odometry, odomCallback3)


	
	rate = rospy.Rate(freq)
	
	dist_01 = []
	dist_02 = []
	dist_03 = []
	dist_12 = []
	dist_13 = []
	dist_23 = []
	
	dist0_obs0 = []
	dist0_obs1 = []
	dist0_obs2 = []
	dist0_obs3 = []
	dist0_obs4 = []
	dist1_obs0 = []
	dist1_obs1 = []
	dist1_obs2 = []
	dist1_obs3 = []
	dist1_obs4 = []
	dist2_obs0 = []
	dist2_obs1 = []
	dist2_obs2 = []
	dist2_obs3 = []
	dist2_obs4 = []
	dist3_obs0 = []
	dist3_obs1 = []
	dist3_obs2 = []
	dist3_obs3 = []
	dist3_obs4 = []

	simulation_time = []
	simulation_v0 = []
	simulation_psidot0 = []
	simulation_x_residue0 = []
	simulation_y_residue0 = []
	simulation_psi_residue0 = []
	simulation_v1 = []
	simulation_psidot1 = []
	simulation_x_residue1 = []
	simulation_y_residue1 = []
	simulation_psi_residue1 = []
	simulation_v2 = []
	simulation_psidot2 = []
	simulation_x_residue2 = []
	simulation_y_residue2 = []
	simulation_psi_residue2 = []
	simulation_v3 = []
	simulation_psidot3 = []
	simulation_x_residue3 = []
	simulation_y_residue3= []
	simulation_psi_residue3 = []

	iter = 0.0
	fig = plt.figure()
	ax1 = fig.add_subplot(1, 1, 1)

	trajx0 = []
	trajy0 = []
	trajx1 = []
	trajy1 = []
	trajx2 = []
	trajy2 = []
	trajx3 = []
	trajy3 = []
	t = time()
	l = 1
	squarex = []
	squarey = []
	d1 = []
	d2 = []
	d3 = []
	d4 = []
	d5 = []
	d6 = []
	config_matrix = [[0, l, 2*l/np.sqrt(2), l], [l, 0, l, 2*l/np.sqrt(2)], [2*l/np.sqrt(2), l, 0, l], [l, 2*l/np.sqrt(2), l, 0]]
	 
	while not rospy.is_shutdown():
		"""if rx0 >= 5 and rx1 >=5 and rx2 >= 5:
			iter += 1
			r = 0.2 * np.sqrt(2)/ 2+0.32
			dist_goal = np.sqrt((goal0[0] - state0[0]) ** 2 + (goal0[1] - state0[1]) ** 2)
			simulation_time.append(iter * 1/freq)
			simulation_v0.append(v0)
			simulation_psidot0.append(wz0)
			simulation_x_residue0.append((abs(state0[0] - goal0[0])))
			simulation_y_residue0.append((abs(state0[1] - goal0[1])))
			simulation_psi_residue0.append((abs(state0[2] - goal0[2])))
			# dist_obs0.append(np.sqrt((state0[0] - obsx[0]) ** 2 + (state0[1] - obsy[0]) ** 2) - r)
			# dist_obs1.append(np.sqrt((state0[0] - obsx[1]) ** 2 + (state0[1] - obsy[1]) ** 2) - r)
			# dist_obs2.append(np.sqrt((state0[0] - obsx[2]) ** 2 + (state0[1] - obsy[2]) ** 2) - r)
			# dist_obs3.append(np.sqrt((state0[0] - obsx[3]) ** 2 + (state0[1] - obsy[3]) ** 2) - r)
			# dist_obs4.append(np.sqrt((state0[0] - obsx[4]) ** 2 + (state0[1] - obsy[4]) ** 2) - r)
			# print(round(simulation_psi_residue0[-1], 2), round(simulation_x_residue0[-1], 2), round(simulation_y_residue0[-1], 2))
			dist_01.append(abs(dist(state0[0], state0[1], state1[0], state1[1]) - 0.7))
			dist_02.append(abs(dist(state0[0], state0[1], state2[0], state2[1]) - 0.7))
			dist_03.append(abs(dist(state0[0], state0[1], state3[0], state3[1]) - 0.7))
			dist_12.append(abs(dist(state1[0], state1[1], state2[0], state2[1]) - 0.7))
			if simulation_x_residue0[-1] < 0.01 and simulation_y_residue0[-1] < 0.01 and simulation_psi_residue0[-1] < 0.01:
				break
			if time() - t >= 30:
				break
			plt.clf()
			ax1 = fig.add_subplot(1, 1, 1)
			ax1.set_aspect(1)
			
			trajx0.append(state0[0])
			trajy0.append(state0[1])
			trajx1.append(state1[0])
			trajy1.append(state1[1])
			trajx2.append(state2[0])
			trajy2.append(state2[1])
			plt.plot(trajx0, trajy0, color='red')
			plt.plot(trajx1, trajy1, color='blue')
			plt.plot(trajx2, trajy2, color='green')
			circle5 = (plt.Circle((state0[0], state0[1]), 0.32, color='orange', fill=False))
			circle6 = (plt.Circle((state1[0], state1[1]), 0.32, color='orange', fill=False))
			circle7 = (plt.Circle((state2[0], state2[1]), 0.32, color='orange', fill=False))

			circle0 = (plt.Circle((obsx[0], obsy[0]), r, color='orange', fill=False))
			circle1 = (plt.Circle((obsx[1], obsy[1]), r, color='orange', fill=False))
			circle2 = (plt.Circle((obsx[2], obsy[2]), r, color='orange', fill=False))
			circle3 = (plt.Circle((obsx[3], obsy[3]), r, color='orange', fill=False))
			circle4 = (plt.Circle((obsx[4], obsy[4]), r, color='orange', fill=False))
			
			for i in range(len(shelfx)):
				plt.plot([shelfx[i] + shelfa[i]/2, shelfx[i] + shelfa[i]/2, shelfx[i] - shelfa[i]/2, shelfx[i] - shelfa[i]/2, shelfx[i] + shelfa[i]/2], [shelfy[i] + shelfb[i]/2, shelfy[i]-shelfb[i]/2, shelfy[i]-shelfb[i]/2, shelfy[i]+shelfb[i]/2, shelfy[i]+shelfb[i]/2])
			

			# circle5 = (plt.Circle((obsx[0], obsy[0]), 0.2, color='orange'))
			# circle6 = (plt.Circle((obsx[1], obsy[1]), 0.2, color='orange'))
			# circle7 = (plt.Circle((obsx[2], obsy[2]), 0.2, color='orange'))
			# circle8 = (plt.Circle((obsx[3], obsy[3]), 0.2, color='orange'))
			# circle9 = (plt.Circle((obsx[4], obsy[4]), 0.2, color='orange'))

			# ax1.add_artist(circle0)
			# ax1.add_artist(circle1)
			# ax1.add_artist(circle2)
			# ax1.add_artist(circle3)
			# ax1.add_artist(circle4)
			ax1.add_artist(circle5)
			ax1.add_artist(circle6)
			ax1.add_artist(circle7)
			# ax1.add_artist(circle8)
			# ax1.add_artist(circle9)
				
			plt.xlim(-10, 10)
			plt.ylim(-10, 10)
			plt.scatter(state0[0], state0[1], linewidths=0.05, color='red')
			plt.scatter(state1[0], state1[1], linewidths=0.05, color='blue')
			plt.scatter(state2[0], state2[1], linewidths=0.05, color='green')
			plt.scatter(goal0[0], goal0[1], linewidths=0.05, color='red')
			plt.scatter(goal1[0], goal1[1], linewidths=0.05, color='blue')
			plt.scatter(goal2[0], goal2[1], linewidths=0.05, color='green')
			plt.xlabel('x')
			plt.ylabel('y')
			plt.plot(states_x0, states_y0, linestyle=':',color='black')
			plt.plot(states_x1, states_y1, linestyle=':',color='black')
			plt.plot(states_x2, states_y2, linestyle=':',color='black')
			#plt.title('Obstacle avoidance')
			plt.draw()
			plt.pause(0.000001)"""

		if rx0 >= 5 and rx1 >= 5 and rx2 >= 5 and rx3 >= 5:
			config_matrix = [[0, l, 2*l/np.sqrt(2), l], [l, 0, l, 2*l/np.sqrt(2)], [2*l/np.sqrt(2), l, 0, l], [l, 2*l/np.sqrt(2), l, 0]]
			plt.clf()
			ax1.set_aspect(1)
			iter+=1
			simulation_time.append(iter * 1/freq)
			dist_goal = np.sqrt((state1[0] - (goal1[0]))**2 + (state1[1] - goal1[1])**2)
			dist_01.append(abs(dist(state0[0], state0[1], state1[0], state1[1]) - config_matrix[0][1]))
			dist_02.append(abs(dist(state0[0], state0[1], state2[0], state2[1]) - config_matrix[0][2]))
			dist_03.append(abs(dist(state0[0], state0[1], state3[0], state3[1]) - config_matrix[0][3]))
			dist_12.append(abs(dist(state1[0], state1[1], state2[0], state2[1]) - config_matrix[1][2]))
			dist_13.append(abs(dist(state1[0], state1[1], state3[0], state3[1]) - config_matrix[1][3]))
			dist_23.append(abs(dist(state2[0], state2[1], state3[0], state3[1]) - config_matrix[2][3]))
			
			d1.append(abs(dist(state0[0], state0[1], state1[0], state1[1])))
			d2.append(abs(dist(state0[0], state0[1], state2[0], state2[1])))
			d3.append(abs(dist(state0[0], state0[1], state3[0], state3[1])))
			d4.append(abs(dist(state1[0], state1[1], state2[0], state2[1])))
			d5.append(abs(dist(state1[0], state1[1], state3[0], state3[1])))
			d6.append(abs(dist(state2[0], state2[1], state3[0], state3[1])))

			plt.plot([state0[0], state1[0], state2[0], state3[0], state0[0]], [state0[1], state1[1], state2[1], state3[1], state0[1]], linestyle ='dotted')
			for i in range(len(squarex)):
				plt.plot(squarex[i], squarey[i], color='blue')
			if iter % 5 == 0:
				squarex.append([state0[0], state1[0], state2[0], state3[0], state0[0]])
				squarey.append([state0[1], state1[1], state2[1], state3[1], state0[1]])
			
			plt.scatter(state0[0], state0[1], linewidths=0.05, color='red')
			plt.scatter(state1[0], state1[1], linewidths=0.05, color='green')
			plt.scatter(state2[0], state2[1], linewidths=0.05, color='blue')
			plt.scatter(state3[0], state3[1], linewidths=0.05, color='orange')
			# plt.scatter(goal1[0]+2, goal1[1], linewidths=0.05, color='black')
			for i in range(len(shelfx)*0):
				plt.plot([shelfx[i] + shelfa[i]/2, shelfx[i] + shelfa[i]/2, shelfx[i] - shelfa[i]/2, shelfx[i] - shelfa[i]/2, shelfx[i] + shelfa[i]/2], [shelfy[i] + shelfb[i]/2, shelfy[i]-shelfb[i]/2, shelfy[i]-shelfb[i]/2, shelfy[i]+shelfb[i]/2, shelfy[i]+shelfb[i]/2])
			
			ax1.set_aspect(1)
			plt.plot(states_x0, states_y0, linestyle=':',color='black')
			plt.plot(states_x1, states_y1, linestyle=':',color='black')
			plt.plot(states_x2, states_y2, linestyle=':',color='black')
			plt.plot(states_x3, states_y3, linestyle=':',color='black')
			
			if(dist_goal < 0.1):
				break
			plt.xlabel('x')
			plt.ylabel('y')
			plt.xlim(-5, 5)
			plt.ylim(-2, 13)
			plt.draw()
			plt.pause(0.000001)
			# print(round(dist_01[-1], 3), round(dist_02[-1], 3), round(dist_03[-1], 3), round(dist_12[-1], 3), round(dist_13[-1], 3), round(dist_23[-1], 3))
		"""if rx0 >= 5 and rx1 >= 5 and rx2 >=5 and rx3 >=5:
			plt.clf()
			ax1 = fig.add_subplot(1, 1, 1)
			ax1.set_aspect(1)
			r = 0.2 * np.sqrt(2)/ 2+0.32
			simulation_time.append(iter * 1/freq)
			simulation_v0.append(v0)
			simulation_psidot0.append(wz0)
			simulation_x_residue0.append((abs(state0[0] - goal0[0])))
			simulation_y_residue0.append((abs(state0[1] - goal0[1])))
			simulation_psi_residue0.append((abs(state0[2] - goal0[2])))
			simulation_v1.append(v1)
			simulation_psidot1.append(wz1)
			simulation_x_residue1.append((abs(state1[0] - goal1[0])))
			simulation_y_residue1.append((abs(state1[1] - goal1[1])))
			simulation_psi_residue1.append((abs(state1[2] - goal1[2])))
			simulation_v2.append(v2)
			simulation_psidot2.append(wz2)
			simulation_x_residue2.append((abs(state2[0] - goal2[0])))
			simulation_y_residue2.append((abs(state2[1] - goal2[1])))
			simulation_psi_residue2.append((abs(state2[2] - goal2[2])))
			simulation_v3.append(v3)
			simulation_psidot3.append(wz3)
			simulation_x_residue3.append((abs(state3[0] - goal3[0])))
			simulation_y_residue3.append((abs(state3[1] - goal3[1])))
			simulation_psi_residue3.append((abs(state3[2] - goal3[2])))
			dist0_obs0.append(np.sqrt((state0[0] - obsx[0]) ** 2 + (state0[1] - obsy[0]) ** 2) - r)
			dist0_obs1.append(np.sqrt((state0[0] - obsx[1]) ** 2 + (state0[1] - obsy[1]) ** 2) - r)
			dist0_obs2.append(np.sqrt((state0[0] - obsx[2]) ** 2 + (state0[1] - obsy[2]) ** 2) - r)
			dist0_obs3.append(np.sqrt((state0[0] - obsx[3]) ** 2 + (state0[1] - obsy[3]) ** 2) - r)
			dist0_obs4.append(np.sqrt((state0[0] - obsx[4]) ** 2 + (state0[1] - obsy[4]) ** 2) - r)

			dist1_obs0.append(np.sqrt((state1[0] - obsx[0]) ** 2 + (state1[1] - obsy[0]) ** 2) - r)
			dist1_obs1.append(np.sqrt((state1[0] - obsx[1]) ** 2 + (state1[1] - obsy[1]) ** 2) - r)
			dist1_obs2.append(np.sqrt((state1[0] - obsx[2]) ** 2 + (state1[1] - obsy[2]) ** 2) - r)
			dist1_obs3.append(np.sqrt((state1[0] - obsx[3]) ** 2 + (state1[1] - obsy[3]) ** 2) - r)
			dist1_obs4.append(np.sqrt((state1[0] - obsx[4]) ** 2 + (state1[1] - obsy[4]) ** 2) - r)
			
			dist2_obs0.append(np.sqrt((state2[0] - obsx[0]) ** 2 + (state2[1] - obsy[0]) ** 2) - r)
			dist2_obs1.append(np.sqrt((state2[0] - obsx[1]) ** 2 + (state2[1] - obsy[1]) ** 2) - r)
			dist2_obs2.append(np.sqrt((state2[0] - obsx[2]) ** 2 + (state2[1] - obsy[2]) ** 2) - r)
			dist2_obs3.append(np.sqrt((state2[0] - obsx[3]) ** 2 + (state2[1] - obsy[3]) ** 2) - r)
			dist2_obs4.append(np.sqrt((state2[0] - obsx[4]) ** 2 + (state2[1] - obsy[4]) ** 2) - r)

			dist3_obs0.append(np.sqrt((state3[0] - obsx[0]) ** 2 + (state3[1] - obsy[0]) ** 2) - r)
			dist3_obs1.append(np.sqrt((state3[0] - obsx[1]) ** 2 + (state3[1] - obsy[1]) ** 2) - r)
			dist3_obs2.append(np.sqrt((state3[0] - obsx[2]) ** 2 + (state3[1] - obsy[2]) ** 2) - r)
			dist3_obs3.append(np.sqrt((state3[0] - obsx[3]) ** 2 + (state3[1] - obsy[3]) ** 2) - r)
			dist3_obs4.append(np.sqrt((state3[0] - obsx[4]) ** 2 + (state3[1] - obsy[4]) ** 2) - r)

			dist_01.append(abs(dist(state0[0], state0[1], state1[0], state1[1]) - 0.7))
			dist_02.append(abs(dist(state0[0], state0[1], state2[0], state2[1]) - 0.7))
			dist_03.append(abs(dist(state0[0], state0[1], state3[0], state3[1]) - 0.7))
			dist_12.append(abs(dist(state1[0], state1[1], state2[0], state2[1]) - 0.7))
			dist_13.append(abs(dist(state1[0], state1[1], state3[0], state3[1]) - 0.7))
			dist_23.append(abs(dist(state2[0], state2[1], state3[0], state3[1]) - 0.7))
			iter += 1
			if simulation_x_residue0[-1] < 0.01 and simulation_y_residue0[-1] < 0.01 and simulation_psi_residue0[-1] < 0.01	and simulation_x_residue1[-1] < 0.01 and simulation_y_residue1[-1] < 0.01 and simulation_psi_residue1[-1] < 0.01 and	simulation_x_residue2[-1] < 0.01 and simulation_y_residue2[-1] < 0.01 and simulation_psi_residue2[-1] < 0.01  and simulation_x_residue3[-1] < 0.01 and simulation_y_residue3[-1] < 0.01 and simulation_psi_residue3[-1] < 0.01:
				break
			if time() - t >= 240:
				break
			circle5 = (plt.Circle((obsx[0], obsy[0]), 0.2*np.sqrt(2)/2, color='black'))
			circle6 = (plt.Circle((obsx[1], obsy[1]), 0.2*np.sqrt(2)/2, color='black'))
			circle7 = (plt.Circle((obsx[2], obsy[2]), 0.2*np.sqrt(2)/2, color='black'))
			circle8 = (plt.Circle((obsx[3], obsy[3]), 0.2*np.sqrt(2)/2, color='black'))
			circle9 = (plt.Circle((obsx[4], obsy[4]), 0.2*np.sqrt(2)/2, color='black'))

			circle0 = (plt.Circle((obsx[0], obsy[0]), 0.2*np.sqrt(2)/2 + 0.25, color='black', fill=False, linestyle='dotted'))
			circle1 = (plt.Circle((obsx[1], obsy[1]), 0.2*np.sqrt(2)/2 + 0.25, color='black', fill=False, linestyle='dotted'))
			circle2 = (plt.Circle((obsx[2], obsy[2]), 0.2*np.sqrt(2)/2 + 0.25, color='black', fill=False, linestyle='dotted'))
			circle3 = (plt.Circle((obsx[3], obsy[3]), 0.2*np.sqrt(2)/2 + 0.25, color='black', fill=False, linestyle='dotted'))
			circle4 = (plt.Circle((obsx[4], obsy[4]), 0.2*np.sqrt(2)/2 + 0.25, color='black', fill=False, linestyle='dotted'))

			circle10 = (plt.Circle((state0[0], state0[1]), 0.35, color='red', fill=False))
			circle11 = (plt.Circle((state1[0], state1[1]), 0.35, color='green', fill=False))
			circle12 = (plt.Circle((state2[0], state2[1]), 0.35, color='blue', fill=False))
			circle13 = (plt.Circle((state3[0], state3[1]), 0.35, color='orange', fill=False))

			circle14 = (plt.Circle((state0[0], state0[1]), 0.35 + 0.5, color='red', fill=False, linestyle='dotted'))
			circle15 = (plt.Circle((state1[0], state1[1]), 0.35 + 0.5, color='green', fill=False, linestyle='dotted'))
			circle16 = (plt.Circle((state2[0], state2[1]), 0.35 + 0.5, color='blue', fill=False, linestyle='dotted'))
			circle17 = (plt.Circle((state3[0], state3[1]), 0.35 + 0.5, color='orange', fill=False, linestyle='dotted'))

			# ax1.add_artist(circle0)
			# ax1.add_artist(circle1)
			# ax1.add_artist(circle2)
			# ax1.add_artist(circle3)
			# ax1.add_artist(circle4)
			ax1.add_artist(circle5)
			ax1.add_artist(circle6)
			ax1.add_artist(circle7)
			ax1.add_artist(circle8)
			ax1.add_artist(circle9)
			ax1.add_artist(circle10)
			ax1.add_artist(circle11)
			ax1.add_artist(circle12)
			ax1.add_artist(circle13)
			# ax1.add_artist(circle14)
			# ax1.add_artist(circle15)
			# ax1.add_artist(circle16)
			# ax1.add_artist(circle17)

			trajx0.append(state0[0])
			trajy0.append(state0[1])
			trajx1.append(state1[0])
			trajy1.append(state1[1])
			trajx2.append(state2[0])
			trajy2.append(state2[1])
			trajx3.append(state3[0])
			trajy3.append(state3[1])

			plt.plot(states_x0, states_y0, linestyle=':',color='black')
			plt.plot(states_x1, states_y1, linestyle=':',color='black')
			plt.plot(states_x2, states_y2, linestyle=':',color='black')
			plt.plot(states_x3, states_y3, linestyle=':',color='black')
			plt.scatter(goal0[0], goal0[1], linewidths=0.05, color='red')
			plt.scatter(goal1[0], goal1[1], linewidths=0.05, color='green')
			plt.scatter(goal2[0], goal2[1], linewidths=0.05, color='blue')
			plt.scatter(goal3[0], goal3[1], linewidths=0.05, color='orange')
			plt.plot(trajx0, trajy0, color='red')
			plt.plot(trajx1, trajy1, color='green')
			plt.plot(trajx2, trajy2, color='blue')
			plt.plot(trajx3, trajy3, color='orange')
			
			plt.xlim(-5, 4)
			plt.ylim(-5, 4)
			plt.title('Goal Reach + Obstacle avoidance')
			plt.draw()
			plt.pause(0.001)"""

		rate.sleep()
	v_max = 1
	psidot_max = 5

	l = 1.5*1.2
	config_matrix0 = [[0, l, 2*l/np.sqrt(2), l], [l, 0, l, 2*l/np.sqrt(2)], [2*l/np.sqrt(2), l, 0, l], [l, 2*l/np.sqrt(2), l, 0]]

	l = 1.5*0.3
	config_matrix1 = [[0, l, 2*l/np.sqrt(2), l], [l, 0, l, 2*l/np.sqrt(2)], [2*l/np.sqrt(2), l, 0, l], [l, 2*l/np.sqrt(2), l, 0]]

	plt.figure(10)
	plt.plot(simulation_time, d1)
	plt.plot([simulation_time[-1], 0], [config_matrix0[0][1], config_matrix0[0][1]])
	plt.plot([simulation_time[-1], 0], [config_matrix1[0][1], config_matrix1[0][1]])
	plt.legend(["dist_01", "max_deform", "min_deform"])

	plt.figure(11)
	plt.plot(simulation_time, d2)
	plt.plot([simulation_time[-1], 0], [config_matrix0[0][2], config_matrix0[0][2]])
	plt.plot([simulation_time[-1], 0], [config_matrix1[0][2], config_matrix1[0][2]])
	plt.legend(["dist_02", "max_deform", "min_deform"])

	plt.figure(12)
	plt.plot(simulation_time, d3)
	plt.plot([simulation_time[-1], 0], [config_matrix0[0][3], config_matrix0[0][3]])
	plt.plot([simulation_time[-1], 0], [config_matrix1[0][3], config_matrix1[0][3]])
	plt.legend(["dist_03", "max_deform", "min_deform"])

	plt.figure(13)
	plt.plot(simulation_time, d4)
	plt.plot([simulation_time[-1], 0], [config_matrix0[1][2], config_matrix0[1][2]])
	plt.plot([simulation_time[-1], 0], [config_matrix1[1][2], config_matrix1[1][2]])
	plt.legend(["dist_12", "max_deform", "min_deform"])

	plt.figure(14)
	plt.plot(simulation_time, d5)
	plt.plot([simulation_time[-1], 0], [config_matrix0[1][3], config_matrix0[1][3]])
	plt.plot([simulation_time[-1], 0], [config_matrix1[1][3], config_matrix1[1][3]])
	plt.legend(["dist_13", "max_deform", "min_deform"])

	plt.figure(15)
	plt.plot(simulation_time, d6)
	plt.plot([simulation_time[-1], 0], [config_matrix0[2][3], config_matrix0[2][3]])
	plt.plot([simulation_time[-1], 0], [config_matrix1[2][3], config_matrix1[2][3]])
	plt.legend(["dist_23", "max_deform", "min_deform"])

	# print(round(np.mean(dist_01), 3), round(np.mean(dist_02), 3), round(np.mean(dist_03), 3), round(np.mean(dist_12), 3), round(np.mean(dist_13), 3), round(np.mean(dist_23), 3))
	# plt.figure(5)
	# plt.title('Distance_robots')
	# plt.plot(simulation_time, dist_01)
	# plt.plot(simulation_time, dist_02)
	# plt.plot(simulation_time, dist_03)
	# plt.plot(simulation_time, dist_12)
	# plt.plot([simulation_time[-1], 0], [0, 0])
	# plt.plot(simulation_time, dist_13)
	# plt.plot(simulation_time, dist_23)
	# plt.legend(["dist_01", "dist_02","dist_03", "dist_12", "dist_13", "dist_23"])
	

	# plt.figure(5)
	# plt.title('residue_goal')
	# plt.plot(simulation_time, simulation_x_residue0)
	# plt.plot(simulation_time, simulation_y_residue0)
	# # plt.plot(simulation_time, simulation_x_residue1)
	# # plt.plot(simulation_time, simulation_y_residue1)
	# # plt.plot(simulation_time, simulation_x_residue2)
	# # plt.plot(simulation_time, simulation_y_residue2)
	# # plt.plot(simulation_time, simulation_x_residue3)
	# # plt.plot(simulation_time, simulation_y_residue3)
	# plt.legend(["res_x0", "res_y0", "res_x1", "res_y1", "res_x2", "res_y2", "res_x3", "res_y3"], loc ="upper right")

	# plt.figure(2)
	# plt.title('v')
	# plt.plot(simulation_time, simulation_v0)
	# # plt.plot(simulation_time, simulation_v1)
	# # plt.plot(simulation_time, simulation_v2)
	# # plt.plot(simulation_time, simulation_v3)
	# plt.plot([0, simulation_time[-1]], [v_max, v_max])
	# plt.plot([0, simulation_time[-1]], [-v_max, -v_max])
	# plt.legend(["linear_velocity", "max_bounds", "min_bounds"], loc ="upper right")

	# plt.figure(3)
	# plt.title('psidot')
	# plt.plot(simulation_time, simulation_psidot0)
	# # plt.plot(simulation_time, simulation_psidot1)
	# # plt.plot(simulation_time, simulation_psidot2)
	# # plt.plot(simulation_time, simulation_psidot3)
	# plt.plot([0, simulation_time[-1]], [psidot_max, psidot_max])
	# plt.plot([0, simulation_time[-1]], [-psidot_max, -psidot_max])
	# plt.legend(["angular_velocity", "max_bounds", "min_bounds"], loc ="upper right")

	# plt.figure(4)
	# plt.title('res_psi')
	# plt.plot(simulation_time, simulation_psi_residue0)
	# # plt.plot(simulation_time, simulation_psi_residue1)
	# # plt.plot(simulation_time, simulation_psi_residue2)
	# # plt.plot(simulation_time, simulation_psi_residue3)
	# plt.legend(["res_psi0", "res_psi1", "res_psi2", "res_psi3"], loc="upper right")

	# plt.figure(9)
	# plt.title('Dist to Obstacles0')
	# plt.plot(simulation_time, dist0_obs0)
	# plt.plot(simulation_time, dist0_obs1)
	# plt.plot(simulation_time, dist0_obs2)
	# plt.plot(simulation_time, dist0_obs3)
	# plt.plot(simulation_time, dist0_obs4)
	# plt.plot([0, simulation_time[-1]], [0, 0])
	# plt.legend(["dist_obs0", "dist_obs1", "dist_obs2", "dist_obs3", "dist_obs4"])	

	# plt.figure(10)
	# plt.title('Dist to Obstacles1')
	# plt.plot(simulation_time, dist1_obs0)
	# plt.plot(simulation_time, dist1_obs1)
	# plt.plot(simulation_time, dist1_obs2)
	# plt.plot(simulation_time, dist1_obs3)
	# plt.plot(simulation_time, dist1_obs4)
	# plt.plot([0, simulation_time[-1]], [0, 0])
	# plt.legend(["dist_obs0", "dist_obs1", "dist_obs2", "dist_obs3", "dist_obs4"])

	# plt.figure(11)
	# plt.title('Dist to Obstacles2')
	# plt.plot(simulation_time, dist2_obs0)
	# plt.plot(simulation_time, dist2_obs1)
	# plt.plot(simulation_time, dist2_obs2)
	# plt.plot(simulation_time, dist2_obs3)
	# plt.plot(simulation_time, dist2_obs4)
	# plt.plot([0, simulation_time[-1]], [0, 0])
	# plt.legend(["dist_obs0", "dist_obs1", "dist_obs2", "dist_obs3", "dist_obs4"])

	# plt.figure(12)
	# plt.title('Dist to Obstacles3')
	# plt.plot(simulation_time, dist3_obs0)
	# plt.plot(simulation_time, dist3_obs1)
	# plt.plot(simulation_time, dist3_obs2)
	# plt.plot(simulation_time, dist3_obs3)
	# plt.plot(simulation_time, dist3_obs4)
	# plt.plot([0, simulation_time[-1]], [0, 0])
	# plt.legend(["dist_obs0", "dist_obs1", "dist_obs2", "dist_obs3", "dist_obs4"])
	plt.xlabel('time')
	plt.ylabel('value')
	np.savetxt("/home/vivek/catkin_ws/src/multi_robot_mpc/results/dist_01.txt", np.asarray(dist_01), delimiter='\n')
	np.savetxt("/home/vivek/catkin_ws/src/multi_robot_mpc/results/dist_02.txt", np.asarray(dist_02), delimiter='\n')
	np.savetxt("/home/vivek/catkin_ws/src/multi_robot_mpc/results/dist_03.txt", np.asarray(dist_03), delimiter='\n')
	np.savetxt("/home/vivek/catkin_ws/src/multi_robot_mpc/results/dist_12.txt", np.asarray(dist_12), delimiter='\n')
	np.savetxt("/home/vivek/catkin_ws/src/multi_robot_mpc/results/dist_13.txt", np.asarray(dist_13), delimiter='\n')
	np.savetxt("/home/vivek/catkin_ws/src/multi_robot_mpc/results/dist_23.txt", np.asarray(dist_23), delimiter='\n')
	plt.show()
	rospy.spin()

