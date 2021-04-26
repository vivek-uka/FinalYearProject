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

v0 = wz0 = v1 = wz1 = v2 = wz2 = v3 = wz3 = 0.0
goal0 = [0.0, 5, 3*np.pi/4]
goal1 = [0.27, -2.2, 0]
goal2= [-2.4, -2.2, -np.pi/4]
goal3 = [-2.4, 0.56, -np.pi/4]

obsx = [-1.7, -0.36, -1.7, -1, -0.36] #[-6, -6, -5, -5, -5.5] 
obsy = [-1.5, -0.36, -0.36, -0.8, -1.5] #[0.5, 1.5, 1.5, 0.5, 1]

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
	l = 0.5
	config_matrix = [[0, l/np.sqrt(2), 2*l/np.sqrt(2), l/np.sqrt(2)], 
					 [l/np.sqrt(2), 0, l/np.sqrt(2), 2*l/np.sqrt(2)], 
					 [2*l/np.sqrt(2), l/np.sqrt(2), 0, l/np.sqrt(2)], 
					 [l/np.sqrt(2), 2*l/np.sqrt(2), l/np.sqrt(2), 0]]
	while not rospy.is_shutdown():
		if rx0 >= 5:
			iter += 1
			dist_goal = np.sqrt((goal0[0] - state0[0]) ** 2 + (goal0[1] - state0[1]) ** 2)
			simulation_time.append(iter * 1/freq)
			simulation_v0.append(v0)
			simulation_psidot0.append(wz0)
			simulation_x_residue0.append((abs(state0[0] - goal0[0])))
			simulation_y_residue0.append((abs(state0[1] - goal0[1])))
			simulation_psi_residue0.append((abs(state0[2] - goal0[2])))
			
			if simulation_x_residue0[-1] < 0.01 and simulation_y_residue0[-1] < 0.01 and simulation_psi_residue0[-1] < 0.01:
				break
			if time() - t >= 20:
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
				

			plt.xlim(-2.5, 2.5)
			plt.ylim(-0.0, 7)
			plt.scatter(state0[0], state0[1], linewidths=0.05)
			plt.scatter(goal0[0], goal0[1], linewidths=0.05, color='green')
			plt.plot(states_x0, states_y0, linestyle=':',color='red')
			plt.title('Obstacle avoidance')
			plt.draw()
			plt.pause(0.000001)

		"""if rx0 >= 5 and rx1 >= 5 and rx2 >= 5 and rx3 >= 5:
			plt.clf()
			iter+=1
			simulation_time.append(iter * 1/freq)
			dist_goal = np.sqrt((state1[0] - (-4.6))**2 + (state1[1] - (-0.5))**2)
			dist_01.append(abs(dist(state0[0], state0[1], state1[0], state1[1]) - config_matrix[0][1]))
			dist_02.append(abs(dist(state0[0], state0[1], state2[0], state2[1]) - config_matrix[0][2]))
			dist_03.append(abs(dist(state0[0], state0[1], state3[0], state3[1]) - config_matrix[0][3]))
			dist_12.append(abs(dist(state1[0], state1[1], state2[0], state2[1]) - config_matrix[1][2]))
			dist_13.append(abs(dist(state1[0], state1[1], state3[0], state3[1]) - config_matrix[1][3]))
			dist_23.append(abs(dist(state2[0], state2[1], state3[0], state3[1]) - config_matrix[2][3]))

			plt.plot([state0[0], state1[0], state2[0], state3[0], state0[0]], [state0[1], state1[1], state2[1], state3[1], state0[1]], linestyle ='dotted')
			plt.scatter(state0[0], state0[1], linewidths=0.05, color='red')
			plt.scatter(state1[0], state1[1], linewidths=0.05, color='green')
			plt.scatter(state2[0], state2[1], linewidths=0.05, color='blue')
			plt.scatter(state3[0], state3[1], linewidths=0.05, color='orange')
			plt.scatter(-4.6, -0.5, linewidths=0.05, color='black')
			plt.plot(states_x0, states_y0, linestyle=':',color='black')
			plt.plot(states_x1, states_y1, linestyle=':',color='black')
			plt.plot(states_x2, states_y2, linestyle=':',color='black')
			plt.plot(states_x3, states_y3, linestyle=':',color='black')
			ax1.set_aspect(1)
			if(dist_goal < 0.05):
				break
			plt.xlabel('x')
			plt.ylabel('y')
			plt.xlim(-7, 0)
			plt.ylim(-4, 6)
			plt.draw()
			plt.pause(0.000001)
			print(round(dist_01[-1], 3), round(dist_02[-1], 3), round(dist_03[-1], 3), round(dist_12[-1], 3), round(dist_13[-1], 3), round(dist_23[-1], 3))"""
		"""if rx0 >= 5 and rx1 >= 5 and rx2 >=5 and rx3 >=5:
			plt.clf()
			ax1 = fig.add_subplot(1, 1, 1)
			ax1.set_aspect(1)
			
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

			iter += 1
			if simulation_x_residue0[-1] < 0.01 and simulation_y_residue0[-1] < 0.01 and simulation_psi_residue0[-1] < 0.01	and simulation_x_residue1[-1] < 0.01 and simulation_y_residue1[-1] < 0.01 and simulation_psi_residue1[-1] < 0.01 and	simulation_x_residue2[-1] < 0.01 and simulation_y_residue2[-1] < 0.01 and simulation_psi_residue2[-1] < 0.01  and simulation_x_residue3[-1] < 0.01 and simulation_y_residue3[-1] < 0.01 and simulation_psi_residue3[-1] < 0.01:
				break
			if time() - t >= 90:
				break
			circle5 = (plt.Circle((obsx[0], obsy[0]), 0.2*np.sqrt(2)/2, color='black'))
			circle6 = (plt.Circle((obsx[1], obsy[1]), 0.2*np.sqrt(2)/2, color='black'))
			circle7 = (plt.Circle((obsx[2], obsy[2]), 0.2*np.sqrt(2)/2, color='black'))
			circle8 = (plt.Circle((obsx[3], obsy[3]), 0.2*np.sqrt(2)/2, color='black'))
			circle9 = (plt.Circle((obsx[4], obsy[4]), 0.2*np.sqrt(2)/2, color='black'))

			circle10 = (plt.Circle((state0[0], state0[1]), 0.1, color='red', fill=False))
			circle11 = (plt.Circle((state1[0], state1[1]), 0.1, color='green', fill=False))
			circle12 = (plt.Circle((state2[0], state2[1]), 0.1, color='blue', fill=False))
			circle13 = (plt.Circle((state3[0], state3[1]), 0.1, color='orange', fill=False))

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
			
			plt.xlim(-2.5, 1)
			plt.ylim(-2.5, 1)
			plt.title('Goal Reach + Obstacle avoidance')
			plt.draw()
			plt.pause(0.001)"""

		rate.sleep()
	v_max = 1
	psidot_max = 5

	print(round(np.mean(dist_01), 3), round(np.mean(dist_02), 3), round(np.mean(dist_03), 3), round(np.mean(dist_12), 3), round(np.mean(dist_13), 3), round(np.mean(dist_23), 3))
	"""plt.figure(5)
	plt.title('distance_error')
	plt.plot(simulation_time, dist_01)
	plt.plot(simulation_time, dist_02)
	plt.plot(simulation_time, dist_03)
	plt.plot(simulation_time, dist_12)
	plt.plot(simulation_time, dist_13)
	plt.plot(simulation_time, dist_23)
	plt.legend(["dist_01", "dist_02", "dist_03", "dist_12", "dist_13", "dist_23"])
	plt.show()
	"""
	plt.figure(5)
	plt.title('residue_goal')
	plt.plot(simulation_time, simulation_x_residue0)
	plt.plot(simulation_time, simulation_y_residue0)
	# plt.plot(simulation_time, simulation_x_residue1)
	# plt.plot(simulation_time, simulation_y_residue1)
	# plt.plot(simulation_time, simulation_x_residue2)
	# plt.plot(simulation_time, simulation_y_residue2)
	# plt.plot(simulation_time, simulation_x_residue3)
	# plt.plot(simulation_time, simulation_y_residue3)
	plt.legend(["res_x0", "res_y0", "res_x1", "res_y1", "res_x2", "res_y2", "res_x3", "res_y3"], loc ="upper right")

	plt.figure(2)
	plt.title('v_controls')
	plt.plot(simulation_time, simulation_v0)
	# plt.plot(simulation_time, simulation_v1)
	# plt.plot(simulation_time, simulation_v2)
	# plt.plot(simulation_time, simulation_v3)
	plt.plot([0, simulation_time[-1]], [v_max, v_max])
	plt.plot([0, simulation_time[-1]], [-v_max, -v_max])
	plt.legend(["linear_velocity", "max_bounds", "min_bounds"], loc ="upper right")

	plt.figure(3)
	plt.title('psidot_controls')
	plt.plot(simulation_time, simulation_psidot0)
	# plt.plot(simulation_time, simulation_psidot1)
	# plt.plot(simulation_time, simulation_psidot2)
	# plt.plot(simulation_time, simulation_psidot3)
	plt.plot([0, simulation_time[-1]], [psidot_max, psidot_max])
	plt.plot([0, simulation_time[-1]], [-psidot_max, -psidot_max])
	plt.legend(["angular_velocity", "max_bounds", "min_bounds"], loc ="upper right")

	plt.figure(4)
	plt.title('res_psi')
	plt.plot(simulation_time, simulation_psi_residue0)
	# plt.plot(simulation_time, simulation_psi_residue1)
	# plt.plot(simulation_time, simulation_psi_residue2)
	# plt.plot(simulation_time, simulation_psi_residue3)
	plt.legend(["res_psi0", "res_psi1", "res_psi2", "res_psi3"], loc="upper right")

	plt.show()
	rospy.spin()

