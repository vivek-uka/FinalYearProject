#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3
import autograd.numpy as np
from autograd import grad
from time import time
from multi_robot_mpc.msg import States
import matplotlib.pyplot as plt


state = [0.5, 0.866025, 1.57]
states_x0 = []
states_y0 = []
states_psi0 = []

states_x2 = []
states_y2 = []
states_psi2 = []

states_x3 = []
states_y3 = []
states_psi3 = []

rx0 = 0
rx1 = 5
rx2 = 0
rx3 = 0

psidot_optimal = 0
v_optimal = 0
class ModelPredictiveControl:

	def __init__(self, x_g, y_g, psi_g, angular_max, linear_max):

		self.horizon = 5
		self.control = 1
		self.dt = 0.5
		self.psidot_max = angular_max
		self.v_max = linear_max
		self.goal = [x_g, y_g]	
		self.pre_states = States()
		self.psi_terminal = psi_g
		self.pub2 = rospy.Publisher('tb3_1/pre_state', States, queue_size=10)
		self.stop = 1
		self.job = 1 # follower = 0 leader = 1
		l = 1 #square config
		self.config_matrix = [[0, l/np.sqrt(2), 2*l/np.sqrt(2), l/np.sqrt(2)], [l/np.sqrt(2), 0, l/np.sqrt(2), 2*l/np.sqrt(2)], [2*l/np.sqrt(2), l/np.sqrt(2), 0, l/np.sqrt(2)], [l/np.sqrt(2), 2*l/np.sqrt(2), l/np.sqrt(2), 0]]

	def optimize(self, state, u, mode,steps=25, lr=0.001, decay=0.9, eps=1e-8):

		dx_mean_sqr = np.zeros(self.horizon*2)

		if mode == "solo":
			dF = grad(self.cost_goal_only)
		elif mode == "multi":
			dF = grad(self.cost_maintain_config) 

		startTime = time()
		for k in range(steps):
		    dx = dF(u, state)
		    dx_mean_sqr = decay * dx_mean_sqr + (1.0 - decay) * dx ** 2
		    if k != steps - 1:
				u -= lr * dx / (np.sqrt(dx_mean_sqr) + eps)
				
		#print("Optimization Time = ", time()-startTime)
		self.pub2.publish(self.pre_states)
		return u

	def cost_maintain_config(self, u, state): 

		psi = [state[2] + u[self.horizon] * self.dt]
		rn = [state[0] + u[0] * np.cos(psi[0]) * self.dt]
		re = [state[1] + u[0] * np.sin(psi[0]) * self.dt]

		for i in range(1, self.horizon):
			psi.append(psi[i-1] + u[self.horizon + i] * self.dt)
			rn.append(rn[i-1] + u[i] * np.cos(psi[i]) * self.dt)
			re.append(re[i-1] + u[i] * np.sin(psi[i]) * self.dt)
		rn = np.array(rn)
		re = np.array(re)
		psi = np.array(psi)

		self.pre_states.x = rn._value
		self.pre_states.y = re._value
		self.pre_states.psi = psi._value
		self.pre_states.x0 = state[0]
		self.pre_states.y0 = state[1]
		self.pre_states.psi0 = state[2]
		
		lamda_1 = np.maximum(np.zeros(self.horizon), -0.0*self.v_max - u[:self.horizon]) + np.maximum(np.zeros(self.horizon), u[:self.horizon] - self.v_max) 
		lamda_2 = np.maximum(np.zeros(self.horizon), -self.psidot_max - u[self.horizon:]) + np.maximum(np.zeros(self.horizon), u[self.horizon:] - self.psidot_max) 
		cost_xy = (rn - self.goal[0]) ** 2 + (re - self.goal[1]) ** 2 
		
		cost_dist = (np.sqrt((states_x0 - rn) ** 2 + (states_y0 - re) ** 2) - self.config_matrix[0][1]) ** 2 + (np.sqrt((states_x2 - rn) ** 2 + (states_y2 - re) ** 2) - self.config_matrix[1][2]) ** 2 + (np.sqrt((states_x3 - rn) ** 2 + (states_y3 - re) ** 2) - self.config_matrix[1][3]) ** 2
		
		cost_ = 100 * lamda_1 + 100 * lamda_2 + 50 * cost_dist + 10 * self.job * cost_xy
		
		cost = np.sum(cost_) 
		
		return cost

	def cost_goal_only(self, u, state):

		psi = [state[2] + u[self.horizon] * self.dt]
		rn = [state[0] + u[0] * np.cos(psi[0]) * self.dt]
		re = [state[1] + u[0] * np.sin(psi[0]) * self.dt]

		for i in range(1, self.horizon):
			psi.append(psi[i-1] + u[self.horizon + i] * self.dt)
			rn.append(rn[i-1] + u[i] * np.cos(psi[i]) * self.dt)
			re.append(re[i-1] + u[i] * np.sin(psi[i]) * self.dt)
		rn = np.array(rn)
		re = np.array(re)
		psi = np.array(psi)

		self.pre_states.x = rn._value
		self.pre_states.y = re._value
		self.pre_states.psi = psi._value
		self.pre_states.x0 = state[0]
		self.pre_states.y0 = state[1]
		self.pre_states.psi0 = state[2]

		lamda_1 = np.maximum(np.zeros(self.horizon), -self.v_max - u[:self.horizon]) + np.maximum(np.zeros(self.horizon), u[:self.horizon] - self.v_max) 
		lamda_2 = np.maximum(np.zeros(self.horizon), -self.psidot_max - u[self.horizon:]) + np.maximum(np.zeros(self.horizon), u[self.horizon:] - self.psidot_max) 
		cost_xy = (rn - self.goal[0]) ** 2 + (re - self.goal[1]) ** 2 	
		cost_psi = (psi - self.psi_terminal) ** 2

		cost_ = 100 * lamda_1 + 100 * lamda_2 + 10 * cost_xy + 0.02 * cost_psi

		cost = np.sum(cost_) 

		return cost


def statesCallback0(data):
	global states_x0, states_y0, states_psi0, rx0

	states_x0 = data.x
	states_y0 = data.y
	states_psi0 = data.psi
	rx0 = 1
	

def statesCallback2(data):
	global states_x2, states_y2, states_psi2, rx2

	states_x2 = data.x
	states_y2 = data.y
	states_psi2 = data.psi
	rx2 = 1
	

def statesCallback3(data):
	global states_x3, states_y3, states_psi3, rx3

	states_x3 = data.x
	states_y3 = data.y
	states_psi3 = data.psi
	rx3 = 1

def odomCallback(data):
	global rx1, state, v_optimal, psidot_optimal

	x = data.pose.pose.position.x
	y = data.pose.pose.position.y

	vx = data.twist.twist.linear.x
	vy = data.twist.twist.linear.y

	wz = data.twist.twist.angular.z

	qx = data.pose.pose.orientation.x
	qy = data.pose.pose.orientation.y
	qz = data.pose.pose.orientation.z
	qw = data.pose.pose.orientation.w

	siny_cosp = 2 * (qw * qz + qx * qy)
	cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
	psi = np.arctan2(siny_cosp, cosy_cosp)

	state[0] = x
	state[1] = y
	state[2] = psi
	
	

	if rx1 == 5:
		rx1 = 1

if __name__ == '__main__':
	
	freq = 10
	rospy.init_node('my_robot1', anonymous='True')	
	rospy.Subscriber('tb3_1/odom', Odometry, odomCallback)

	rospy.Subscriber('tb3_0/pre_state', States, statesCallback0)
	rospy.Subscriber('tb3_2/pre_state', States, statesCallback2)
	rospy.Subscriber('tb3_3/pre_state', States, statesCallback3)

	pub = rospy.Publisher('tb3_1/cmd_vel', Twist, queue_size=10)
	

	rate = rospy.Rate(freq)

	myRobot = ModelPredictiveControl(-5, -1.0, 0.0, 2.84, 0.22)
	u = np.zeros(2*myRobot.horizon)
		
	mode = "multi"
	while not rospy.is_shutdown():
		dist_goal = np.sqrt((state[0] - myRobot.goal[0]) ** 2 + (state[1] - myRobot.goal[1]) ** 2)

		if rx1 == 1.0:
			myRobot.pre_states.x = np.full(myRobot.horizon, state[0])
			myRobot.pre_states.y = np.full(myRobot.horizon, state[1])
			myRobot.pre_states.psi = np.full(myRobot.horizon, state[2])
			myRobot.pub2.publish(myRobot.pre_states)
			
		
		if (rx0 and rx2 and rx3) or mode == "solo":
			rx1 = 0.0				
			u = myRobot.optimize(state, u, mode)			
			pub.publish(Twist(Vector3(u[0], 0, 0),Vector3(0, 0, u[myRobot.horizon])))
		
		rate.sleep()

	
	rospy.spin()
