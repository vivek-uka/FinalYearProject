import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3
import autograd.numpy as np
from autograd import grad
from time import time
from multi_robot_mpc.msg import States



state = [0, 0, 0]
states_x1 = []
states_y1 = []
states_psi1 = []

states_x2 = []
states_y2 = []
states_psi2 = []

states_x3 = []
states_y3 = []
states_psi3 = []

rx = 0



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
		self.pub2 = rospy.Publisher('tb3_0/pre_state', States, queue_size=10)
		self.stop = 1

	def optimize(self, state, u, steps=25, lr=0.001, decay=0.9, eps=1e-8):

		dx_mean_sqr = np.zeros(self.horizon*2)
		dF = grad(self.cost_function) 

		startTime = time()
		for k in range(steps):
		    dx = dF(u, state)
		    dx_mean_sqr = decay * dx_mean_sqr + (1.0 - decay) * dx ** 2
		    if k != steps - 1:
				u -= lr * dx / (np.sqrt(dx_mean_sqr) + eps)
				
		print("Optimization Time = ", time()-startTime)
		return u

	def cost_function(self, u, state): 
		
		psi = [state[2] + u[self.horizon] * self.dt]
		for i in range(1, self.horizon):
			psi.append(psi[i-1] + u[self.horizon + i] * self.dt)
		
		rn = state[0] + np.array([u[i] * np.cos(psi[i]) * self.dt for i in range(self.horizon)], dtype=float)
		re = state[1] + np.array([u[i] * np.sin(psi[i]) * self.dt for i in range(self.horizon)], dtype=float)
			
		self.pre_states.x = rn._value
		self.pre_states.y = re._value
		self.pre_states.psi = psi
		self.pub2.publish(self.pre_states)
		
		lamda_1 = np.maximum(np.zeros(self.horizon), -self.v_max - u[:self.horizon]) + np.maximum(np.zeros(self.horizon), u[:self.horizon] - self.v_max) 
		lamda_2 = np.maximum(np.zeros(self.horizon), -self.psidot_max - u[self.horizon:]) + np.maximum(np.zeros(self.horizon), u[self.horizon:] - self.psidot_max) 
		
		cost_ = 5.0 * (rn - self.goal[0])**2 + 5.0 * (re - self.goal[1])**2 + 10 * lamda_1 + 10 * lamda_2
		
		cost = np.sum(cost_) + (psi[-1] - self.psi_terminal) ** 2
		
		return cost



def statesCallback1(data):
	global states_x1, states_y1, states_psi1

	states_x1 = data.x
	states_y1 = data.y
	states_psi1 = data.psi

def statesCallback2(data):
	global states_x2, states_y2, states_psi2

	states_x2 = data.x
	states_y2 = data.y
	states_psi2 = data.psi

def statesCallback3(data):
	global states_x3, states_y3, states_psi3

	states_x3 = data.x
	states_y3 = data.y
	states_psi3 = data.psi

def odomCallback(data):
	global rx, state

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

	rx = 1

if __name__ == '__main__':
	
	freq = 5
	rospy.init_node('my_robot0', anonymous='True')	
	rospy.Subscriber('tb3_0/odom', Odometry, odomCallback)

	rospy.Subscriber('tb3_1/pre_state', States, statesCallback1)
	rospy.Subscriber('tb3_2/pre_state', States, statesCallback2)
	rospy.Subscriber('tb3_3/pre_state', States, statesCallback3)

	pub = rospy.Publisher('tb3_0/cmd_vel', Twist, queue_size=10)
	

	rate = rospy.Rate(freq)

	myRobot = ModelPredictiveControl(0.0, 2.0, 0.0, 2.84, 0.22)
	v = np.zeros(myRobot.horizon)
	psidot = np.zeros(myRobot.horizon)
	
	u = np.hstack((v, psidot))	
	while not rospy.is_shutdown():
		if rx:
			u = myRobot.optimize(state, u)	
			dist = np.sqrt((state[0] - myRobot.goal[0]) ** 2 + (state[1] - myRobot.goal[1]) ** 2)
			res_psi = abs(state[2] - myRobot.psi_terminal)		
			if dist >= 0.05 or res_psi >= 1*np.pi/180.0:
				pub.publish(Twist(Vector3(u[0], 0, 0),Vector3(0, 0, u[myRobot.horizon])))
			else:
				print("STOPPPPPPP")
				pub.publish(Twist(Vector3(0, 0, 0),Vector3(0, 0, 0)))
				
			print("x, y, psi", state[0], state[1], state[2] * 180.0 / np.pi)
			print("v, psidot", u[0], u[myRobot.horizon])
		rate.sleep()

	
	rospy.spin()
