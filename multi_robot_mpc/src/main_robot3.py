import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3
import autograd.numpy as np
from autograd import grad
from time import time
from multi_robot_mpc.msg import States

state = [0, 0, 0]

rx = 0

class ModelPredictiveControl:

	def __init__(self, x_g, y_g, angular_max, linear_max):

		self.horizon = 5
		self.control = 1
		self.dt = 0.5
		self.psidot_max = angular_max
		self.v_max = linear_max
		self.goal = [x_g, y_g]	
		self.pre_states = States()
		
	def optimize(self, state, u, steps=25, lr=0.001, decay=0.9, eps=1e-8):

		dx_mean_sqr = np.zeros(u.shape)
		dF = grad(self.cost_function, 1) 

		startTime = time()
		for k in range(steps):
		    dx = dF(state, u)
		    dx_mean_sqr = decay * dx_mean_sqr + (1.0 - decay) * dx ** 2
		    if k != steps - 1:
				u -= lr * dx / (np.sqrt(dx_mean_sqr) + eps)
				u = self.applyConstraint(u)
		print("Optimization Time = ", time()-startTime)
		return u

	def cost_function(self, state, u): 

		psi = state[2] + np.array([(i + 1) * u[2*i + 1] * self.dt for i in range(self.horizon)], dtype=float)
		rn = state[0] + np.array([u[2*i + 0] * np.cos(psi[i]) * self.dt for i in range(self.horizon)], dtype=float)
		re = state[1] + np.array([u[2*i + 0] * np.sin(psi[i]) * self.dt for i in range(self.horizon)], dtype=float)
		
		arr = np.vstack((rn, re, psi))		
		self.pre_states.x = rn._value
		self.pre_states.y = re._value
		self.pre_states.psi = psi._value

		cost_ = (rn - self.goal[0])**2 + (re - self.goal[1])**2
		cost = np.sum(cost_)

		return cost

	def applyConstraint(self, u):
		
		for i in range(self.horizon):
			if u[2*i + 0] >= self.v_max:
				u[2*i + 0] = self.v_max
			elif u[2*i + 0] <= -self.v_max:
				u[2*i + 0] = -self.v_max

			if u[2*i + 1] >= self.psidot_max:
				u[2*i + 1] = self.psidot_max
			elif u[2*i + 1] <= -self.psidot_max:
				u[2*i + 1] = -self.psidot_max
		return u


def callback(data):
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
	rospy.init_node('my_robot3', anonymous='True')	
	rospy.Subscriber('tb3_3/odom', Odometry, callback)	
	pub = rospy.Publisher('tb3_3/cmd_vel', Twist, queue_size=10)
	pub2 = rospy.Publisher('tb3_3/pre_state', States, queue_size=10)

	rate = rospy.Rate(freq)

	myRobot = ModelPredictiveControl(5.0, 0.0, 2.84, 0.22)
	u = np.zeros(myRobot.horizon*2)	
	
	while not rospy.is_shutdown():
		if rx:
			u = myRobot.optimize(state, u)
			pub2.publish(myRobot.pre_states)
			#pub.publish(Twist(Vector3(u[0], 0, 0),Vector3(0, 0, u[1])))
			print("x , y ", state[0], state[1])
			print("v, psidot", u[0], u[1])
		rate.sleep()

	
	rospy.spin()
