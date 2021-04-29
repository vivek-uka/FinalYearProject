import autograd.numpy as np
from autograd import grad
from time import time
from math import *
import matplotlib.pyplot as plt

pre_x = np.array([])
pre_y = np.array([])
pre_psi = np.array([])
goal = [1, 1]
psi_terminal = -pi/4

obsx = [1, 0, 0.25, 0.5, 1.5]  # [-6, -6, -5, -5, -5.5]
obsy = [-0.25, 0.5, 1.0, 0, 0]  # [0.5, 1.5, 1.5, 0.5, 1]
r = [0.1 * np.sqrt(2) / 2]

def optimize(state, u, u_optimal, horizon, dt, steps=25, lr=0.007, decay=0.9, eps=1e-8):

	dx_mean_sqr = np.zeros(horizon*2)

	dF = grad(cost_func) 

	startTime = time()
	for k in range(steps):
		dx = dF(u, state, u_optimal[0], u_optimal[1], horizon, dt)
		dx_mean_sqr = decay * dx_mean_sqr + (1.0 - decay) * dx ** 2
		if k != steps - 1:
			u -= lr * dx / (np.sqrt(dx_mean_sqr) + eps)
	t = time()
	print("RMS PROP", horizon)
	print("Comp time",t - startTime)		
	return u


def cost_func(u, state, v_optimal, psidot_optimal, horizon, dt):
    global pre_x, pre_y, pre_psi
    v_max = 1
    psidot_max = 5

    rr = 0.0

    psi = [state[2] + u[horizon] * dt]
    rn = [state[0] + u[0] * np.cos(psi[0]) * dt]
    re = [state[1] + u[0] * np.sin(psi[0]) * dt]

    for i in range(1, horizon):
        psi.append(psi[i - 1] + u[horizon + i] * dt)
        rn.append(rn[i - 1] + u[i] * np.cos(psi[i]) * dt)
        re.append(re[i - 1] + u[i] * np.sin(psi[i]) * dt)

    rn = np.array(rn)
    re = np.array(re)
    psi = np.array(psi)

    lamda_1 = np.maximum(np.zeros(horizon), -v_max - u[:horizon]) + np.maximum(np.zeros(horizon), u[:horizon] - v_max)
    lamda_2 = np.maximum(np.zeros(horizon), -psidot_max - u[horizon:]) + np.maximum(np.zeros(horizon), u[horizon:] - psidot_max)

    cost_xy = (rn - goal[0]) ** 2 + (re - goal[1]) ** 2
    cost_smoothness_a = (np.hstack((u[0] - v_optimal, np.diff(u[0:horizon]))) / dt) ** 2
    cost_smoothness_w = (np.hstack((u[horizon] - psidot_optimal, np.diff(u[horizon:]))) / dt) ** 2
    cost_psi = (psi - psi_terminal) ** 2

    dist_obs = np.array([np.sqrt((rn - np.array(obsx[i])) ** 2 + (re - np.array(obsy[i])) ** 2) for i in range(len(obsx))], dtype=float)
    cost_obs = ((r[0] + rr + 0.1 - dist_obs)/(abs(r[0] + rr + 0.1 - dist_obs)+0.000000000000001) + 1) * (1/dist_obs)
    cost_obs = np.sum(cost_obs, axis=0)

    cost_ = 200 * lamda_1 + 200 * lamda_2 + 50 * cost_xy + 100 * cost_xy[-1] + 50 * cost_smoothness_w + 50 * cost_smoothness_a + 2 * cost_psi[-1] + 50 * cost_obs
    cost = np.sum(cost_)

    pre_x = rn._value
    pre_y = re._value
    pre_psi = psi._value

    return cost


if __name__ == '__main__':
    horizon = 100
    dt = 0.5
    state = [0, 0, pi/2]
    u_optimal = [0, 0]
    u = np.zeros(2 * horizon)
    u = optimize(state, u, u_optimal, horizon, dt)

    # print("Norm linear", np.linalg.norm(np.hstack((u[0] - 0, np.diff(u[0:horizon]))) / dt))
    # print("Norm angular", np.linalg.norm(np.hstack((u[horizon] - 0, np.diff(u[horizon:]))) / dt))
    # print("Final position cost", np.sqrt((pre_x[-1] - goal[0]) ** 2 + (pre_y[-1] - goal[1]) ** 2))

    fig = plt.figure()
    ax1 = fig.add_subplot(1, 1, 1)
    ax1.set_aspect(1)

    circle0 = (plt.Circle((obsx[0], obsy[0]), r[0], color='orange'))
    circle1 = (plt.Circle((obsx[1], obsy[1]), r[0], color='orange'))
    circle2 = (plt.Circle((obsx[2], obsy[2]), r[0], color='orange'))
    circle3 = (plt.Circle((obsx[3], obsy[3]), r[0], color='orange'))
    circle4 = (plt.Circle((obsx[4], obsy[4]), r[0], color='orange'))


    ax1.add_artist(circle0)
    ax1.add_artist(circle1)
    ax1.add_artist(circle2)
    ax1.add_artist(circle3)
    ax1.add_artist(circle4)

    plt.xlim(-1, 5)
    plt.ylim(-1, 5)
    plt.scatter([0], [0], linewidths=0.01)
    plt.scatter([1], [1], linewidths=0.01)
    plt.plot(pre_x, pre_y)

    # plt.show()


# import autograd.numpy as np
# from autograd import grad
# from time import time
# import matplotlib.pyplot as plt

# pre_x = np.array([])
# pre_y = np.array([])
# pre_psi = np.array([])
# goal = [1, 1]
# psi_terminal = 0

# def optimize(state, u, u_optimal, horizon, dt, steps=25, lr=0.007, decay=0.9, eps=1e-8):

# 		dx_mean_sqr = np.zeros(horizon*2)

# 		dF = grad(cost_func) 

# 		startTime = time()
# 		for k in range(steps):
# 		    dx = dF(u, state, u_optimal[0], u_optimal[1], horizon, dt)
# 		    dx_mean_sqr = decay * dx_mean_sqr + (1.0 - decay) * dx ** 2
# 		    if k != steps - 1:
# 				u -= lr * dx / (np.sqrt(dx_mean_sqr) + eps)
# 		print("RMS")
# 		print("Comp time",time() - startTime)		
# 		return u

# def cost_func(u, state, v_optimal, psidot_optimal, horizon, dt):

# 	global pre_x, pre_y, pre_psi
# 	v_max = 1
# 	psidot_max = 5
# 	r = [0.2 * np.sqrt(2)/2]
# 	rr = 0.35

	
	
# 	obsx = [0.25, 0.5, 0.34, -1, -2.4]#[-2.5, 0.34, 0.34, -1, -2.4]#[-6, -6, -5, -5, -5.5] 
# 	obsy = [0.5, 0.25, -2.27, -0.8, -2.2]#[0.5, 0.51, -2.27, -0.8, -2.2]#[0.5, 1.5, 1.5, 0.5, 1]

# 	psi = [state[2] + u[horizon] * dt]
# 	rn = [state[0] + u[0] * np.cos(psi[0]) * dt]
# 	re = [state[1] + u[0] * np.sin(psi[0]) * dt]

# 	for i in range(1, horizon):
# 		psi.append(psi[i-1] + u[horizon + i] * dt)
# 		rn.append(rn[i-1] + u[i] * np.cos(psi[i]) * dt)
# 		re.append(re[i-1] + u[i] * np.sin(psi[i]) * dt)

# 	rn = np.array(rn)
# 	re = np.array(re)
# 	psi = np.array(psi)

# 	lamda_1 = np.maximum(np.zeros(horizon), -v_max - u[:horizon]) + np.maximum(np.zeros(horizon), u[:horizon] - v_max) 
# 	lamda_2 = np.maximum(np.zeros(horizon), -psidot_max - u[horizon:]) + np.maximum(np.zeros(horizon), u[horizon:] - psidot_max) 

# 	cost_xy = (rn - goal[0]) ** 2 + (re - goal[1]) ** 2
# 	cost_smoothness_a = (np.hstack((u[0] - v_optimal, np.diff(u[0:horizon])))/dt) ** 2
# 	cost_smoothness_w = (np.hstack((u[horizon] - psidot_optimal, np.diff(u[horizon:])))/dt)**2
# 	cost_psi = (psi - psi_terminal) ** 2

# 	# dist_obs = np.array([np.sqrt((rn - np.array(obsx[i])) ** 2 + (re - np.array(obsy[i])) ** 2) for i in range(len(obsx))], dtype=float)
# 	# cost_obs = ((r[0] + rr + 0.25 - dist_obs)/(abs(r[0] + rr + 0.25 - dist_obs)+0.000000000000001) + 1) * (1/dist_obs)
# 	# cost_obs = np.sum(cost_obs, axis=0)

# 	cost_ = 200 * lamda_1 + 200 * lamda_2 + 50 * cost_xy + 100 * cost_xy[-1] + 50 * cost_smoothness_w + 50 * cost_smoothness_a + 2 * cost_psi[-1] #+ 50 * cost_obs
# 	cost = np.sum(cost_)
	
# 	pre_x = rn._value
# 	pre_y = re._value
# 	pre_psi = psi._value

# 	return cost

# if __name__ == '__main__':

# 	horizon = 50 # 10, 20, 25, 35, 50, 75, 100
# 	dt = 0.5
# 	state = [0, 0, 0]
# 	u_optimal = [0 , 0]
# 	u = np.zeros(2*horizon)
# 	u = optimize(state, u, u_optimal, horizon, dt)
	
# 	print("Norm linear",np.linalg.norm(np.hstack((u[0] - 0, np.diff(u[0:horizon])))/dt))
# 	print("Norm angular",np.linalg.norm(np.hstack((u[horizon] - 0, np.diff(u[horizon:])))/dt))
# 	print("Final position cost",np.sqrt((pre_x[-1] - goal[0]) ** 2 + (pre_y[-1] - goal[1]) ** 2))
# 	# u[0:horizon] ---> v
# 	# u[horizon:] ---> psidot

# 	# norm of linear acceleration //np.linalg.norm(arr)
# 	# norm of angular acceleration
# 	# final position cost dist to goal (pre_x[-1])
# 	# computation time

# 	plt.figure(1)
# 	plt.xlim(-1, 5)
# 	plt.ylim(-1, 5)
# 	plt.scatter([0], [0])
# 	plt.scatter([1], [1])
# 	plt.plot(pre_x, pre_y)
# 	plt.show()