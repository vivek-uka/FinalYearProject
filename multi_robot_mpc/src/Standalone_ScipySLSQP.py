import numpy as np
from time import time
from scipy.optimize import minimize
from math import *
import matplotlib.pyplot as plt
from numpy import savetxt

variable = -0.1 # 0, 0.1, 0.4, 0.5, 0.6
total_time = 0

pre_x = []
pre_y = []
pre_psi = []

horizon = 14 # 10, 20, 25, 35, 50, 75, 100
dt = 0.5
state = [0, 0, variable*np.pi]
goal = [1, 1]
#psi_terminal = -pi/4

u = np.zeros(2 * horizon)
v_optimal = 0
psidot_optimal = 0

obsx = [0.5, 1, 0.5, 0, 0.5]  # [-6, -6, -5, -5, -5.5]
obsy = [0, 0.5, 0.5, 0.5, 1]  # [0.5, 1.5, 1.5, 0.5, 1]
r = [0.10 * np.sqrt(2) / 2]



def cost_func(u):  # , state, v_optimal, psidot_optimal, horizon,dt):
    global pre_x, pre_y, pre_psi

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

    cost_xy = (rn - goal[0]) ** 2 + (re - goal[1]) ** 2
    cost_smoothness_a = (np.hstack((u[0] - v_optimal, np.diff(u[0:horizon]))) / dt) ** 2
    cost_smoothness_w = (np.hstack((u[horizon] - psidot_optimal, np.diff(u[horizon:]))) / dt) ** 2
 #   cost_psi = (psi - psi_terminal) ** 2

    dist_obs = np.array([np.sqrt((rn - np.array(obsx[i])) ** 2 + (re - np.array(obsy[i])) ** 2) for i in range(len(obsx))], dtype=float)
    cost_obs = ((r[0] + rr + 0.1 - dist_obs)/(abs(r[0] + rr + 0.1 - dist_obs)+0.000000000000001) + 1) * (1/dist_obs)
    cost_obs = np.sum(cost_obs, axis=0)

    cost = (50 * cost_xy) + (500 * cost_xy[-1]) + 1000 * (cost_smoothness_w) + 1000 * (cost_smoothness_a) + 50 * cost_obs
    cost = np.sum(cost)

    pre_x = rn
    pre_y = re
    pre_psi = psi

    return cost


b1 = [-1, 1]
b2 = [-5, 5]
bnds = []
for i in range(horizon):
    bnds.append(b1)
for i in range(horizon):
    bnds.append(b2)
bnds = tuple(bnds)


sol = minimize(cost_func, u, method='SLSQP', bounds=bnds)  # , constraints = const)


if __name__ == '__main__':
    sol.x = np.zeros(2 * horizon)
    dist = 1000
    X = []
    Y = []
    Psi = []
    V = []
    Psi_dot = []
    loops = 0
    loop_time = 0
    Time = []

    while (dist > 0.05):
        dist = (np.sqrt((state[0] - goal[0]) ** 2 + (state[1] - goal[1]) ** 2))
        start_t = time()
        sol = minimize(cost_func, u, method='SLSQP', bounds=bnds)
        loop_time = (time() - start_t)
        total_time += loop_time
        Time.append(loop_time)
        u = sol.x
        X.append(state[0])
        Y.append(state[1])
        Psi.append(state[2])
        V.append(u[0])
        Psi_dot.append(u[horizon])
        state[0] = pre_x[0]
        state[1] = pre_y[0]
        state[2] = pre_psi[0]
        loops += 1


    X_dash = np.diff(X)/dt
    Y_dash = np.diff(Y)/dt
    sum = 0
    for i in range(len(X_dash)):
        sum += np.sqrt((X_dash[i] * X_dash[i]) + (Y_dash[i] * Y_dash[i]))

    TIME = np.asarray([Time])

    name = "multi_robot_mpc/results/optimizer/scipy-slsqp/SLSQP_config_time" + str(variable)  # str(int(state[2]*180/np.pi))
    name1 = "multi_robot_mpc/results/optimizer/scipy-slsqp/SLSQP_config" + str(variable)  # str(int(state[2]*180/np.pi))
    average_loop_time = total_time / loops
    arc_length = sum
    Norm_linear = np.linalg.norm(np.hstack((X[0] - 0, np.diff(X))) / dt)
    Norm_angular = np.linalg.norm(np.hstack((Y[0] - 0, np.diff(Y))) / dt)
    Final_position_cost = np.sqrt((pre_x[-1] - goal[0]) ** 2 + (pre_y[-1] - goal[1]) ** 2)

    DATA = np.asarray([average_loop_time, Norm_linear, Norm_angular, Final_position_cost, arc_length])

    savetxt(name, TIME, delimiter='\n')
    savetxt(name1, DATA, delimiter='\n')

    # print("arc lenth", sum)
    # print(" Average loop time", total_time/loops)
    # print("total time", total_time)
    # print("Norm linear", np.linalg.norm(np.hstack((X[0] - 0, np.diff(X))) / dt))
    # print("Norm angular", np.linalg.norm(np.hstack((Y[0] - 0, np.diff(Y))) / dt))
    # print("Final position cost", np.sqrt((pre_x[-1] - goal[0]) ** 2 + (pre_y[-1] - goal[1]) ** 2))
    #


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

    plt.xlim(-0.5, 1.5)
    plt.ylim(-0.5, 1.5)
    plt.scatter([0], [0], linewidths=0.001)
    plt.scatter([1], [1], linewidths=0.001)
    plt.plot(X, Y)

    plt.show()
