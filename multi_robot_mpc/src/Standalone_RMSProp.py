import autograd.numpy as np
from autograd import grad
from time import time
import matplotlib.pyplot as plt
from numpy import savetxt

total_time = 0
pre_x = np.array([])
pre_y = np.array([])
pre_psi = np.array([])
goal = [1, 1]
psi_terminal = 0
variable = -0.1                # 0, 0.1, 0.4, 0.5, 0.6
obsx = [0.5, 1, 0.5, 0, 0.5]  # [-6, -6, -5, -5, -5.5]
obsy = [0, 0.5, 0.5, 0.5, 1]  # [0.5, 1.5, 1.5, 0.5, 1]
r = [0.10 * np.sqrt(2) / 2]

def optimize(state, u, u_optimal, horizon, dt, steps= 12,  lr=0.007, decay=0.9, eps=1e-8):
    global total_time
    dx_mean_sqr = np.zeros(horizon * 2)

    dF = grad(cost_func)

    #startTime = time()
    for k in range(steps):
        dx = dF(u, state, u_optimal[0], u_optimal[1], horizon, dt)
        dx_mean_sqr = decay * dx_mean_sqr + (1.0 - decay) * dx ** 2
        if k != steps - 1:
            u -= lr * dx / (np.sqrt(dx_mean_sqr) + eps)
    #print("RMS")
    #print("Comp time", time() - startTime)
    #total_time += (time() - startTime)
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
    lamda_2 = np.maximum(np.zeros(horizon), -psidot_max - u[horizon:]) + np.maximum(np.zeros(horizon),
                                                                                    u[horizon:] - psidot_max)

    cost_xy = (rn - goal[0]) ** 2 + (re - goal[1]) ** 2
    cost_smoothness_a = (np.hstack((u[0] - v_optimal, np.diff(u[0:horizon]))) / dt) ** 2
    cost_smoothness_w = (np.hstack((u[horizon] - psidot_optimal, np.diff(u[horizon:]))) / dt) ** 2


    dist_obs = np.array([np.sqrt((rn - np.array(obsx[i])) ** 2 + (re - np.array(obsy[i])) ** 2) for i in range(len(obsx))], dtype=float)
    cost_obs = ((r[0] + rr + 0.1 - dist_obs)/(abs(r[0] + rr + 0.1 - dist_obs)+0.000000000000001) + 1) * (1/dist_obs)
    cost_obs = np.sum(cost_obs, axis=0)

    cost_ = 200 * lamda_1 + 200 * lamda_2 + 50 * cost_xy + 500 * cost_xy[
        -1] + 1000 * cost_smoothness_w + 1000 * cost_smoothness_a + 50 * cost_obs
    cost = np.sum(cost_)

    pre_x = rn._value
    pre_y = re._value
    pre_psi = psi._value

    return cost


if __name__ == '__main__':
    horizon = 14
    dt = 0.5
    state = [0, 0, variable*np.pi]
    u_optimal = [0, 0]
    u = np.zeros(2 * horizon)
    dist = 1000
    X = []
    Y =[]
    Psi = []
    V =[]
    Psi_dot = []
    loop_time = 0
    Time = []
    loops = 0

    while(dist>0.05):
        dist = (np.sqrt((state[0]- goal[0])**2 + (state[1]- goal[1])**2) )
        startTime = time()
        u = optimize(state, u, u_optimal, horizon, dt)
        loop_time = (time() - startTime)
        total_time += loop_time
        Time.append(loop_time)
        X.append(state[0])
        Y.append(state[1])
        Psi.append(state[2])
        V.append(u[0])
        Psi_dot.append(u[horizon])
        state[0] = pre_x[0]
        state[1] = pre_y[0]
        state[2] = pre_psi[0]
        u_optimal = [u[0],u[horizon]]
        loops += 1


    X_dash = np.diff(X) / dt
    Y_dash = np.diff(Y) / dt
    sum = 0
    for i in range(len(X_dash)):
        sum += np.sqrt((X_dash[i] * X_dash[i]) + (Y_dash[i] * Y_dash[i]))

    TIME = np.asarray([Time])
    name = "multi_robot_mpc/results/optimizer/rmsprop/config_time" + str(variable) # str(int(state[2]*180/np.pi))
    name1 = "multi_robot_mpc/results/optimizer/rmsprop/config" + str(variable) #str(int(state[2]*180/np.pi))
    average_loop_time =  total_time/loops
    arc_length = sum
    Norm_linear = np.linalg.norm(np.hstack((V[0] - 0, np.diff(V))) / dt)
    Norm_angular = np.linalg.norm(np.hstack((Psi_dot[0] - 0, np.diff(Psi_dot))) / dt)
    Final_position_cost = np.sqrt((pre_x[-1] - goal[0]) ** 2 + (pre_y[-1] - goal[1]) ** 2)

    DATA = np.asarray([average_loop_time, Norm_linear, Norm_angular, Final_position_cost, arc_length])

    savetxt(name, TIME, delimiter='\n')
    savetxt(name1, DATA, delimiter='\n')
    # print("arc lenth", sum)
    # print(" Average loop time", total_time/loops)
    # print("total time", total_time)
    # print("Norm linear", np.linalg.norm(np.hstack((V[0] - 0, np.diff(V))) / dt))
    # print("Norm angular", np.linalg.norm(np.hstack((Psi_dot[0] - 0, np.diff(Psi_dot))) / dt))
    # print("Final position cost", np.sqrt((pre_x[-1] - goal[0]) ** 2 + (pre_y[-1] - goal[1]) ** 2))# + (pre_psi[-1] - psi_terminal)**2)
    # #pint(u[0:horizon])
    # #print(u[horizon:])

    # norm of linear acceleration //np.linalg.norm(arr)
    # norm of angular acceleration
    # final position cost dist to goal (pre_x[-1])
    # computation time

    fig = plt.figure()
    ax1 = fig.add_subplot(1, 1, 1)
    ax1.set_aspect(1)

    circle0 = (plt.Circle((obsx[0], obsy[0]), r[0], color='orange'))
    circle1 = (plt.Circle((obsx[1], obsy[1]), r[0], color='orange'))
    circle2 = (plt.Circle((obsx[2], obsy[2]), r[0], color='orange'))
    circle3 = (plt.Circle((obsx[3], obsy[3]), r[0], color='orange'))
    circle4 = (plt.Circle((obsx[4], obsy[4]), r[0], color='orange'))

    # circle5 = (plt.Circle((obsx[0], obsy[0]), r[0] + 0.1, fill=False))
    # circle6 = (plt.Circle((obsx[1], obsy[1]), r[0] + 0.1, fill=False))
    # circle7 = (plt.Circle((obsx[2], obsy[2]), r[0] + 0.1, fill=False))
    # circle8 = (plt.Circle((obsx[3], obsy[3]), r[0] + 0.1, fill=False))
    # circle9 = (plt.Circle((obsx[4], obsy[4]), r[0] + 0.1, fill=False))

    ax1.add_artist(circle0)
    ax1.add_artist(circle1)
    ax1.add_artist(circle2)
    ax1.add_artist(circle3)
    ax1.add_artist(circle4)
    # ax1.add_artist(circle5)
    # ax1.add_artist(circle6)
    # ax1.add_artist(circle7)
    # ax1.add_artist(circle8)
    # ax1.add_artist(circle9)

    plt.xlim(-0.5, 1.5)
    plt.ylim(-0.5, 1.5)
    plt.scatter([0], [0], linewidths=0.001)
    plt.scatter([1], [1], linewidths=0.001)
    plt.plot(X, Y)

    plt.show()
