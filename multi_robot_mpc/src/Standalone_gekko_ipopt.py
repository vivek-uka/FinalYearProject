from gekko import GEKKO
import numpy as np
import matplotlib.pyplot as plt  
from time import time 
from numpy import savetxt

variable = -0.7   # 0, 0.1, 0.4, 0.5, 0.6
horizon = 14
dt = 0.5
obsx = [0.5, 1, 0.5, 0, 0.5]  
obsy = [0, 0.5, 0.5, 0.5, 1]  
r = [0.10 * np.sqrt(2) / 2]
goal = [1,1]
psiTerminal = 0
state = [0, 0, variable*(np.pi)]

m = GEKKO(remote=False)
m.time = np.linspace(0,horizon*dt,horizon)

v = m.MV(value=0, lb=-1, ub=1)
v.STATUS = 1 
v.DCOST = 500


psidot = m.MV(value=0, lb=-5, ub=5)
psidot.STATUS = 1  
psidot.DCOST = 500


x = m.CV(value=state[0])
x.STATUS = 1
m.options.CV_TYPE = 2 
x.SP = 1     
x.TR_INIT = 0 
x.TAU = 0     

y = m.CV(value=state[1])
y.STATUS = 1
m.options.CV_TYPE = 2 
y.SP = 1     
y.TR_INIT = 0 
y.TAU = 0     

psi = m.CV(value=state[2])
psi.STATUS = 0
m.options.CV_TYPE = 2 
psi.SP = 0     
psi.TR_INIT = 0 
psi.TAU = 0     

m.options.MAX_ITER = 5000
m.Equation(psi.dt() == psidot)
m.Equation(x.dt() == v*m.cos(psi))
m.Equation(y.dt() == v*m.sin(psi))

rr = 0
dist_obs0 = m.sqrt((x - np.array(obsx[0])) ** 2 + (y - np.array(obsy[0])) ** 2) 
dist_obs1 = m.sqrt((x - np.array(obsx[1])) ** 2 + (y - np.array(obsy[1])) ** 2) 
dist_obs2 =  m.sqrt((x - np.array(obsx[2])) ** 2 + (y - np.array(obsy[2])) ** 2)
dist_obs3 = m.sqrt((x - np.array(obsx[3])) ** 2 + (y - np.array(obsy[3])) ** 2)
dist_obs4 =  m.sqrt((x - np.array(obsx[4])) ** 2 + (y - np.array(obsy[4])) ** 2) 

cost_obs0 = ((r[0] + rr + 0.1 - dist_obs0)/(m.abs(r[0] + rr + 0.1 - dist_obs0)+0.000000000000001) + 1) * (1/dist_obs0) ** 2
cost_obs1 = ((r[0] + rr + 0.1 - dist_obs1)/(m.abs(r[0] + rr + 0.1 - dist_obs1)+0.000000000000001) + 1) * (1/dist_obs1) ** 2
cost_obs2 = ((r[0] + rr + 0.1 - dist_obs2)/(m.abs(r[0] + rr + 0.1 - dist_obs2)+0.000000000000001) + 1) * (1/dist_obs2) ** 2
cost_obs3 = ((r[0] + rr + 0.1 - dist_obs3)/(m.abs(r[0] + rr + 0.1 - dist_obs3)+0.000000000000001) + 1) * (1/dist_obs3) ** 2
cost_obs4 = ((r[0] + rr + 0.1 - dist_obs4)/(m.abs(r[0] + rr + 0.1 - dist_obs4)+0.000000000000001) + 1) * (1/dist_obs4) ** 2
cost_obs = cost_obs0 + cost_obs1 + cost_obs2 + cost_obs3 + cost_obs4
cost_xy = (x - goal[0]) ** 2 + (y - goal[1]) ** 2

m.options.IMODE = 6 
k= 250
m.Obj(cost_obs + 5500*cost_xy)





if __name__ == '__main__':
    dist = 1000
    X = []
    Y =[]
    Psi = []
    V =[]
    Psi_dot = []
    loop_time = 0
    Time = []
    loops = 0
    total_time = 0

    fig = plt.figure()
    while(dist>0.05):
        dist = (np.sqrt((state[0]- goal[0])**2 + (state[1]- goal[1])**2) )
        startTime = time()
        m.solve(disp=False)
        loop_time = (time() - startTime)
        total_time += loop_time
        Time.append(loop_time)
        state[0] = x.value[0]
        state[1] = y.value[0]
        state[2] = psi.value[0]
        V.append(v.value[1])
        Psi_dot.append(psidot.value[1])
        X.append(state[0])
        Y.append(state[1])
        Psi.append(state[2])
        loops += 1


   
        
        
    X_dash = np.diff(X) / dt
    Y_dash = np.diff(Y) / dt
    sum = 0

    for i in range(len(X_dash)):
        sum += np.sqrt((X_dash[i] * X_dash[i]) + (Y_dash[i] * Y_dash[i]))

    TIME = np.asarray([Time])
    name = "multi_robot_mpc/results/optimizer/gekko/Gekko_config_time" + str(variable)  # str(int(state[2]*180/np.pi))
    name1 = "multi_robot_mpc/results/optimizer/gekko/Gekko_config" + str(variable)  # str(int(state[2]*180/np.pi))
    average_loop_time = total_time / loops
    arc_length = sum
    Norm_linear = np.linalg.norm(np.hstack((V[0] - 0, np.diff(V))) / dt)
    Norm_angular = np.linalg.norm(np.hstack((Psi_dot[0] - 0, np.diff(Psi_dot))) / dt)
    Final_position_cost = np.sqrt((X[-1] - goal[0]) ** 2 + (Y[-1] - goal[1]) ** 2)

    DATA = np.asarray([average_loop_time, Norm_linear, Norm_angular, Final_position_cost, arc_length])

    savetxt(name, TIME, delimiter='\n')
    savetxt(name1, DATA, delimiter='\n')
    # print("arc lenth", sum)
    # print(" Average loop time", total_time/loops)
    # print("total time", total_time)
    # print("Norm linear", np.linalg.norm(np.hstack((V[0] - 0, np.diff(V))) / dt))
    # print("Norm angular", np.linalg.norm(np.hstack((Psi_dot[0] - 0, np.diff(Psi_dot))) / dt))
    # print("Final position cost", np.sqrt((X[-1] - goal[0]) ** 2 + (Y[-1] - goal[1]) ** 2))

    ax1 = fig.add_subplot(1, 1, 1)
    ax1.set_aspect(1)

    circle0 = (plt.Circle((obsx[0], obsy[0]), r[0], color='orange'))
    circle1 = (plt.Circle((obsx[1], obsy[1]), r[0], color='orange'))
    circle2 = (plt.Circle((obsx[2], obsy[2]), r[0], color='orange'))
    circle3 = (plt.Circle((obsx[3], obsy[3]), r[0], color='orange'))
    circle4 = (plt.Circle((obsx[4], obsy[4]), r[0], color='orange'))

    circle5 = (plt.Circle((obsx[0], obsy[0]), r[0] + 0.1, fill=False))
    circle6 = (plt.Circle((obsx[1], obsy[1]), r[0] + 0.1, fill=False))
    circle7 = (plt.Circle((obsx[2], obsy[2]), r[0] + 0.1, fill=False))
    circle8 = (plt.Circle((obsx[3], obsy[3]), r[0] + 0.1, fill=False))
    circle9 = (plt.Circle((obsx[4], obsy[4]), r[0] + 0.1, fill=False))
    #
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
    #
    plt.xlim(-0.5, 1.5)
    plt.ylim(-0.5, 1.5)
    plt.scatter([0], [0], linewidths=0.001)
    plt.scatter([1], [1], linewidths=0.001)
    plt.plot(X, Y)
    plt.plot(x.value, y.value)
    plt.show()
  
