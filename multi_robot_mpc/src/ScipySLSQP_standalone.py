import numpy as np
from time import time
from scipy.optimize import minimize
import matplotlib.pyplot as plt

pre_x = []
pre_y = []
pre_psi = []

horizon = 10
u = np.zeros(2*horizon)
state = [0, 0, 0]
v_optimal = 0
psidot_optimal = 0
dt = 0.5  

def cost_func (u):#, state, v_optimal, psidot_optimal, horizon,dt):
    global pre_x, pre_y, pre_psi

    goal = [1, 1]
    psi_terminal = 0

    obsx = [-2.5, 0.34, 0.34, -1, -2.4]#[-6, -6, -5, -5, -5.5] 
    obsy = [0.5, 0.51, -2.27, -0.8, -2.2]#[0.5, 1.5, 1.5, 0.5, 1]
    r = [0.2 * np.sqrt(2)/2]
    rr = 0.35

    psi = [state[2] + u[horizon] * dt]
    rn = [state[0] + u[0] * np.cos(psi[0]) * dt]
    re = [state[1] + u[0] * np.sin(psi[0]) * dt]
    
    for i in range(1, horizon):
        psi.append(psi[i-1] + u[horizon + i] * dt)
        rn.append(rn[i-1] + u[i] * np.cos(psi[i]) * dt)
        re.append(re[i-1] + u[i] * np.sin(psi[i]) * dt)
        
    rn = np.array(rn)
    re = np.array(re)
    psi = np.array(psi)
    
    cost_xy = (rn - goal[0])**2 + (re - goal[1])**2
    cost_smoothness_a = (np.hstack((u[0] - v_optimal, np.diff(u[0:horizon])))/dt)**2
    cost_smoothness_w = (np.hstack((u[horizon] - psidot_optimal, np.diff(u[horizon:])))/dt)**2
    cost_psi = (psi - psi_terminal) ** 2
    
    dist_obs = np.array([np.sqrt((rn - np.array(obsx[i])) ** 2 + (re - np.array(obsy[i])) ** 2) for i in range(len(obsx))], dtype=float)
    cost_obs = ((r[0] + rr + 0.25 - dist_obs)/(abs(r[0] + rr + 0.25 - dist_obs)+0.000000000000001) + 1) * (1/dist_obs)
    cost_obs = np.sum(cost_obs, axis=0)

    cost = (50 * cost_xy) + (100 * cost_xy[-1]) + 50*(cost_smoothness_w) + 50*(cost_smoothness_a) + (2 * cost_psi[-1]) + 50 * cost_obs
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

t = time()
for i in range(1):
    sol = minimize(cost_func, u, method = 'SLSQP', bounds = bnds)#, constraints = const)
    # print(sol.x)
print(time() - t)
print(np.linalg.norm(sol.x[0:horizon]))
print(np.linalg.norm(sol.x[horizon:]))
plt.figure(1)
plt.xlim(-1, 5)
plt.ylim(-1, 5)
plt.scatter([0], [0])
plt.scatter([1], [1])
plt.plot(pre_x, pre_y)
# plt.show()