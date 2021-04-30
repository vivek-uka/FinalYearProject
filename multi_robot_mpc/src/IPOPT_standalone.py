from gekko import GEKKO
import numpy as np
import matplotlib.pyplot as plt  
from time import time 

horizon = 100
dt = 0.5
obsx = [0.5, 1, 0.5, 0, 0.5]  # [-6, -6, -5, -5, -5.5]
obsy = [0, 0.5, 0.5, 0.5, 1]  # [0.5, 1.5, 1.5, 0.5, 1]
r = [0.10 * np.sqrt(2) / 2]

m = GEKKO(remote=False)
m.time = np.linspace(0,horizon*dt,horizon)

# Manipulated variable
v = m.MV(value=0, lb=-1, ub=1)
v.STATUS = 1 # allow optimizer to change
v.DCOST = 15 # smooth out gas pedal movement

psidot = m.MV(value=0, lb=-5, ub=5)
psidot.STATUS = 1  # allow optimizer to change
psidot.DCOST = 15 # smooth out gas pedal movement

# Controlled Variable
x = m.CV(value=0)
x.STATUS = 0  # add the SP to the objective
m.options.CV_TYPE = 2 # squared error
x.SP = 1     # set point
x.TR_INIT = 1 # set point trajectory
x.TAU = 0     # time constant of trajectory

y = m.CV(value=0)
y.STATUS = 0  # add the SP to the objective
m.options.CV_TYPE = 2 # squared error
y.SP = 1     # set point
y.TR_INIT = 0 # set point trajectory
y.TAU = 0 

psi = m.CV(value=0)
psi.STATUS = 0  # add the SP to the objective
m.options.CV_TYPE = 2 # squared error
psi.SP = 0     # set point
psi.TR_INIT = 0 # set point trajectory
psi.TAU = 0

# Process model
m.Equation(psi.dt() == psidot)
m.Equation(x.dt() == v*m.cos(psi))
m.Equation(y.dt() == v*m.sin(psi))

rr = 0
dist_obs = m.sqrt((x - np.array(obsx[0])) ** 2 + (y - np.array(obsy[0])) ** 2) +\
            m.sqrt((x - np.array(obsx[1])) ** 2 + (y - np.array(obsy[1])) ** 2) +\
            m.sqrt((x - np.array(obsx[2])) ** 2 + (y - np.array(obsy[2])) ** 2) +\
            m.sqrt((x - np.array(obsx[3])) ** 2 + (y - np.array(obsy[3])) ** 2) +\
            m.sqrt((x - np.array(obsx[4])) ** 2 + (y - np.array(obsy[4])) ** 2) 

cost_obs = ((r[0] + rr + 0.25 - dist_obs)/(m.abs(r[0] + rr + 0.25 - dist_obs)+0.000000000000001) + 1) * (1/dist_obs)
cost_xy = (x - 1) ** 2 + (y - 1) ** 2
cost_psi = (psi - 0) ** 2


m.options.IMODE = 6 # control
m.Obj(100*cost_xy + cost_psi)# + 1000*cost_obs)
t = time()
m.solve(disp=False)
# print(psi.value[-1] - 0)
print("Time",time()-t)
# print("Horizon", horizon)
# print("Linaer",np.linalg.norm(np.hstack((v.value[0] - 0, np.diff(v.value))) / dt))
# print("Angular",np.linalg.norm(np.hstack((psidot.value[0] - 0, np.diff(psidot.value))) / dt))
print("Final position cost", np.sqrt((x.value[-1] - 1) ** 2 + (y.value[-1] - 1) ** 2))
fig = plt.figure()
ax1 = fig.add_subplot(1, 1, 1)
ax1.set_aspect(1)
plt.plot(x.value, y.value)
plt.scatter([0], [0], linewidths = 0.001)
plt.scatter([1], [1], linewidths = 0.001)

circle0 = (plt.Circle((obsx[0], obsy[0]), r[0], color='orange'))
circle1 = (plt.Circle((obsx[1], obsy[1]), r[0], color='orange'))
circle2 = (plt.Circle((obsx[2], obsy[2]), r[0], color='orange'))
circle3 = (plt.Circle((obsx[3], obsy[3]), r[0], color='orange'))
circle4 = (plt.Circle((obsx[4], obsy[4]), r[0], color='orange'))

#circle5 = (plt.Circle((obsx[0], obsy[0]), r[0] + 0.1, fill=False))
#circle6 = (plt.Circle((obsx[1], obsy[1]), r[0] + 0.1, fill=False))
#circle7 = (plt.Circle((obsx[2], obsy[2]), r[0] + 0.1, fill=False))
#circle8 = (plt.Circle((obsx[3], obsy[3]), r[0] + 0.1, fill=False))
#circle9 = (plt.Circle((obsx[4], obsy[4]), r[0] + 0.1, fill=False))

ax1.add_artist(circle0)
ax1.add_artist(circle1)
ax1.add_artist(circle2)
ax1.add_artist(circle3)
ax1.add_artist(circle4)
# plt.show()