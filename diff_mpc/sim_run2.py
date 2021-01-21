from model2d import*
from basics import*
#import matlab.engine
from copy import deepcopy
import matplotlib.pyplot as plt
from multiprocessing import Process, Manager

# ___________________________________mpc objects for 2 FWAs_____________________________________________________________
mpc = [ModelPredictiveControl(0, 0, 5, 5, np.pi/4, 0), ModelPredictiveControl(5.0, 0.0, 0, 5, 3*np.pi/4, 1)]#, ModelPredictiveControl(6, 5, 0, 0, -3*np.pi/4, 2)]


# __________________________________run simulation agent0_______________________________________________________________
def optimize_agent0(optimizeFlag, x, y, z, roll, pitch, yaw, predicted_x, predicted_y, u):

    constraint = [[-1.0, 1.0],  # angular heading
                  [0.05, 1.0]]  # velocity constraint

    while True:
       mpc[0].obs = [x[1], y[1]]


       if optimizeFlag[0] == 1:
            mpc[0].u = mpc[0].optimize(mpc[0].prev_state, mpc[0].u, constraint)
            mpc[0].u_optimal = np.array([mpc[0].u[0][0], mpc[0].u[1][0]])
            mpc[0].u = delete_append(mpc[0].u)
            optimizeFlag[0] = 0
            predicted_x[0] = (mpc[0].rn)._value
            predicted_y[0] = (mpc[0].re)._value
            u[0] = mpc[0].u_optimal

       elif optimizeFlag[0] == 0:
            mpc[0].fwa_state = mpc[0].plant_model(mpc[0].prev_state, mpc[0].dt, mpc[0].u_optimal[0], mpc[0].u_optimal[1])

            x[0] = mpc[0].fwa_state[0]
            y[0] = mpc[0].fwa_state[1]
            yaw[0] = mpc[0].fwa_state[2]

            mpc[0].prev_state = deepcopy(mpc[0].fwa_state)
            optimizeFlag[0] = -1

            mpc[0].dist_goal = dist(mpc[0].fwa_state[0], mpc[0].fwa_state[1], mpc[0].point[0], mpc[0].point[1])
            if mpc[0].dist_goal < 1.0:
                constraint[1][0] = 0.0



def optimize_agent1(optimizeFlag, x, y, z, roll, pitch, yaw, predicted_x, predicted_y, u):

    constraint = [[-1.0, 1.0],  # angular heading
                  [0.05, 1.0]]  # velocity constraint

    while True:
        mpc[1].obs = [x[0], y[0]]

        if optimizeFlag[1] == 1:
            mpc[1].u = mpc[1].optimize(mpc[1].prev_state, mpc[1].u, constraint)
            mpc[1].u_optimal = np.array([mpc[1].u[0][0], mpc[1].u[1][0]])
            mpc[1].u = delete_append(mpc[1].u)
            optimizeFlag[1] = 0

            predicted_x[1] = (mpc[1].rn)._value
            predicted_y[1] = (mpc[1].re)._value
            u[1] = mpc[1].u_optimal

        elif optimizeFlag[1] == 0:
            mpc[1].fwa_state = mpc[1].plant_model(mpc[1].prev_state, mpc[1].dt, mpc[1].u_optimal[0], mpc[1].u_optimal[1])

            x[1] = mpc[1].fwa_state[0]
            y[1] = mpc[1].fwa_state[1]
            yaw[1] = mpc[1].fwa_state[2]

            mpc[1].prev_state = deepcopy(mpc[1].fwa_state)
            optimizeFlag[1] = -1

            mpc[1].dist_goal = dist(mpc[1].fwa_state[0], mpc[1].fwa_state[1], mpc[1].point[0], mpc[1].point[1])

            if mpc[1].dist_goal < 1.0:
                constraint[1][0] = 0.0

# ____________________________________________visualization_____________________________________________________________

def visualize(optimizeFlag, x, y, z, roll, pitch, yaw, predicted_x, predicted_y, u):

    fig = plt.figure()
    ax1 = fig.add_subplot(1, 1, 1)
   #eng = matlab.engine.start_matlab()
    x0 = []
    y0 = []
    x1 = []
    y1 = []

    while True:

        if optimizeFlag[0] == -1 and optimizeFlag[1] == -1:
            #eng.visualize(x_[0], y_[0], z_[0], roll_[0], pitch_[0], yaw_[0], nargout=0)
            x0.append(x[0])
            y0.append(y[0])

            x1.append(x[1])
            y1.append(y[1])

            sep = dist(x[0],y[0],x[1],y[1])

            plt.clf()

            plt.xlim(-1, 20)
            plt.ylim(-1, 20)
            plt.xlabel('x in (m)')
            plt.ylabel('y in (m)')

            circle1 = plt.Circle((x[1], y[1]), 1.5, color='g', fill=False, linestyle='dotted')
            circle2 = plt.Circle((x[1], y[1]), 0.5, color='g', fill=False)
            circle3 = plt.Circle((x[0], y[0]), 1.5, color='b', fill=False, linestyle='dotted')
            circle4 = plt.Circle((x[0], y[0]), 0.5, color='b', fill=False)

            ax1 = fig.add_subplot(1, 1, 1)
            ax1.add_artist(circle1)
            ax1.add_artist(circle2)
            ax1.add_artist(circle3)
            ax1.add_artist(circle4)
            ax1.set_aspect(1)

            plt.text(0, 15, 'Agent 0 \n heading = %s deg \n vel = %s m/s \n (x,y) = (%s ,%s)' % (round(yaw[0]*180/np.pi, 2), round(u[0][1],2),round(x[0],2), round(y[0],2)))
            plt.text(10, 15,'Agent 1 \n heading = %s deg \n vel = %s m/s \n (x,y) = (%s ,%s)' % (round(yaw[1]*180/np.pi, 2), round(u[0][1],2),round(x[1], 2), round(y[1], 2)))
            plt.text(10, 0, 'separation = %s m' % (round(sep, 2)))

            plt.plot([mpc[0].start[0], mpc[0].point[0]], [mpc[0].start[1], mpc[0].point[1]], color='b', linestyle='dotted')
            plt.plot([mpc[1].start[0], mpc[1].point[0]], [mpc[1].start[1], mpc[1].point[1]], color='g', linestyle='dotted')

            #plt.plot(predicted_x[0], predicted_y[0], color='b', linestyle='dotted')
            #plt.plot(predicted_x[1], predicted_y[1], color='g', linestyle='dotted')

            #print("y=", predicted_y[0])
            #print("x=", predicted_x[0])
            plt.scatter(x0, y0, s=9, color='b')
            plt.scatter(x1, y1, s=9, color='g')
            plt.draw()
            optimizeFlag[0] = 1
            optimizeFlag[1] = 1
            plt.pause(0.01)




if __name__ == "__main__":
    manager = Manager()

    optimizeFlag = manager.list()
    x = manager.list()
    y = manager.list()
    z = manager.list()
    roll = manager.list()
    pitch = manager.list()
    yaw = manager.list()
    predicted_x = manager.list()
    predicted_y = manager.list()
    u = manager.list()

    for i in range(len(mpc)):
        optimizeFlag.append(1)
        x.append(mpc[i].prev_state[0])
        y.append(mpc[i].prev_state[1])
        z.append(0.0)
        roll.append(0.0)
        pitch.append(0.0)
        yaw.append(mpc[i].prev_state[2])
        predicted_x.append(np.zeros(mpc[i].horizon))
        predicted_y.append(np.zeros(mpc[i].horizon))
        u.append(np.zeros(mpc[i].num_inputs))

    process_visualize = Process(target=visualize, args=(optimizeFlag, x, y, z, roll, pitch, yaw, predicted_x, predicted_y, u))
    process_optimize_agent0 = Process(target=optimize_agent0, args=(optimizeFlag, x, y, z, roll, pitch, yaw, predicted_x, predicted_y, u))
    process_optimize_agent1 = Process(target=optimize_agent1, args=(optimizeFlag, x, y, z, roll, pitch, yaw, predicted_x, predicted_y, u))

    process_visualize.start()
    process_optimize_agent0.start()
    process_optimize_agent1.start()

    process_visualize.join()
    process_optimize_agent0.join()
    process_optimize_agent1.join()






