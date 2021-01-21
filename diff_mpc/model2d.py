from basics import *
from autograd import grad
from time import time

class ModelPredictiveControl:
    horizon = 5
    control = 1
    num_inputs = 2
    dt = 0.5
    optimizeFlag = 1
    dist_goal = 0.0

    fwa_state = np.zeros(3, dtype='float')
    u_optimal = np.zeros(2, dtype='float')
    prev_state = np.zeros(3, dtype='float')
    u = np.zeros([num_inputs, horizon], dtype='float')

    rn = np.zeros(horizon)
    re = np.zeros(horizon)

    def __init__(self, x1, y1, x2, y2, yaw, id):
        self.agent = id
        self.trace_line(x1, y1, x2, y2, yaw)
        self.environment()


    def trace_line(self, x1, y1, x2, y2, yaw):
        self.point = [x2, y2]
        self.start = [x1, y1]
        self.reference = [y1-y2, x2-x1, - (x2-x1)*y2 - (y1-y2)*x2]
        self.prev_state = [x1, y1, yaw]


    def environment(self):
        self.obs = [1000,1000]
        self.safe_dist = 1.0
        self.radius = 0.5


    def plant_model(self, prev_state, dt, angular_heading, V):

        rn_ = prev_state[0] + V * np.cos(prev_state[2]) * dt
        re_ = prev_state[1] + V * np.sin(prev_state[2]) * dt
        psi_ = prev_state[2] + angular_heading * dt

        state = np.array([rn_, re_, psi_])

        return state

    def cost_function(self, state, u): #line tracing cost function

        dt = np.array([self.dt*(i+1) for i in range(self.horizon)])
        psi = state[2] + np.array([(i + 1) * self.dt * u[0][i] for i in range(self.horizon)])
        yaw = np.arctan2(self.point[1] - state[1], self.point[0] - state[0])

        self.rn = state[0] + u[1] * np.cos(psi) * dt
        self.re = state[1] + u[1] * np.sin(psi) * dt

        dist = (self.rn - self.obs[0])**2 + (self.re - self.obs[1])**2 - (self.safe_dist + self.radius)**2
        dist = np.where(dist < 0.0, dist, 0)

        obs_cost = np.sum(dist)

        if obs_cost == 0.0:
            """if state[2] > 0.0 and yaw < 0.0:
                yaw += 2*np.pi
            elif state[2] < 0.0 and yaw > 0.0:
                yaw -= 2*np.pi"""
            cost_ = (psi - yaw) ** 2
            cost_ += 5.0*((self.rn - self.point[0]) ** 2 + (self.re - self.point[1]) ** 2)
        else:
            cost_ = np.exp((dist) ** 2)

        cost = np.sum(cost_)

        return cost

    def optimize(self, state, x, constraint, steps=50, lr=0.001, decay=0.9, eps=1e-8):

        dx_mean_sqr = np.zeros(x.shape)
        dF = grad(self.cost_function, 1)  # gradient of cost function

        startTime = time()
        for k in range(steps):
            dx = dF(state, x)

            dx_mean_sqr = decay * dx_mean_sqr + (1.0 - decay) * dx ** 2
            if k != steps - 1:
                x -= lr * dx / (np.sqrt(dx_mean_sqr) + eps)
                x = self.apply_constraint(x, constraint)
        print("OptimizationTime=", time() - startTime, "Agent =", self.agent)
        return x

    def apply_constraint(self, x, constraint):

        for j in range(self.num_inputs):
            for i in range(self.horizon):
                if x[j][i] < constraint[j][0]:
                    x[j][i] = constraint[j][0]
                elif x[j][i] > constraint[j][1]:
                    x[j][i] = constraint[j][1]

        return x