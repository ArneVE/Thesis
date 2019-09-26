# Include Required Libraries
import numpy as np
import matplotlib.pyplot as plt
import scipy
from scipy.integrate import odeint
from scipy.signal import correlate
from math import sin, cos, sqrt, exp, pi
import csv


class hopf():
    def __init__(self, params, X0):
        self.alpha = params[0]
        self.beta = params[1]
        self.w_swing = params[2]  # frequency of the swing phase
        self.w_stance = params[3]    # frequency of the stance phase
        self.theta = params[4]  # required phase difference between the 2 oscillators
        self.current_ED = params[5]
        self.current_FP = params[6]
        self.mu = params[13]
        self.k = params[14]  # coupling between phases of oscillators
        self.X0 = X0    # List with initial condition
        self.b = 100


    # Function for set of Hopf Equations
    # Returns rate of change of x and y wrt time as array of floats
    def update(self, X, t):
        x1 = X[0]
        y1 = X[1]
        x2 = X[2]
        y2 = X[3]
        r1 = sqrt(x1**2 + y1**2)
        r2 = sqrt(x2**2 + y2**2)
        delta1 = y2*cos(self.theta) - x2*sin(self.theta)
        delta2 = y1*cos(-self.theta) - x1*sin(-self.theta)
        w1 = self.w_stance/(exp(-self.b*x1) + 1) + self.w_swing/(exp(self.b*x1) + 1)
        w2 = self.w_stance/(exp(-self.b*x2) + 1) + self.w_swing/(exp(self.b*x2) + 1)
        dx1dt = self.alpha*(self.mu - r1**2)*x1 - w1*y1
        dy1dt = self.beta*(self.mu - r1**2)*y1 + w1*x1 + self.k*delta1
        dx2dt = self.alpha*(self.mu - r2**2)*x2 - w2*y2
        dy2dt = self.beta*(self.mu - r2**2)*y2 + w2*x2 + self.k*delta2
        return np.array([dx1dt, dy1dt, dx2dt, dy2dt])


    # Function to solve Hopf Equations
    def solve(self, X0, tstart=0, tend=10, tnum=10000):
        t = np.linspace(tstart, tend, tnum)
        sol = odeint(self.update, self.X0, t)
        x1 = sol[:, 0]
        y1 = sol[:, 1]
        x2 = sol[:, 2]
        y2 = sol[:, 3]
        return [x1, y1, x2, y2, t]


    # Function to plot Solution
    def SolutionPlot(self, x1, x2, t):
        fig = plt.figure(1)
        plt.clf()
        plt.plot(t[::], self.current_ED*x1[::])
        plt.plot(t[::], self.current_FP*x2[::])
        plt.xlabel('Time [s]')
        plt.ylabel('Current [unit]')
        plt.legend(('CPG ED', 'CPG FP'),loc=2)
        #plt.legend((r'$\alpha$ = 0.1', r'$\alpha$ = 1', r'$\alpha$ = 5', r'$\alpha$ = 10', r'$\alpha$ = 20', r'$\alpha$ = 30'))
        #plt.savefig('hopf-1.png'

        # fig = plt.figure(2)
        # plt.clf()
        # plt.plot(t[::], np.sign(x1[::])*)
        # plt.plot(t[::], self.current_FP*x2[::])
        # plt.xlabel('Time [s]')
        # plt.ylabel('Current [unit]')
        # plt.legend(('CPG ED', 'CPG FP'),loc=2)

        plt.show()
        return fig


if __name__ == '__main__':
    # Main Processing
    #     a      b      w_swing   w_stance      theta   I_ED  I_FP  PWM_ED_POS  PWM_FP_POS  PWM_ED_NEG  PWM_FP_NEG  start_CPG1 start_CPG2  mu  k
    lb = [0.001, 0.001, 2*pi/1.5,   2*pi/1.5,   pi/2,   10,    10,  200,        200,        200,        200,        -1,        -1,         1,  1]   # lower bounds of the tunable parameters
    ub = [30,    30,    2*pi/0.8, 2*pi/0.8,     4*pi/2, 150,   150, 600,        600,        600,        600,         1,         1,         1,  1]   # upper bounds of the tunable parameters
    X0 = [0.0, -1.0, 0.0, 1.0]

    # optimal parameters defined by CMA-ES
    best = [0.13548073672404304, 0.35325991516773203, 0.27983333374094416, 0.9948610659597079, 0.22210453778191153, 0.8563338959285824, 0.7609336926599861, 0.3280657736873627, 0.9878437579740545, 0.35881809237940555, 0.7788594082865437, 0.9777547548093148, 0.12138980866280139, 0, 0]

    denormalized_best = np.multiply((np.array(ub) - np.array(lb)), best) + np.array(lb)
    print(denormalized_best)
    cpg = hopf(denormalized_best, X0 = [0.01, 1.0, 0.0, -1.0])
    [x1, _, x2, _, t] = cpg.solve(X0)
    cpg.SolutionPlot(x1, x2, t)
