# MIT License

# Copyright (c) [2022] [Luc Jaulin] 

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

# available at https://www.ensta-bretagne.fr/jaulin/roblib.py
# For help : https://www.ensta-bretagne.fr/jaulin/python.html
# used in KalMOOC :  https://www.ensta-bretagne.fr/jaulin/kalmooc.html
# used in RobMOOC :  https://www.ensta-bretagne.fr/jaulin/robmooc.html
# used in KalMOOC :  https://www.ensta-bretagne.fr/jaulin/inmooc.html


import numpy as np
import matplotlib.pyplot as plt
from numpy import pi, cos, sin, \
     array, zeros, ones, vstack, hstack, linspace
from matplotlib.pyplot import *
from scipy.linalg import expm
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.gridspec as gridspec
import threading


SIMU_UPDATE_FRQ = 1000
BODY_SCALER = 4
WORLD_SCALER = 5
TRAJECTORY_MARKER_SIZE=0.03
UNIT_SCALER=1

class Viz:
    def __init__(self, l=0.046) :
        self.state_dot = np.zeros(12)
        self.t = 0
        self.l = l
        self.pose_history=np.array([[0,0,0,0,0,0]])  
        self.state_dot_window = 20
        self.state_dot_update_frq = SIMU_UPDATE_FRQ
        self.gs = gridspec.GridSpec(nrows=4, ncols=10)
        self.fig = plt.figure(figsize=(10, 5))

        # self.ax3D = self.fig.add_subplot(1, 2, 1,projection="3d")
        # self.ax2D = [self.fig.add_subplot(2, 2, 2), self.fig.add_subplot(2, 2, 4)]

        self.ax3D = self.fig.add_subplot(self.gs[:, :7], projection="3d")
        self.ax2D = [self.fig.add_subplot(self.gs[0, 7:])]
        self.ax2D[0].set_xlim(0, self.state_dot_window)
        self.ax2D[0].set_ylim(-10/UNIT_SCALER, 10/UNIT_SCALER)
        # self.ax2D[0].legend()
        self.ax2D.append(self.fig.add_subplot(self.gs[1, 7:], sharex = self.ax2D[0]))
        self.ax2D[1].set_ylim(-60/UNIT_SCALER, 60/UNIT_SCALER)
        # self.ax2D[1].legend()
        self.ax2D.append(self.fig.add_subplot(self.gs[2, 7:], sharex = self.ax2D[0]))
        self.ax2D[2].set_ylim(-60/UNIT_SCALER, 60/UNIT_SCALER)
        # self.ax2D[2].legend()
        self.ax2D.append(self.fig.add_subplot(self.gs[3, 7:], sharex = self.ax2D[0]))
        self.ax2D[3].set_ylim(-60/UNIT_SCALER, 60/UNIT_SCALER)
        # self.ax2D[3].legend()
        # self.ax3D = self.fig.add_subplot(3, 1, 1, projection="3d")
        # self.ax2D = [self.fig.add_subplot(1, 2, 2)]
        # self.ax2D.append(self.fig.add_subplot(2, 2, 3, sharex = self.ax2D[0]))
        # self.ax2D.append(self.fig.add_subplot(3, 2, 4, sharex = self.ax2D[0]))
        # self.ax2D.append(self.fig.add_subplot(4, 2, 5, sharex = self.ax2D[0]))


        # ax.set_title("Plot of Position")
        plt.style.use('seaborn-white')

        self.time_buffer = []
        self.v_buffer = [[],[],[]]
        self.v_dot_buffer = [[],[],[]] 
        self.w_buffer = [[],[],[]]
        self.w_dot_buffer = [[],[],[]] 
        
        self.v_lines = [self.ax2D[0].plot([], [], label='Xv', color='red')[0],
                            self.ax2D[0].plot([], [], label='Yv', color='green')[0],
                            self.ax2D[0].plot([], [], label='Zv', color='blue')[0],]
        self.v_dot_lines = [self.ax2D[1].plot([], [], label='Xa', color='red',linestyle='dashed')[0],
                                self.ax2D[1].plot([], [], label='Ya', color='green',linestyle='dashed')[0],
                                self.ax2D[1].plot([], [], label='Za', color='blue',linestyle='dashed')[0]]
        self.w_lines = [self.ax2D[2].plot([], [], label='Φ_dot', color='red')[0],
                        self.ax2D[2].plot([], [], label='θ_dot', color='green')[0],
                        self.ax2D[2].plot([], [], label='ψ_dot', color='blue')[0], ]
        self.w_dot_lines = [self.ax2D[3].plot([], [], label='Φ_ddot', color='red',linestyle='dashed')[0],
                            self.ax2D[3].plot([], [], label='θ_ddot', color='green',linestyle='dashed')[0],
                            self.ax2D[3].plot([], [], label='ψ_ddot', color='blue',linestyle='dashed')[0]]

    def drone_state_plot(self, state_dot, t):
        temp_v = state_dot[0:3]/UNIT_SCALER
        temp_v_dot = state_dot[3:6]/UNIT_SCALER
        temp_w = state_dot[6:9]/UNIT_SCALER
        temp_w_dot = state_dot[9:12]/UNIT_SCALER

        self.time_buffer.append(t)
        for i in range(3):
            self.v_buffer[i].append(temp_v[i])
            self.v_dot_buffer[i].append(temp_v_dot[i])
            self.w_buffer[i].append(temp_w[i])
            self.w_dot_buffer[i].append(temp_w_dot[i])

        if len(self.time_buffer) > self.state_dot_window:
            self.time_buffer.pop(0)
            for i in range(3):
                self.v_buffer[i].pop(0)
                self.v_dot_buffer[i].pop(0)
                self.w_buffer[i].pop(0)
                self.w_dot_buffer[i].pop(0)

            #   For number of 2D plots
            for i in range(4):
                self.ax2D[i].set_xlim(self.time_buffer[0], self.time_buffer[-1])

        # set legend
        for i in range(4):
            self.ax2D[i].legend(loc='upper right')

        # update the data for each line
        for i in range(3):
            self.v_lines[i].set_data(self.time_buffer, self.v_buffer[i])
            self.v_dot_lines[i].set_data(self.time_buffer, self.v_dot_buffer[i])
            self.w_lines[i].set_data(self.time_buffer, self.v_buffer[i])
            self.w_dot_lines[i].set_data(self.time_buffer, self.v_dot_buffer[i])

    def add1(self, M):
        return vstack((M, ones(M.shape[1])))

    def ToH(self, R):  # transformation matrix to homogenous
        H = hstack((R, array([[0], [0], [0]])))
        V = vstack((H, array([0, 0, 0, 1])))
        return V

    def tran3H(self, x, y, z):
        return array([[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]])

    def eulerH(self, angles):
        Ad_i = self.adjoint(array([1, 0, 0]))
        Ad_j = self.adjoint(array([0, 1, 0]))
        Ad_k = self.adjoint(array([0, 0, 1]))
        M = expm(angles[2] * Ad_k) @ expm(angles[1] * Ad_j) @ expm(angles[0] * Ad_i)
        return self.ToH(M)
    
    def adjoint(self, w):
        w = w.flatten()
        return array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])

    def draw3H(self, M, col, shadow=False, mirror=1):  # mirror=-1 in case z in directed downward
        self.ax3D.plot(mirror * M[0], M[1], mirror * M[2], color=col)
        if shadow: self.ax3D.plot(mirror * M[0], M[1], 0 * M[2], color='gray')

    def circle3H(self, r):
        n = 10
        θ = linspace(0, 2 * pi, n)
        x = r * cos(θ) + array(n * [0])
        y = r * sin(θ) + array(n * [0])
        z = zeros(n)
        return self.add1(array([x, y, z]))

    def draw_trajectory(self, pose):
        self.pose_history= np.vstack((self.pose_history,np.array(pose)))
        x=self.pose_history[:,0]
        y=self.pose_history[:,1]
        z=self.pose_history[:,2]
        dx=dy=dz=np.ones(1)*TRAJECTORY_MARKER_SIZE
        self.ax3D.bar3d(x, y, z, dx, dy, dz, color="C1")

    def draw_quadrotor3D(self, x, l):
        Ca = hstack((self.circle3H(0.3 * l), [[0.3 * l, -0.3 * l], [0, 0], [0, 0], [1, 1]]))  # the disc + the blades
        T = self.tran3H(*x[0:3]) @ self.eulerH(-x[3:6])
        C0 = T @ self.tran3H(0, l, 0) @ self.eulerH(np.zeros(3)) @ Ca  # we rotate the blades
        C1 = T @ self.tran3H(-l, 0, 0) @ self.eulerH(np.zeros(3)) @ Ca
        C2 = T @ self.tran3H(0, -l, 0) @ self.eulerH(np.zeros(3)) @ Ca
        C3 = T @ self.tran3H(l, 0, 0) @ self.eulerH(np.zeros(3)) @ Ca
        M = T @ self.add1(array([[l, -l, 0, 0, 0], [0, 0, 0, l, -l], [0, 0, 0, 0, 0]]))
        self.draw3H( M, 'grey', shadow=True)  # body
        self.draw3H( C0, 'green', shadow=True)
        self.draw3H( C1, 'black', shadow=True)
        self.draw3H( C2, 'red', shadow=True)
        self.draw3H( C3, 'blue', shadow=True)
        self.draw_trajectory(x)

    def draw_quadri(self, X, state_dot, t):
        self.ax3D.clear()
        ech = 1 * WORLD_SCALER
        self.ax3D.set_xlim3d(-ech, ech)
        self.ax3D.set_ylim3d(-ech, ech)
        self.ax3D.set_zlim3d(0, ech)
        self.draw_quadrotor3D(X, BODY_SCALER * self.l)
        self.drone_state_plot(state_dot, t)
        # make the left 3D panel larger
        # plt.tight_layout()
        plt.pause(1/SIMU_UPDATE_FRQ)
