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
from numpy import pi, cos, sin, array, zeros, ones, vstack, hstack, linspace
from matplotlib.pyplot import *
from scipy.linalg import expm
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.gridspec as gridspec

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import mpl_toolkits.mplot3d.axes3d as p3
from IPython.display import HTML
from config import *


class Viz:
    def __init__(
        self,
        mapdata,
        l=drone_length,
        dashboard_mode=DASHBOARD_MODE,
        show_2Dgraph=True,
    ):
        self.state_dot = np.zeros(12)
        self.t = 0
        self.l = l
        self.pose_history = None
        self.state_dot_window = 20
        self.state_dot_update_frq = SIMU_UPDATE_FRQ
        self.gs = gridspec.GridSpec(nrows=4, ncols=10)
        self.show_2Dgraph = show_2Dgraph
        self.world_size = mapdata["boundaries"]
        self.obs_origin = mapdata["obstacle_origin"]
        self.obs_size = mapdata["obstacle_size"]
        self.fig = plt.figure(figsize=(11, 5))

        self.env_time = 0
        self.dashboard_mode = dashboard_mode

        if not self.show_2Dgraph:
            self.ax3D = Axes3D(self.fig)  # self.fig.add_subplot( projection="3d")
        else:
            self.ax3D = self.fig.add_subplot(self.gs[:, :7], projection="3d")
            self.ax2D = [self.fig.add_subplot(self.gs[0, 7:])]
            self.ax2D[0].set_ylim(-10 / UNIT_SCALER, 10 / UNIT_SCALER)
            self.ax2D[0].set_xlim(0, self.state_dot_window)
            self.ax2D.append(self.fig.add_subplot(self.gs[1, 7:], sharex=self.ax2D[0]))
            self.ax2D[1].set_ylim(-70 / UNIT_SCALER, 70 / UNIT_SCALER)
            self.ax2D.append(self.fig.add_subplot(self.gs[2, 7:], sharex=self.ax2D[0]))
            self.ax2D[2].set_ylim(-30 / UNIT_SCALER, 30 / UNIT_SCALER)
            self.ax2D.append(self.fig.add_subplot(self.gs[3, 7:], sharex=self.ax2D[0]))
            self.ax2D[3].set_ylim(-180 / UNIT_SCALER, 180 / UNIT_SCALER)

            self.time_buffer = []
            self.v_buffer = [[], [], []]
            self.v_dot_buffer = [[], [], []]
            self.w_buffer = [[], [], []]
            self.w_dot_buffer = [[], [], []]

            self.v_lines = [
                self.ax2D[0].plot([], [], label="Xv", color="red")[0],
                self.ax2D[0].plot([], [], label="Yv", color="green")[0],
                self.ax2D[0].plot([], [], label="Zv", color="blue")[0],
            ]
            self.v_dot_lines = [
                self.ax2D[1].plot([], [], label="Xa", color="red", linestyle="dashed")[
                    0
                ],
                self.ax2D[1].plot(
                    [], [], label="Ya", color="green", linestyle="dashed"
                )[0],
                self.ax2D[1].plot([], [], label="Za", color="blue", linestyle="dashed")[
                    0
                ],
            ]
            self.w_lines = [
                self.ax2D[2].plot([], [], label="Φ_dot", color="red")[0],
                self.ax2D[2].plot([], [], label="θ_dot", color="green")[0],
                self.ax2D[2].plot([], [], label="ψ_dot", color="blue")[0],
            ]
            self.w_dot_lines = [
                self.ax2D[3].plot(
                    [], [], label="Φ_ddot", color="red", linestyle="dashed"
                )[0],
                self.ax2D[3].plot(
                    [], [], label="θ_ddot", color="green", linestyle="dashed"
                )[0],
                self.ax2D[3].plot(
                    [], [], label="ψ_ddot", color="blue", linestyle="dashed"
                )[0],
            ]

        plt.style.use("seaborn-white")
        self.all_plots = []

    def drone_state_plot(self, state_dot, t):
        temp_v = state_dot[0:3] / UNIT_SCALER
        temp_v_dot = state_dot[3:6] / UNIT_SCALER
        temp_w = state_dot[6:9] / UNIT_SCALER
        temp_w_dot = state_dot[9:12] / UNIT_SCALER

        self.time_buffer.append(t)
        for i in range(3):
            self.v_buffer[i].append(temp_v[i])
            self.v_dot_buffer[i].append(temp_v_dot[i])
            self.w_buffer[i].append(temp_w[i])
            self.w_dot_buffer[i].append(temp_w_dot[i])

        # if len(self.time_buffer) > self.state_dot_window:
        #     self.time_buffer.pop(0)
        #     for i in range(3):
        #         self.v_buffer[i].pop(0)
        #         self.v_dot_buffer[i].pop(0)
        #         self.w_buffer[i].pop(0)
        #         self.w_dot_buffer[i].pop(0)
        self.env_time += 1
        if self.dashboard_mode == "local_data":
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
        else:
            for i in range(4):
                self.ax2D[i].set_xlim(0, self.time_buffer[-1])

        # set legend
        for i in range(4):
            self.ax2D[i].legend(loc="upper right", fontsize="5")

        # update the data for each line
        for i in range(3):
            self.v_lines[i].set_data(self.time_buffer, self.v_buffer[i])
            self.v_dot_lines[i].set_data(self.time_buffer, self.v_dot_buffer[i])
            self.w_lines[i].set_data(self.time_buffer, self.w_buffer[i])
            self.w_dot_lines[i].set_data(self.time_buffer, self.w_dot_buffer[i])

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

    def draw3H(
        self, M, col, shadow=False, mirror=1
    ):  # mirror=-1 in case z in directed downward
        self.ax3D.plot(mirror * M[0], M[1], mirror * M[2], color=col)
        if shadow:
            self.ax3D.plot(mirror * M[0], M[1], 0 * M[2], color="gray")

    def circle3H(self, r):
        n = 10
        θ = linspace(0, 2 * pi, n)
        x = r * cos(θ) + array(n * [0])
        y = r * sin(θ) + array(n * [0])
        z = zeros(n)
        return self.add1(array([x, y, z]))

    def draw_trajectory_Barplot(self, pose):
        if self.pose_history is None:
            self.pose_history = np.array(pose)
            x = self.pose_history[0]
            y = self.pose_history[1]
            z = self.pose_history[2]
        else:
            self.pose_history = np.vstack((self.pose_history, np.array(pose)))
            x = self.pose_history[:, 0]
            y = self.pose_history[:, 1]
            z = self.pose_history[:, 2]
        dx = dy = dz = np.ones(1) * TRAJECTORY_MARKER_SIZE
        self.ax3D.bar3d(x, y, z, dx, dy, dz, color="C1")
        # self.ax3D.plot(x, y, z, color="coral", linewidth=0.9)

    def draw_obs(self):
        for origin, size in zip(self.obs_origin, self.obs_size):
            self.ax3D.bar3d(
                origin[0],
                origin[1],
                origin[2],
                size[0],
                size[1],
                size[2],
                alpha=0.15,
                color="grey",
                edgecolor="red",
            )

    def draw_planned_trajectory(self, traj_path):
        # display waypoints and navigation trajectory
        self.ax3D.plot(*zip(*traj_path), "y--", c="green", linewidth=1)
        self.ax3D.scatter(*zip(*traj_path), c="black", s=10)

    def draw_robot_trajectory(self, pose):
        if self.pose_history is None:
            self.pose_history = np.array(pose)
            x = self.pose_history[0]
            y = self.pose_history[1]
            z = self.pose_history[2]
        else:
            self.pose_history = np.vstack((self.pose_history, np.array(pose)))
            x = self.pose_history[:, 0]
            y = self.pose_history[:, 1]
            z = self.pose_history[:, 2]

        self.ax3D.plot(x, y, z, color="coral", linewidth=1.2)

    def draw_quadrotor3D(self, x, l):
        Ca = hstack(
            (self.circle3H(0.3 * l), [[0.3 * l, -0.3 * l], [0, 0], [0, 0], [1, 1]])
        )  # the disc + the blades
        T = self.tran3H(*x[0:3]) @ self.eulerH(-x[3:6])
        C0 = (
            T @ self.tran3H(0, l, 0) @ self.eulerH(np.zeros(3)) @ Ca
        )  # we rotate the blades
        C1 = T @ self.tran3H(-l, 0, 0) @ self.eulerH(np.zeros(3)) @ Ca
        C2 = T @ self.tran3H(0, -l, 0) @ self.eulerH(np.zeros(3)) @ Ca
        C3 = T @ self.tran3H(l, 0, 0) @ self.eulerH(np.zeros(3)) @ Ca
        M = T @ self.add1(array([[l, -l, 0, 0, 0], [0, 0, 0, l, -l], [0, 0, 0, 0, 0]]))
        self.draw3H(M, "grey", shadow=True)  # body
        self.draw3H(C0, "black", shadow=True)
        self.draw3H(C1, "red", shadow=True)
        self.draw3H(C2, "red", shadow=True)
        self.draw3H(C3, "blue", shadow=True)
        self.draw_robot_trajectory(x)

        if AGENT_NUMBER > 1:
            Dx = x.copy()
            Dx[0:2] = Dx[0:2] - 2
            Dx[3] += 1
            Da = hstack(
                (self.circle3H(0.3 * l), [[0.3 * l, -0.3 * l], [0, 0], [0, 0], [1, 1]])
            )  # the disc + the blades
            DT = self.tran3H(*Dx[0:3]) @ self.eulerH(-Dx[3:6])
            D0 = (
                DT @ self.tran3H(0, l, 0) @ self.eulerH(np.zeros(3)) @ Ca
            )  # we rotate the blades
            D1 = DT @ self.tran3H(-l, 0, 0) @ self.eulerH(np.zeros(3)) @ Ca
            D2 = DT @ self.tran3H(0, -l, 0) @ self.eulerH(np.zeros(3)) @ Ca
            D3 = DT @ self.tran3H(l, 0, 0) @ self.eulerH(np.zeros(3)) @ Ca
            DM = DT @ self.add1(
                array([[l, -l, 0, 0, 0], [0, 0, 0, l, -l], [0, 0, 0, 0, 0]])
            )
            self.draw3H(DM, "grey", shadow=True)  # body
            self.draw3H(D0, "black", shadow=True)
            self.draw3H(D1, "green", shadow=True)
            self.draw3H(D2, "green", shadow=True)
            self.draw3H(D3, "blue", shadow=True)
            self.draw_robot_trajectory(Dx)

    def draw_quadri(self, X, state_dot, waypoints, t):
        self.ax3D.clear()
        self.ax3D.set_xlim3d(0, self.world_size[0])
        self.ax3D.set_ylim3d(0, self.world_size[1])
        self.ax3D.set_zlim3d(0, self.world_size[2])
        self.draw_obs()
        self.draw_planned_trajectory(waypoints)
        self.draw_quadrotor3D(X, BODY_SCALER * self.l)
        if self.show_2Dgraph:
            self.drone_state_plot(state_dot, t)
        plt.tight_layout()
        plt.pause(1 / SIMU_UPDATE_FRQ)

    def plot_state_and_des_state(self, data):
        # Separate state, des_state, orientation_des and time
        state = np.vstack([item[0] for item in data])
        des_state = np.vstack([item[1] for item in data])
        orientation_des = np.vstack([item[2] for item in data])
        time = np.hstack([item[3] for item in data])

        # Determine the minimum length among the sequences
        min_length = min(len(state), len(des_state), len(orientation_des), len(time))

        # Trim the sequences to the minimum length
        state = state[:min_length]
        des_state = des_state[:min_length]
        orientation_des = orientation_des[:min_length]
        time = time[:min_length]

        # Initialize subplots
        fig, axs = plt.subplots(4, 3, figsize=(15, 20))

        labels = [
            "x",
            "y",
            "z",
            "x-dot",
            "y-dot",
            "z-dot",
            "phi",
            "theta",
            "psi",
            "phi-dot",
            "theta-dot",
            "psi-dot",
        ]

        # Plot state and des_state
        for i in range(3):
            axs[0, i].plot(time, state[:, i], label="State")
            axs[0, i].plot(time, des_state[:, i], label="Desired State")
            axs[0, i].set_xlabel("t")
            axs[0, i].set_ylabel(labels[i])
            axs[0, i].legend()

        for i in range(3):
            axs[1, i].plot(time, state[:, i + 3], label="State")
            # axs[1, i].plot(time, des_state[:, i+3], label='Desired State')
            axs[1, i].set_xlabel("t")
            axs[1, i].set_ylabel(labels[i + 3])
            axs[1, i].legend()

        # Plot state and orientation_des
        for i in range(3):
            axs[2, i].plot(time, state[:, i + 6], label="State")
            axs[2, i].plot(time, orientation_des[:, i], label="Desired Orientation")
            axs[2, i].set_xlabel("t")
            axs[2, i].set_ylabel(labels[i + 6])
            axs[2, i].legend()

        for i in range(3):
            axs[3, i].plot(time, state[:, i + 9], label="State")
            # axs[3, i].plot(time, orientation_des[:, i+3], label='Desired Orientation')
            axs[3, i].set_xlabel("t")
            axs[3, i].set_ylabel(labels[i + 9])
            axs[3, i].legend()

        plt.tight_layout()
        plt.show()

    def make_animation(self):
        ani = animation.ArtistAnimation(self.fig, self.allplots, interval=50, blit=True)
        fn = "Quadcopter_Trajectory"
        ani.save("%s.mp4" % (fn), writer="ffmpeg", fps=1000 / 50)
        ani.save("%s.gif" % (fn), writer="imagemagick", fps=1000 / 50)
