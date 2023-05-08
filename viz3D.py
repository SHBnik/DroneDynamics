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


SIMU_UPDATE_FRQ = 1000
BODY_SCALER = 4
WORLD_SCALER = 2

class Viz:
    def __init__(self, l=0.046) :
        self.fig = plt.figure()
        self.ax = Axes3D(self.fig)
        self.l = l

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
        self.ax.plot(mirror * M[0], M[1], mirror * M[2], color=col)
        if shadow: self.ax.plot(mirror * M[0], M[1], 0 * M[2], color='gray')

    def circle3H(self, r):
        n = 10
        θ = linspace(0, 2 * pi, n)
        x = r * cos(θ) + array(n * [0])
        y = r * sin(θ) + array(n * [0])
        z = zeros(n)
        return self.add1(array([x, y, z]))

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



    def draw_quadri(self, X, state_dot, t):
        self.ax.clear()
        ech = 1 * WORLD_SCALER
        self.ax.set_xlim3d(-ech, ech)
        self.ax.set_ylim3d(-ech, ech)
        self.ax.set_zlim3d(0, ech)
        self.draw_quadrotor3D(X, BODY_SCALER * self.l)
        plt.pause(1/SIMU_UPDATE_FRQ)