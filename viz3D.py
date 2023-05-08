# available at https://www.ensta-bretagne.fr/jaulin/roblib.py
# For help : https://www.ensta-bretagne.fr/jaulin/python.html
# used in KalMOOC :  https://www.ensta-bretagne.fr/jaulin/kalmooc.html
# used in RobMOOC :  https://www.ensta-bretagne.fr/jaulin/robmooc.html
# used in KalMOOC :  https://www.ensta-bretagne.fr/jaulin/inmooc.html


import numpy as np
import matplotlib.pyplot as plt
from numpy import mean, pi, cos, sin, sqrt, tan, arctan, arctan2, tanh, arcsin, arccos, \
    exp, dot, array, log, inf, eye, zeros, ones, inf, size, \
    arange, reshape, vstack, hstack, diag, median, \
    sign, sum, meshgrid, cross, linspace, append, round, trace
from matplotlib.pyplot import *
from numpy.random import randn, rand
from numpy.linalg import inv, det, norm, eig, qr
from scipy.linalg import sqrtm, expm, logm, norm, block_diag

from scipy.signal import place_poles
from mpl_toolkits.mplot3d import Axes3D
from math import factorial
from matplotlib.patches import Ellipse, Rectangle, Circle, Wedge, Polygon, Arc
from matplotlib.collections import PatchCollection


# Unicode https://en.wikipedia.org/wiki/List_of_Unicode_characters
# αβδεθλΛμρτφψωΓ

def add1(M):
    return vstack((M, ones(M.shape[1])))


def ToH(R):  # transformation matrix to homogenous
    H = hstack((R, array([[0], [0], [0]])))
    V = vstack((H, array([0, 0, 0, 1])))
    return V


def tran3H(x, y, z):
    return array([[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]])

def eulerH(φ, θ, ψ):
    Ad_i = adjoint(array([1, 0, 0]))
    Ad_j = adjoint(array([0, 1, 0]))
    Ad_k = adjoint(array([0, 0, 1]))
    M = expm(ψ * Ad_k) @ expm(θ * Ad_j) @ expm(φ * Ad_i)
    return ToH(M)


def adjoint(w):
    w = w.flatten()
    return array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])


def draw3H(ax, M, col, shadow=False, mirror=1):  # mirror=-1 in case z in directed downward
    ax.plot(mirror * M[0], M[1], mirror * M[2], color=col)
    if shadow: ax.plot(mirror * M[0], M[1], 0 * M[2], color='gray')

def circle3H(r):
    n = 10
    θ = linspace(0, 2 * pi, n)
    x = r * cos(θ) + array(n * [0])
    y = r * sin(θ) + array(n * [0])
    z = zeros(n)
    return add1(array([x, y, z]))

def draw_quadrotor3D(ax, x, α, l):
    Ca = hstack((circle3H(0.3 * l), [[0.3 * l, -0.3 * l], [0, 0], [0, 0], [1, 1]]))  # the disc + the blades
    T = tran3H(*x[0:3]) @ eulerH(*x[3:6])
    C0 = T @ tran3H(0, l, 0) @ eulerH(0, 0, α[0]) @ Ca  # we rotate the blades
    C1 = T @ tran3H(-l, 0, 0) @ eulerH(0, 0, -α[1]) @ Ca
    C2 = T @ tran3H(0, -l, 0) @ eulerH(0, 0, α[2]) @ Ca
    C3 = T @ tran3H(l, 0, 0) @ eulerH(0, 0, -α[3]) @ Ca
    M = T @ add1(array([[l, -l, 0, 0, 0], [0, 0, 0, l, -l], [0, 0, 0, 0, 0]]))
    draw3H(ax, M, 'grey', True, -1)  # body
    draw3H(ax, C0, 'green', True, -1)
    draw3H(ax, C1, 'black', True, -1)
    draw3H(ax, C2, 'red', True, -1)
    draw3H(ax, C3, 'blue', True, -1)


