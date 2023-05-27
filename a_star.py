from math import pi, sqrt
from collections import OrderedDict
from scipy.spatial.distance import euclidean
from config import DIAGONAL_PATH
from map import MapReader
import numpy as np
from config import *


def neighbors(current, boundaries, check_bound=False,diagonal=DIAGONAL_PATH):
    # define the list of 4 neighbors: left,right,up,down
    # neighbors = [(0,0,1),(0, 1,0), (1, 0,0), (0, -1,0), (-1, 0,0), (0,0,-1)]

    X_lim, Y_lim, Z_lim = boundaries
    if not diagonal:
        neighbors = [
            (0, 0, 1),
            (0, 1, 0),
            (1, 0, 0),
            (0, -1, 0),
            (-1, 0, 0),
            (0, 0, -1),
        ]
    else:
        # print("in")
        neighbors = [
            (0, 0, 1),
            (0, 1, 1),
            (1, 1, 1),
            (1, 0, 1),
            (1, -1, 1),
            (0, -1, 1),
            (-1, -1, 1),
            (-1, 0, 1),
            (-1, 1, 1),
            (0, 1, 0),
            (1, 1, 0),
            (1, 0, 0),
            (1, -1, 0),
            (0, -1, 0),
            (-1, -1, 0),
            (-1, 0, 0),
            (0, 0, -1),
            (0, 1, -1),
            (1, 1, -1),
            (1, 0, -1),
            (1, -1, -1),
            (0, -1, -1),
            (-1, -1, -1),
            (-1, 0, -1),
        ]

    if not check_bound:
        # print("in  2")
        return [(current[0] + nbr[0], current[1] + nbr[1], current[2] + nbr[2]) for nbr in neighbors]

    res = []
    for nbr in neighbors:
        flag_x = abs(current[0] + nbr[0] - X_lim/2) <= X_lim/2
        flag_y = abs(current[1] + nbr[1] - Y_lim/2) <= Y_lim/2
        flag_z = abs(current[2] + nbr[2] - Z_lim/2) <= Z_lim/2
        if flag_x and flag_y and flag_z:
            res.append((current[0] + nbr[0], current[1] + nbr[1], current[2] + nbr[2]))
    return res


def heuristic_distance(candidate, goal):
    # distance:
    # print("candidate is:", candidate)
    # print("goal is:", goal)
    # res = sum(abs(val1 - val2) for val1, val2 in zip(candidate, goal))
    # print("val1 =",candidate)
    # print("val2 =", val2)
    res = euclidean(candidate, goal)
    # print("result is:",res)
    return res

def transition_cost(pta, ptb):
    return euclidean(pta, ptb)

def set_to_list(set):
    return [list(e) for e in set]


def compute_slope_3d(pta, ptb):
    # print("pta,ptb is",pta,ptb)
    slope_x = ptb[0] - pta[0] + 1e6
    slope_y = ptb[1] - pta[1]
    slope_z = ptb[2] - pta[2]

    # Check for division by zero to avoid ZeroDivisionError
    if slope_x == 0 and slope_y == 0 and slope_z == 0:
        return "The line is not well-defined."

    if slope_x == 0:
        slope = 0
    else:
        slope = slope_y / slope_x  # Calculate the slope in the xy-plane

    return slope, slope_z


def merge_waypoints(path):
    # prune_path = [path[0]]
    # print("==",path)
    path.append((float("inf"), float("-inf"), float("inf")))
    # print("==", path)
    # prune_path = [path[0]]
    prune_path = []
    last_slope, _, _, last_slope_z = 0, 0, 0, 0

    for wid in range(1, len(path)):
        slope, slope_z = compute_slope_3d(path[wid - 1], path[wid])
        if last_slope == slope and last_slope_z == slope_z:
            continue
        else:
            prune_path.append(path[wid - 1])
        last_slope = slope
        last_slope_z = slope_z
    return prune_path


def get_path_from_A_star(start, goal, take_off_height, obstacles, boundaries, prune, check_bound=False):
    # def get_path_from_A_star(start, goal, take_off_height, obstacles):
    # input  start: integer 3-tuple of the current grid, e.g., (0, 0,0)
    #        goal: integer 2-tuple  of the goal grid, e.g., (5, 1,0)
    #        obstacles: a list of grids marked as obstacles, e.g., [(2, -1), (2, 0), ...]
    # output path: a list of grids connecting start to goal, e.g., [(1, 0), (1, 1), ...]
    #   note that the path should contain the goal but not the start
    #   e.g., the path from (0, 0) to (2, 2) should be [(1, 0), (1, 1), (2, 1), (2, 2)]

    # Open List
    take_off_start = (start[0], start[1], start[2] + take_off_height)
    open_list = [(0, take_off_start)]  # ( cost, (start) )

    # Past Cost List
    past_cost = {}
    past_cost[take_off_start] = 0

    # Parent List
    parent = {}
    parent[start] = None

    # Closed List
    closed_list = []

    # print("check_bound is ",check_bound)
    while open_list:
        current = open_list.pop(0)[1]
        closed_list.append(current)
        # Check if current point is the goal
        if current == goal:
            # Sort Open List(outside For loop)
            open_list.sort()
            break
        for candidate in neighbors(current, boundaries, check_bound):
            if candidate in obstacles:
                continue
            if candidate in closed_list:
                continue
            # print(" [current]=", current)
            # print(" [past_cost]=", past_cost)
            # print(" past_cost[current]=", past_cost[current])

            # don't do it here, waste of time
            # if candidate[0] >=X_lim or candidate[1] >=Y_lim or candidate[2] >=Z_lim:
            #     continue

            # for non-diagonal move
            # tentative_past_cost = past_cost[current] + 1

            # for diagonal move
            tentative_past_cost = past_cost[current] + transition_cost(current,candidate)

            # if candidate not in past_cost or tentative_past_cost < past_cost[current]:
            if candidate not in past_cost or tentative_past_cost < past_cost[candidate]:
                # if tentative_past_cost < past_cost[candidate]:
                past_cost[candidate] = tentative_past_cost
                parent[candidate] = current
                new_cost = past_cost[candidate] + heuristic_distance(candidate, goal)
                open_list.append((new_cost, candidate))

    # Path Reconstruction (no start in the path)
    # goal through parent list

    path = [goal]
    while path[-1] != take_off_start:
        # print(parent[path[-1]])
        path.append(parent[path[-1]])
    path.reverse()
    # path.insert(0,(-1,-1,-1))
    path.insert(0, start)
    # path.remove(start)
    if prune:
        path = merge_waypoints(path)
    return set_to_list(path)
