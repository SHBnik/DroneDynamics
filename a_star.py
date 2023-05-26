from math import pi, sqrt
from collections import OrderedDict
from scipy.spatial.distance import euclidean
from config import DIAGONAL_PATH

def neighbors(current, diagonal=DIAGONAL_PATH):
    # define the list of 4 neighbors: left,right,up,down
    # neighbors = [(0,0,1),(0, 1,0), (1, 0,0), (0, -1,0), (-1, 0,0), (0,0,-1)]

    if not diagonal:
        neighbors = [(0, 0, 1), (0, 1, 0), (1, 0, 0), (0, -1, 0), (-1, 0, 0), (0, 0, -1)]
    else:
        # print("in")
        neighbors = [(0, 0, 1),(0, 1, 1), (1,1,1),(1,0,1),(1,-1,1),(0,-1,1),(-1,-1,1),(-1,0,1),(-1,1,1),
                     (0, 1, 0), (1, 1, 0), (1, 0, 0), (1, -1, 0), (0, -1, 0), (-1, -1, 0), (-1, 0, 0),
                     (0, 0, -1), (0, 1, -1), (1, 1, -1), (1, 0, -1), (1, -1, -1), (0, -1, -1), (-1, -1, -1), (-1, 0, -1) ]


    return [(current[0] + nbr[0], current[1] + nbr[1], current[2] + nbr[2]) for nbr in neighbors]
    # neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0)]
    # return [ (current[0]+nbr[0], current[1]+nbr[1]) for nbr in neighbors ]


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


def set_to_list(set):
    return [list(e) for e in set]

def compute_slope_3d(pta,ptb):
    # print("pta,ptb is",pta,ptb)
    slope_x = (ptb[0] - pta[0])
    slope_y = (ptb[1] - pta[1])
    slope_z = (ptb[2] - pta[2])

    # Check for division by zero to avoid ZeroDivisionError
    if slope_x == 0 and slope_y == 0 and slope_z == 0:
        return "The line is not well-defined."

    if slope_x==0:
        slope = 0
    else:
        slope = slope_y / slope_x  # Calculate the slope in the xy-plane

    return slope,  slope_z


def merge_waypoints(path):
    # prune_path = [path[0]]
    # print("==",path)
    path.append((float('inf'),float('-inf'),float('inf')))
    # print("==", path)
    # prune_path = [path[0]]
    prune_path=[]
    last_slope, _, _, last_slope_z= 0,0,0,0

    for wid in range(1,len(path)):
        slope, slope_z = compute_slope_3d(path[wid-1],path[wid])
        if last_slope==slope and last_slope_z ==slope_z:
            continue
        else:
            prune_path.append(path[wid-1])
        last_slope =slope
        last_slope_z = slope_z
    return prune_path

def get_path_from_A_star(start, goal, take_off_height , obstacles, boundaries,  prune, check_bound=False):
# def get_path_from_A_star(start, goal, take_off_height, obstacles):
    # input  start: integer 3-tuple of the current grid, e.g., (0, 0,0)
    #        goal: integer 2-tuple  of the goal grid, e.g., (5, 1,0)
    #        obstacles: a list of grids marked as obstacles, e.g., [(2, -1), (2, 0), ...]
    # output path: a list of grids connecting start to goal, e.g., [(1, 0), (1, 1), ...]
    #   note that the path should contain the goal but not the start
    #   e.g., the path from (0, 0) to (2, 2) should be [(1, 0), (1, 1), (2, 1), (2, 2)]

    # Open List
    take_off_start =(start[0],start[1],start[2]+take_off_height)
    open_list = [(0, take_off_start)]  # ( cost, (start) )

    # Past Cost List
    past_cost = {}
    past_cost[take_off_start] = 0

    # Parent List
    parent = {}
    parent[start] = None

    # Closed List
    closed_list = []

    while open_list:
        current = open_list.pop(0)[1]
        closed_list.append(current)
        # Check if current point is the goal
        if current == goal:
            # Sort Open List(outside For loop)
            open_list.sort()
            break
        for candidate in neighbors(current):
            if candidate in obstacles:
                continue
            if candidate in closed_list:
                continue
            # print(" [current]=", current)
            # print(" [past_cost]=", past_cost)
            # print(" past_cost[current]=", past_cost[current])
            tentative_past_cost = past_cost[current] + 1
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
    path.insert(0,start)
    # path.remove(start)
    if prune:
        path = merge_waypoints(path)
    return set_to_list(path)


if __name__ == '__main__':
    # start = (0, 0)  # this is a tuple data structure in Python initialized with 2 integers
    # goal = (-5, -2)
    # obstacles = [(-2, 1), (-2, 0), (-2, -1), (-2, -2), (-4, -2), (-4, -3)]
    # path = get_path_from_A_star(start, goal, obstacles)
    # print(path)
    #[(0, 1), (0, 2), (-1, 2), (-2, 2), (-3, 2), (-3, 1), (-3, 0), (-3, -1), (-4, -1), (-5, -1), (-5, -2)]

    start = (0, 0,0)  # this is a tuple data structure in Python initialized with 2 integers
    goal = (5, 2,10)
    obstacles = [(-2, 1,0), (-2, 0,0), (-2, -1,0), (-2, -2,0), (-4, -2,0), (-4, -3,0)]
    take_off_height=3
    # path = get_path_from_A_star(start, goal, take_off_height,obstacles)

    boundaries= (20,20,20)
    # take_off_height=1

    # goal = (4, 4, 9)
    path = get_path_from_A_star(start, goal, take_off_height , obstacles, boundaries, prune=False, check_bound=False)
    print(path)
#     [(0, 0, 0), (0, 0, 3), (0, 0, 4), (0, 0, 5), (0, 0, 6), (0, 0, 7), (0, 0, 8), (0, 0, 9), (0, 0, 10),
#      -->  (0, 1, 10), (0, 2, 10), (1, 2, 10), (2, 2, 10), (3, 2, 10), (4, 2, 10), (5, 2, 10)]


    # cordsA = (1, 1)
    # cordsB = (2, 2)
    # print(euclidean(cordsA, cordsB))

    # 26 neighbors result
    # [[0, 0, 0], [0, 0, 3], [0, 0, 4], [0, 0, 5], [1, 1, 6], [2, 2, 7], [3, 3, 8], [4, 3, 9], [5, 2, 10]]
    # goal = (4, 4, 9)
    path = get_path_from_A_star(start, goal, take_off_height, obstacles, boundaries, prune=True, check_bound=False)
    print("after prune")
    print(path)
    # [[0, 0, 0], [0, 0, 3], [0, 0, 5], [3, 3, 8], [4, 3, 9], [5, 2, 10]]