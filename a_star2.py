import numpy as np
from collections import defaultdict


class AStar:
    def __init__(self, c_all, c_obs, resolution=np.array([1, 1, 1])):
        self.c_free = {tuple(map(int, point / resolution)) for point in c_all} - {
            tuple(map(int, point / resolution)) for point in c_obs
        }
        self.resolution = resolution
        self.came_from = {}
        self.cost_so_far = defaultdict(lambda: float("inf"))

    def heuristic(self, a, b):
        return np.linalg.norm(np.array(a) - np.array(b))

    def a_star_search(self, start, goal):
        start, goal = tuple(map(int, start / self.resolution)), tuple(
            map(int, goal / self.resolution)
        )
        frontier = [(0, start)]
        self.came_from[start] = None
        self.cost_so_far[start] = 0

        while frontier:
            min_index = np.argmin([i[0] for i in frontier])
            _, current = frontier[min_index]
            frontier = frontier[:min_index] + frontier[min_index + 1 :]

            if current == goal:
                break

            for next in self.get_neighbors(current):
                if next not in self.c_free:
                    continue
                new_cost = self.cost_so_far[current] + self.heuristic(current, next)
                if new_cost < self.cost_so_far[next]:
                    self.cost_so_far[next] = new_cost
                    priority = new_cost + self.heuristic(goal, next)
                    frontier.append((priority, next))
                    self.came_from[next] = current

        return self.reconstruct_path(start, goal)

    def get_neighbors(self, current):
        deltas = [-1, 0, 1]
        neighbors = [
            (current[0] + dx, current[1] + dy, current[2] + dz)
            for dx in deltas
            for dy in deltas
            for dz in deltas
        ]
        neighbors.remove(current)  # a point is not its own neighbor
        return neighbors

    def get_neighbors(self, current):
        deltas = [-1, 0, 1]
        neighbors = [
            (current[0] + dx, current[1] + dy, current[2] + dz)
            for dx in deltas
            for dy in deltas
            for dz in deltas
        ]
        neighbors.remove(current)  # a point is not its own neighbor
        return neighbors

    def reconstruct_path(self, start, goal):
        current = goal
        path = []
        while current != start:
            path.append(np.array(current) * self.resolution)
            current = self.came_from[current]
        path.append(
            np.array(start) * self.resolution
        )  # don't forget to add the start position
        path.reverse()
        return self.remove_collinear_points(np.array(path))

    def collinear(self, p1, p2, p3):
        x1, y1, z1 = p1
        x2, y2, z2 = p2
        x3, y3, z3 = p3

        return (
            abs((y2 - y1) * (z3 - z1) - (y3 - y1) * (z2 - z1)) < 1e-2
            and abs((z2 - z1) * (x3 - x1) - (z3 - z1) * (x2 - x1)) < 1e-2
            and abs((x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1)) < 1e-2
        )

    def remove_collinear_points(self, path):
        new_path = [path[0]]
        for i in range(1, len(path) - 1):
            if not self.collinear(path[i - 1], path[i], path[i + 1]):
                new_path.append(path[i])
        new_path.append(path[-1])
        return np.array(new_path)

    # #   Another version of A*
    # a_star2 = AStar(
    #     c_all,
    #     c_obs,
    #     # config.map_resolution
    # )
    # waypoints = a_star2.a_star_search(np.hstack([q0[0:2], zt]), qh[0:3])
