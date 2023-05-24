import cvxpy as cp
import numpy as np
from timer import Timer


# class MinJerk:
#     def __init__(self, traj_T, zt, waypoints, start_pos, goal_pos):
#         self.T = traj_T
#         print(self.poly_mat(self.T))
#         self.polyinv = np.linalg.inv(self.poly_mat(self.T))

#         self.waypoints = waypoints
#         self.waypoint_counter = 0
#         # self.start_pos = start_pos
#         self.goal_pos = goal_pos

#         self.coefs = np.zeros((6, 3))
#         self.boundary_cond = np.array(
#             [
#                 [start_pos[0], waypoints[0, 0], 0, 0, 0, 0],
#                 [start_pos[1], waypoints[0, 1], 0, 0, 0, 0],
#                 [zt, waypoints[0, 2], 0, 0, 0, 0],
#             ]
#         ).T
#         self.prev_boundary_cond = np.copy(self.boundary_cond)

#         self.is_done = False

#         self.traj_timer = None

#     def poly_mat(self, T):
#         return np.array(
#             [
#                 [0, 0, 0, 0, 0, 1],
#                 [T**5, T**4, T**3, T**2, T, 1],
#                 [0, 0, 0, 0, 1, 0],
#                 [5 * T**4, 4 * T**3, 3 * T**2, 2 * T, 1, 0],
#                 [0, 0, 0, 2, 0, 0],
#                 [20 * T**3, 12 * T**2, 6 * T, 2, 0, 0],
#             ]
#         )

#     def poly_coef(self, boundary_cond):
#         a = np.dot(self.polyinv, boundary_cond)
#         return a

#     def next_waypoint(self):
#         self.boundary_cond[0, :] = self.prev_boundary_cond[1, :]
#         self.boundary_cond[2, :] = self.prev_boundary_cond[3, :]
#         self.boundary_cond[4, :] = self.prev_boundary_cond[5, :]
#         self.boundary_cond[1, :] = self.waypoints[self.waypoint_counter, :]

#         self.coefs = self.poly_coef(self.boundary_cond)
#         print(self.boundary_cond)

#     def trajectroy(self):
#         if_cond, t = self.traj_timer.is_fire_time()
#         if if_cond:
#             self.waypoint_counter += 1
#             if self.waypoint_counter == len(self.waypoints):
#                 #   TODO: finish mission got to idle
#                 self.is_done = True
#             else:
#                 self.next_waypoint()

#         des_state = np.dot(self.poly_mat(t)[[1, 3, 5]], self.coefs)
#         #   [x y z x-dot y-dot z-dot x-ddot y-ddot z-ddot]
#         return des_state.flatten()

#     def start_minjerk_timer(self):
#         self.traj_timer = Timer(self.T)
#         #   initial
#         self.next_waypoint()


# class MinJerk:
#     def __init__(self, traj_T, zt, waypoints, start_pos, goal_pos):
#         self.T = traj_T
#         self.coefs = self.calculate_coefs(start_pos, zt, waypoints, traj_T)
#         print(self.coefs[:, :, 0])
#         self.waypoints = waypoints
#         self.waypoint_counter = 0
#         self.goal_pos = goal_pos

#         self.is_done = False

#         self.traj_timer = None

#     def poly_mat(self, T):
#         return np.array(
#             [
#                 [0, 0, 0, 0, 0, 1],
#                 [T**5, T**4, T**3, T**2, T, 1],
#                 [0, 0, 0, 0, 1, 0],
#                 [5 * T**4, 4 * T**3, 3 * T**2, 2 * T, 1, 0],
#                 [0, 0, 0, 2, 0, 0],
#                 [20 * T**3, 12 * T**2, 6 * T, 2, 0, 0],
#             ]
#         )

#     def trajectroy(self):
#         if_cond, t = self.traj_timer.is_fire_time()
#         if if_cond:
#             if self.waypoint_counter == len(self.waypoints) - 1:
#                 self.is_done = True
#             else:
#                 self.waypoint_counter += 1

#         des_state = np.dot(
#             self.poly_mat(t)[[1, 3, 5]], self.coefs[:, self.waypoint_counter, :]
#         )
#         #   [x y z x-dot y-dot z-dot x-ddot y-ddot z-ddot]
#         return des_state.flatten()

#     def start_minjerk_timer(self):
#         self.traj_timer = Timer(self.T)

#     # fmt: off


#     def calculate_coefs(self, start_pos, zt, waypoints, T):
#         n = len(waypoints)
#         boundary = np.zeros((3 * n + 3, 3))
#         poly = np.zeros((3 * n + 3, 6 * n, 3))

#         boundary[0, 0:2] = start_pos[0:2]
#         boundary[0, 2] = zt
#         boundary[1, 0:3] = waypoints[0, 0:3]
#         for i, point in enumerate(waypoints):
#             if i == 0:
#                 continue
#             boundary[6 + 3 * (i - 1), 0:3] = point[0:3]

#         for i in range(n):
#             if i == 0:
#                 poly[0:6, 0:6, 0] = poly[0:6, 0:6, 1] = poly[
#                     0:6, 0:6, 2
#                 ] = self.poly_mat(T)
#                 poly[3, 6 * (i + 1) + 4, 0:3] = -1
#                 poly[5, 6 * (i + 1) + 3, 0:3] = -2
#                 continue

#             poly[3 * i + 3 : 3 * i + 6, 6 * i : 6 * i + 6, 0] = poly[
#                 3 * i + 3 : 3 * i + 6, 6 * i : 6 * i + 6, 1
#             ] = poly[3 * i + 3 : 3 * i + 6, 6 * i : 6 * i + 6, 2] = self.poly_mat(T)[
#                 [1, 3, 5]
#             ]
#             if i != n - 1:
#                 poly[4 + 3 * i, 6 * (i + 1) + 4, 0:3] = -1
#                 poly[5 + 3 * i, 6 * (i + 1) + 3, 0:3] = -2

#         coefx = np.linalg.lstsq(poly[:, :, 0], boundary[:, 0], rcond=None)[0]
#         coefy = np.linalg.lstsq(poly[:, :, 1], boundary[:, 1], rcond=None)[0]
#         coefz = np.linalg.lstsq(poly[:, :, 2], boundary[:, 2], rcond=None)[0]

#         return np.hstack([coefx, coefy, coefz]).reshape((6,n,3))


class MinJerk:
    def __init__(self, traj_T, zt, waypoints, start_pos, goal_pos):
        self.T = traj_T
        self.coefs = self.calculate_coefs(start_pos, zt, waypoints, traj_T)
        print(self.coefs[:, :, 0])
        self.waypoints = waypoints
        self.waypoint_counter = 0
        self.goal_pos = goal_pos

        self.is_done = False

        self.traj_timer = None

    def poly_mat(self, T):
        return np.array(
            [
                [0, 0, 0, 0, 0, 1],
                [T**5, T**4, T**3, T**2, T, 1],
                [0, 0, 0, 0, 1, 0],
                [5 * T**4, 4 * T**3, 3 * T**2, 2 * T, 1, 0],
                [0, 0, 0, 2, 0, 0],
                [20 * T**3, 12 * T**2, 6 * T, 2, 0, 0],
            ]
        )

    def trajectroy(self):
        if_cond, t = self.traj_timer.is_fire_time()
        if if_cond:
            if self.waypoint_counter == len(self.waypoints) - 1:
                self.is_done = True
            else:
                self.waypoint_counter += 1

        des_state = np.dot(
            self.poly_mat(t)[[1, 3, 5]], self.coefs[:, self.waypoint_counter, :]
        )
        #   [x y z x-dot y-dot z-dot x-ddot y-ddot z-ddot]
        return des_state.flatten()

    def start_minjerk_timer(self):
        self.traj_timer = Timer(self.T)

    # fmt: off


    def calculate_coefs(self, start_pos, zt, waypoints, T):
        n = len(waypoints)
        boundary = np.zeros((3 * n + 3, 3))
        poly = np.zeros((3 * n + 3, 6 * n, 3))

        boundary[0, 0:2] = start_pos[0:2]
        boundary[0, 2] = zt
        boundary[1, 0:3] = waypoints[0, 0:3]
        for i, point in enumerate(waypoints):
            if i == 0:
                continue
            boundary[6 + 3 * (i - 1), 0:3] = point[0:3]

        for i in range(n):
            if i == 0:
                poly[0:6, 0:6, 0] = poly[0:6, 0:6, 1] = poly[
                    0:6, 0:6, 2
                ] = self.poly_mat(T)
                poly[3, 6 * (i + 1) + 4, 0:3] = -1
                poly[5, 6 * (i + 1) + 3, 0:3] = -2
                continue

            poly[3 * i + 3 : 3 * i + 6, 6 * i : 6 * i + 6, 0] = poly[
                3 * i + 3 : 3 * i + 6, 6 * i : 6 * i + 6, 1
            ] = poly[3 * i + 3 : 3 * i + 6, 6 * i : 6 * i + 6, 2] = self.poly_mat(T)[
                [1, 3, 5]
            ]
            if i != n - 1:
                poly[4 + 3 * i, 6 * (i + 1) + 4, 0:3] = -1
                poly[5 + 3 * i, 6 * (i + 1) + 3, 0:3] = -2

        coefx = np.linalg.lstsq(poly[:, :, 0], boundary[:, 0], rcond=None)[0]
        coefy = np.linalg.lstsq(poly[:, :, 1], boundary[:, 1], rcond=None)[0]
        coefz = np.linalg.lstsq(poly[:, :, 2], boundary[:, 2], rcond=None)[0]

        return np.hstack([coefx, coefy, coefz]).reshape((6,n,3))


import numpy as np


def calculate_coefs(self, waypoints, T):
    n = len(waypoints)
    # Define your waypoints, times, initial and final velocities and accelerations
    # times = np.array([0, 5, 10, 15])  # Time for each waypoint
    times = np.zeros(n)
    velocities = np.zeros(n)  # Velocity at waypoints
    accelerations = np.zeros(n)  # Acceleration at waypoints

    for i in range(n):
        times[i] = i * T
        if i == 0 or i == n - 1:
            continue
        velocities[i] = None
        accelerations[i] = None
    print(times)
    print(velocities)
    print(accelerations)
    # We assume zero velocity and acceleration at the start and end, and we don't know the velocity and acceleration at the intermediate waypoints
    # velocities = np.array([0, None, None, 0])  # Velocity at waypoints
    # accelerations = np.array([0, None, None, 0])  # Acceleration at waypoints

    # Define the variable for the coefficients of the polynomials
    coeffs = cp.Variable((6, n))

    # Define the objective function
    objective = cp.Minimize(cp.sum_squares(coeffs[0, :]))

    constraints = []
    for i in range(n):
        T = times[i]

        # Add constraints for the position, velocity and acceleration at each waypoint
        constraints += [
            coeffs[5, i]
            + coeffs[4, i] * T
            + coeffs[3, i] * T**2
            + coeffs[2, i] * T**3
            + coeffs[1, i] * T**4
            + coeffs[0, i] * T**5
            == waypoints[i],
        ]

        if not velocities[i].isnan():
            constraints += [
                coeffs[4, i]
                + 2 * coeffs[3, i] * T
                + 3 * coeffs[2, i] * T**2
                + 4 * coeffs[1, i] * T**3
                + 5 * coeffs[0, i] * T**4
                == velocities[i],
            ]

        if not accelerations[i].isnan():
            constraints += [
                2 * coeffs[3, i]
                + 6 * coeffs[2, i] * T
                + 12 * coeffs[1, i] * T**2
                + 20 * coeffs[0, i] * T**3
                == accelerations[i],
            ]

    # Add constraints for the velocity and acceleration continuity at the intermediate waypoints
    for i in range(n - 1):
        T = times[i + 1]

        # Velocity continuity
        constraints += [
            coeffs[4, i]
            + 2 * coeffs[3, i] * T
            + 3 * coeffs[2, i] * T**2
            + 4 * coeffs[1, i] * T**3
            + 5 * coeffs[0, i] * T**4
            == coeffs[4, i + 1],
        ]

        # Acceleration continuity
        constraints += [
            2 * coeffs[3, i]
            + 6 * coeffs[2, i] * T
            + 12 * coeffs[1, i] * T**2
            + 20 * coeffs[0, i] * T**3
            == 2 * coeffs[3, i + 1],
        ]

    # Define and solve the problem
    prob = cp.Problem(objective, constraints)
    result = prob.solve()

    # Get the coefficients
    return coeffs.value


print(calculate_coefs(1, np.array([1, 2, 3]), 5))
