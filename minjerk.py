import cvxpy as cp
import numpy as np
from timer import Timer


class MinJerkNonContinuous:
    def __init__(self, waypoints, traj_T_Kp=1):
        # self.T = traj_T
        self.traj_T_Kp = traj_T_Kp
        self.ideal_T = 0
        # print(self.poly_mat(self.T))
        # self.polyinv = np.linalg.inv(self.poly_mat(self.T))

        self.waypoints = waypoints
        self.waypoint_counter = 1

        self.coefs = np.zeros((6, 3))
        self.boundary_cond = np.array(
            [
                [waypoints[0, 0], waypoints[1, 0], 0, 0, 0, 0],
                [waypoints[0, 1], waypoints[1, 1], 0, 0, 0, 0],
                [waypoints[0, 2], waypoints[1, 2], 0, 0, 0, 0],
            ]
        ).T
        self.prev_boundary_cond = np.copy(self.boundary_cond)

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

    def poly_coef(self, boundary_cond, T):
        a = np.dot(np.linalg.inv(self.poly_mat(T)), boundary_cond)
        return a

    def next_waypoint(self):
        self.boundary_cond[0, :] = self.prev_boundary_cond[1, :]
        self.boundary_cond[2, :] = self.prev_boundary_cond[3, :]
        self.boundary_cond[4, :] = self.prev_boundary_cond[5, :]
        self.boundary_cond[1, :] = self.waypoints[self.waypoint_counter, :]
        new_T = self.euclidean_t_est(
            self.waypoints[self.waypoint_counter - 1],
            self.waypoints[self.waypoint_counter],
        )
        self.ideal_T += new_T
        self.coefs = self.poly_coef(self.boundary_cond, self.ideal_T)
        self.prev_boundary_cond = np.copy(self.boundary_cond)

        # print("new_t", self.ideal_T)
        # print(self.boundary_cond)
        self.traj_timer.set_period(self.ideal_T)
        # self.traj_timer.reset()

    def trajectroy(self, robot_cur_pose):
        if_cond, t = self.traj_timer.is_fire_time()
        if if_cond:
            self.print_poses(robot_cur_pose)
            self.waypoint_counter += 1
            if self.waypoint_counter == len(self.waypoints):
                self.is_done = True
            else:
                self.next_waypoint()

        des_state = np.dot(self.poly_mat(t)[[1, 3, 5]], self.coefs)
        #   [x y z x-dot y-dot z-dot x-ddot y-ddot z-ddot]
        return des_state.flatten()

    def start_minjerk_timer(self):
        self.traj_timer = Timer(self.ideal_T)  # 1 dummy number
        #   initial
        self.next_waypoint()

    def euclidean_t_est(self, point1, point2):
        return np.linalg.norm(point1 - point2) * self.traj_T_Kp

    def print_poses(self, robot_cur_pose):
        waypoint_pose = np.hstack(
            [self.waypoints[self.waypoint_counter], np.array([0, 0, 0])]
        )
        print("---> Waypoint: {0}".format(self.waypoint_counter))
        print(
            "---> Robot    Pose: {0}".format(
                np.array_str(robot_cur_pose, precision=2, suppress_small=True)
            )
        )
        print(
            "---> Waypoint Pose: {0}".format(
                np.array_str(waypoint_pose, precision=2, suppress_small=True)
            )
        )
        print()


class MinJerkContinuous:
    def __init__(self, waypoints, traj_T_Kp=1):
        self.traj_T_Kp = traj_T_Kp
        self.waypoints = waypoints
        self.waypoint_counter = 0
        self.times = np.zeros(len(waypoints))
        self.T_total = self.calc_t_total()
        self.coefs = self.calculate_3d_coefs(waypoints)
        print(self.times, self.waypoints)

        self.is_done = False

        self.traj_timer = None

    def calc_t_total(self):
        t_total = 0
        for i in range(len(self.waypoints) - 1):
            t_total += self.euclidean_t_est(self.waypoints[i], self.waypoints[i + 1])
            self.times[i + 1] = (i + 1) * 9  # t_total
        return t_total

    def poly_mat(self, T):
        return np.array(
            [
                [1, T, T**2, T**3, T**4, T**5],
                [0, 1, 2 * T, 3 * T**2, 4 * T**3, 5 * T**4],
                [0, 0, 2, 6 * T, 12 * T**2, 20 * T**3],
            ]
        )

    def trajectroy(self, robot_cur_pose):
        if_cond, t = self.traj_timer.is_fire_time()
        # t = self.traj_timer.current_time()
        if if_cond:
            if self.waypoint_counter == len(self.waypoints) - 1:
                self.is_done = True
            else:
                self.print_poses(robot_cur_pose)
                self.waypoint_counter += 1
                self.traj_timer.set_period(self.times[self.waypoint_counter - 1])

        _des_state = []
        for i in range(3):
            _des_state.append(
                np.dot(
                    self.poly_mat(t),
                    self.coefs[i][:, self.waypoint_counter],
                )
            )
        # print(_des_state)
        #   [x y z x-dot y-dot z-dot x-ddot y-ddot z-ddot]
        des_state = np.zeros(9)
        des_state[[0, 3, 6]] = _des_state[0].T
        des_state[[1, 4, 7]] = _des_state[1].T
        des_state[[2, 5, 8]] = _des_state[2].T
        return des_state

    def start_minjerk_timer(self):
        self.traj_timer = Timer(self.times[0])

    # fmt: off
    # fmt: on

    def calculate_3d_coefs(self, waypoints):
        # Define number of waypoints
        n = len(waypoints)

        T = self.times
        print(T)
        # Define the coefficients of the 5th degree polynomial for each dimension
        a_x = cp.Variable((6, n - 1))
        a_y = cp.Variable((6, n - 1))
        a_z = cp.Variable((6, n - 1))

        # Objective is to minimize jerk (third derivative of position with respect to time)
        objective = cp.Minimize(
            cp.sum(
                [
                    cp.sum_squares(a_x[5, :]),
                    cp.sum_squares(a_y[5, :]),
                    cp.sum_squares(a_z[5, :]),
                ]
            )
        )

        # Define the constraints
        constraints = []

        # Add constraints that ensure we hit each waypoint at the correct time
        for i in range(n - 1):
            constraints.append(
                a_x[:, i]
                @ np.array(
                    [
                        1,
                        T[i + 1],
                        T[i + 1] ** 2,
                        T[i + 1] ** 3,
                        T[i + 1] ** 4,
                        T[i + 1] ** 5,
                    ]
                )
                == waypoints[i + 1, 0]
            )
            constraints.append(
                a_y[:, i]
                @ np.array(
                    [
                        1,
                        T[i + 1],
                        T[i + 1] ** 2,
                        T[i + 1] ** 3,
                        T[i + 1] ** 4,
                        T[i + 1] ** 5,
                    ]
                )
                == waypoints[i + 1, 1]
            )
            constraints.append(
                a_z[:, i]
                @ np.array(
                    [
                        1,
                        T[i + 1],
                        T[i + 1] ** 2,
                        T[i + 1] ** 3,
                        T[i + 1] ** 4,
                        T[i + 1] ** 5,
                    ]
                )
                == waypoints[i + 1, 2]
            )

        # Add constraints for smooth and continuous velocity and acceleration
        for i in range(n - 2):
            constraints.append(
                a_x[:, i]
                @ np.array(
                    [
                        0,
                        1,
                        2 * T[i + 1],
                        3 * T[i + 1] ** 2,
                        4 * T[i + 1] ** 3,
                        5 * T[i + 1] ** 4,
                    ]
                )
                == a_x[:, i + 1]
                @ np.array(
                    [
                        0,
                        1,
                        2 * T[i + 2],
                        3 * T[i + 2] ** 2,
                        4 * T[i + 2] ** 3,
                        5 * T[i + 2] ** 4,
                    ]
                )
            )
            constraints.append(
                a_y[:, i]
                @ np.array(
                    [
                        0,
                        1,
                        2 * T[i + 1],
                        3 * T[i + 1] ** 2,
                        4 * T[i + 1] ** 3,
                        5 * T[i + 1] ** 4,
                    ]
                )
                == a_y[:, i + 1]
                @ np.array(
                    [
                        0,
                        1,
                        2 * T[i + 2],
                        3 * T[i + 2] ** 2,
                        4 * T[i + 2] ** 3,
                        5 * T[i + 2] ** 4,
                    ]
                )
            )
            constraints.append(
                a_z[:, i]
                @ np.array(
                    [
                        0,
                        1,
                        2 * T[i + 1],
                        3 * T[i + 1] ** 2,
                        4 * T[i + 1] ** 3,
                        5 * T[i + 1] ** 4,
                    ]
                )
                == a_z[:, i + 1]
                @ np.array(
                    [
                        0,
                        1,
                        2 * T[i + 2],
                        3 * T[i + 2] ** 2,
                        4 * T[i + 2] ** 3,
                        5 * T[i + 2] ** 4,
                    ]
                )
            )
            constraints.append(
                a_x[:, i]
                @ np.array(
                    [0, 0, 2, 6 * T[i + 1], 12 * T[i + 1] ** 2, 20 * T[i + 1] ** 3]
                )
                == a_x[:, i + 1]
                @ np.array(
                    [0, 0, 2, 6 * T[i + 2], 12 * T[i + 2] ** 2, 20 * T[i + 2] ** 3]
                )
            )
            constraints.append(
                a_y[:, i]
                @ np.array(
                    [0, 0, 2, 6 * T[i + 1], 12 * T[i + 1] ** 2, 20 * T[i + 1] ** 3]
                )
                == a_y[:, i + 1]
                @ np.array(
                    [0, 0, 2, 6 * T[i + 2], 12 * T[i + 2] ** 2, 20 * T[i + 2] ** 3]
                )
            )
            constraints.append(
                a_z[:, i]
                @ np.array(
                    [0, 0, 2, 6 * T[i + 1], 12 * T[i + 1] ** 2, 20 * T[i + 1] ** 3]
                )
                == a_z[:, i + 1]
                @ np.array(
                    [0, 0, 2, 6 * T[i + 2], 12 * T[i + 2] ** 2, 20 * T[i + 2] ** 3]
                )
            )

        # Initial and final velocity and acceleration constraints
        constraints += [
            a_x[1, 0] == 0,
            a_x[2, 0] == 0,
            a_x[1, -1] == 0,
            a_x[2, -1] == 0,
        ]
        constraints += [
            a_y[1, 0] == 0,
            a_y[2, 0] == 0,
            a_y[1, -1] == 0,
            a_y[2, -1] == 0,
        ]
        constraints += [
            a_z[1, 0] == 0,
            a_z[2, 0] == 0,
            a_z[1, -1] == 0,
            a_z[2, -1] == 0,
        ]

        # Define the problem
        problem = cp.Problem(objective, constraints)

        # Solve the problem
        problem.solve()

        return a_x.value, a_y.value, a_z.value

    def euclidean_t_est(self, point1, point2):
        return np.linalg.norm(point1 - point2) * self.traj_T_Kp

    def print_poses(self, robot_cur_pose):
        waypoint_pose = np.hstack(
            [self.waypoints[self.waypoint_counter], np.array([0, 0, 0])]
        )
        print("---> Waypoint: {0}".format(self.waypoint_counter))
        print(
            "---> Robot    Pose: {0}".format(
                np.array_str(robot_cur_pose, precision=2, suppress_small=True)
            )
        )
        print(
            "---> Waypoint Pose: {0}".format(
                np.array_str(waypoint_pose, precision=2, suppress_small=True)
            )
        )
        print()
