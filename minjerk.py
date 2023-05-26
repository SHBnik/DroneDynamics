import cvxpy as cp
import numpy as np
from timer import Timer


class MinJerkNonContinuous:
    def __init__(self, traj_T, waypoints):
        self.T = traj_T
        # print(self.poly_mat(self.T))
        self.polyinv = np.linalg.inv(self.poly_mat(self.T))

        self.waypoints = waypoints
        self.waypoint_counter = 0

        self.coefs = np.zeros((6, 3))
        self.boundary_cond = np.array(
            [
                [waypoints[0, 0], waypoints[1, 0], 0, 0, 0, 0],
                [waypoints[0, 0], waypoints[1, 1], 0, 0, 0, 0],
                [waypoints[0, 0], waypoints[1, 2], 0, 0, 0, 0],
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

    def poly_coef(self, boundary_cond):
        a = np.dot(self.polyinv, boundary_cond)
        return a

    def next_waypoint(self):
        self.boundary_cond[0, :] = self.prev_boundary_cond[1, :]
        self.boundary_cond[2, :] = self.prev_boundary_cond[3, :]
        self.boundary_cond[4, :] = self.prev_boundary_cond[5, :]
        self.boundary_cond[1, :] = self.waypoints[self.waypoint_counter, :]

        self.coefs = self.poly_coef(self.boundary_cond)
        self.prev_boundary_cond = np.copy(self.boundary_cond)
        # print(self.boundary_cond)

    def trajectroy(self):
        if_cond, t = self.traj_timer.is_fire_time()
        if if_cond:
            self.waypoint_counter += 1
            if self.waypoint_counter == len(self.waypoints):
                #   TODO: finish mission got to idle
                self.is_done = True
            else:
                self.next_waypoint()

        des_state = np.dot(self.poly_mat(t)[[1, 3, 5]], self.coefs)
        #   [x y z x-dot y-dot z-dot x-ddot y-ddot z-ddot]
        return des_state.flatten()

    def start_minjerk_timer(self):
        self.traj_timer = Timer(self.T)
        #   initial
        self.next_waypoint()

    def euclidean_t_est(self, point1, point2):
        point1, point2 = np.array(point1), np.array(point2)
        return np.linalg.norm(point1 - point2)


class MinJerkContinuous:
    def __init__(self, traj_T, waypoints):
        self.T = traj_T
        self.coefs = self.calculate_3d_coefs(waypoints, traj_T)

        self.waypoints = waypoints
        self.waypoint_counter = 1

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
        t = self.traj_timer.current_time()
        if if_cond:
            if self.waypoint_counter == len(self.waypoints) - 1:
                self.is_done = True
            else:
                self.waypoint_counter += 1
        _des_state = []
        for i in range(3):
            _des_state.append(
                np.dot(
                    self.poly_mat(t)[[1, 3, 5]],
                    self.coefs[i][:, self.waypoint_counter],
                )
            )
        #   [x y z x-dot y-dot z-dot x-ddot y-ddot z-ddot]
        des_state = np.zeros(9)
        des_state[[0, 3, 6]] = _des_state[0].T
        des_state[[1, 4, 7]] = _des_state[1].T
        des_state[[2, 5, 8]] = _des_state[2].T
        return des_state

    def start_minjerk_timer(self):
        self.traj_timer = Timer(self.T)

    # fmt: off


    def calculate_3d_coefs(self, waypoints, T):
        # coef = np.zeros((3, 6,len(waypoints)))
        coefx = np.zeros((6,len(waypoints)))
        coefy = np.zeros((6,len(waypoints)))
        coefz = np.zeros((6,len(waypoints)))
        coefx = self.calculate_1d_coefs(waypoints[:, 0], T)
        coefy = self.calculate_1d_coefs(waypoints[:, 1], T)
        coefz = self.calculate_1d_coefs(waypoints[:, 2], T)

        return coefx,coefy,coefz


    def calculate_1d_coefs(self, waypoints, traj_T):
        n = len(waypoints)
        # Define your waypoints, times, initial and final velocities and accelerations
        # times = np.array([0, 5, 10, 15])  # Time for each waypoint
        times = np.zeros(n)
        velocities = np.zeros(n)  # Velocity at waypoints
        accelerations = np.zeros(n)  # Acceleration at waypoints

        for i in range(n):
            times[i] = i * traj_T
            # if i == 0 or i == n - 1:
            #     continue
            # velocities[i] = -100  #   dummy number
            # accelerations[i] = 

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
            if i == 0 or i == n-1:

            # if velocities[i] != -100:
                constraints += [
                    coeffs[4, i]
                    + 2 * coeffs[3, i] * T
                    + 3 * coeffs[2, i] * T**2
                    + 4 * coeffs[1, i] * T**3
                    + 5 * coeffs[0, i] * T**4
                    == velocities[i],
                ]

            # if accelerations[i] != -100:
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
