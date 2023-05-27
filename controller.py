import numpy as np
import threading
from timer import Timer, Counter
from trajectory import TrajectoryGenerator2, TrajectoryGenerator1


class PDPosition:
    def __init__(self, kp, kd):
        self.k = np.array([kp, kd, 1])

    def update(self, signal, des):
        """
        signal = [s s-dot s-ddot]
        des = [d d-dot d-ddot]
        """
        signal[2] = 0
        return np.sum(np.dot(des - signal, self.k))


class PDVelocity:
    def __init__(self, kp, kd):
        self.k = np.array([1, kp, kd])

    def update(self, signal, des):
        """
        signal = [s s-dot s-ddot]
        des = [d d-dot d-ddot]
        """
        return np.sum(np.dot(des - signal, self.k))


class DroneController:
    def __init__(
        self,
        drone_state_time_func,
        drone_u_func,
        mass,
        g,
        I,
        tc,
        q0,
        qh,
        th,
        zt,
        waypoints,
        T_traj,
    ):
        self.sensor_feedback = drone_state_time_func
        self.u1, self.u2 = drone_u_func

        self.mass = mass
        self.g = g
        self.I = I

        # self.xPD        = PDPosition(993.8, 189.377)
        # self.yPD        = PDPosition(993.8, 189.377)
        # self.zPD        = PDPosition(300,   35.43)
        # self.phiPD      = PDPosition(10000, 185)
        # self.thetaPD    = PDPosition(10000, 185)
        # self.psiPD      = PDPosition(452,   42)

        self.xPD = PDPosition(250, 100)
        self.yPD = PDPosition(250, 100)
        self.zPD = PDPosition(300, 35.43)
        self.phiPD = PDPosition(10000, 200)  # 185)
        self.thetaPD = PDPosition(10000, 200)  # 185)
        self.psiPD = PDPosition(10000, 200)  # 185)

        self.tc = tc
        self.controller_outter_timer = Timer(tc)
        #   Inner loop should be 10x faster than outter loop
        self.controller_inner_timer = Timer(tc / 10)

        #   phase   1
        # self.trajectory = TrajectoryGenerator1(q0, qh, zt, th)
        #   phase   2

        self.trajectory = TrajectoryGenerator2(
            q0, qh, zt, waypoints, T_traj, T_hover=th
        )
        self.mission_ended = False

    def atitude_controller(self, feedback, des):
        """
        des = [phi theta psi psi-dot]
        feedback = [x y z x-dot y-dot z-dot phi theta psi phi-dot theta-dot psi-dot]
        """
        u2 = np.zeros(3)
        w_dot = np.array(
            [
                self.phiPD.update(feedback[[6, 9, 6]], np.array([des[0], 0, 0])),
                self.thetaPD.update(feedback[[7, 10, 7]], np.array([des[1], 0, 0])),
                self.psiPD.update(feedback[[8, 11, 8]], np.array([des[2], des[3], 0])),
            ]
        )
        w = feedback[9:12]
        u2 = np.dot(self.I, w_dot) + np.cross(w, np.dot(self.I, w))
        return u2

    def postition_controller(self, feedback, des):
        """
        des = [x y z x-dot y-dot z-dot x-ddot y-ddot z-ddot]
        feedback = [x y z x-dot y-dot z-dot phi theta psi phi-dot theta-dot psi-dot]
        """
        u1 = self.mass * (self.g + self.zPD.update(feedback[[2, 5, 2]], des[[2, 5, 8]]))
        phi = -(self.mass / u1) * self.yPD.update(feedback[[1, 4, 1]], des[[1, 4, 7]])
        theta = feedback[8] * phi - (self.mass / u1) * self.xPD.update(
            feedback[[0, 3, 0]], des[[0, 3, 6]]
        )
        orientation = np.array([phi, theta, 0, 0])

        return u1, orientation

    def controller_loop(self):
        #   Initial pose and time
        state, time = self.sensor_feedback()
        terminator = 1
        while True:
            #   Outter loop to controll the position
            if self.controller_outter_timer.is_fire():
                des_state = self.trajectory.next_point(state)

                u1, orientation_des = self.postition_controller(state, des_state[0:9])
                orientation_des[2:4] = des_state[9:11]
                #   Limmit the roll and pitch to [-50 50]
                orientation_des[0:2] = np.clip(orientation_des[0:2], -0.872, 0.872)
                #   Apply the thrust
                if self.trajectory.is_mission_done:
                    terminator = 0
                    self.mission_ended = True

                self.u1(u1 * terminator)

                #   Inner loop to controll the atitude
                if self.controller_inner_timer.is_fire():
                    #   Get position, orientation and time as the feedback from ODE
                    state, time = self.sensor_feedback()
                    self.u2(
                        self.atitude_controller(state, orientation_des) * terminator
                    )

            self.controller_outter_timer.small_delay()

    def start_controller(self):
        self.controller_thread = threading.Thread(target=self.controller_loop)
        self.controller_thread.daemon = True
        self.controller_thread.start()
