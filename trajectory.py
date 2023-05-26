import numpy as np
import threading
from timer import Timer, Counter
from minjerk import MinJerkContinuous, MinJerkNonContinuous



#   phase 1
class TrajectoryGenerator1:
    def __init__(self, q0, qh, zt, T, dx=1):
        self.q0 = q0
        self.qh = qh
        self.zt = zt
        self.T = T
        self.dx = dx
        self.e2e_timer = Counter()

        self.stage = 0
        self.func = [
            self.__take_off,
            self.__coast,
            self.__hover,
            self.__land,
            self.__print_time,
        ]

        self.hover_timer = None
        self.hover_flag = True

        self.is_mission_done = False

    def __take_off(self, cur_pose):
        new_goal_pose = np.copy(self.q0)
        new_goal_pose[2] = self.zt

        if np.abs(cur_pose[2] - self.zt) < 1e-3:
            self.stage += 1

        return new_goal_pose

    def __set_hover_timer(self):
        self.hover_timer = Timer(self.T)

    def __hover(self, cur_pose):
        if self.hover_flag:
            self.__set_hover_timer()
            self.hover_flag = False

        if self.hover_timer.is_fire():
            self.hover_flag = True
            self.stage += 1

        return self.qh

    def __coast(self, cur_pose):
        new_goal_pose = np.copy(self.qh)
        new_goal_pose[0:3] = cur_pose[0:3] + self.dx
        new_goal_pose[0:3] = np.minimum(new_goal_pose[0:3], self.qh[0:3])

        #   Cheat
        cur_pose[3:5] = 0
        cur_pose[2] = self.qh[2]

        if np.all(np.abs(cur_pose - self.qh) < 1e-2):
            self.stage += 1

        return new_goal_pose

    def __land(self, cur_pose):
        new_goal_pose = np.copy(self.qh)
        new_goal_pose[2] = 0

        if np.abs(cur_pose[2] - 0) < 1e-3:
            self.stage += 1

        return new_goal_pose

    def __print_time(self, cur_pose):
        if not self.is_mission_done:
            self.is_mission_done = True
            print("Time elapsed : {0:.2f} ms".format(self.e2e_timer.stop() * 1000))

        new_goal_pose = np.copy(self.qh)
        new_goal_pose[2] = 0
        return new_goal_pose

    def next_point(self, cur_state):
        des_state = np.zeros(11)
        cur_pose = np.zeros(6)

        cur_pose = cur_state[[0, 1, 2, 6, 7, 8]]
        new_goal_pose = self.func[self.stage](cur_pose)

        #   x y z
        des_state[0:3] = new_goal_pose[0:3]
        #   psi
        des_state[9] = new_goal_pose[5]
        return des_state


#   phase 2
class TrajectoryGenerator2:
    def __init__(
        self,
        start_pose,
        goal_pose,
        zt,
        waypoints,
        Traj_T,
        T_hover=0,
        dx=1,
        dy=1,
        dz=1,
    ):
        self.start_pose = start_pose
        self.goal_pose = goal_pose
        self.zt = zt
        self.T = T_hover
        self.e2e_timer = Counter()

        self.stage = 0
        self.func = [
            self.__take_off,
            self.__coast,
            self.__hover,
            self.__land,
            self.__idle,
        ]

        self.hover_timer = None
        self.hover_flag = True

        self.coast_flag = True

        self.is_mission_done = False

        #   Road map generation with A*
        # a_star = Astar(dx, dy, dz, c_obs, c_free)
        # self.waypoints  = a_star.generate_roadmap(start_pose, goal_pose)
        self.waypoints = np.array([[0, 0, 0],[2, 2, 3], [3, 3, 2], [3, 3, 3]])
        # print("waypoints is", waypoints)
        self.waypoints = np.array(waypoints)

        self.minjerk = MinJerkContinuous(
            Traj_T, self.zt, self.waypoints, self.start_pose, self.goal_pose
        )
        # self.minjerk = MinJerkNonContinuous(
        #     Traj_T, self.zt, self.waypoints, self.start_pose, self.goal_pose
        # )

    def __take_off(self, cur_pose):
        new_goal_pose = np.copy(self.start_pose)
        new_goal_pose[2] = self.zt

        if np.abs(cur_pose[2] - self.zt) < 1e-2:
            self.__print_time("take off")
            self.stage += 1

        return self.make_des_state(new_goal_pose)

    def __set_hover_timer(self):
        self.hover_timer = Timer(self.T)

    def __hover(self, cur_pose):
        if self.hover_flag:
            self.__set_hover_timer()
            self.hover_flag = False

        if self.hover_timer.is_fire():
            self.hover_flag = True
            self.__print_time("hover")
            self.stage += 1

        return self.make_des_state(self.goal_pose)

    def __coast(self, cur_pose):
        """Min Jerk is implimented here"""
        if self.coast_flag:
            self.coast_flag = False
            self.minjerk.start_minjerk_timer()

        new_des_state = self.minjerk.trajectroy()

        if self.minjerk.is_done:
            self.__print_time("trajectory execution")
            self.stage += 1

        return np.hstack([new_des_state, np.array([0, 0])])

    def __land(self, cur_pose):
        new_goal_pose = np.copy(self.goal_pose)
        new_goal_pose[2] = 0

        if np.abs(cur_pose[2] - 0) < 1e-3:
            self.__print_time("land")
            self.is_mission_done = True
            self.stage += 1

        return self.make_des_state(new_goal_pose)

    def __idle(self, cur_pose):
        new_goal_pose = np.copy(self.goal_pose)
        new_goal_pose[2] = 0
        return self.make_des_state(new_goal_pose)

    def __print_time(self, stage):
        print(
            "Time elapsed for stage {0}: {1:.2f} ms".format(
                stage, self.e2e_timer.now() * 1000
            )
        )

    def make_des_state(self, pose):
        des_state = np.zeros(11)
        #   x y z
        des_state[0:3] = pose[0:3]
        #   psi
        des_state[9] = pose[5]
        return des_state

    def next_point(self, cur_state):
        cur_pose = np.zeros(6)

        cur_pose = cur_state[[0, 1, 2, 6, 7, 8]]
        new_des_state = self.func[self.stage](cur_pose)

        return new_des_state
