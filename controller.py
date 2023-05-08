import numpy as np
import threading
from timer import Timer, Counter

#   TODO: fine tune PID
#   TODO: fix landing (after landing robot shouldnt move)

class PDPosition:
    def __init__(self,kp,kd):
        self.k = np.array([kp, kd, 1])
    def update(self, signal, des):
        ''' 
        signal = [s s-dot s-ddot]
        des = [d d-dot d-ddot] 
        '''
        signal[2] = 0
        return np.sum(np.dot(des - signal, self.k))

class PDVelocity:
    def __init__(self,kp,kd):
        self.k = np.array([1, kp, kd])
    def update(self, signal, des):
        ''' 
        signal = [s s-dot s-ddot]
        des = [d d-dot d-ddot] 
        '''
        return np.sum(np.dot(des - signal, self.k))



class TrajectoryGenerator:
    def __init__(self, q0, qh, zt, T, dx = 1):
        self.q0 = q0
        self.qh = qh
        self.zt = zt
        self.T = T
        self.dx = dx
        self.e2e_timer = Counter()

        self.stage = 0
        self.func = [self.__take_off, self.__coast, 
                     self.__hover, self.__land, self.__print_time]
        
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

        if np.all(np.abs(cur_pose - self.qh) < 5e-3):
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
            print('Time elapsed : {0:.2f} ms'.format(self.e2e_timer.stop() * 1000))

        new_goal_pose = np.copy(self.qh) 
        new_goal_pose[2] = 0
        return new_goal_pose

    def next_point(self, cur_state):
        des_state = np.zeros(11)
        cur_pose = np.zeros(6)

        cur_pose = cur_state[[0, 1, 2,
                              6, 7, 8]]
        new_goal_pose = self.func[self.stage](cur_pose)

        #   x y z
        des_state[0:3] = new_goal_pose[0:3]
        #   psi
        des_state[9] =  new_goal_pose[5] 
        return des_state



class DroneController:
    def __init__(self, drone_state_time_func, drone_u_func, mass, g, I, tc, q0, qh, th, zt):
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

        
        self.xPD        = PDPosition(300,   70)
        self.yPD        = PDPosition(300,   70)
        self.zPD        = PDPosition(300,   35.43)
        self.phiPD      = PDPosition(10000, 185)
        self.thetaPD    = PDPosition(10000, 185)
        self.psiPD      = PDPosition(452,   42)


        self.tc = tc
        self.controller_outter_timer = Timer(tc)
        #   Inner loop should be 10x faster than outter loop
        self.controller_inner_timer = Timer(tc / 10)

        self.trajectory = TrajectoryGenerator(q0, qh, zt, th)
        


    def atitude_controller(self, feedback, des):
        '''
        des = [phi theta psi psi-dot]
        feedback = [x y z x-dot y-dot z-dot phi theta psi phi-dot theta-dot psi-dot]
        '''
        u2 = np.zeros(3)
        w_dot = np.array([self.phiPD.update(feedback[[6, 9, 6]], np.array([des[0], 0, 0])), 
                          self.thetaPD.update(feedback[[7, 10, 7]], np.array([des[1], 0, 0])), 
                          self.psiPD.update(feedback[[8, 11, 8]], np.array([des[2], des[3], 0]))])
        w = feedback[9:12]
        u2 = np.dot(self.I, w_dot) + np.cross(w, np.dot(self.I, w))
        return u2


    def postition_controller(self, feedback, des):
        '''
        des = [x y z x-dot y-dot z-dot x-ddot y-ddot z-ddot]
        feedback = [x y z x-dot y-dot z-dot phi theta psi phi-dot theta-dot psi-dot]
        '''
        u1 = self.mass * (self.g + self.zPD.update(feedback[[2, 5, 2]], des[[2, 5, 8]]))
        phi = - (self.mass / u1) * self.yPD.update(feedback[[1, 4, 1]], des[[1, 4, 7]])
        theta = feedback[8] * phi - (self.mass / u1) * self.xPD.update(feedback[[0, 3, 0]], des[[0, 3, 6]])
        orientation = np.array([phi, theta, 0, 0])
        
        return u1, orientation




    def controller_loop(self):
        #   Initial pose and time
        state, time = self.sensor_feedback()
        while True:
            #   Outter loop to controll the position
            if self.controller_outter_timer.is_fire():
                
                des_pose = self.trajectory.next_point(state)
                
                u1, orientation_des = self.postition_controller(state, des_pose[0:9])
                orientation_des[2:4] = des_pose[9:11]
                #   Limmit the roll and pitch to [-80 80]
                orientation_des[0:2] = np.clip(orientation_des[0:2], -0.872, 0.872)
                #   Apply the thrust
                if not self.trajectory.is_mission_done:
                    self.u1(u1)
                else:
                    self.u1(0)

                #   Inner loop to controll the atitude
                if self.controller_inner_timer.is_fire():
                    #   Get position, orientation and time as the feedback from ODE
                    state, time = self.sensor_feedback()
                    
                    if not self.trajectory.is_mission_done:
                        self.u2(self.atitude_controller(state, orientation_des))
                    else:
                        self.u2(np.array([0, 0, 0]))

            self.controller_outter_timer.small_delay()





    def start_controller(self):
        self.controller_thread = threading.Thread(target=self.controller_loop)
        self.controller_thread.daemon = True
        self.controller_thread.start()