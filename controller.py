import numpy as np
import threading
from timer import Timer

#   TODO: write the trajectory generator

class PDPosition:
    def __init__(self,kp,kd):
        self.k = np.array([kp, kd, 1])
    def update(self, signal, des):
        ''' 
        signal = [s s-dot s-ddot]
        des = [d d-dot d-ddot] 
        '''
        signal[2] = 0
        return np.dot(des - signal, self.k)

class PDVelocity:
    def __init__(self,kp,kd):
        self.k = np.array([1, kp, kd])
    def update(self, signal, des):
        ''' 
        signal = [s s-dot s-ddot]
        des = [d d-dot d-ddot] 
        '''
        return np.dot(des - signal, self.k)



class DroneController:
    def __init__(self, drone_pose_time_func, drone_u_func, tc):
        self.sensor_feedback = drone_pose_time_func
        self.u1, self.u2 = drone_u_func



        self.tc = tc
        self.controller_outter_timer = Timer(tc)
        #   Inner loop should be 10x faster than outter loop
        self.controller_inner_timer = Timer(tc / 10)

    def atitude_controller(self):
        pass

    def postition_controller(self):
        pass





    def controller_loop(self,to):
        #   Initial pose and time
        pose, time = self.sensor_feedback()
        while True:
            #   Outter loop to controll the position
            if self.controller_outter_timer.is_fire():
                u1, orientation = self.postition_controller()
                #   Apply the thrust
                self.u1(u1)
                #   Inner loop to controll the atitude
                if self.controller_inner_timer.is_fire():
                    #   Get position, orientation and time as the feedback from ODE
                    pose, time = self.sensor_feedback()
                    self.u2(self.atitude_controller())

            self.controller_outter_timer.small_delay()

    def start_controller(self):
        self.controller_thread = threading.Thread(target=self.controller_loop)
        self.controller_thread.daemon = True
        self.controller_thread.start()