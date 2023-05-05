import numpy as np
from scipy.integrate import ode
from timer import Timer
import threading



class Drone:
    def __init__(self, mass, I, length, kf, km, g, initial_pose, to):
        self.mass = mass 
        self.Ib = I
        self.invIb = np.linalg.inv(I)
        self.length = length
        self.kf = kf
        self.km = km
        self.gamma = km/kf
        self.g = g

        # self.wb = np.zeros((3,1))
        # self.F = np.zeros ((4,1))
        # self.F_u_coef = np.array([
        #     [1,               1,              1,              1],               #   Thrust
        #     [0,               self.length,    0,              -self.length],    #   MomentX
        #     [-self.length,    0,              self.length,    0],               #   MomentY
        #     [self.gamma,      -self.gamma,    self.gamma,     -self.gamma]])    #   MomentZ

        
        self.u1 = 0 
        self.u2 = np.array([0, 0, 0])
        
        self.state = np.zeros(12)
        self.state[0:3] = initial_pose[0:3]
        self.state[6:9] = initial_pose[3:6]
        self.ode =  ode(self.state_dot).set_integrator('vode', nsteps=500,method='bdf')
        
        self.to = to
        self.ode_timer = Timer(to)






    def u1(self, F, F_u_coef):
        return np.dot(F_u_coef[0], F)
    
    def u2(self, F, F_u_coef):
        return np.dot(F_u_coef[1:3], F)
    

    #   R = R(psi,z)*R(theta,y)*R(phi,x)
    def rotation_matrix(self,angles):
        cos_psi =   np.cos(angles[2])
        sin_psi =   np.sin(angles[2])
        cos_theta = np.cos(angles[1])
        sin_theta = np.sin(angles[1])
        cos_phi =   np.cos(angles[0])
        sin_phi =   np.sin(angles[0])
        return  np.array([
            [cos_psi*cos_theta,     -sin_psi * cos_phi - sin_phi * sin_theta,       sin_psi * sin_phi - sin_theta * cos_phi],
            [0,                     cos_psi * cos_phi,                              -cos_psi * sin_phi],
            [0,                     cos_theta * sin_phi,                            cos_theta * cos_phi]])

    #   Dynamics funciton
    def state_dot(self, t, state, u1, u2):
        _state_dot = np.zeros(12)
        #   x-dot   y-dot   z-dot
        _state_dot[0:3] = state[3:6]
        #   x-ddot   y-ddot   z-ddot  
        _state_dot[3:6] = np.dot(self.rotation_matrix(state[6:9]), 
                                    np.array([0, 0, u1/self.mass])) + np.array([0, 0, -self.g])
        #   phi-dot  theta-dot  psi-dot
        _state_dot[6:9] = state[9:12]
        #   phi-ddot  theta-ddot  psi-ddot
        _state_dot[9:12] = np.dot(self.invIb, 
                                     u2 - np.cross(state[9:12], np.dot(self.Ib, state[9:12])))
        
        return _state_dot
        
    def wrap_angle(self,angels):
        return( (( angels + np.pi) % (2 * np.pi )) - np.pi )

    def update_dynamics(self, u1=0, u2=np.array([0, 0, 0])):
            self.ode.set_initial_value(self.state ,0).set_f_params(u1, u2)
            temp_state = self.ode.integrate(self.ode.t + self.to)
            #   To keep the angels in [-pi, pi] range
            temp_state[6:9] = self.wrap_angle(temp_state[6:9])
            #   To prevent the drone fall into the ground
            temp_state[2] = max(0 ,temp_state[2])
            if temp_state[2] == 0:
                temp_state[5] = max(0 ,temp_state[5])
            self.state = temp_state


    def set_u1(self, u1):
        self.u1 = u1
        
    def set_u2(self, u2):
        self.u2 = u2

    def get_pose_time(self):
        pose = np.zeros(6)
        pose[0:3] = self.state[0:3]
        pose[3:6] = self.state[6:9]
        return pose, self.ode_timer.current_time()


    def ode_loop(self):
        while True:
            if self.ode_timer.is_fire():
                self.update_dynamics(u1 = self.u1, u2 = self.u2)

            self.ode_timer.small_delay()

    def start_ode(self):
        self.ode_thread = threading.Thread(target=self.ode_loop)
        self.ode_thread.daemon = True
        self.ode_thread.start()

