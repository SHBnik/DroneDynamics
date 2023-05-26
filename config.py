# fmt: off
import numpy as np

#   CrazyFly 2.0                                    #unit

#   CrazyFly mass 
drone_mass = 0.030                                  #kg

#   CraxyFly arm length
drone_length = 0.046                                #m

#   CrazyFly inertia matrix
I = np.array([[1.43e-5,     0,          0       ],
              [0,           1.43e-5,    0       ],
              [0,           0,          2.89e-5 ]]) #kg.m^2

#   CrazyFly coefficients
kf = 6.11e-8                                        #N/rpm^2
km = 1.5e-9                                         #N/rpm^2


#   gravity accleration
g = 9.8

#   Initial pose
#              x, y, z
# q0 = np.array([1.0, 1.0, 0.0,    #Position                #m
#                0.0, 0.0, 0.0])   #Orientation             #rad

q0 = np.array([0.0, 0.0, 0.0,    #Position                #m
               0.0, 0.0, 0.0])   #Orientation             #rad

#   Hovering pose
#              x, y, z
# qh = np.array([3.0, 3.0, 3.0,    #Position                #m
#                0.0, 0.0, 0.0])   #Orientation             #rad

qh = np.array([5.0, 2.0, 10.0,    #Position                #m
               0.0, 0.0, 0.0])   #Orientation             #rad

#   Take-off altitude
zt = 1                                                   #m


#   Simulation time ratio (if = 1 simulation time = real time, lowwer simiulation is slower)
ode_scalar = 1


#   Hovering time
th = 1 / ode_scalar                                 #s


#   Controller frequency
tc = 0.002 / ode_scalar                             #s
fc = 1/tc                                           #hz

#   ODE solver frequency
to = 0.001                                          #s
fo = 1/to                                           #hz


#   Simulation
SIMU_UPDATE_FRQ = 1000
BODY_SCALER = 4
WORLD_SCALER = 11
TRAJECTORY_MARKER_SIZE = 0.03
UNIT_SCALER = 1
MULTI_AGENT = 1
DASHBOARD_MODE = 'global_data'
# DASHBOARD_MODE = 'local_data'

#   Trajectroy params
T_traj = 2#5.5


#   A* neighbors params
DIAGONAL_PATH=True
PRUNE_PATH=True