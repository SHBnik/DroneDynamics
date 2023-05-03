import numpy as np


#   CrazyFly 2.0                                #unit

#   CrazyFly mass 
drone_mass = 0.030                              #kg

#   CraxyFly arm length
drone_length = 0.046                            #m

#   CrazyFly inertia matrix
np.array([[1.43e-5,     0,          0       ],
          [0,           1.43e-5,    0       ],
          [0,           0,          2.89e-5 ]]) #kg.m^2

#   CrazyFly coefficients
kf = 6.11e-8                                    #N/rpm^2
km = 1.5e-9                                     #N/rpm^2

#   Initial pose
#              x, y, z
q0 = np.array([0, 0, 0,    #Position            #m  
               0, 0, 0])   #Orientation         #rad

#   Hovering pose
#              x, y, z
qh = np.array([0, 0, 0,    #Position            #m
               0, 0, 0])   #Orientation         #rad

#   Hovering time
th = 5                                          #s

#   Take-off altitude
zt = 1                                          #m


#   Controller frequency
tc = 0.02                                       #s
fc = 1/tc                                       #hz

#   ODE solver frequency
td = 0.01                                      #s
fd = 1/td                                       #hz