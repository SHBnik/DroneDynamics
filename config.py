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