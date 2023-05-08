import config
import argparse
import numpy as np
from timer import Timer, Counter
from visualizer import Viz
from dynamics import Drone
from controller import DroneController
import time 
import matplotlib.pyplot as plt
import viz3D
from mpl_toolkits.mplot3d import Axes3D


SIMU_UPDATE_FRQ=1000






def draw_quadri(ax, X):
    ax.clear()
    ech = 2
    ax.set_xlim3d(-ech, ech)
    ax.set_ylim3d(-ech, ech)
    ax.set_zlim3d(0, 1*ech)
    arm=0.046
    viz3D.draw_quadrotor3D(ax, X, np.array([[0,0,0,0]]).T, 5 * arm)





def Run(q0, qh, th, zt, to, tc):
    #   end to end timer Tt
    e2e_timer = Counter()

    #   Start the ODE solver aka simulation
    drone = Drone(config.drone_mass, config.I, config.drone_length, 
                  config.kf, config.km, config.g, q0, to, config.ode_scalar)

    #   Start the drone controller
    controller = DroneController(drone.get_state_time, 
                                 [drone.set_u1, drone.set_u2], config.drone_mass, 
                                 config.g, config.I, tc, q0, qh, th, zt)
    
    #   Run the visualizer
    __viz = Viz()


    drone.start_ode()
    controller.start_controller()

    fig = plt.figure()
    ax = Axes3D(fig)
   
    while True:
        pose, t = drone.get_pose_time()
        pose[2] = -pose[2]
        draw_quadri(ax, pose)
        plt.pause(1/SIMU_UPDATE_FRQ)
        # __viz.plot(pose, t)
        # time.sleep(0.01)

    









    
            
























def arg_to_np(arg):
    return np.array(arg)



if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Drone Dynamics")
    parser.add_argument("-d", "--debug", help="turn on debug mode", action="store_true")
    parser.add_argument("-n", "--noinput", action="store_true")
    parser.add_argument('--q0', nargs='+', type=float)
    parser.add_argument('--qh', nargs='+', type=float)
    parser.add_argument("--th", type=float)
    parser.add_argument("--zt", type=float)
    args = parser.parse_args()

    if args.noinput:
        Run(config.q0, config.qh, config.th, config.zt, config.to, config.tc)
    else:
        Run(arg_to_np(args.q0), arg_to_np(args.qh), args.th, args.zt, config.to, config.tc)