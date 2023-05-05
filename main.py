import config
import argparse
import numpy as np
from timer import Timer, Counter
from visualizer import Viz
from dynamics import Drone
from controller import DroneController
import time 














def Run(q0, qh, th, zt, to, tc):
    #   TODO: should the update rate of contoller and the ode be different? 
    controller_timer = Timer(tc)
    #   end to end timer Tt
    e2e_timer = Counter()

    #   Start the ODE solver aka simulation
    drone = Drone(config.drone_mass, config.I, config.drone_length, 
                  config.kf, config.km, config.g, q0)

    #   Start the drone controller
    controller = DroneController(drone.get_pose_time, 
                                 [drone.set_u1, drone.set_u2], tc)
    
    #   Run the visualizer
    __viz = Viz()


    drone.start_ode(to)
    while True:
        pose = drone.get_pose() 
        print(pose)
        if pose[2] == 0:
            break
        time.sleep(0.001)
    

    print('Time elapsed : {0:.2f} ms'.format(e2e_timer.stop() * 1000))
            
























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