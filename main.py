import config
import argparse
import numpy as np
from timer import Timer
 
















def Run(q0, qh, th, zt, td, tc):
    #   TODO: should the update rate of contoller and the ode be different? 
    controller_timer = Timer(tc)
    ode_timer = Timer(td)
    while True: 
        #   Run this loop every td time for the ode solver
        if ode_timer.is_fire():
            pass
























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
        Run(config.q0, config.qh, config.th, config.zt, config.td, config.tc)
    else:
        Run(arg_to_np(args.q0), arg_to_np(args.qh), args.th, args.zt, config.td, config.tc)