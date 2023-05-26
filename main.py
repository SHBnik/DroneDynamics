import config
import argparse
import numpy as np
from timer import Counter
from dynamics import Drone
from controller import DroneController
from viz3D import Viz
import time


def Run(q0, qh, th, zt, to, tc, show_2Dgraph):
    #   Start the ODE solver aka simulation
    drone = Drone(
        config.drone_mass,
        config.I,
        config.drone_length,
        config.kf,
        config.km,
        config.g,
        q0,
        to,
        config.ode_scalar,
    )

    #   Map reader

    #   Start the drone controller
    controller = DroneController(
        drone.get_state_time,
        [drone.set_u1, drone.set_u2],
        config.drone_mass,
        config.g,
        config.I,
        tc,
        q0,
        qh,
        th,
        zt,
        0,
        0,
        config.T_traj,
    )

    #   Run the visualizer
    __viz = Viz(show_2Dgraph=show_2Dgraph)

    drone.start_ode()
    controller.start_controller()
    while True:
        pose, t = drone.get_pose_time()
        state_dot, t = drone.get_state_dot_time()
        __viz.draw_quadri(pose, state_dot, t)

        if controller.mission_ended:
            time.sleep(3)
            break


def arg_to_np(arg):
    return np.array(arg)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Drone Dynamics")
    parser.add_argument("--d", action="store_true", default=False)
    parser.add_argument("-n", "--noinput", action="store_true")
    parser.add_argument("--q0", nargs="+", type=float)
    parser.add_argument("--qh", nargs="+", type=float)
    parser.add_argument("--th", type=float)
    parser.add_argument("--zt", type=float)
    args = parser.parse_args()

    if args.noinput:
        Run(config.q0, config.qh, config.th, config.zt, config.to, config.tc, args.d)
    else:
        Run(
            arg_to_np(args.q0),
            arg_to_np(args.qh),
            args.th,
            args.zt,
            config.to,
            config.tc,
            args.d
        )
