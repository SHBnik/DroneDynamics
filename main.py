import config
import argparse
import numpy as np
from timer import Counter
from dynamics import Drone
from controller import DroneController
from viz3D import Viz
from map import MapReader
import time
from a_star import get_path_from_A_star


def Run(q0, qh, th, zt, to, tc, show_2Dgraph):
    #   Map reader
    map_data = MapReader.read_map("Environment.txt")  # , config.robot_size)

    c_obs, c_all = MapReader.generate_c(map_data, config.map_resolution)

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

    ###################################################
    # #   Map reader #TODO:
    # obstacles=None
    # boundaries=None

    # # for test purpose
    # obstacles = [(-2, 1, 0), (-2, 0, 0), (-2, -1, 0), (-2, -2, 0), (-4, -2, 0), (-4, -3, 0)]
    # boundaries = (20, 20, 20)
    # start =  tuple(q0[:3])
    # goal = tuple(qh[:3])
    # # waypoints = get_path_from_A_star((0, 0,0), (5, 2,10),zt, obstacles, boundaries)

    # waypoints = get_path_from_A_star(start, goal, zt, obstacles, , config.PRUNE_PATH)
    # print("navigation path is :",waypoints)
    ######################################################

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
        c_all,
        c_obs,
        waypoints,
        config.T_traj,
        config.map_resolution,
    )

    #   Run the visualizer
    __viz = Viz(map_data, controller.planned_traj_path, show_2Dgraph=show_2Dgraph)

    drone.start_ode()
    controller.start_controller()
    while True:
        pose, t = drone.get_pose_time()
        state_dot, t = drone.get_state_dot_time()
        __viz.draw_quadri(pose, state_dot, t)

        if controller.mission_ended:
            time.sleep(5)
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
    args.d = True
    args.noinput = True

    if args.noinput:
        # print("in")
        Run(config.q0, config.qh, config.th, config.zt, config.to, config.tc, args.d)
    else:
        Run(
            arg_to_np(args.q0),
            arg_to_np(args.qh),
            args.th,
            args.zt,
            config.to,
            config.tc,
            args.d,
        )
