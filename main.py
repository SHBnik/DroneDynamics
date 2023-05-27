import config
import argparse
import numpy as np
from timer import Counter
from dynamics import Drone
from controller import DroneController
from viz3D import Viz
from map import MapReader
import time
from a_star2 import AStar
import a_star
import warnings

warnings.filterwarnings("ignore")


def __print_time(timer, stage):
    print("Time elapsed for stage {0}: {1:.2f} ms".format(stage, timer.now() * 1000))


def Run(q0, qh, th, zt, to, tc, show_2Dgraph):
    component_timer = Counter()

    #   Map reader
    map_data = MapReader.read_map(
        "Environment.txt",
        # config.robot_size
    )

    c_obs, c_all = MapReader.generate_c(
        map_data,
        # config.map_resolution
    )

    __print_time(component_timer, "Map generation")

    #   Way point generation with A*
    waypoints = np.array(
        a_star.get_path_from_A_star(
            tuple(q0[0:3]),
            tuple(qh[0:3]),
            zt,
            [tuple(point) for point in c_obs],
            map_data["boundaries"],
            prune=True,
            check_bound=True
        )
    )

    __print_time(component_timer, "A*")

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
        waypoints,
        config.traj_T_Kp,
    )

    #   Run the visualizer
    __viz = Viz(map_data, show_2Dgraph=show_2Dgraph)

    drone.start_ode()
    controller.start_controller()

    while True:
        pose = drone.get_pose_time()[0]
        state_dot, t = drone.get_state_dot_time()
        __viz.draw_quadri(pose, state_dot, waypoints, t)

        if controller.mission_ended:
            time.sleep(5)
            break

    __viz.plot_state_and_des_state(controller.state_des_state)


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
    # args.d = True
    # args.noinput = True

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
