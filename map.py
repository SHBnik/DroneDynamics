import numpy as np


class MapReader:
    @staticmethod
    def read_map(path, robot_size=np.array([0, 0, 0])):
        boundaries = None
        obstacle_origin = []
        obstacle_size = []

        with open(path, "r") as file:
            lines = file.readlines()

            for line in lines:
                line = line.strip()

                #   Remove comments indicated by '#'
                if "#" in line:
                    line = line.split("#")[0].strip()

                if line == "":
                    continue

                if line == "EOF":
                    break

                if boundaries is None:
                    boundaries = np.array(list(map(float, line.split(";"))))
                else:
                    origin, size = line.split(":")
                    obstacle_origin.append(
                        np.array(list(map(float, origin.split(";"))))
                    )
                    size = np.array(list(map(float, size.split(";"))))
                    size += robot_size
                    obstacle_size.append(size)

        obstacle_origin = np.array(obstacle_origin)
        obstacle_size = np.array(obstacle_size)

        return {
            "obstacle_origin": obstacle_origin,
            "obstacle_size": obstacle_size,
            "boundaries": boundaries,
        }

    @staticmethod
    def generate_c(_map, resolution):
        x = np.linspace(
            0,
            _map["boundaries"][0],
            int(_map["boundaries"][0] / resolution[0]) + 1,
        )
        y = np.linspace(
            0,
            _map["boundaries"][1],
            int(_map["boundaries"][1] / resolution[1]) + 1,
        )
        z = np.linspace(
            0,
            _map["boundaries"][2],
            int(_map["boundaries"][2] / resolution[2]) + 1,
        )

        # Create a meshgrid of the Cartesian coordinates
        X, Y, Z = np.meshgrid(x, y, z, indexing="ij")

        # Initialize arrays to store points inside and outside the obstacle
        c_obs_all = []

        # Plot points inside each box
        for obs_origin, obs_size in zip(_map["obstacle_origin"], _map["obstacle_size"]):
            mask_inside_obs = np.logical_and.reduce(
                (
                    X >= obs_origin[0],
                    X <= obs_origin[0] + obs_size[0],
                    Y >= obs_origin[1],
                    Y <= obs_origin[1] + obs_size[1],
                    Z >= obs_origin[2],
                    Z <= obs_origin[2] + obs_size[2],
                )
            )

            c_obs = np.vstack(
                (X[mask_inside_obs], Y[mask_inside_obs], Z[mask_inside_obs])
            ).T

            # Append the points to the arrays
            c_obs_all.append(c_obs)

        c_all = np.vstack((X.ravel(), Y.ravel(), Z.ravel())).T
        c_obs_all = np.concatenate(c_obs_all)

        return c_obs_all, c_all
