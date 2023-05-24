
import numpy as np

class MapReader:
    
    @classmethod
    def read_map(path):
        boundaries = None
        obstacle_origin = []
        obstacle_size = []

        with open(path, 'r') as file:
            lines = file.readlines()

            for line in lines:
                line = line.strip()

                #   Remove comments indicated by '#'
                if '#' in line:
                    line = line.split('#')[0].strip()

                if line == '':
                    continue

                if line == 'EOF':
                    break

                if boundaries is None:
                    boundaries = np.array(list(map(float, line.split(';'))))
                else:
                    origin, size = line.split(':')
                    obstacle_origin.append(np.array(list(map(float, origin.split(';')))))
                    obstacle_size.append(np.array(list(map(float, size.split(';')))))


        obstacle_origin = np.array(obstacle_origin)
        obstacle_size = np.array(obstacle_size)

        return {'obstacle_origin':obstacle_origin,'obstacle_size':obstacle_origin, 'boundaries':boundaries}


    @classmethod
    def generate_c_free(_map, dx, dy, dz):
        x = np.linspace(0, _map['boundaries'][0], int(_map['boundaries'][0]/dx) +1)
        y = np.linspace(0, _map['boundaries'][1], int(_map['boundaries'][1]/dy) +1)
        z = np.linspace(0, _map['boundaries'][2], int(_map['boundaries'][2]/dz) +1)

        # Create a meshgrid of the Cartesian coordinates
        X, Y, Z = np.meshgrid(x, y, z, indexing='ij')


        # Initialize arrays to store points inside and outside the obstacle
        c_obs_all = []
        c_free_all = []

        # Plot points inside and outside each box
        for obs_origin, obs_size in zip(_map['obstacle_origin'], _map['obstacle_size']):
            mask_inside_obs = np.logical_and.reduce((X >= obs_origin[0], X <= obs_origin[0] + obs_size[0],
                                                    Y >= obs_origin[1], Y <= obs_origin[1] + obs_size[1],
                                                    Z >= obs_origin[2], Z <= obs_origin[2] + obs_size[2]))

            c_obs = np.vstack((X[mask_inside_obs], Y[mask_inside_obs], Z[mask_inside_obs])).T
            c_free = np.vstack((X[~mask_inside_obs], Y[~mask_inside_obs], Z[~mask_inside_obs])).T

            # Append the points to the arrays
            c_obs_all.append(c_obs)
            c_free_all.append(c_free)
        c_obs_all = np.concatenate(c_obs_all)
        c_free_all = np.concatenate(c_free_all)
        return c_obs_all, c_free_all
            

