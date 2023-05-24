# Quadcopter Simulation and Control
This is a project for course EE260 "Advanced Robotics".

## Description
In this project, we simulate a Crazyflie quadcopter in Python from scratch. This project has three main parts:
- Dynamics: solves the dynamic equations of the robot using an ODE solver.
- Controller: implements a PD controller to control the drone to a desired pose.
- Visualization: plots the drone in a 3D space.

### Prerequisites
- Python 3.7

### Installing
To install the required dependencies, run the following command:

```
pip install -r requirements.txt
```

### Running
To run the simulation, you need to provide the following parameters:
- `q0`: Initial pose `[x, y, z, phi, theta, psi]`
- `qh`: Goal pose `[x, y, z, phi, theta, psi]`
- `th`: Hovering time
- `zt`: Take-off height
- `d` : Display velocity and accelartion on 2d graph

```
python main.py --q0 1.5 2.5 .4 0 0 0 --qh 1 1 0 0 2 3 --th 5 --zt 2 --d
```
Alternatively, you can define these parameters in the `config.py` file and run:
```
python main.py -n --d
```
If you use the `--d` option, the simulation speed will decrease, and the robot may lag. For better performance, you can run the simulation without the `--d` option.

![](https://github.com/SHBnik/DroneDynamics/blob/main/src/demo.gif)

## Credits
This project uses the visualization library from Eric Jaulin's works, available at https://www.ensta-bretagne.fr/jaulin/roblib.py.