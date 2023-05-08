import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import random

# create a figure and axis object
fig, axes = plt.subplots( nrows=2, figsize=(12, 5))
display_window=30
update_frq=1000

# set axis limits and labels
for ax in axes:
    ax.set_xlim(0, display_window)
    ax.set_ylim(-60, 70)
    ax.set_title("Plot of Position")
    plt.style.use('seaborn-white')
    # ax.xlabel("Time")
    # ax.ylabel("Position")

# create empty lists to store the data for each line
t1, t2,x_dot_buffer, y_dot_buffer, x_dd_buffer,y_dd_buffer = [], [], [],[],[],[]
env_time = range(0, 1000)

xdot = [random.randint(-10, 0) + (i ** 1.7) / 130 for i in range(0, 1000, 2)]
ydot = [random.randint(0, 10) + (i ** 1.5) / 110 for i in range(0, 1000, 2)]

xdd = [random.randint(-10, 0) + (i ** 1.7) / 130 for i in range(0, 1000, 2)]
ydd = [random.randint(0, 10) + (i ** 1.5) / 110 for i in range(0, 1000, 2)]


# create two lines with initial empty data and labels
# xdot_line, = axes.plot([], [], label='Red Line', color='red')
# ydot_line, = axes.plot([], [], label='Green Line', color='green')
xdot_line1, = axes[0].plot([], [], label='Red Line', color='red')
ydot_line1, = axes[0].plot([], [], label='Green Line', color='green')

xdot_line2, = axes[1].plot([], [], label='Red Line', color='red')
ydot_line2, = axes[1].plot([], [], label='Green Line', color='green')



# define the animation function
def animate(i):
    print("i=",i)
    t1.append(env_time[i])
    t2.append(env_time[i])
    print("t1 is", t1)
    print("t2 is", t2)
    x_dot_buffer.append(xdot[i])
    y_dot_buffer.append(ydot[i])

    x_dd_buffer.append(xdd[i])
    y_dd_buffer.append(ydd[i])

    # update the data for each line
    xdot_line1.set_data(t1, x_dot_buffer)
    ydot_line1.set_data(t1, y_dot_buffer)

    xdot_line2.set_data(t2, x_dd_buffer)
    ydot_line2.set_data(t2, y_dd_buffer)

    # update the legend with the labels for each line
    # for ax in axes:
    #     ax.legend(loc='upper right')
    #     if i >=display_window:
    #         t.pop(0)
    #         x_dot_buffer.pop(0)
    #         y_dot_buffer.pop(0)
    #         x_dd_buffer.pop(0)
    #         y_dd_buffer.pop(0)
    #         # print("x[0] is", t[0])
    #         # print("x[-1] is", t[-1])
    #         ax.set_xlim(t[0], t[-1])

    axes[0].legend(loc='upper right')
    if i >=display_window:
        t1.pop(0)
        x_dot_buffer.pop(0)
        y_dot_buffer.pop(0)
        axes[0].set_xlim(t1[0], t1[-1])

    axes[1].legend(loc='upper right')
    if i >=display_window:
        t2.pop(0)
        x_dd_buffer.pop(0)
        y_dd_buffer.pop(0)
        axes[1].set_xlim(t2[0], t2[-1])


    plt.pause(1/update_frq)

# create the animation object
ani = FuncAnimation(fig, animate,  repeat=True)
# show the final animation
plt.show()