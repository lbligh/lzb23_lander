# uncomment the next line if running in a notebook
# %matplotlib inline
import numpy as np
import matplotlib.pyplot as plt
import time


def main():

    # mass, spring constant, initial positions and velocities
    m = 1
    k = 1
    x_euler = x_verlet = 0
    v_euler = v_verlet = 1

    # simulation time, timestep and time
    t_max = 1000
    dt = 0.01
    t_array = np.arange(0, t_max, dt)
    ana_t = np.arange(0, t_max, 0.1)

    # analytical constants
    omega = np.sqrt(k/m)
    B = 1/omega

    # initialise empty lists to record trajectories
    euler_x_list = []
    euler_v_list = []
    verlet_x_list = []

    # initialise verlet velocity list - handled differently
    verlet_v_list = [v_verlet]

    # Integration
    for t in t_array:

        # append current state to trajectories
        euler_x_list.append(x_euler)
        euler_v_list.append(v_euler)
        verlet_x_list.append(x_verlet)

        # calculate new euler position and velocity
        a_euler = -k * x_euler / m
        x_euler = x_euler + dt * v_euler
        v_euler = v_euler + dt * a_euler

        # calculate new verlet postitions
        a_verlet = -k * x_verlet / m
        if t == 0:
            x_prev = x_verlet
            x_verlet = x_verlet + dt * v_verlet
        else:
            temp = x_verlet
            x_verlet = 2*x_verlet - x_prev + dt*dt*a_verlet
            x_prev = temp

    # calculate verlet velocities from positions
    for i in range(1, len(verlet_x_list)-1):
        verlet_v_list.append(
            (verlet_x_list[i+1]-verlet_x_list[i-1])/(2*dt)
        )
    verlet_v_list.append(
        (verlet_x_list[-1]-verlet_x_list[-2])/dt
    )

    print(verlet_x_list[0], verlet_v_list[0],
          t_array[0], euler_v_list[0], euler_x_list[0])

    # convert trajectory lists into arrays, so they can be sliced (useful for Assignment 2)
    euler_x_array = np.array(euler_x_list)
    euler_v_array = np.array(euler_v_list)
    x_array = np.array(verlet_x_list)
    v_array = np.array(verlet_v_list)

    # plot the position-time graphs
    fig, ax = plt.subplots()
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')
    ax.plot(t_array, x_array, label='Verlet')
    ax.plot(ana_t, B*np.sin(omega*ana_t), label='Analytical')
    plt.title(f'Position-time graph - verlet, dt = {dt}')
    ax.legend()

    fig, ax = plt.subplots()
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')
    ax.plot(t_array, euler_x_array, label='Euler')
    ax.plot(ana_t, B*np.sin(omega*ana_t), label='Analytical')
    plt.title(f'Position-time graph - euler, dt = {dt}')
    ax.legend()

    fig, ax = plt.subplots()
    plt.xlabel('time (s)')
    plt.ylabel('velocity (m\s)')
    ax.plot(t_array, v_array, label='Verlet')
    ax.plot(ana_t, np.cos(omega*ana_t), label='Analytical')
    plt.title(f'Velocity-time graph - verlet, dt = {dt}')
    ax.legend()

    fig, ax = plt.subplots()
    plt.xlabel('time (s)')
    plt.ylabel('velocity (m\s)')
    ax.plot(t_array, euler_v_array, label='Euler')
    ax.plot(ana_t, np.cos(omega*ana_t), label='Analytical')
    plt.title(f'Velocity-time graph - euler, dt = {dt}')
    ax.legend()

    plt.show()


if __name__ == '__main__':
    main()
