# uncomment the next line if running in a notebook
# %matplotlib inline
import numpy as np
import matplotlib.pyplot as plt
import time

G = 6.673e-11
M = 6.42e23
R = 3386000.0


def f_g(m, r):
    global G
    global M
    return (-1*G*M*m/(np.linalg.norm(r)**2))*r/np.linalg.norm(r)


def hit_ground(pos):
    global R
    return np.linalg.norm(pos) <= R


def main():
    simul1()
    simul2()
    simul3()
    simul4()
    plt.show()


def simul1():
    global G
    global M
    global R

    t_max = 100
    dt = 0.001

    init_pos = np.array([100+R, 0, 0])

    init_v = np.array([0, 0, 0])

    t_array, euler_x_array, euler_v_array, verlet_x_array, verlet_v_array = simulate(
        init_pos, init_v, t_max, dt)

    print(verlet_x_array.shape, verlet_v_array.shape,
          t_array.shape, euler_v_array.shape, euler_x_array.shape)

    # print(euler_x_array)
    # print(t_array[verlet_x_array.shape[0]-1])
    # print(np.linalg.norm(euler_v_array[-1]))
    # print(np.linalg.norm(verlet_v_array[-1]))

    fig, ax = plt.subplots()
    ax.plot(t_array[:len(verlet_x_array)], verlet_x_array[:, 0])


def simul2():
    global G
    global M
    global R

    t_max = 10000
    dt = 0.5

    init_pos = np.array([1e6+R, 0, 0])

    v = np.sqrt(G*M/np.linalg.norm(init_pos))

    init_v = np.array([0, v, 0])

    t_array, euler_x_array, euler_v_array, verlet_x_array, verlet_v_array = simulate(
        init_pos, init_v, t_max, dt)

    print(verlet_x_array.shape, verlet_v_array.shape,
          t_array.shape, euler_v_array.shape, euler_x_array.shape)

    # print(euler_x_array)
    # print(t_array[verlet_x_array.shape[0]-1])
    # print(np.linalg.norm(euler_v_array[-1]))
    # print(np.linalg.norm(verlet_v_array[-1]))

    fig, ax = plt.subplots()
    fig.set_size_inches(6, 6)
    mars = plt.Circle((0, 0), R, color='r', label='Mars')
    ax.add_patch(mars)
    ax.plot(verlet_x_array[:, 0], verlet_x_array[:, 1], label='Path')
    plt.text(0, 0, 'Planet', ha="center")
    plt.legend()
    ax.set_aspect('equal')
    plt.axis('square')


def simul3():
    global G
    global M
    global R

    t_max = 1000000
    dt = 20

    init_pos = np.array([1e6+R, 0, 0])

    v = np.sqrt(1.5*G*M/np.linalg.norm(init_pos))

    init_v = np.array([0, v, 0])

    t_array, euler_x_array, euler_v_array, verlet_x_array, verlet_v_array = simulate(
        init_pos, init_v, t_max, dt)

    print(verlet_x_array.shape, verlet_v_array.shape,
          t_array.shape, euler_v_array.shape, euler_x_array.shape)

    # print(euler_x_array)
    # print(t_array[verlet_x_array.shape[0]-1])
    # print(np.linalg.norm(euler_v_array[-1]))
    # print(np.linalg.norm(verlet_v_array[-1]))

    fig, ax = plt.subplots()
    fig.set_size_inches(6, 6)
    mars = plt.Circle((0, 0), R, color='r', label='Mars')
    ax.add_patch(mars)
    ax.plot(verlet_x_array[:, 0], verlet_x_array[:, 1], label='Path')
    plt.text(0, 0, 'Planet', ha="center")
    plt.legend()
    ax.set_aspect('equal')
    plt.axis('square')


def simul4():
    global G
    global M
    global R

    t_max = 1000
    dt = 0.1

    ratio = np.array([1, 1, 0])
    init_pos = ratio * R/np.linalg.norm(ratio)

    v = np.sqrt(2*G*M/np.linalg.norm(init_pos))*2

    init_v = v*np.array([-1, 1, 0])/np.linalg.norm(np.array([-1, 1, 0]))

    t_array, euler_x_array, euler_v_array, verlet_x_array, verlet_v_array = simulate(
        init_pos, init_v, t_max, dt)

    print(verlet_x_array.shape, verlet_v_array.shape,
          t_array.shape, euler_v_array.shape, euler_x_array.shape)

    # print(euler_x_array)
    # print(t_array[verlet_x_array.shape[0]-1])
    # print(np.linalg.norm(euler_v_array[-1]))
    # print(np.linalg.norm(verlet_v_array[-1]))

    fig, ax = plt.subplots()
    fig.set_size_inches(6, 6)
    mars = plt.Circle((0, 0), R, color='r', label='Mars')
    ax.add_patch(mars)
    ax.plot(verlet_x_array[:, 0], verlet_x_array[:, 1], label='Path')
    plt.text(0, 0, 'Planet', ha="center")
    plt.legend()
    ax.set_aspect('equal')
    plt.axis('square')


def simulate(init_pos, init_v, t_max, dt):
    global G
    global M
    global R
    # mass, initial position and initial velocity
    m = 1
    x_euler = x_verlet = init_pos
    v_euler = v_verlet = init_v

    # simulation time, timestep and time
    # t_max = 100
    # dt = 0.001
    t_array = np.arange(0, t_max, dt)

    # initialise lists to record trajectories
    euler_x_array = np.array([x_euler])
    euler_v_array = np.array([v_euler])
    verlet_x_array = np.array([x_verlet])
    verlet_v_array = np.array([v_verlet])

    # Euler Integration
    for t in t_array[1:]:

        # calculate new euler position and velocity
        a_euler = f_g(m, x_euler) / m
        x_euler = x_euler + dt * v_euler
        v_euler = v_euler + dt * a_euler

        euler_x_array = np.append(euler_x_array, np.array([x_euler]), axis=0)
        euler_v_array = np.append(euler_v_array, np.array([v_euler]), axis=0)

        if hit_ground(x_euler):
            break

    # Verlet Integration
    x_prev = x_verlet
    x_verlet = x_verlet + dt * v_verlet
    verlet_x_array = np.append(verlet_x_array, np.array([x_verlet]), axis=0)

    for t in t_array[2:]:
        # calculate new verlet postitions
        a_verlet = f_g(m, x_verlet) / m
        temp = x_verlet
        x_verlet = 2*x_verlet - x_prev + dt*dt*a_verlet
        x_prev = temp

        verlet_x_array = np.append(
            verlet_x_array, np.array([x_verlet]), axis=0)

        if hit_ground(x_verlet):
            break

    # calculate verlet velocities from positions
    for i in range(1, len(verlet_x_array)-1):
        v = (verlet_x_array[i+1]-verlet_x_array[i-1])/(2*dt)
        verlet_v_array = np.append(verlet_v_array, np.array([v]), axis=0)

    verlet_v_array = np.append(
        verlet_v_array, np.array([(verlet_x_array[-1]-verlet_x_array[-2])/dt]), axis=0
    )

    return t_array, euler_x_array, euler_v_array, verlet_x_array, verlet_v_array


if __name__ == '__main__':
    main()
