#include <iostream>
#include <fstream>
#include <vector>

using namespace std;

int main()
{

  // declare variables
  double m, k, x, x_prev, temp, v, t_max, dt, t, a;
  vector<double> t_list, x_list, v_list;

  // mass, spring constant, initial position and velocity
  m = 1;
  k = 1;
  x = 0;
  v = 1;

  // simulation time and timestep
  t_max = 100;
  dt = 0.1;

  t_list.push_back(0);
  x_list.push_back(x);
  v_list.push_back(v);

  // Euler integration
  // for (t = dt; t <= t_max; t = t + dt)
  // {
  //   // calculate new position and velocity
  //   a = -k * x / m;
  //   x = x + dt * v;
  //   v = v + dt * a;

  //   // append current state to trajectories
  //   t_list.push_back(t);
  //   x_list.push_back(x);
  //   v_list.push_back(v);
  // }

  // Verlet integration
  x_prev = x;
  x = x + dt * v;
  t_list.push_back(dt);
  x_list.push_back(x);

  for (t = dt * 2; t <= t_max; t = t + dt)
  {
    // calculate new position and velocity
    a = -k * x / m;
    temp = x;
    x = 2 * x - x_prev + dt * dt * a;
    x_prev = temp;

    // append current state to trajectories
    t_list.push_back(t);
    x_list.push_back(x);
  }
  for (int i = 1; i < t_list.size() - 1; i = i + 1)
  {
    v = (x_list[i + 1] - x_list[i - 1]) / (2 * dt);
    v_list.push_back(v);
  }
  v_list.push_back((x_list[-1] - x_list[-2]) / dt);

  cout << t_list.size() << endl;
  cout << x_list.size() << endl;
  cout << v_list.size() << endl;

  // Write the trajectories to file
  ofstream fout;
  fout.open("trajectories.txt");
  if (fout)
  { // file opened successfully
    for (int i = 0; i < t_list.size(); i = i + 1)
    {
      fout << t_list[i] << ' ' << x_list[i] << ' ' << v_list[i] << endl;
    }
  }
  else
  { // file did not open successfully
    cout << "Could not open trajectory file for writing" << endl;
  }

  system("python3 /Users/lzb/Documents/University/Cambridge/1st\\ Year/Computing/lander/plt.py");
  /* The file can be loaded and visualised in Python as follows:

  import numpy as np
  import matplotlib.pyplot as plt
  results = np.loadtxt('trajectories.txt')
  plt.figure(1)
  plt.clf()
  plt.xlabel('time (s)')
  plt.grid()
  plt.plot(results[:, 0], results[:, 1], label='x (m)')
  plt.plot(results[:, 0], results[:, 2], label='v (m/s)')
  plt.legend()
  plt.show()

  */
}
