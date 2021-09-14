// Mars lander simulator
// Version 1.11
// Mechanical simulation functions
// Gabor Csanyi and Andrew Gee, August 2019

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation, to make use of it
// for non-commercial purposes, provided that (a) its original authorship
// is acknowledged and (b) no modified versions of the source code are
// published. Restriction (b) is designed to protect the integrity of the
// exercise for future generations of students. The authors would be happy
// to receive any suggested modifications by private correspondence to
// ahg@eng.cam.ac.uk and gc121@eng.cam.ac.uk.

#include "lander.h"

void autopilot(void)
// Autopilot to adjust the engine throttle, parachute and attitude control
{
  double descent_rate, Kh, Kp, Kd, Ki, deriv, e, h, delta, P_out, current_mass, current_weight, prop, integ, der;
  static double sum_errors, previous_error;

  if (simulation_time <= 0.5)
  {
    previous_error = 0;
  }

  Kh = 23.0e-3;
  Kp = 2.3e-1;
  Kd = 0.5e-1;
  Ki = 0.01e-3;

  current_mass = UNLOADED_LANDER_MASS + fuel * FUEL_CAPACITY * FUEL_DENSITY;
  current_weight = GRAVITY * MARS_MASS * current_mass / (position.abs() * position.abs());
  delta = current_weight / MAX_THRUST;

  h = position.abs() - MARS_RADIUS;
  descent_rate = velocity * position.norm();

  e = -1 * (1.0 + Kh * h + descent_rate);
  deriv = (e - previous_error) / delta_t;
  cout << "acc: " << descent_rate << " target: " << -(0.5 + Kh * h) << " e: " << e << endl;

  prop = Kp * e;
  integ = Ki * sum_errors;
  der = Kd * deriv;

  cout << "prop: " << prop << " int: " << integ << " dev: " << der << endl;

  P_out = prop + der + integ;

  if (P_out <= -1 * delta)
    throttle = 0;
  else if (P_out < 1 - delta && P_out > -1 * delta)
    throttle = delta + P_out;
  else
    throttle = 1;

  if (safe_to_deploy_parachute() && parachute_status == NOT_DEPLOYED && h <= 10000)
  {
    parachute_status = DEPLOYED;
  }

  cout
      << "p_out: " << P_out << endl
      << "thr: " << throttle << " sum_errors: " << sum_errors << endl;

  ofstream fout;
  fout.open("trajectory5.txt", std::ios_base::app);
  if (fout)
  { // file opened successfully
    fout << simulation_time << ' ' << h << ' ' << descent_rate << ' ' << throttle
         << ' ' << prop << ' ' << integ << ' ' << deriv << endl;
  }
  else
  { // file did not open successfully
    cout << "failed to open file" << endl;
  }
  cout << endl;
  if (e * e < 600 * 600)
    sum_errors += ((e + previous_error) / 2) * delta_t;
  else
    sum_errors = 0;
  previous_error = e;
  attitude_stabilization();
}

void numerical_dynamics(void)
// This is the function that performs the numerical integration to update the
// lander's pose. The time step is delta_t (global variable).
{
  vector3d F_t, F_d, F_g, a, F_res, new_position;
  static vector3d prev_position;
  double current_mass;
  bool euler = false;

  current_mass = UNLOADED_LANDER_MASS + fuel * FUEL_CAPACITY * FUEL_DENSITY;

  F_t = thrust_wrt_world();
  F_g = -1 * GRAVITY * MARS_MASS * (current_mass)*position / (position.abs2() * position.abs());
  F_d = -0.5 * atmospheric_density(position) * pow(LANDER_SIZE, 2) * M_PI * velocity.abs() * velocity;
  if (parachute_status == DEPLOYED)
    F_d *= (DRAG_COEF_LANDER + DRAG_COEF_CHUTE);
  else
    F_d *= DRAG_COEF_LANDER;

  if (simulation_time == 0)
  {
    ofstream fout;
    fout.open("trajectory5.txt");
  }

  F_res = F_t + F_g + F_d;
  a = F_res / current_mass;

  if (euler) // Use Euler method
  {
    position = position + delta_t * velocity;
    velocity = velocity + delta_t * a;
  }
  else // Use Verlet method
  {
    if (simulation_time == 0)
    {
      cout << "using verlet" << endl;
      new_position = position + delta_t * velocity;
      velocity = velocity + delta_t * a;
    }
    else
    {
      new_position = 2 * position - prev_position + delta_t * delta_t * a;
      velocity = (new_position - position) / delta_t;
    }
    prev_position = position;
    position = new_position;
  }

  // Here we can apply an autopilot to adjust the thrust, parachute and attitude
  if (autopilot_enabled)
    autopilot();

  // Here we can apply 3-axis stabilization to ensure the base is always pointing downwards
  if (stabilized_attitude)
    attitude_stabilization();
}

void initialize_simulation(void)
// Lander pose initialization - selects one of 10 possible scenarios
{
  // The parameters to set are:
  // position - in Cartesian planetary coordinate system (m)
  // velocity - in Cartesian planetary coordinate system (m/s)
  // orientation - in lander coordinate system (xyz Euler angles, degrees)
  // delta_t - the simulation time step
  // boolean state variables - parachute_status, stabilized_attitude, autopilot_enabled
  // scenario_description - a descriptive string for the help screen

  scenario_description[0] = "circular orbit";
  scenario_description[1] = "descent from 10km";
  scenario_description[2] = "elliptical orbit, thrust changes orbital plane";
  scenario_description[3] = "polar launch at escape velocity (but drag prevents escape)";
  scenario_description[4] = "elliptical orbit that clips the atmosphere and decays";
  scenario_description[5] = "descent from 200km";
  scenario_description[6] = "";
  scenario_description[7] = "";
  scenario_description[8] = "";
  scenario_description[9] = "";

  switch (scenario)
  {

  case 0:
    // a circular equatorial orbit
    position = vector3d(1.2 * MARS_RADIUS, 0.0, 0.0);
    velocity = vector3d(0.0, -3247.087385863725, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 1:
    // a descent from rest at 10km altitude
    position = vector3d(0.0, -(MARS_RADIUS + 10000.0), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    break;

  case 2:
    // an elliptical polar orbit
    position = vector3d(0.0, 0.0, 1.2 * MARS_RADIUS);
    velocity = vector3d(3500.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 3:
    // polar surface launch at escape velocity (but drag prevents escape)
    position = vector3d(0.0, 0.0, MARS_RADIUS + LANDER_SIZE / 2.0);
    velocity = vector3d(0.0, 0.0, 5027.0);
    orientation = vector3d(0.0, 0.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 4:
    // an elliptical orbit that clips the atmosphere each time round, losing energy
    position = vector3d(0.0, 0.0, MARS_RADIUS + 100000.0);
    velocity = vector3d(4000.0, 0.0, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 5:
    // a descent from rest at the edge of the exosphere
    position = vector3d(0.0, -(MARS_RADIUS + EXOSPHERE), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    break;

  case 6:
    break;

  case 7:
    break;

  case 8:
    break;

  case 9:
    break;
  }
}
