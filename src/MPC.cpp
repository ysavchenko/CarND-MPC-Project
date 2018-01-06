#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

const int VAR_X = 0;
const int VAR_Y = 1;
const int VAR_PSI = 2;
const int VAR_V = 3;
const int VAR_CTE = 4;
const int VAR_EPSI = 5;
const int VAR_DELTA = 6;
const int VAR_A = 7;
std::vector<int> STATE_VARS = { VAR_X, VAR_Y, VAR_PSI, VAR_V, VAR_CTE, VAR_EPSI };
std::vector<int> ALL_VARS = { VAR_X, VAR_Y, VAR_PSI, VAR_V, VAR_CTE, VAR_EPSI, VAR_DELTA, VAR_A };

const double AVG_SPEED = 15;
const double MAX_STEERING = 25. * M_PI / 180.;

// TODO: Set the timestep length and duration
size_t N = 15;
double dt = 0.2;

// Return index of particular variable in array
int var_index(int type, int timestep) {
  int index = type * N + timestep;
  if (type == VAR_A) index -= 1;  // We use -1 here because variable before acceleration (delta) has N-1 steps
  return index;
}
int var_index(int type) {
  return var_index(type, 0);
}

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    
    // Now we'll fill fg vector. It should contain the following:
    // fg[0] = f(x), which is the function we're uptimizing (cost)
    // and for i=0,…,ng−1, 
    // fg[1 + i] = gi(x), which is the constraints (initial state and then differences between t and t-1 for each state variable)

    // Step 1: Calculate cost
    fg[0] = 0;
    for (int timestep = 0; timestep < N; timestep++) {
      // Minimize position error
      fg[0] += CppAD::pow(vars[var_index(VAR_CTE, timestep)], 2) * 1000;
      // Minimize angle error
      fg[0] += CppAD::pow(vars[var_index(VAR_EPSI, timestep)], 2);
      // Try to have the same speed
      fg[0] += CppAD::pow(vars[var_index(VAR_V, timestep)] - AVG_SPEED, 2);
      // Try to minimize use of actuators
      if (timestep < N - 1) {
        fg[0] += CppAD::pow(vars[var_index(VAR_DELTA, timestep)], 2);
        fg[0] += CppAD::pow(vars[var_index(VAR_A, timestep)], 2);
      }
      // Try to minimize difference between actuators
      if (timestep < N - 2) { // We use N - 2 because we access the next timestep in the loop (first -1) and actuators variable vector has length N - 1 (second -1)
        fg[0] += CppAD::pow(vars[var_index(VAR_DELTA, timestep)] - vars[var_index(VAR_DELTA, timestep + 1)], 2);
        fg[0] += CppAD::pow(vars[var_index(VAR_A, timestep)] - vars[var_index(VAR_A, timestep + 1)], 2);
      }
    }

    // Step 2: Set up g function of constraints
    AD<double> last[ALL_VARS.size()];
    for (int timestep = 0; timestep < N; timestep++) {
      for (int var : STATE_VARS) {
        fg[1 + var_index(var, timestep)] = vars[var_index(var, timestep)];
      }

      if (timestep != 0) {
        // Now subtract values calculated using our kinematic model
        // x[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
        // y[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
        // psi[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
        // v[t] = v[t-1] + a[t-1] * dt
        // cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
        // epsi[t] = psi[t-1] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt
        fg[1 + var_index(VAR_X, timestep)] -= last[VAR_X] + last[VAR_V] * CppAD::cos(last[VAR_PSI]) * dt;
        fg[1 + var_index(VAR_Y, timestep)] -= last[VAR_Y] + last[VAR_V] * CppAD::sin(last[VAR_PSI]) * dt;
        fg[1 + var_index(VAR_PSI, timestep)] -= last[VAR_PSI] + last[VAR_V] / Lf * last[VAR_DELTA] * dt;
        fg[1 + var_index(VAR_V, timestep)] -= last[VAR_V] + last[VAR_A] * dt;

        // Calculate f and its derivative
        AD<double> f_last = coeffs[0];
        f_last += coeffs[1] * last[VAR_X];
        f_last += coeffs[2] * CppAD::pow(last[VAR_X], 2);
        f_last += coeffs[3] * CppAD::pow(last[VAR_X], 3);
        AD<double> psides_last = coeffs[1];
        psides_last += coeffs[2] * last[VAR_X] * 2;
        psides_last += coeffs[3] * CppAD::pow(last[VAR_X], 2) * 3;
        psides_last = CppAD::atan(psides_last);

        fg[1 + var_index(VAR_CTE)] -= f_last - last[VAR_Y] + last[VAR_V] * CppAD::sin(last[VAR_PSI]) * dt;
        fg[1 + var_index(VAR_EPSI)] -= last[VAR_PSI] - psides_last + last[VAR_V] * last[VAR_DELTA] / Lf * dt;
      }

      if (timestep < N - 1) {
        for (int var : ALL_VARS) {
          last[var] = vars[var_index(var, timestep)];
        }
      }
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

const int STATE_SIZE = 6;
const int ACTUATORS_SIZE = 2;

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs, std::vector<double> &mpc_x_vals, std::vector<double> &mpc_y_vals) {
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // Set the number of model variables (includes both states and inputs).
  size_t n_vars = N * STATE_SIZE + (N - 1) * ACTUATORS_SIZE;
  // TODO: Set the number of constraints
  size_t n_constraints = N * STATE_SIZE;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }
  for (int i = 0; i < state.size(); i++) {
    vars[var_index(i)] = state[i];
  }

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // Set lower and upper limits for variables.
  for (int timestep = 0; timestep < N; timestep++) {
    for (int var : STATE_VARS) {
      vars_lowerbound[var_index(var, timestep)] = -numeric_limits<double>::max();
      vars_upperbound[var_index(var, timestep)] = numeric_limits<double>::max();
    }
    if (timestep < N - 1) {
      vars_lowerbound[var_index(VAR_DELTA, timestep)] = -MAX_STEERING;
      vars_upperbound[var_index(VAR_DELTA, timestep)] = MAX_STEERING;
      vars_lowerbound[var_index(VAR_A, timestep)] = -1;
      vars_upperbound[var_index(VAR_A, timestep)] = 1;
    }
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  for (int i = 0; i < state.size(); i++) {
    constraints_lowerbound[var_index(i, 0)] = state[i];
    constraints_upperbound[var_index(i, 0)] = state[i];
  }

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  //options += "String  linear_solver mumps\n";
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // Fill predicted position vectors
  for (int timestep = 0; timestep < N; timestep++) {
    mpc_x_vals.push_back(solution.x[var_index(VAR_X, timestep)]);
    mpc_y_vals.push_back(solution.x[var_index(VAR_Y, timestep)]);
  }

  // Return the first actuator values. The variables can be accessed with
  return { solution.x[var_index(VAR_DELTA, 0)], solution.x[var_index(VAR_A, 0)] };
}

void MPC::ApplyLatency(double &px, double &py, double& psi, double& v, double steering, double throttle, double latency) {
  px += v * cos(psi) * latency;
  py += v * sin(psi) * latency;
  psi += v * steering / Lf * latency;
  v += throttle * latency;
}