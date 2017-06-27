#include "MPC.h"
#include "params.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

params param;
// TODO: Set the timestep length and duration
size_t N = param.N;
double dt = param.dt;

const double Lf = 2.67;

double ref_cte = param.ref_cte;
double ref_epsi = param.ref_epsi;
double ref_v = param.ref_v;

size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
// Dummy variables for cte_limit2 and cte_limit3
size_t cte_cor_start2 = epsi_start + N;
size_t cte_cor_start3 = cte_cor_start2 + N;
size_t delta_start = cte_cor_start3 + N;
size_t a_start = delta_start + N - 1;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.
    fg[0] = 0;

    for (uint t = 0; t < N; t++) {
      fg[0] += param.c_cte * CppAD::pow(vars[cte_start+t],2);
      // Add cost for cte > cte_limit2
      fg[0] += param.c_cte2 * CppAD::pow(vars[cte_start+t] - vars[cte_cor_start2+t],2);
      // Add cost for cte > cte_limit3
      fg[0] += param.c_cte3 * CppAD::pow(vars[cte_start+t] - vars[cte_cor_start3+t],2);
      fg[0] += param.c_epsi * CppAD::pow(vars[epsi_start+t],2);
      fg[0] += param.c_v * CppAD::pow(vars[v_start+t] - ref_v,2);
    }

    for (uint t = 0; t < N - 1; t++) {
      fg[0] += param.c_str * CppAD::pow(vars[delta_start + t],2);
      fg[0] += param.c_a * CppAD::pow(vars[a_start + t], 2);
    }

    for (uint t = 0; t < N - 2; t++) {
      fg[0] += param.c_str_d * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += param.c_a_d * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }

    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // The rest of the constraints
    for (uint t = 1; t < N; t++) {
      // The state at time t+1
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];

      // The state at time t
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];

      // Actuation at time t
      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0 = vars[a_start + t - 1];

      AD<double> f0 = coeffs[0] +
                      (coeffs[1] * x0) +
                      (coeffs[2] * x0 * x0) +
                      (coeffs[3] * x0 * x0 * x0);
      AD<double> slope0 = coeffs[1] +
                          (coeffs[2] * 2 * x0) +
                          (coeffs[3] * 3 * x0 * x0);
      AD<double> psides0 = CppAD::atan(slope0);
      AD<double> slope1 = coeffs[1] +
                          (coeffs[2] * 2 * x1) +
                          (coeffs[3] * 3 * x1 * x1);
      AD<double> psides1 = CppAD::atan(slope1);

      // Add additional cost for slope to start turning in advance
      fg[0] += param.c_cte_m1 * CppAD::pow(cte0 - (param.c_cte_mm1 * slope1), 2);

      // Recall the equations for the model:
      // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      // v_[t+1] = v[t] + a[t] * dt
      // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
    }


  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {
  this->counter = 0;
  this->max_cte = 0.0;
  this->max_epsi = 0.0;
  this->max_steering = 0.0;
}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  //size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  this->counter++;
  this->total_speed += v;
  if (fabs(cte) > this->max_cte) {
    this->max_cte = fabs(cte);
  }
  if (fabs(epsi) > this->max_epsi) {
    this->max_epsi = fabs(epsi);
  }
  if (v > this->max_speed) {
    this->max_speed = v;
  }

  // The number of variables
  // 6 * N for model variables
  // 2 * (N - 1) for actuations
  // 2 * N for dummy cte limit variables
  //size_t n_vars = N * 8 + (N - 1) * 2;
  size_t n_vars = N * 8 + (N - 1) * 2;
  // Number of constraints
  size_t n_constraints = N * 6;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (uint i = 0; i < n_vars; i++) {
    vars[i] = 0.0;
  }

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // TODO: Set lower and upper limits for variables.
  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (uint i = 0; i < cte_cor_start2; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // Dummy variables should not be greater than the limits
  for (uint i = 0; i < N; i++) {
    vars_lowerbound[cte_cor_start2 + i] = -param.cte_limit2;
    vars_upperbound[cte_cor_start2 + i] = param.cte_limit2;

    vars_lowerbound[cte_cor_start3 + i] = -param.cte_limit3;
    vars_upperbound[cte_cor_start3 + i] = param.cte_limit3;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  // NOTE: Feel free to change this to something else.
  for (uint i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  // Acceleration/decceleration upper and lower limits.
  // NOTE: Feel free to change this to something else.
  for (uint i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (uint i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;

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
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  // TODO CHANGE max_cpu_time
  options += "Numeric max_cpu_time          0.1\n";

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
  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  vector<double> result;

  //double steering = solution.x[delta_start+param.delay_steps];
  //double throttle = solution.x[a_start+param.delay_steps];
  double steering = solution.x[delta_start];
  double throttle = solution.x[a_start];
    if (fabs(steering) > this->max_steering) {
    this->max_steering = fabs(steering);
  }

  std::cout << this->counter << " ";
  std::cout << "Cost: " << cost << "; STR:";
  std::cout << steering;
  std::cout << "; THR:"<<throttle;
  std::cout << "; spd:"<<v / 0.44704;
  std::cout << "; cte:"<<cte<<"; epsi:"<<epsi;
  std::cout << "; avr_spd:"<<this->total_speed / 0.44704 / this->counter;
  std::cout << "; max_spd:"<<this->max_speed / 0.44704;
  std::cout << "; max_cte:"<<this->max_cte;
  std::cout << "; max_epsi:"<<this->max_epsi;
  std::cout << "; max_str:"<<this->max_steering;
  std::cout<<std::endl;

  if (!ok) {
    std::cout<<std::endl<<" SOLUTION FAILED "<<std::endl<<std::endl;
  }

  result.push_back(steering);
  result.push_back(throttle);
  for (uint i = 0; i < N - 1; i++) {
    result.push_back(solution.x[x_start + i + 1]);
    result.push_back(solution.x[y_start + i + 1]);
  }
  return result;
}
