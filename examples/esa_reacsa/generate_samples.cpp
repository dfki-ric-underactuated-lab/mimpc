#include "sim/VizForces.hpp"
#include "systems/REACSA.hpp"
#include "solvers/SCIPSolver.hpp"
#include "sim/Simulation.hpp"
#include <math.h>
#include <iostream>

#include <cnpy.h>

using namespace mimpc;
using namespace mimpc::simulation;
using namespace mimpc::systems;

int main()
{
  unsigned int num_train = 10000;
  unsigned int num_test = 2000;
  unsigned int num_valid = 100;
  unsigned int num_transfer = 100;
  
  static constexpr int N = 20;
  static constexpr int max_steps_on = 3;
  static constexpr int min_steps_on = 1;
  static constexpr int min_steps_off = 2;
  static constexpr int num_break_trusts = 1;
  double rw_bound = 150.0 * reacsa_constants::RPM_2_RADPS;

  // TODO: also alternate weights
  REACSA::StateVec state_weight = {20.0, 20.0, 2., 0.0, 0.0, 1.0, 0.0};
  REACSA::StateVec state_final_weight = {40.0, 40.0, 2.0, 0.0, 0.0, 1.0, 0.0};
  REACSA::InputVec input_weight = {0.1, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0};

  REACSA::StateVec init_state;
  REACSA::StateVec target_state;

  using namespace reacsa_constants;

  double v_bound;
  double x_bound;

  // TODO: latency compensation?
  REACSA reacsa;
  SCIPSolver<REACSA, N, min_steps_on, min_steps_off, max_steps_on, 0, BACKWARD_EULER> solver(
      state_weight,
      state_final_weight,
      input_weight,
      target_state,
      COST_TYPE::L1Linear,
      reacsa,
      0.1);

  reacsa.get_limitcycle_v_bounds(num_break_trusts, v_bound, x_bound);
  double theta_d_bound = reacsa.get_angular_velocity_bound(rw_bound * RPM_2_RADPS);

  double rw_speed_min = -VELOCITY_REACTION_WHEEL_MAX * RPM_2_RADPS;
  double rw_speed_max = -VELOCITY_REACTION_WHEEL_MIN * RPM_2_RADPS;

  double rw_speed_range = (rw_speed_max - rw_speed_min);
  double rw_speed_range_half = rw_speed_range / 2.0;
  double rw_speed_center = rw_speed_min + rw_speed_range_half;



  Eigen::Matrix<double, REACSA::NUM_BIN_INPUTS, solver.history_depth> thruster_input_history;

  // create randomizers
  std::random_device rd; // Will be used to obtain a seed for the random number engine
  std::mt19937 gen(rd());

  std::uniform_real_distribution<double> x_value(-reacsa_constants::LENGTH_FLATFLOOR_X / 2. + 1.5 * reacsa_constants::RADIUS_REACSA, reacsa_constants::LENGTH_FLATFLOOR_X / 2. - 1.5 * reacsa_constants::RADIUS_REACSA);
  std::uniform_real_distribution<double> y_value(-reacsa_constants::LENGTH_FLATFLOOR_Y / 2. + 1.5 * reacsa_constants::RADIUS_REACSA, reacsa_constants::LENGTH_FLATFLOOR_Y / 2. - 1.5 * reacsa_constants::RADIUS_REACSA);
  std::uniform_real_distribution<double> theta_value(-M_PI, M_PI);
  std::uniform_real_distribution<double> x_dot_value(-reacsa_constants::VELOCITY_REACSA_MAX * DEG_2_RAD, reacsa_constants::VELOCITY_REACSA_MAX * DEG_2_RAD);
  std::uniform_real_distribution<double> y_dot_value(-reacsa_constants::VELOCITY_REACSA_MAX * DEG_2_RAD, reacsa_constants::VELOCITY_REACSA_MAX * DEG_2_RAD);
  std::uniform_real_distribution<double> theta_dot_value(-reacsa_constants::ROTATIONAL_VELOCITY_REACSA_MAX, +reacsa_constants::ROTATIONAL_VELOCITY_REACSA_MAX); // TODO: to correct unit
  std::uniform_real_distribution<double> rw_speed_value(rw_speed_min,rw_speed_max);          // TODO: to correct unit

  std::uniform_int_distribution<char> thruster_was_on(0, 1);
  std::uniform_int_distribution<unsigned int> thruster_was_on_for_steps(min_steps_on, max_steps_on);
  std::uniform_int_distribution<unsigned int> thruster_was_off_for_steos(min_steps_off, solver.history_depth);

  REACSA::StateVec state_const_lb = {-LENGTH_FLATFLOOR_X / 2. + 1.5 * RADIUS_REACSA,
                                     -LENGTH_FLATFLOOR_Y / 2. + 1.5 * RADIUS_REACSA,
                                     -ROTATION_REACSA_MAX * DEG_2_RAD * 3,
                                     -VELOCITY_REACSA_MAX,
                                     -VELOCITY_REACSA_MAX,
                                     -ROTATIONAL_VELOCITY_REACSA_MAX * DEG_2_RAD,
                                     rw_speed_min};
  REACSA::StateVec state_const_ub = {LENGTH_FLATFLOOR_X / 2 - 1.5 * RADIUS_REACSA,
                                     LENGTH_FLATFLOOR_Y / 2. - 1.5 * RADIUS_REACSA,
                                     ROTATION_REACSA_MAX * DEG_2_RAD * 3,
                                     VELOCITY_REACSA_MAX,
                                     VELOCITY_REACSA_MAX,
                                     ROTATIONAL_VELOCITY_REACSA_MAX * DEG_2_RAD,
                                     rw_speed_max};
  REACSA::StateVec state_final_lb = {state_const_lb(0),
                                     state_const_lb(1),
                                     state_const_lb(2),
                                     -v_bound,
                                     -v_bound,
                                     -theta_d_bound,
                                     (rw_speed_center)-rw_bound};
  REACSA::StateVec state_final_ub = {state_const_ub(0),
                                     state_const_ub(1),
                                     state_const_ub(2),
                                     v_bound,
                                     v_bound,
                                     theta_d_bound,
                                     (rw_speed_center) + rw_bound};

  for (unsigned int k = 0; k < REACSA::NUM_STATES; k++)
  {
    solver.addStateConstraintOnIndex(k, state_const_lb(k), state_const_ub(k));
    solver.addInputConstraintOnIndex(k, -TORQUE_REACTION_WHEEL_MAX, TORQUE_REACTION_WHEEL_MAX);
  }
  solver.addStateConstraintOnStep(N, state_final_lb,
                                  state_final_ub);

  for (unsigned int i = 0; i < (num_train + num_test + num_valid + num_transfer) ; i++)
  {
    init_state = {x_value(gen), y_value(gen), theta_value(gen), x_dot_value(gen), y_dot_value(gen), theta_dot_value(gen), rw_speed_value(gen)};
    target_state = {x_value(gen), y_value(gen), theta_value(gen), 0, 0, 0, 0};

    for (unsigned int t = 0; t < REACSA::NUM_BIN_INPUTS; t++)
    {
      if (static_cast<bool>(thruster_was_on(gen)))
      {
        auto num_on = std::min(thruster_was_on_for_steps(gen), static_cast<unsigned int>(solver.history_depth));
        thruster_input_history(t, Eigen::all).setZero();
        if(num_on > 0){
        thruster_input_history(t, Eigen::lastN(num_on)).setOnes();
        }
      }
      else
      {
        auto num_off = std::min(thruster_was_off_for_steos(gen), static_cast<unsigned int>(solver.history_depth));
        thruster_input_history(t, Eigen::all).setOnes();
        if(num_off > 0){
         thruster_input_history(t, Eigen::lastN(num_off)).setZero();
        }
      }
    }

    solver.setState(init_state);                                // updates the system (linearizes)
    solver.addStateConstraintOnStep(0, init_state, init_state); // sets inital condition

    std::string data_case = "train";
    if(i > num_train){
      data_case = "test";
    }else if(i > (num_train + num_test)){
      data_case = "valid";
    }else if(i > (num_train + num_test + num_valid)){
      data_case = "transfer";
    }

    solver.writeProblemToFile(fmt::format("data/instances/mimpc/{}/instances_{}.lp", data_case,i));
    std::cout << "Wrote problem " << i << " of " << num_train << std::endl;
  }
}
