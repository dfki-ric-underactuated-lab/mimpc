#include "sim/VizForces.hpp"
#include "systems/REACSA.hpp"
#include "solvers/SCIPSolver.hpp"
#include "sim/Simulation.hpp"
#include <math.h>
#include <iostream>
#include <fstream>
#include <filesystem>

#include <cnpy.h>

using namespace mimpc;
using namespace mimpc::simulation;
using namespace mimpc::systems;

int main()
{
  unsigned int num_train = 10000;
  unsigned int num_test = 2000;
  unsigned int num_valid = 200;
  unsigned int num_transfer = 1000;

  static constexpr int N = 40;
  static constexpr int max_steps_on = 3;
  static constexpr int min_steps_on = 1;
  static constexpr int min_steps_off = 2;
  static constexpr int num_break_trusts = 1;
  static constexpr double system_dt = 0.1;
  double rw_bound = 150.0 * reacsa_constants::RPM_2_RADPS;

  // TODO: also alternate weights
  REACSA::StateVec state_weight = {20.0, 20.0, 2., 0.0, 0.0, 1.0, 0.0};
  REACSA::StateVec state_final_weight = {40.0, 40.0, 2.0, 0.0, 0.0, 1.0, 0.0};

  double rw_weight = 0.1;
  double thrust_weight = 0.1;
  REACSA::InputVec input_weight = {rw_weight, thrust_weight, thrust_weight, thrust_weight, thrust_weight, thrust_weight, thrust_weight, thrust_weight, thrust_weight};
  REACSA::StateVec init_state;
  REACSA::StateVec target_state;
  init_state.setZero();
  target_state.setZero();

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
      system_dt);

  reacsa.get_limitcycle_v_bounds(num_break_trusts, v_bound, x_bound);
  double theta_d_bound = reacsa.get_angular_velocity_bound(rw_bound);

  double rw_speed_min = -VELOCITY_REACTION_WHEEL_MAX * RPM_2_RADPS;
  double rw_speed_max = -VELOCITY_REACTION_WHEEL_MIN * RPM_2_RADPS;

  double rw_speed_range = (rw_speed_max - rw_speed_min);
  double rw_speed_range_half = rw_speed_range / 2.0;
  double rw_speed_center = rw_speed_min + rw_speed_range_half;

  double max_feasible_lin_vel = v_bound;
  for (unsigned int i = 0; i < ((N - 1) / (max_steps_on + min_steps_off)); i++)
  {
    v_bound += 2 * reacsa_constants::FORCE_THRUSTER * max_steps_on / reacsa_constants::MASS_REACSA;
  }

  Eigen::Matrix<double, REACSA::NUM_BIN_INPUTS, solver.history_depth> thruster_input_history;

  // create randomizers
  //std::random_device rd; // Will be used to obtain a seed for the random number engine
  std::mt19937 gen; // Default seeded to always generate the same things

  std::uniform_real_distribution<double> x_value(-reacsa_constants::LENGTH_FLATFLOOR_X / 2. + 1.5 * reacsa_constants::RADIUS_REACSA, reacsa_constants::LENGTH_FLATFLOOR_X / 2. - 1.5 * reacsa_constants::RADIUS_REACSA);
  std::uniform_real_distribution<double> y_value(-reacsa_constants::LENGTH_FLATFLOOR_Y / 2. + 1.5 * reacsa_constants::RADIUS_REACSA, reacsa_constants::LENGTH_FLATFLOOR_Y / 2. - 1.5 * reacsa_constants::RADIUS_REACSA);
  std::uniform_real_distribution<double> theta_value(-M_PI, M_PI);
  std::uniform_real_distribution<double> x_dot_value(-max_feasible_lin_vel, max_feasible_lin_vel);
  std::uniform_real_distribution<double> y_dot_value(-max_feasible_lin_vel, max_feasible_lin_vel);
  std::uniform_real_distribution<double> theta_dot_value(-ROTATIONAL_VELOCITY_REACSA_MAX * DEG_2_RAD, ROTATIONAL_VELOCITY_REACSA_MAX * DEG_2_RAD); // TODO: to correct unit
  std::uniform_real_distribution<double> rw_speed_value(rw_speed_min, rw_speed_max);                                                               // TODO: to correct unit

  std::uniform_int_distribution<char> thruster_was_on(0, 1);
  std::uniform_int_distribution<unsigned int> thruster_was_on_for_steps(min_steps_on, max_steps_on);
  std::uniform_int_distribution<unsigned int> thruster_was_off_for_steps(min_steps_off, solver.history_depth);
  
  std::uniform_real_distribution<double> thrust_cost(0.01, 2.0);

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
  }

  solver.addInputConstraintOnIndex(0, -TORQUE_REACTION_WHEEL_MAX, TORQUE_REACTION_WHEEL_MAX);

  solver.addStateConstraintOnStep(N, state_final_lb,
                                  state_final_ub);
  
  std::filesystem::create_directory("data/");                                
  std::filesystem::create_directory("data/instances/");
  std::filesystem::create_directory("data/instances/mimpc/");
  std::filesystem::create_directory("data/instances/mimpc/test/");
  std::filesystem::create_directory("data/instances/mimpc/valid/");
  std::filesystem::create_directory("data/instances/mimpc/transfer/");

  Eigen::Matrix<double, Eigen::Dynamic, REACSA::NUM_STATES, Eigen::RowMajor> init_states(num_train, REACSA::NUM_STATES);
  Eigen::Matrix<double, Eigen::Dynamic, REACSA::NUM_STATES, Eigen::RowMajor> target_states(num_train, REACSA::NUM_STATES);
  Eigen::Matrix<double, Eigen::Dynamic, REACSA::NUM_BIN_INPUTS * solver.history_depth, Eigen::RowMajor> thruster_histories(num_train, REACSA::NUM_BIN_INPUTS * solver.history_depth);
  Eigen::VectorXi nnodes_processed(num_train);
  Eigen::VectorXi sol_node_ns(num_train);
  Eigen::VectorXi sol_times(num_train);
  Eigen::Matrix<double, Eigen::Dynamic, REACSA::NUM_INPUTS, Eigen::RowMajor> input_weights(num_train, REACSA::NUM_INPUTS);
  
  std::ofstream sol_file;
  sol_file.open("data/instances/mimpc/train/instance_solutions.json", std::ios::trunc);
  sol_file << "{";

  std::ofstream sol_file_valid;
  sol_file_valid.open("data/instances/mimpc/valid/instance_solutions.json", std::ios::trunc);
  sol_file_valid << "{";

  std::ofstream nnodes_file;
  nnodes_file.open("data/instances/mimpc/train/instance_nnodes.json", std::ios::trunc);
  nnodes_file << "{";

  for (unsigned int i = 1; i < (num_train + num_test + num_valid + num_transfer + 1); i++)
  {
    init_state = {x_value(gen), y_value(gen), theta_value(gen), x_dot_value(gen), y_dot_value(gen), theta_dot_value(gen), rw_speed_value(gen)};
    target_state = {x_value(gen), y_value(gen), theta_value(gen), 0, 0, 0, 0};

    thrust_weight = thrust_cost(gen);
    input_weight = {rw_weight, thrust_weight, thrust_weight,thrust_weight,thrust_weight,thrust_weight,thrust_weight,thrust_weight,thrust_weight};
    solver.setInputWeights(input_weight);

    for (unsigned int t = 0; t < REACSA::NUM_BIN_INPUTS; t++)
    {
      if (static_cast<bool>(thruster_was_on(gen)))
      {
        auto num_on = std::min(thruster_was_on_for_steps(gen), static_cast<unsigned int>(solver.history_depth));
        thruster_input_history(t, Eigen::all).setZero();
        if (num_on > 0)
        {
          thruster_input_history(t, Eigen::lastN(num_on)).setOnes();
        }
      }
      else
      {
        auto num_off = std::min(thruster_was_off_for_steps(gen), static_cast<unsigned int>(solver.history_depth));
        thruster_input_history(t, Eigen::all).setOnes();
        if (num_off > 0)
        {
          thruster_input_history(t, Eigen::lastN(num_off)).setZero();
        }
      }
    }

    solver.setState(init_state);                                // updates the system (linearizes)
    solver.addStateConstraintOnStep(0, init_state, init_state); // sets inital condition
    solver.setSetPoint(target_state);
    Eigen::Matrix<double, REACSA::NUM_INPUTS, solver.history_depth> input_history;
    input_history.setZero();
    input_history(Eigen::seq(1,Eigen::last), Eigen::all) = thruster_input_history;
    solver.setInputHistory(input_history);

    Eigen::Matrix<double, 9, N, 1> olu;
    Eigen::Matrix<double, 7, N+1, 1> olx;
    double obj_value;

    auto ret = solver.solve(Eigen::Matrix<double, 9, N>::Zero(), Eigen::Matrix<double, 7, N+1>::Zero(), olu, olx, obj_value);
    auto &  infos = solver.getSCIPInfos();

    std::string data_case = "train";
    unsigned int out_idx = i;
    if (i > num_train)
    {
      data_case = "valid";
      out_idx = i - num_train;
    }
    if (i > (num_train + num_valid))
    {
      data_case = "test";
      out_idx = i - (num_train + num_valid);
    }
    if (i > (num_train + num_valid + num_test))
    {
      data_case = "transfer";
      out_idx = i - (num_train + num_test + num_valid);
    }

    if(infos.sol_node_n <= 1 && (data_case == "train" || data_case == "valid")){
    std::cout << "nodes_processed to less - repeat " << i << std::flush;
     i--;
     std::cout << "\r";
     continue;
    }
    if (ret != SOLVER_RETURN::OPTIMAL)
    {
      std::cout << "not optimal (" << ret << ") - repeat" << i << std::flush;
      i--;
      std::cout << "\r";
    }
    else
    {
      auto file = fmt::format("data/instances/mimpc/{}/instance_{}.lp", data_case, out_idx);
      solver.setProblemName(file);
      solver.writeProblemToFile(file);
      if(i <= num_train){
        sol_file << "\"" << file << "\": " << obj_value << ", ";
        nnodes_file  << "\"" << file << "\": " <<  infos.sol_node_n << ", ";
        init_states(i - 1, Eigen::all) = init_state;
        target_states(i - 1, Eigen::all) = target_state;
        nnodes_processed(i - 1) = infos.nodes_processed;
        sol_node_ns(i - 1) = infos.sol_node_n;
        sol_times(i - 1) = infos.sol_time;
        input_weights(i -1, Eigen::all) = input_weight;
        thruster_histories(i - 1, Eigen::all) = thruster_input_history.transpose().reshaped(1, REACSA::NUM_BIN_INPUTS * solver.history_depth);
      }else if(i <= num_train + num_valid){
        sol_file_valid << "\"" << file << "\": " << obj_value << ", ";
      }
      if(i == num_train){
          sol_file << "}";
          sol_file.close();

          nnodes_file << "}";
          nnodes_file.close();

          cnpy::npz_save("data/instances/mimpc/train/instaces_info.npz", "init_states", init_states.data(), {num_train, REACSA::NUM_STATES}, "w");
          cnpy::npz_save("data/instances/mimpc/train/instaces_info.npz", "target_states", target_states.data(), {num_train, REACSA::NUM_STATES},"a");
          cnpy::npz_save("data/instances/mimpc/train/instaces_info.npz", "nnodes_processed", nnodes_processed.data(), {num_train},"a");
          cnpy::npz_save("data/instances/mimpc/train/instaces_info.npz", "sol_nodes", sol_node_ns.data(), {num_train},"a");
          cnpy::npz_save("data/instances/mimpc/train/instaces_info.npz", "sol_times", sol_times.data(), {num_train},"a");
          cnpy::npz_save("data/instances/mimpc/train/instaces_info.npz", "input_weights", input_weights.data(), {num_train, REACSA::NUM_INPUTS} ,"a");
          cnpy::npz_save("data/instances/mimpc/train/instaces_info.npz", "thruster_histories", thruster_histories.data(), {num_train, REACSA::NUM_BIN_INPUTS * solver.history_depth},"a");
          
      }

      if(i == num_train + num_valid){
        sol_file_valid << "}";
        sol_file_valid.close();
      }

      if(infos.nodes_processed <= 1){
        std::cout << "\033[1;31m";
      }else if(infos.nodes_processed <= 3){
        std::cout << "\033[1;33m";
      }
      std::cout << fmt::format("Wrote problem {}, in {} (Found best sol in node {} on depth {} after {}, found by {} - total nodes processed {})", i, data_case, infos.sol_node_n, infos.sol_depth, infos.sol_time, infos.sol_found_by, infos.nodes_processed) << std::endl;
      std::cout << "\033[0m\n";



      if(false){
        std::cout << "--------------\t" << "x, y, theta, x_d, y_d, theta_d, rw_speed" << std::endl;
        std::cout << "Init state:   \t" << init_state.transpose() << std::endl;
        std::cout << "Target state: \t" << target_state.transpose() << std::endl; 
        std::cout << "Input u0 : \t" << olu(0, Eigen::all) << std::endl;
        std::cout << "Input u1 : \t" << olu(1, Eigen::all) << std::endl;
        std::cout << "Input u2 : \t" << olu(2, Eigen::all) << std::endl;
        std::cout << "Input u3 : \t" << olu(3, Eigen::all) << std::endl;
        std::cout << "Input u4 : \t" << olu(4, Eigen::all) << std::endl;
        std::cout << "Input u5 : \t" << olu(5, Eigen::all) << std::endl;
        std::cout << "Input u6 : \t" << olu(6, Eigen::all) << std::endl;
        std::cout << "Input u7 : \t" << olu(7, Eigen::all) << std::endl;
        std::cout << "Input u8 : \t" << olu(8, Eigen::all) << std::endl;        
        std::cout << "Input t0 : \t" << olu(Eigen::all, 0).transpose() << std::endl;
      }

    }
  }


  
}
