#pragma once

#include "MPC.hpp"
#include <chrono>
#include <iostream>
#include <fmt/format.h>

namespace mimpc {
    template<class SystemType, class SolverType>
    requires
    std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>> &&
    std::derived_from<SolverType, Solver<SystemType, SolverType::PRED_STEPS, SolverType::MIN_STEPS_ON, SolverType::MIN_STEPS_OFF, SolverType::MAX_STEPS_ON, SolverType::NUM_STEPS_SOLVER_DELAY, SolverType::INTEGRATION_SCHEME>>
    MPC<SystemType, SolverType>::MPC(
            SolverType &solver)
            :solver_(solver) {
        last_x_.setZero();
        last_u_.setZero();
        history_.setZero();
    }

    template<class SystemType, class SolverType>
    requires
    std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>> &&
    std::derived_from<SolverType, Solver<SystemType, SolverType::PRED_STEPS, SolverType::MIN_STEPS_ON, SolverType::MIN_STEPS_OFF, SolverType::MAX_STEPS_ON, SolverType::NUM_STEPS_SOLVER_DELAY, SolverType::INTEGRATION_SCHEME>>
      SOLVER_RETURN
    MPC<SystemType, SolverType>::control(const SystemType::StateVec &state,
                                         SystemType::InputVec &input,
                                         std::string &stats) {

        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        solver_.setState(state); //updates the system (linearizes)
        std::chrono::steady_clock::time_point set_state = std::chrono::steady_clock::now();
        solver_.addStateConstraintOnStep(0, state, state); //sets inital condition
        std::chrono::steady_clock::time_point set_init_cons = std::chrono::steady_clock::now();
        if (SolverType::history_depth > 0) {
            solver_.setInputHistory(
                    history_(Eigen::all, Eigen::seqN(0, SolverType::history_depth))); //sets thruster usage in history
        }
        if (SolverType::NUM_STEPS_SOLVER_DELAY > 0) {
            solver_.setNextInputs(history_.template rightCols<SolverType::NUM_STEPS_SOLVER_DELAY>());
        }
        std::chrono::steady_clock::time_point set_bin_hist = std::chrono::steady_clock::now();
        SOLVER_RETURN result = solver_.solve(last_u_, last_x_, last_u_, last_x_);
        std::chrono::steady_clock::time_point solve = std::chrono::steady_clock::now();

        long int solve_time = std::chrono::duration_cast<std::chrono::milliseconds>(solve - set_bin_hist).count();

        static int i = 0;
        std::printf("Control cycle %d", i++);

        std::printf(
                "\t \t \t \n set_state: %ld [ms] \n set_ini_cons: %ld [ms] \n set_bin_hist: %ld [ms] \n solve: %ld [ms] \n",
                std::chrono::duration_cast<std::chrono::milliseconds>(set_state - begin).count(),
                std::chrono::duration_cast<std::chrono::milliseconds>(set_init_cons - set_state).count(),
                std::chrono::duration_cast<std::chrono::milliseconds>(set_bin_hist - set_init_cons).count(),
                solve_time
        );

        stats = fmt::format("Took (realtime) [{} ms] ", solve_time);

        switch (result) {
            case USER_INTERRUPT:
              std::cout << "Solver was interrupted externally." << std::endl;
            break;
            case TIME_LIMIT_WITH_SOLUTION:
                stats += "Solver was early stopped! Feasible but NO ";
                [[fallthrough]];
            case OPTIMAL:
                stats += "OPTIMAL Solution found and proven.";
                break;
            case NO_SOLUTION:
                stats += "NO Solution found, going to use old solutions as long as possible. Otherwise zero output!";
                for (unsigned int n = 0; n < (SolverType::PRED_STEPS - 1); n++) {
                    last_u_.col(n) = last_u_.col(n + 1);
                }
                last_u_.col(SolverType::PRED_STEPS - 1).setZero();
                break;
        }
        input = last_u_(Eigen::all, solver_.getStepsToCompensateControllerDelay());
        for (unsigned int idx_col = 0; idx_col < (history_.cols() - 1); idx_col++) {
            history_.col(idx_col) = history_.col(idx_col + 1);
        }
        history_(Eigen::all, Eigen::last) = input;

        std::cout << stats.c_str() << std::endl;
        return result; //Propagating result for information purposes and to handle interrupt
    }
}