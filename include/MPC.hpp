#pragma once

#include "Solver.hpp"
#include <type_traits>
#include <concepts>

namespace mimpc {
/**
 * This class provides a highlevel mpc interface.
 *
 * @tparam SystemType the system type which shall be controlled by the mpc must be of type System
 * @tparam SolverType the underlying solver which solves the optimization problem, must be of type Solver
 */
    template<class SystemType, class SolverType> requires
    std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>> &&
    std::derived_from<SolverType, Solver<SystemType, SolverType::PRED_STEPS, SolverType::MIN_STEPS_ON, SolverType::MIN_STEPS_OFF, SolverType::MAX_STEPS_ON, SolverType::NUM_STEPS_SOLVER_DELAY, SolverType::INTEGRATION_SCHEME>>
    class MPC {
    public:
        using SYSTEM_TYPE = SystemType;
        using SOLVER_TYPE = SolverType;
    private:
        SolverType &solver_;
        Eigen::Matrix<double, SystemType::NUM_STATES, SolverType::PRED_STEPS + 1, Eigen::RowMajor> last_x_;
        Eigen::Matrix<double, SystemType::NUM_CONT_INPUTS +
                              SystemType::NUM_BIN_INPUTS, SolverType::PRED_STEPS, Eigen::RowMajor> last_u_;
        Eigen::Matrix<double,
                SystemType::NUM_CONT_INPUTS + SystemType::NUM_BIN_INPUTS,
                SolverType::history_depth + SolverType::NUM_STEPS_SOLVER_DELAY> history_;
    public:
        /**
         * Instantiates the MPC.
         *
         * @param solver a reference to the instance of the underlying optimization problem. The optimization problem needs to be configured via the low level interface.
         */
        explicit MPC(SolverType &solver);

        /**
         * Starts the solving process based on the current state and returns the optimal input.
         *
         * @param state the current system state
         * @param input the optimal input
         * @param stats statistics about the solving process
         *
         * @param returns the solver status even though no solutions are already handled by this class by using the old predictions
         */
        SOLVER_RETURN control(const SystemType::StateVec &state, SystemType::InputVec &input, std::string &stats);

        virtual ~MPC() = default;
    };
};

#include "MPC.tpp"