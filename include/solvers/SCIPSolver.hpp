#pragma once

#include <scip/scip.h>
#include <scip/scipdefplugins.h>
#include <array>
#include <type_traits>
#include <concepts>

#include "../Solver.hpp"

namespace mimpc {

    /**
     * Provides a solver based on the open source SCIP Solver (https://scipopt.org/)
     */
    template<class SystemType, int N, int min_steps_on, int min_steps_off, int max_steps_on, int num_steps_solver_delay, INTEGRATION_SCHEME integration_scheme> requires
    std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>>
    class SCIPSolver
            : public Solver<SystemType, N, min_steps_on, min_steps_off, max_steps_on, num_steps_solver_delay, integration_scheme> {
    public:
        using SolverBase = Solver<SystemType, N, min_steps_on, min_steps_off, max_steps_on, num_steps_solver_delay, integration_scheme>;
    private:
        SCIP *scip_problem_;
        std::array<std::array<SCIP_VAR *, N + 1>, SystemType::NUM_STATES> scip_state_vars_;
        std::array<std::array<SCIP_VAR *, N>, SystemType::NUM_INPUTS> scip_input_vars_;
        std::array<std::array<SCIP_CONS *, SystemType::NUM_STATES>, N> scip_dynamic_const_;
        std::array<std::array<SCIP_CONS *,
                SolverBase::history_depth + N -
                max_steps_on>, SystemType::NUM_BIN_INPUTS> scip_pattern_timing_max_const_;
        std::array<std::array<SCIP_CONS *, 1 + N - 2>, SystemType::NUM_BIN_INPUTS> scip_pattern_timing_min_const_;

        std::array<std::array<SCIP_VAR *, N + 1>, SystemType::NUM_STATES> scip_T_state_;
        std::array<std::array<SCIP_VAR *, N>, SystemType::NUM_INPUTS> scip_T_input_;
        std::array<std::array<SCIP_CONS *, N + 1>, SystemType::NUM_STATES> scip_Tconsp_state_;
        std::array<std::array<SCIP_CONS *, N + 1>, SystemType::NUM_STATES> scip_Tconsm_state_;
        std::array<std::array<SCIP_CONS *, N>, SystemType::NUM_INPUTS> scip_Tconsp_input_;
        std::array<std::array<SCIP_CONS *, N>, SystemType::NUM_INPUTS> scip_Tconsm_input_;


        std::array<std::array<SCIP_VAR *, N + 1>, SystemType::NUM_STATES> scip_QuadCostErr_states_;
        std::array<std::array<SCIP_CONS *, N + 1>, SystemType::NUM_STATES> scip_QuadCostErrCons_states_;

        SCIP_CONS *scip_QuadCostCons_;
        SCIP_Var *scip_QuadCostSum_;

        COST_TYPE cost_type_;
        bool compensate_controller_delay_;

        const SystemType &system_;
        double system_dt_;
        Eigen::Vector<double, SystemType::NUM_STATES> state_;
        Eigen::Vector<double, SystemType::NUM_STATES> state_cost_weights_;
        Eigen::Vector<double, SystemType::NUM_STATES> final_state_cost_weights_;
        Eigen::Vector<double, SystemType::NUM_INPUTS> input_cost_weights_;
        Eigen::Vector<double, SystemType::NUM_STATES> set_point_;

        void setCost(COST_TYPE cost_type) override;

    public:

        /**
         * Initializes a new SCIP Solver
         * @param state_weights vector that contains the weights for each state error
         * @param final_weights vector that contains the final weights for each state error
         * @param input_weights vector that contains the input weights
         * @param set_point the target state
         * @param cost_type the type of the cost function
         * @param system the concrete system
         * @param system_dt the timestep with which the system dynamics are discretized
         */
        SCIPSolver(const typename SystemType::StateVec &state_weights,
                   const typename SystemType::StateVec &final_weights,
                   const typename SystemType::InputVec &input_weights, const typename SystemType::StateVec &set_point,
                   COST_TYPE cost_type,
                   const SystemType &system, double system_dt);

        virtual ~SCIPSolver();

        void addInputConstraintOnIndex(int index, double lb, double ub) override;

        void addStateConstraintOnIndex(int index, double lb, double ub) override;

        void addInputConstraintOnStep(int step, const typename SystemType::InputVec &lb,
                                      const typename SystemType::InputVec &ub) override;

        void addStateConstraintOnStep(int step, const typename SystemType::StateVec &lb,
                                      const typename SystemType::StateVec &ub) override;

        void setFinalWeights(const typename SystemType::StateVec &final_weights) override;

        void setStateWeights(const typename SystemType::StateVec &state_weights) override;

        void setInputWeights(const typename SystemType::InputVec &state_weights) override;

        void setSetPoint(const typename SystemType::StateVec &set_point) override;

        void setState(const typename SystemType::StateVec &state) override;

        void setInputHistory(
                const typename Eigen::Matrix<double, SystemType::NUM_INPUTS, SolverBase::history_depth> &bin_input_hist) override;

        void setNextInputs(const typename Eigen::Matrix<double,
                SystemType::NUM_INPUTS, num_steps_solver_delay> &next_inputs) override;

        void setSolverTimeLimit(double max_seconds) override;

        SOLVER_RETURN
        solve(const typename Eigen::Matrix<double, SystemType::NUM_INPUTS, N, Eigen::RowMajor> &last_open_loop_input,
              const typename Eigen::Matrix<double, SystemType::NUM_STATES,
                      N + 1, Eigen::RowMajor> &last_open_loop_state,
              typename Eigen::Matrix<double, SystemType::NUM_INPUTS, N, Eigen::RowMajor> &open_loop_input,
              typename Eigen::Matrix<double, SystemType::NUM_STATES,
                      N + 1, Eigen::RowMajor> &open_loop_state) const override;

        unsigned int getStepsToCompensateControllerDelay() override;

        void writeProblemToFile(const std::string & file_path);
    };
};

#include "SCIPSolver.tpp"