#pragma once

#include <drake/solvers/mathematical_program.h>
#include <drake/solvers/solve.h>
#include <drake/solvers/solver_options.h>
#include <drake/solvers/solver_type.h>
#include <drake/solvers/clp_solver.h>

#include "SimpleSigmaDeltaModulator.hpp"
#include "LimetingSigmaDeltaModulator.hpp"

#include <array>

#include "../Solver.hpp"

namespace mimpc
{

    /**
     * Drake-backed MPC solver skeleton (LP for L1 via slacks, QP for L2).
     * NOTE: This skeleton only lays out the structure; you implement the details.
     */
    template <class SystemType, int N,
              int min_steps_on, int min_steps_off, int max_steps_on,
              int num_steps_solver_delay, INTEGRATION_SCHEME integration_scheme>
        requires std::derived_from<SystemType,
                                   System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>>
    class DrakeSolver final
        : public Solver<SystemType, N, min_steps_on, min_steps_off, max_steps_on,
                        num_steps_solver_delay, integration_scheme>
    {
        static_assert(integration_scheme == FORWARD_EULER,
                      "This skeleton targets FORWARD_EULER.");

    public:
        using SolverBase = Solver<SystemType, N, min_steps_on, min_steps_off, max_steps_on, num_steps_solver_delay, integration_scheme>;

        DrakeSolver(const typename SystemType::StateVec &state_weights,
                    const typename SystemType::StateVec &final_weights,
                    const typename SystemType::InputVec &input_weights,
                    const typename SystemType::StateVec &set_point,
                    COST_TYPE cost_type,
                    const SystemType &system,
                    double system_dt,
                    double controller_dt,
                    LimetingSigmaDeltaModulators<SystemType::NUM_BIN_INPUTS> & sdm

        );

        ~DrakeSolver() override = default;

        // ---- constraints (bulk & per-step) ----
        void addInputConstraintOnIndex(int index, double lb, double ub) override;
        void addStateConstraintOnIndex(int index, double lb, double ub) override;
        void addInputConstraintOnStep(int step,
                                      const typename SystemType::InputVec &lb,
                                      const typename SystemType::InputVec &ub) override;
        void addStateConstraintOnStep(int step,
                                      const typename SystemType::StateVec &lb,
                                      const typename SystemType::StateVec &ub) override;

        // ---- weights / cost / setpoint ----
        void setFinalWeights(const typename SystemType::StateVec &final_weights) override;
        void setStateWeights(const typename SystemType::StateVec &state_weights) override;
        void setInputWeights(const typename SystemType::InputVec &input_weights) override;
        void setCost(COST_TYPE cost_type) override; // switching allowed if you implement both paths
        void setSetPoint(const typename SystemType::StateVec &set_point) override;

        // ---- runtime data ----
        void setState(const typename SystemType::StateVec &state) override;
        void setInputHistory(const Eigen::Matrix<double, SystemType::NUM_INPUTS, SolverBase::history_depth> &bin_input_hist) override;
        void setNextInputs(const Eigen::Matrix<double, SystemType::NUM_INPUTS, num_steps_solver_delay> &next_inputs) override;
        void setSolverTimeLimit(double max_seconds) override;
        unsigned int getStepsToCompensateControllerDelay() override;

        // ---- solve ----
        SOLVER_RETURN solve(
            const Eigen::Matrix<double, SystemType::NUM_INPUTS, N, Eigen::RowMajor> &last_open_loop_input,
            const Eigen::Matrix<double, SystemType::NUM_STATES, N + 1, Eigen::RowMajor> &last_open_loop_state,
            Eigen::Matrix<double, SystemType::NUM_INPUTS, N, Eigen::RowMajor> &open_loop_input,
            Eigen::Matrix<double, SystemType::NUM_STATES, N + 1, Eigen::RowMajor> &open_loop_state) const override;

    private:
        // --- fixed data / sizes
        const SystemType system_;
        const double dt_;
        const double controller_dt_;

        COST_TYPE cost_type_;
        SystemType::StateVec final_weights_, weights_, set_point_;
        SystemType::InputVec input_weights_;
        drake::solvers::ClpSolver solver_;
        drake::solvers::SolverOptions solver_options_;

        LimetingSigmaDeltaModulators<SystemType::NUM_BIN_INPUTS> sigma_delta_modulator_;

        // Drake program and solver
        drake::solvers::MathematicalProgram prog_;
        drake::solvers::MatrixDecisionVariable<SystemType::NUM_STATES, N + 1> states_;
        drake::solvers::MatrixDecisionVariable<SystemType::NUM_INPUTS, N> inputs_;

        std::array<std::shared_ptr<drake::solvers::LinearEqualityConstraint>, N> dynamic_cons_;

        std::array<std::shared_ptr<drake::solvers::LinearConstraint>, N + 1> l1_state_cost_cons_;
        std::array<std::shared_ptr<drake::solvers::LinearConstraint>, N> l1_input_cost_cons_;

        std::array<std::array<std::shared_ptr<drake::solvers::BoundingBoxConstraint>, N + 1>, SystemType::NUM_STATES> state_constraints_;
        std::array<std::array<std::shared_ptr<drake::solvers::BoundingBoxConstraint>, N>, SystemType::NUM_INPUTS> input_constraints_;
    };

} // namespace mimpc

// Template implementation
#include "DrakeSolver.tpp"