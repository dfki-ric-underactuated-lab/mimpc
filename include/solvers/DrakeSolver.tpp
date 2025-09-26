#include "DrakeSolver.hpp"

#pragma once
#include <limits>
#include <iostream>

namespace mimpc
{

    // ---------- ctor ----------
    template <class SystemType, int N, int min_steps_on, int min_steps_off, int max_steps_on, int num_steps_solver_delay, INTEGRATION_SCHEME integration_scheme>
        requires std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>>
    DrakeSolver<SystemType, N, min_steps_on, min_steps_off, max_steps_on, num_steps_solver_delay, integration_scheme>::DrakeSolver(
        const typename SystemType::StateVec &state_weights,
        const typename SystemType::StateVec &final_weights,
        const typename SystemType::InputVec &input_weights,
        const typename SystemType::StateVec &set_point,
        COST_TYPE cost_type,
        const SystemType &system,
        double system_dt,
        double controller_dt,
        LimetingSigmaDeltaModulators<SystemType::NUM_BIN_INPUTS> &sigma_delta_Modulator,
        int mi_informed)
        : weights_(state_weights),
          final_weights_(final_weights),
          input_weights_(input_weights),
          set_point_(set_point),
          system_(system),
          dt_(system_dt),
          controller_dt_(controller_dt),
          cost_type_(cost_type),
          sigma_delta_modulator_(sigma_delta_Modulator),
          mi_informed_(mi_informed)
    {

        states_ = prog_.NewContinuousVariables<SystemType::NUM_STATES, N + 1>("x");
        inputs_ = prog_.NewContinuousVariables<SystemType::NUM_INPUTS, N>("u");
        auto state = SystemType::StateVec::Zero();
        Eigen::Matrix<double, SystemType::NUM_STATES, SystemType::NUM_STATES * 2 + SystemType::NUM_INPUTS> A_full;
        A_full.template block<SystemType::NUM_STATES, SystemType::NUM_STATES>(0, 0) = Eigen::Matrix<double, SystemType::NUM_STATES, SystemType::NUM_STATES>::Identity() +
                                                                                      (dt_ * system.getA(state));
        A_full.template block<SystemType::NUM_STATES, SystemType::NUM_INPUTS>(0, SystemType::NUM_STATES) = dt_ * system.getB(state);

        A_full.template block<SystemType::NUM_STATES, SystemType::NUM_STATES>(0, SystemType::NUM_STATES + SystemType::NUM_INPUTS) = -1 * Eigen::Matrix<double, SystemType::NUM_STATES, SystemType::NUM_STATES>::Identity();

        // Add dynamics & constraints
        for (unsigned int n = 0; n < N; n++)
        {
            drake::solvers::VectorDecisionVariable<2 * SystemType::NUM_STATES + SystemType::NUM_INPUTS> vars;
            vars << states_.template block<SystemType::NUM_STATES, 1>(0, n), inputs_.template block<SystemType::NUM_INPUTS, 1>(0, n), states_.template block<SystemType::NUM_STATES, 1>(0, n + 1);
            dynamic_cons_[n] = prog_.AddLinearEqualityConstraint(A_full, Eigen::Vector<double, SystemType::NUM_STATES>::Zero(), vars).evaluator();
            for (unsigned int i = 0; i < SystemType::NUM_CONT_INPUTS; i++)
            {
                input_constraints_[i][n] = prog_.AddBoundingBoxConstraint(-std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), inputs_(i, n)).evaluator();
            }
            for (unsigned int i = SystemType::NUM_CONT_INPUTS; i < SystemType::NUM_INPUTS; i++)
            {
                input_constraints_[i][n] = prog_.AddBoundingBoxConstraint(0, 1, inputs_(i, n)).evaluator();
            }
            for (unsigned int i = 0; i < SystemType::NUM_STATES; i++)
            {
                state_constraints_[i][n] = prog_.AddBoundingBoxConstraint(-std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), states_(i, n)).evaluator();
            }
            l1_state_cost_cons_[n] = std::get<2>(prog_.AddL1NormCostInEpigraphForm(state_weights.asDiagonal().toDenseMatrix(), -1 * state_weights.asDiagonal().toDenseMatrix() * set_point, states_.template block<SystemType::NUM_STATES, 1>(0, n))).evaluator();
            l1_input_cost_cons_[n] = std::get<2>(prog_.AddL1NormCostInEpigraphForm(input_weights.asDiagonal().toDenseMatrix(), Eigen::Vector<double, SystemType::NUM_INPUTS>::Zero(), inputs_.template block<SystemType::NUM_INPUTS, 1>(0, n))).evaluator();
        }
        for (unsigned int i = 0; i < SystemType::NUM_STATES; i++)
        {
            state_constraints_[i][N] = prog_.AddBoundingBoxConstraint(0, 0, states_(i, N)).evaluator();
        }

        l1_state_cost_cons_[N] = std::get<2>(prog_.AddL1NormCostInEpigraphForm(final_weights.asDiagonal().toDenseMatrix(), -1 * final_weights.asDiagonal().toDenseMatrix() * set_point, states_.template block<SystemType::NUM_STATES, 1>(0, N))).evaluator();
    }

    // ---------- public setters / modifiers ----------
    template <class SystemType, int N, int min_steps_on, int min_steps_off, int max_steps_on, int num_steps_solver_delay, INTEGRATION_SCHEME integration_scheme>
        requires std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>>
    void DrakeSolver<SystemType, N, min_steps_on, min_steps_off, max_steps_on, num_steps_solver_delay, integration_scheme>::addInputConstraintOnIndex(int index, double lb, double ub)
    {
        for (unsigned int n = 0; n < N; n++)
        {
            input_constraints_[index][n]->UpdateLowerBound(Eigen::Vector<double, 1>(lb));
            input_constraints_[index][n]->UpdateUpperBound(Eigen::Vector<double, 1>(ub));
        }
    }

    template <class SystemType, int N, int min_steps_on, int min_steps_off, int max_steps_on, int num_steps_solver_delay, INTEGRATION_SCHEME integration_scheme>
        requires std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>>
    void DrakeSolver<SystemType, N, min_steps_on, min_steps_off, max_steps_on, num_steps_solver_delay, integration_scheme>::addStateConstraintOnIndex(int index, double lb, double ub)
    {
        for (unsigned int n = 0; n <= N; n++)
        {
            state_constraints_[index][n]->UpdateLowerBound(Eigen::Vector<double, 1>(lb));
            state_constraints_[index][n]->UpdateUpperBound(Eigen::Vector<double, 1>(ub));
        }
    }

    template <class SystemType, int N, int min_steps_on, int min_steps_off, int max_steps_on, int num_steps_solver_delay, INTEGRATION_SCHEME integration_scheme>
        requires std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>>
    void DrakeSolver<SystemType, N, min_steps_on, min_steps_off, max_steps_on, num_steps_solver_delay, integration_scheme>::addInputConstraintOnStep(
        int step, const typename SystemType::InputVec &lb, const typename SystemType::InputVec &ub)
    {
        for (unsigned int i = 0; i < SystemType::NUM_INPUTS; i++)
        {
            input_constraints_[i][step]->UpdateLowerBound(lb.template segment<1>(i));
            input_constraints_[i][step]->UpdateUpperBound(ub.template segment<1>(i));
        }
    }

    template <class SystemType, int N, int min_steps_on, int min_steps_off, int max_steps_on, int num_steps_solver_delay, INTEGRATION_SCHEME integration_scheme>
        requires std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>>
    void DrakeSolver<SystemType, N, min_steps_on, min_steps_off, max_steps_on, num_steps_solver_delay, integration_scheme>::addStateConstraintOnStep(
        int step, const typename SystemType::StateVec &lb, const typename SystemType::StateVec &ub)
    {
        for (unsigned int i = 0; i < SystemType::NUM_STATES; i++)
        {
            state_constraints_[i][step]->UpdateLowerBound(lb.template segment<1>(i));
            state_constraints_[i][step]->UpdateUpperBound(ub.template segment<1>(i));
        }
    }

    template <class SystemType, int N, int min_steps_on, int min_steps_off, int max_steps_on, int num_steps_solver_delay, INTEGRATION_SCHEME integration_scheme>
        requires std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>>
    void DrakeSolver<SystemType, N, min_steps_on, min_steps_off, max_steps_on, num_steps_solver_delay, integration_scheme>::setFinalWeights(const typename SystemType::StateVec &final_weights)
    {
        final_weights_ = final_weights;

        // Reverse engeneert from drake code:

        Eigen::MatrixXd A = final_weights.asDiagonal().toDenseMatrix();
        Eigen::VectorXd b = -1 * final_weights.asDiagonal().toDenseMatrix() * set_point_;

        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(A.rows(), A.rows());
        Eigen::MatrixXd A_full(2 * A.rows(), A.rows() + A.cols());
        A_full.topRows(A.rows()) << -I, A;
        A_full.bottomRows(A.rows()) << -I, -1 * A;

        Eigen::VectorXd lb = Eigen::VectorXd::Constant(
            2 * A.rows(), -std::numeric_limits<double>::infinity());
        Eigen::VectorXd ub(2 * A.rows());
        ub.head(A.rows()) = -b;
        ub.tail(A.rows()) = b;
        l1_state_cost_cons_[N]->UpdateCoefficients(A_full, lb, ub);
    }

    template <class SystemType, int N, int min_steps_on, int min_steps_off, int max_steps_on, int num_steps_solver_delay, INTEGRATION_SCHEME integration_scheme>
        requires std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>>
    void DrakeSolver<SystemType, N, min_steps_on, min_steps_off, max_steps_on, num_steps_solver_delay, integration_scheme>::setStateWeights(const typename SystemType::StateVec &state_weights)
    {
        weights_ = state_weights;

        // Reverse engeneert from drake code:

        Eigen::MatrixXd A = weights_.asDiagonal().toDenseMatrix();
        Eigen::VectorXd b = -1 * weights_.asDiagonal().toDenseMatrix() * set_point_;

        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(A.rows(), A.rows());
        Eigen::MatrixXd A_full(2 * A.rows(), A.rows() + A.cols());
        A_full.topRows(A.rows()) << -I, A;
        A_full.bottomRows(A.rows()) << -I, -1 * A;

        Eigen::VectorXd lb = Eigen::VectorXd::Constant(
            2 * A.rows(), -std::numeric_limits<double>::infinity());
        Eigen::VectorXd ub(2 * A.rows());
        ub.head(A.rows()) = -b;
        ub.tail(A.rows()) = b;
        for (unsigned int n = 0; n < N; n++)
        {
            l1_state_cost_cons_[n]->UpdateCoefficients(A_full, lb, ub);
        }
    }

    template <class SystemType, int N, int min_steps_on, int min_steps_off, int max_steps_on, int num_steps_solver_delay, INTEGRATION_SCHEME integration_scheme>
        requires std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>>
    void DrakeSolver<SystemType, N, min_steps_on, min_steps_off, max_steps_on, num_steps_solver_delay, integration_scheme>::setInputWeights(const typename SystemType::InputVec &input_weights)
    {
        input_weights_ = input_weights;
        // Reverse engeneert from drake code:

        Eigen::MatrixXd A = input_weights_.asDiagonal().toDenseMatrix();
        Eigen::VectorXd b = Eigen::Vector<double, SystemType::NUM_INPUTS>::Zero();

        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(A.rows(), A.rows());
        Eigen::MatrixXd A_full(2 * A.rows(), A.rows() + A.cols());
        A_full.topRows(A.rows()) << -I, A;
        A_full.bottomRows(A.rows()) << -I, -A;

        Eigen::VectorXd lb = Eigen::VectorXd::Constant(
            2 * A.rows(), -std::numeric_limits<double>::infinity());
        Eigen::VectorXd ub(2 * A.rows());
        ub.head(A.rows()) = -b;
        ub.tail(A.rows()) = b;
        for (unsigned int n = 0; n < N; n++)
        {
            l1_input_cost_cons_[n]->UpdateCoefficients(A_full, lb, ub);
        }
    }

    template <class SystemType, int N, int min_steps_on, int min_steps_off, int max_steps_on, int num_steps_solver_delay, INTEGRATION_SCHEME integration_scheme>
        requires std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>>
    void DrakeSolver<SystemType, N, min_steps_on, min_steps_off, max_steps_on, num_steps_solver_delay, integration_scheme>::setCost(COST_TYPE cost_type)
    {
        cost_type_ = cost_type;
        throw std::logic_error("Function not yet implemented");
    }

    template <class SystemType, int N, int min_steps_on, int min_steps_off, int max_steps_on, int num_steps_solver_delay, INTEGRATION_SCHEME integration_scheme>
        requires std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>>
    void DrakeSolver<SystemType, N, min_steps_on, min_steps_off, max_steps_on, num_steps_solver_delay, integration_scheme>::setSetPoint(const typename SystemType::StateVec &set_point)
    {
        set_point_ = set_point;
        setStateWeights(weights_);
        setFinalWeights(final_weights_);
        sigma_delta_modulator_.reset();
    }

    template <class SystemType, int N, int min_steps_on, int min_steps_off, int max_steps_on, int num_steps_solver_delay, INTEGRATION_SCHEME integration_scheme>
        requires std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>>
    void DrakeSolver<SystemType, N, min_steps_on, min_steps_off, max_steps_on, num_steps_solver_delay, integration_scheme>::setState(const typename SystemType::StateVec &state)
    {

        Eigen::Matrix<double, SystemType::NUM_STATES, SystemType::NUM_STATES * 2 + SystemType::NUM_INPUTS> A;
        A.template block<SystemType::NUM_STATES, SystemType::NUM_STATES>(0, 0) = Eigen::Matrix<double, SystemType::NUM_STATES, SystemType::NUM_STATES>::Identity() +
                                                                                 (dt_ * system_.getA(state));
        A.template block<SystemType::NUM_STATES, SystemType::NUM_INPUTS>(0, SystemType::NUM_STATES) = dt_ * system_.getB(state);

        A.template block<SystemType::NUM_STATES, SystemType::NUM_STATES>(0, SystemType::NUM_STATES + SystemType::NUM_INPUTS) = -1 * Eigen::Matrix<double, SystemType::NUM_STATES, SystemType::NUM_STATES>::Identity();

        // [A, B, -I] * [x, u, x+] = 0

        // A*x + B*u + B*U_sd =  I*x+
        // A*x + B*u -I*x+  = - B*U_sd
        // [A, B, -I] * [x, u, x+] = - B * [u_sd]

        Eigen::Matrix<double, SystemType::NUM_INPUTS, N> sigma_delta_inputs;
        Eigen::Matrix<double, SystemType::NUM_BIN_INPUTS, N> limits;
        sigma_delta_inputs.setZero();
        if (mi_informed_ >= 1)
        {
            sigma_delta_modulator_.template GetFutureFirings<N>(sigma_delta_inputs.template block<SystemType::NUM_BIN_INPUTS, N>(SystemType::NUM_CONT_INPUTS, 0), limits, dt_);
        }
        for (unsigned int n = 0; n < N; n++)
        {

            dynamic_cons_[n]->UpdateCoefficients(A, -1 * dt_ * system_.getB(state) * sigma_delta_inputs.col(n));
            // std::cout << "SDM input at step " << n << ": " << sigma_delta_inputs.col(n).transpose() << std::endl;

            for (unsigned int i = SystemType::NUM_CONT_INPUTS; i < SystemType::NUM_INPUTS; i++)
            {
                if (mi_informed_ >= 3)
                {

                    if (sigma_delta_inputs(i, n) > 0.5)
                    {
                        limits(i - SystemType::NUM_CONT_INPUTS, n) = 0;
                    }
                }
                if (mi_informed_ >= 2)
                {
                    input_constraints_[i][n]->UpdateUpperBound(Eigen::Vector<double, 1>(limits(i - SystemType::NUM_CONT_INPUTS, n)));
                }
            }
        }

        for (unsigned int i = 0; i < SystemType::NUM_STATES; i++)
        {
            state_constraints_[i][0]->UpdateLowerBound(state.template segment<1>(i));
            state_constraints_[i][0]->UpdateUpperBound(state.template segment<1>(i));
        }
    }

    template <class SystemType, int N, int min_steps_on, int min_steps_off, int max_steps_on, int num_steps_solver_delay, INTEGRATION_SCHEME integration_scheme>
        requires std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>>
    void DrakeSolver<SystemType, N, min_steps_on, min_steps_off, max_steps_on, num_steps_solver_delay, integration_scheme>::setInputHistory(
        const Eigen::Matrix<double, SystemType::NUM_INPUTS, SolverBase::history_depth> &bin_input_hist)
    {
        (void)bin_input_hist;
        // TODO: not needed if you don't model dwell-time here; otherwise add constraints
    }

    template <class SystemType, int N, int min_steps_on, int min_steps_off, int max_steps_on, int num_steps_solver_delay, INTEGRATION_SCHEME integration_scheme>
        requires std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>>
    void DrakeSolver<SystemType, N, min_steps_on, min_steps_off, max_steps_on, num_steps_solver_delay, integration_scheme>::setNextInputs(
        const Eigen::Matrix<double, SystemType::NUM_INPUTS, num_steps_solver_delay> &next_inputs)
    {
        (void)next_inputs;
        // TODO: if you compensate solver delay by pre-applying inputs, add appropriate constraints
    }

    template <class SystemType, int N, int min_steps_on, int min_steps_off, int max_steps_on, int num_steps_solver_delay, INTEGRATION_SCHEME integration_scheme>
        requires std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>>
    void DrakeSolver<SystemType, N, min_steps_on, min_steps_off, max_steps_on, num_steps_solver_delay, integration_scheme>::setSolverTimeLimit(double max_seconds)
    {
        (void)max_seconds;
        // TODO: set in solver_opts_ for your chosen solver (OSQP/Clarabel/Gurobi/etc.)
    }

    template <class SystemType, int N, int min_steps_on, int min_steps_off, int max_steps_on, int num_steps_solver_delay, INTEGRATION_SCHEME integration_scheme>
        requires std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>>
    unsigned int DrakeSolver<SystemType, N, min_steps_on, min_steps_off, max_steps_on, num_steps_solver_delay, integration_scheme>::getStepsToCompensateControllerDelay()
    {
        return 0;
    }

    // ---------- solve ----------
    template <class SystemType, int N, int min_steps_on, int min_steps_off, int max_steps_on, int num_steps_solver_delay, INTEGRATION_SCHEME integration_scheme>
        requires std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>>
    SOLVER_RETURN DrakeSolver<SystemType, N, min_steps_on, min_steps_off, max_steps_on, num_steps_solver_delay, integration_scheme>::solve(
        const Eigen::Matrix<double, SystemType::NUM_INPUTS, N, Eigen::RowMajor> &last_open_loop_input,
        const Eigen::Matrix<double, SystemType::NUM_STATES, N + 1, Eigen::RowMajor> &last_open_loop_state,
        Eigen::Matrix<double, SystemType::NUM_INPUTS, N, Eigen::RowMajor> &open_loop_input,
        Eigen::Matrix<double, SystemType::NUM_STATES, N + 1, Eigen::RowMajor> &open_loop_state) const
    {
        for (int k = 0; k <= N; ++k)
            const_cast<drake::solvers::MathematicalProgram &>(prog_).SetInitialGuess(states_.col(k), last_open_loop_state.col(k));
        for (int k = 0; k < N; ++k)
            const_cast<drake::solvers::MathematicalProgram &>(prog_).SetInitialGuess(inputs_.col(k), last_open_loop_input.col(k));

        const auto result = solver_.Solve(prog_);

        if (!result.is_success())
        {
            // std::cout << result.get_solver_details<drake::solvers::ClpSolver>().status << std::endl
            //           << prog_.to_string() << std::endl;

            return SOLVER_RETURN::NO_SOLUTION;
        }

        // TODO: extract x, u into open_loop_state/input
        for (int k = 0; k <= N; ++k)
            open_loop_state.col(k) = result.GetSolution(states_.col(k));
        for (int k = 0; k < N; ++k)
            open_loop_input.col(k) = result.GetSolution(inputs_.col(k));

        open_loop_input.template block<SystemType::NUM_BIN_INPUTS, 1>(SystemType::NUM_CONT_INPUTS, 0) = const_cast<LimetingSigmaDeltaModulators<SystemType::NUM_BIN_INPUTS> &>(sigma_delta_modulator_).modulate_continuous_force(result.GetSolution(inputs_.template block<SystemType::NUM_BIN_INPUTS, 1>(SystemType::NUM_CONT_INPUTS, 0)), controller_dt_);

        return SOLVER_RETURN::OPTIMAL;
    }

} // namespace mimpc
