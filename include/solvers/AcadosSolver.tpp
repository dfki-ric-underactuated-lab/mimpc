#pragma once

#include "AcadosSolver.hpp"
#include "acados_c/dense_qp_interface.h"
#include "acados/ocp_qp/ocp_qp_full_condensing.h"
#include "acados/utils/print.h"
#include <blasfeo.h>

namespace mimpc
{
    template <class SystemType, int N, int min_steps_on, int min_steps_off, int max_steps_on, int num_steps_solver_delay, INTEGRATION_SCHEME integration_scheme>
        requires std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>>
    AcadosSolver<SystemType, N, min_steps_on, min_steps_off, max_steps_on, num_steps_solver_delay, integration_scheme>::AcadosSolver(
        const typename SystemType::StateVec &state_weights,
        const typename SystemType::StateVec &final_weights,
        const typename SystemType::InputVec &input_weights,
        const typename SystemType::StateVec &set_point,
        COST_TYPE cost_type,
        const SystemType &system,
        double system_dt,
        ocp_qp_solver_t solver,
        int condensing_N,
        std::string hpipm_mode,
        int warm_start) : cost_type_(cost_type),
                          system_(system),
                          system_dt_(system_dt),
                          state_cost_weights_(state_weights),
                          final_state_cost_weights_(final_weights),
                          input_cost_weights_(input_weights),
                          set_point_(set_point),
                          condensing_N_(condensing_N),
                          hpipm_mode_(hpipm_mode),
                          warm_start_(warm_start)

    {
        static_assert(integration_scheme == FORWARD_EULER);
        // get matrices
        auto A = system_.getA(state_);
        auto B = system_.getB(state_);

        A_ = Eigen::Matrix<double, SystemType::NUM_STATES, SystemType::NUM_STATES>::Identity() +
             (system_dt_ * A);
        B_ = system_dt_ * B;
        Q_ = state_cost_weights_.asDiagonal();
        R_ = input_cost_weights_.asDiagonal();
        Qf_ = final_state_cost_weights_.asDiagonal();

        q_ = -(Q_ * set_point);
        qf_ = -(Qf_ * set_point);

        lbu_.setConstant(-ACADOS_INFTY);
        ubu_.setConstant(ACADOS_INFTY);
        lbuf_.setConstant(-ACADOS_INFTY);
        ubuf_.setConstant(ACADOS_INFTY);
        lbx_.setConstant(-ACADOS_INFTY);
        ubx_.setConstant(ACADOS_INFTY);
        lbxf_.setConstant(-ACADOS_INFTY);
        ubxf_.setConstant(ACADOS_INFTY);

        inputIdentity_.setIdentity();
        stateIdentity_.setIdentity();

        // This cheks are important to later pass the pointer to the acados calls
        assert(A_.innerStride() == 1);
        assert(A_.outerStride() == SystemType::NUM_STATES);
        assert(B_.innerStride() == 1);
        assert(B_.outerStride() == SystemType::NUM_STATES);

        assert(R_.innerStride() == 1);
        assert(R_.outerStride() == SystemType::NUM_INPUTS);
        assert(Q_.innerStride() == 1);
        assert(Q_.outerStride() == SystemType::NUM_STATES);
        assert(Qf_.innerStride() == 1);
        assert(Qf_.outerStride() == SystemType::NUM_STATES);

        // It really depends on the used cost type
        if (cost_type_ == COST_TYPE::L2Quadratic)
        {

            // Solver planned to use
            solver_plan_.qp_solver = solver;
            // Create dimensions of QP
            ocp_dims_ = ocp_qp_dims_create(N);
            solver_config_ = ocp_qp_xcond_solver_config_create(solver_plan_);
            int nx = SystemType::NUM_STATES; // This is necessary in order to not lose const qualifiers
            int nu = SystemType::NUM_INPUTS; // TODO: do they have to be in object?
            int nbu = SystemType::NUM_INPUTS;
            int nbx = SystemType::NUM_STATES;
            // inital value constraint for x
            ocp_qp_dims_set(solver_config_, ocp_dims_, 0, "nbx", &nx);
            for (int k = 0; k <= N; k++)
            {
                // State size
                ocp_qp_dims_set(solver_config_, ocp_dims_, k, "nx", &nx);
                // Input size
                ocp_qp_dims_set(solver_config_, ocp_dims_, k, "nu", &nu);
                // Input constraints
                ocp_qp_dims_set(solver_config_, ocp_dims_, k, "nbu", &nbu);
                ocp_qp_dims_set(solver_config_, ocp_dims_, k, "nbx", &nbx);
            }
            // create qp in based on the dimensions
            qp_in_ = ocp_qp_in_create(ocp_dims_);
            for (unsigned int i = SystemType::NUM_CONT_INPUTS; i < SystemType::NUM_INPUTS; i++)
            {
                lbu_[i] = 0;
                ubu_[i] = 1;
            }

            // set the values that won't be changed
            for (int k = 0; k < N; k++)
            {
                ocp_qp_in_set(solver_config_, qp_in_, k, const_cast<char *>("A"), A_.data());
                ocp_qp_in_set(solver_config_, qp_in_, k, const_cast<char *>("B"), B_.data());
                ocp_qp_in_set(solver_config_, qp_in_, k, const_cast<char *>("R"), R_.data());
                ocp_qp_in_set(solver_config_, qp_in_, k, const_cast<char *>("Q"), Q_.data());
                ocp_qp_in_set(solver_config_, qp_in_, k, const_cast<char *>("Jbu"), inputIdentity_.data());
                ocp_qp_in_set(solver_config_, qp_in_, k, const_cast<char *>("Jbx"), stateIdentity_.data());
                ocp_qp_in_set(solver_config_, qp_in_, k, const_cast<char *>("lbu"), lbu_.data());
                ocp_qp_in_set(solver_config_, qp_in_, k, const_cast<char *>("ubu"), ubu_.data());
                ocp_qp_in_set(solver_config_, qp_in_, k, const_cast<char *>("lbx"), lbx_.data());
                ocp_qp_in_set(solver_config_, qp_in_, k, const_cast<char *>("ubx"), ubx_.data());
                ocp_qp_in_set(solver_config_, qp_in_, k, const_cast<char *>("q"), q_.data());
            }
            ocp_qp_in_set(solver_config_, qp_in_, N, const_cast<char *>("Q"), Qf_.data());
            ocp_qp_in_set(solver_config_, qp_in_, N, const_cast<char *>("Jbx"), stateIdentity_.data());

            ocp_qp_in_set(solver_config_, qp_in_, N, const_cast<char *>("lbx"), lbxf_.data());
            ocp_qp_in_set(solver_config_, qp_in_, N, const_cast<char *>("ubx"), ubxf_.data());
            ocp_qp_in_set(solver_config_, qp_in_, N, const_cast<char *>("q"), qf_.data());

            // create solver
            solver_dims_ = ocp_qp_xcond_solver_dims_create_from_ocp_qp_dims(solver_config_, ocp_dims_);
            solver_opts_ = ocp_qp_xcond_solver_opts_create(solver_config_, solver_dims_);

            if (solver < FULL_CONDENSING_HPIPM)
            { // this is the first solver after partials
                assert(condensing_N > 0);
                assert(condensing_N <= N);
                std::cout << "Set condensing to " << condensing_N << std::endl;
                ocp_qp_xcond_solver_opts_set(solver_config_, reinterpret_cast<ocp_qp_xcond_solver_opts *>(solver_opts_), "cond_N", &condensing_N);
            }
            if (solver == PARTIAL_CONDENSING_HPIPM || solver == FULL_CONDENSING_HPIPM)
            {
                assert(hpipm_mode == "SPEED_ABS" or hpipm_mode == "SPEED" or hpipm_mode == "BALANCE" or hpipm_mode == "ROBUST");
                std::cout << "Set hpipm mode to " << hpipm_mode.c_str() << std::endl;
                // char hpipm_mode[] = "SPEED_ABS";
                ocp_qp_xcond_solver_opts_set(solver_config_, reinterpret_cast<ocp_qp_xcond_solver_opts *>(solver_opts_), "hpipm_mode", (void *)hpipm_mode.c_str());
            }

            ocp_qp_xcond_solver_opts_set(solver_config_, reinterpret_cast<ocp_qp_xcond_solver_opts *>(solver_opts_), "warm_start", &warm_start);

            qp_solver_ = ocp_qp_create(solver_config_, solver_dims_, solver_opts_);

            // create problem out
            qp_out_ = ocp_qp_out_create(ocp_dims_);

            // Print Dimensons
            std::cout << " ---> Original problem dimensions: " << std::endl;
            print_ocp_qp_dims(ocp_dims_);
            std::cout << " <--- " << std::endl;
        }
        else if (cost_type_ == COST_TYPE::L1Linear)
        {
            //
        }
    }

    template <class SystemType, int N, int min_steps_on, int min_steps_off, int max_steps_on, int num_steps_solver_delay, INTEGRATION_SCHEME integration_scheme>
        requires std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>>
    AcadosSolver<SystemType, N, min_steps_on, min_steps_off, max_steps_on, num_steps_solver_delay, integration_scheme>::AcadosSolver(const AcadosSolver &other):
    A_(other.A_),
    B_(other.B_),
    lbu_(other.lbu_),
    ubu_(other.ubu_),
    ubuf_(other.ubuf_),
    lbuf_(other.lbuf_),
    lbx_(other.lbx_),
    ubx_(other.ubx_),
    ubxf_(other.ubxf_),
    lbxf_(other.lbxf_),
    Q_(other.Q_),
    R_(other.R_),
    Qf_(other.Qf_),
    q_(other.q_),
    qf_(other.qf_),
    stateIdentity_(other.stateIdentity_),
    inputIdentity_(other.inputIdentity_),
    solver_plan_(other.solver_plan_),
    condensing_N_(other.condensing_N_),
    hpipm_mode_(other.hpipm_mode_),
    warm_start_(other.warm_start_),
    cost_type_(other.cost_type_),
    compensate_controller_delay_(other.compensate_controller_delay_),
    system_(other.system_),
    system_dt_(other.system_dt_),
    state_(other.state_),
    state_cost_weights_(other.state_cost_weights_),
    final_state_cost_weights_(other.final_state_cost_weights_),
    input_cost_weights_(other.input_cost_weights_),
    set_point_(other.set_point_)
    {
        // We have to recreate the acados related elements
        ocp_dims_ = ocp_qp_dims_create(N);
        d_ocp_qp_dim_copy_all(other.ocp_dims_, ocp_dims_);
        solver_config_ = ocp_qp_xcond_solver_config_create(other.solver_plan_);

        int nx = SystemType::NUM_STATES; // This is necessary in order to not lose const qualifiers
        int nu = SystemType::NUM_INPUTS; // TODO: do they have to be in object?
        int nbu = SystemType::NUM_INPUTS;
        int nbx = SystemType::NUM_STATES;
        ocp_qp_dims_set(solver_config_, ocp_dims_, 0, "nbx", &nx);
        for (int k = 0; k <= N; k++)
        {
            // State size
            ocp_qp_dims_set(solver_config_, ocp_dims_, k, "nx", &nx);
            // Input size
            ocp_qp_dims_set(solver_config_, ocp_dims_, k, "nu", &nu);
            // Input constraints
            ocp_qp_dims_set(solver_config_, ocp_dims_, k, "nbu", &nbu);
            ocp_qp_dims_set(solver_config_, ocp_dims_, k, "nbx", &nbx);
        }

        solver_dims_ = ocp_qp_xcond_solver_dims_create_from_ocp_qp_dims(solver_config_, ocp_dims_);
        solver_opts_ = ocp_qp_xcond_solver_opts_create(solver_config_, solver_dims_);

        if (solver_plan_.qp_solver < FULL_CONDENSING_HPIPM)
        { // this is the first solver after partials
            std::cout << "Set condensing to " << condensing_N_ << std::endl;
            ocp_qp_xcond_solver_opts_set(solver_config_, reinterpret_cast<ocp_qp_xcond_solver_opts *>(solver_opts_), "cond_N", &condensing_N_);
        }
        if (solver_plan_.qp_solver == PARTIAL_CONDENSING_HPIPM || solver_plan_.qp_solver == FULL_CONDENSING_HPIPM)
        {
            assert(hpipm_mode_ == "SPEED_ABS" or hpipm_mode_ == "SPEED" or hpipm_mode_ == "BALANCE" or hpipm_mode_ == "ROBUST");
            std::cout << "Set hpipm mode to " << hpipm_mode_.c_str() << std::endl;
            // char hpipm_mode_[] = "SPEED_ABS";
            ocp_qp_xcond_solver_opts_set(solver_config_, reinterpret_cast<ocp_qp_xcond_solver_opts *>(solver_opts_), "hpipm_mode", (void *)hpipm_mode_.c_str());
        }

        ocp_qp_xcond_solver_opts_set(solver_config_, reinterpret_cast<ocp_qp_xcond_solver_opts *>(solver_opts_), "warm_start", &warm_start_);

        qp_solver_ = ocp_qp_create(solver_config_, solver_dims_, solver_opts_);
        qp_in_ = ocp_qp_in_create(other.ocp_dims_);
        d_ocp_qp_copy_all(other.qp_in_, qp_in_);
        qp_out_ = ocp_qp_out_create(other.ocp_dims_);
    }

    template <class SystemType, int N, int min_steps_on, int min_steps_off, int max_steps_on, int num_steps_solver_delay, INTEGRATION_SCHEME integration_scheme>
        requires std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>>
    void
    AcadosSolver<SystemType, N, min_steps_on, min_steps_off, max_steps_on, num_steps_solver_delay, integration_scheme>::addInputConstraintOnIndex(
        int index, double lb, double ub)
    {
        lbu_[index] = lb;
        ubu_[index] = ub;
        for (int n = 0; n < (N - 1); n++)
        {
            ocp_qp_in_set(solver_config_, qp_in_, n, const_cast<char *>("lbu"), lbu_.data());
            ocp_qp_in_set(solver_config_, qp_in_, n, const_cast<char *>("ubu"), ubu_.data());
        }
    }

    template <class SystemType, int N, int min_steps_on, int min_steps_off, int max_steps_on, int num_steps_solver_delay, INTEGRATION_SCHEME integration_scheme>
        requires std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>>
    void
    AcadosSolver<SystemType, N, min_steps_on, min_steps_off, max_steps_on, num_steps_solver_delay, integration_scheme>::addStateConstraintOnIndex(
        int index, double lb, double ub)
    {
        lbx_[index] = lb;
        ubx_[index] = ub;
        for (int n = 0; n < (N + 1); n++)
        {
            ocp_qp_in_set(solver_config_, qp_in_, n, const_cast<char *>("lbx"), lbx_.data());
            ocp_qp_in_set(solver_config_, qp_in_, n, const_cast<char *>("ubx"), ubx_.data());
        }
    }

    template <class SystemType, int N, int min_steps_on, int min_steps_off, int max_steps_on, int num_steps_solver_delay, INTEGRATION_SCHEME integration_scheme>
        requires std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>>
    void
    AcadosSolver<SystemType, N, min_steps_on, min_steps_off, max_steps_on, num_steps_solver_delay, integration_scheme>::addInputConstraintOnStep(
        int step, const typename SystemType::InputVec &lb, const typename SystemType::InputVec &ub)
    {
        lbu_ = lb;
        ubu_ = ub;
        ocp_qp_in_set(solver_config_, qp_in_, step, const_cast<char *>("lbu"), lbu_.data());
        ocp_qp_in_set(solver_config_, qp_in_, step, const_cast<char *>("ubu"), ubu_.data());
    }

    template <class SystemType, int N, int min_steps_on, int min_steps_off, int max_steps_on, int num_steps_solver_delay, INTEGRATION_SCHEME integration_scheme>
        requires std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>>
    void
    AcadosSolver<SystemType, N, min_steps_on, min_steps_off, max_steps_on, num_steps_solver_delay, integration_scheme>::addStateConstraintOnStep(
        int step, const typename SystemType::StateVec &lb, const typename SystemType::StateVec &ub)
    {
        lbx_ = lb;
        ubx_ = ub;
        ocp_qp_in_set(solver_config_, qp_in_, step, const_cast<char *>("lbx"), lbx_.data());
        ocp_qp_in_set(solver_config_, qp_in_, step, const_cast<char *>("ubx"), ubx_.data());
    }

    template <class SystemType, int N, int min_steps_on, int min_steps_off, int max_steps_on, int num_steps_solver_delay, INTEGRATION_SCHEME integration_scheme>
        requires std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>>
    void
    AcadosSolver<SystemType, N, min_steps_on, min_steps_off, max_steps_on, num_steps_solver_delay, integration_scheme>::setFinalWeights(
        const typename SystemType::StateVec &final_weights)
    {
        final_state_cost_weights_ = final_weights;
        switch (cost_type_)
        {
        case L1Linear:
            throw std::logic_error("Function not yet implemented");
            break;
        case L2Quadratic:
            Qf_ = final_weights.asDiagonal();
            ocp_qp_in_set(solver_config_, qp_in_, N, const_cast<char *>("Q"), Qf_.data());
            break;
        }
    }

    template <class SystemType, int N, int min_steps_on, int min_steps_off, int max_steps_on, int num_steps_solver_delay, INTEGRATION_SCHEME integration_scheme>
        requires std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>>
    void
    AcadosSolver<SystemType, N, min_steps_on, min_steps_off, max_steps_on, num_steps_solver_delay, integration_scheme>::setStateWeights(
        const typename SystemType::StateVec &state_weights)
    {
        state_cost_weights_ = state_weights;
        switch (cost_type_)
        {
        case L1Linear:
            throw std::logic_error("Function not yet implemented");
            break;
        case L2Quadratic:
            Q_ = state_weights.asDiagonal();
            for (unsigned int i = 0; i < N; i++)
            {
                ocp_qp_in_set(solver_config_, qp_in_, i, const_cast<char *>("Q"), Q_.data());
            }
            break;
        }
    }

    template <class SystemType, int N, int min_steps_on, int min_steps_off, int max_steps_on, int num_steps_solver_delay, INTEGRATION_SCHEME integration_scheme>
        requires std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>>
    void
    AcadosSolver<SystemType, N, min_steps_on, min_steps_off, max_steps_on, num_steps_solver_delay, integration_scheme>::setInputWeights(
        const typename SystemType::InputVec &input_weights)
    {
        input_cost_weights_ = input_weights;
        switch (cost_type_)
        {
        case L1Linear:
            throw std::logic_error("Function not yet implemented");
            break;
        case L2Quadratic:
            R_ = input_weights.asDiagonal();
            break;
        }
    }

    template <class SystemType, int N, int min_steps_on, int min_steps_off, int max_steps_on, int num_steps_solver_delay, INTEGRATION_SCHEME integration_scheme>
        requires std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>>
    void
    AcadosSolver<SystemType, N, min_steps_on, min_steps_off, max_steps_on, num_steps_solver_delay, integration_scheme>::setCost(
        [[maybe_unused]] COST_TYPE cost_type)
    {
        throw std::logic_error("Function not yet implemented");
    }

    template <class SystemType, int N, int min_steps_on, int min_steps_off, int max_steps_on, int num_steps_solver_delay, INTEGRATION_SCHEME integration_scheme>
        requires std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>>
    void
    AcadosSolver<SystemType, N, min_steps_on, min_steps_off, max_steps_on, num_steps_solver_delay, integration_scheme>::setSetPoint(
        const typename SystemType::StateVec &set_point)
    {
        set_point_ = set_point;
        switch (cost_type_)
        {
        case L1Linear:
            throw std::logic_error("Function not yet implemented");
            break;
        case L2Quadratic:
            q_ = -(Q_ * set_point);
            qf_ = -(Qf_ * set_point);
            for (unsigned int i = 0; i < N; i++)
            {
                ocp_qp_in_set(solver_config_, qp_in_, i, const_cast<char *>("q"), q_.data());
            }
            ocp_qp_in_set(solver_config_, qp_in_, N, const_cast<char *>("q"), qf_.data());
            break;
        }
    }

    template <class SystemType, int N, int min_steps_on, int min_steps_off, int max_steps_on, int num_steps_solver_delay, INTEGRATION_SCHEME integration_scheme>
        requires std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>>
    void
    AcadosSolver<SystemType, N, min_steps_on, min_steps_off, max_steps_on, num_steps_solver_delay, integration_scheme>::setState(
        const typename SystemType::StateVec &state)
    {
        // update state
        state_ = state;
        // update dynamics accordingly
        if (integration_scheme == BACKWARD_EULER)
        {
            throw std::logic_error("Function not yet implemented");
        }
        else if (integration_scheme == FORWARD_EULER)
        {
            std::function<void(unsigned int, unsigned int, double)> updateAfun = [this](unsigned int row,
                                                                                        unsigned int col, double val)
            {
                if (row == col)
                {
                    A_(row, col) = 1.0 + system_dt_ * val;
                }
                else
                {
                    A_(row, col) = system_dt_ * val;
                }
            };
            std::function<void(unsigned int, unsigned int, double)> updateBfun = [this](unsigned int row,
                                                                                        unsigned int col, double val)
            {
                B_(row, col) = system_dt_ * val;
            };

            system_.updateA(state, updateAfun);
            system_.updateB(state, updateBfun);

            for (unsigned int n = 0; n < N; n++)
            {
                ocp_qp_in_set(solver_config_, qp_in_, n, const_cast<char *>("A"), A_.data());
                ocp_qp_in_set(solver_config_, qp_in_, n, const_cast<char *>("B"), B_.data());
            }
        }
        ocp_qp_in_set(solver_config_, qp_in_, 0, const_cast<char *>("lbx"), state_.data());
        ocp_qp_in_set(solver_config_, qp_in_, 0, const_cast<char *>("ubx"), state_.data());
    }

    template <class SystemType, int N, int min_steps_on, int min_steps_off, int max_steps_on, int num_steps_solver_delay, INTEGRATION_SCHEME integration_scheme>
        requires std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>>
    void
    AcadosSolver<SystemType, N, min_steps_on, min_steps_off, max_steps_on, num_steps_solver_delay, integration_scheme>::setInputHistory(
        const Eigen::Matrix<double, SystemType::NUM_INPUTS, SolverBase::history_depth> &bin_input_hist)
    {
        (void)bin_input_hist;
        // TODO:
    }

    template <class SystemType, int N, int min_steps_on, int min_steps_off, int max_steps_on, int num_steps_solver_delay, INTEGRATION_SCHEME integration_scheme>
        requires std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>>
    void
    AcadosSolver<SystemType, N, min_steps_on, min_steps_off, max_steps_on, num_steps_solver_delay, integration_scheme>::setSolverTimeLimit(
        double max_seconds)
    {
        (void)max_seconds;
        // TODO:
    }

    template <class SystemType, int N, int min_steps_on, int min_steps_off, int max_steps_on, int num_steps_solver_delay, INTEGRATION_SCHEME integration_scheme>
        requires std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>>
    void
    AcadosSolver<SystemType, N, min_steps_on, min_steps_off, max_steps_on, num_steps_solver_delay, integration_scheme>::setNextInputs(
        const Eigen::Matrix<double, SystemType::NUM_INPUTS, num_steps_solver_delay> &next_inputs)
    {
        (void)next_inputs;
        // TODO:
    }

    template <class SystemType, int N, int min_steps_on, int min_steps_off, int max_steps_on, int num_steps_solver_delay, INTEGRATION_SCHEME integration_scheme>
        requires std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>>
    AcadosSolver<SystemType, N, min_steps_on, min_steps_off, max_steps_on, num_steps_solver_delay, integration_scheme>::~AcadosSolver()
    {
        ocp_qp_xcond_solver_dims_free(solver_dims_);
        ocp_qp_dims_free(ocp_dims_);
        ocp_qp_xcond_solver_config_free(solver_config_);
        ocp_qp_xcond_solver_opts_free(reinterpret_cast<ocp_qp_xcond_solver_opts *>(solver_opts_));
        ocp_qp_in_free(qp_in_);
        ocp_qp_out_free(qp_out_);
        ocp_qp_solver_destroy(qp_solver_);
    }

    template <class SystemType, int N, int min_steps_on, int min_steps_off, int max_steps_on, int num_steps_solver_delay, INTEGRATION_SCHEME integration_scheme>
        requires std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>>
    SOLVER_RETURN
    AcadosSolver<SystemType, N, min_steps_on, min_steps_off, max_steps_on, num_steps_solver_delay, integration_scheme>::solve(
        const typename Eigen::Matrix<double, SystemType::NUM_INPUTS, N, Eigen::RowMajor> &last_open_loop_input,
        const typename Eigen::Matrix<double, SystemType::NUM_STATES, N + 1, Eigen::RowMajor> &last_open_loop_state,
        typename Eigen::Matrix<double, SystemType::NUM_INPUTS, N, Eigen::RowMajor> &open_loop_input,
        typename Eigen::Matrix<double, SystemType::NUM_STATES, N + 1, Eigen::RowMajor> &open_loop_state) const
    {
        (void)last_open_loop_input;
        (void)last_open_loop_state;

        int acados_return = ocp_qp_solve(qp_solver_, qp_in_, qp_out_);
        SOLVER_RETURN result;
        if (acados_return == ACADOS_SUCCESS)
        {
            result = SOLVER_RETURN::OPTIMAL;
        }
        else if (acados_return == ACADOS_MAXITER || acados_return == ACADOS_TIMEOUT)
        {
            result = SOLVER_RETURN::TIME_LIMIT_WITH_SOLUTION;
        }
        else
        {
            result = SOLVER_RETURN::NO_SOLUTION;
        }

        if (acados_return == ACADOS_SUCCESS)
        {
            Eigen::Vector<double, SystemType::NUM_STATES> state;
            Eigen::Vector<double, SystemType::NUM_INPUTS> input;
            for (int k = 0; k < N; k++)
            {
                d_ocp_qp_sol_get_u(k, qp_out_, input.data());
                d_ocp_qp_sol_get_x(k, qp_out_, state.data());
                open_loop_state(Eigen::all, k) = state;
                open_loop_input(Eigen::all, k) = input;
            }
            d_ocp_qp_sol_get_x(N, qp_out_, state.data());
            open_loop_state(Eigen::all, N) = state;
        }
        else
        {
            std::cout << "acados failed: " << acados_return << std::endl;
        }
        return result;
    }

    template <class SystemType, int N, int min_steps_on, int min_steps_off, int max_steps_on, int num_steps_solver_delay, INTEGRATION_SCHEME integration_scheme>
        requires std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>>
    unsigned int
    AcadosSolver<SystemType, N, min_steps_on, min_steps_off, max_steps_on, num_steps_solver_delay, integration_scheme>::getStepsToCompensateControllerDelay()
    {
        return 0;
    }

}
