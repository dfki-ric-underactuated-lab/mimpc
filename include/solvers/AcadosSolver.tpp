#pragma once

#include "AcadosSolver.hpp"
#include "acados_c/dense_qp_interface.h"
#include "acados/ocp_qp/ocp_qp_full_condensing.h"
#include "acados/utils/print.h"
#include <blasfeo.h>
#include <cstdio> // FILE, fopen, fclose, stdout, perror

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
        int warm_start,
        double controller_dt,
        LimetingSigmaDeltaModulators<SystemType::NUM_BIN_INPUTS> &sdm) : cost_type_(cost_type),
                                                                         system_(system),
                                                                         system_dt_(system_dt),
                                                                         state_cost_weights_(state_weights),
                                                                         final_state_cost_weights_(final_weights),
                                                                         input_cost_weights_(input_weights),
                                                                         set_point_(set_point),
                                                                         condensing_N_(condensing_N),
                                                                         hpipm_mode_(hpipm_mode),
                                                                         warm_start_(warm_start),
                                                                         controller_dt_(controller_dt),
                                                                         sigma_delta_modulator_(sdm)
    {
        static_assert(integration_scheme == FORWARD_EULER);
        // It really depends on the used cost type
        // get matrices
        auto A = system_.getA(state_);
        auto B = system_.getB(state_);

        // This cheks are important to later pass the pointer to the acados calls
        assert(l2_cost_members_.A_.innerStride() == 1);
        assert(l2_cost_members_.A_.outerStride() == SystemType::NUM_STATES);
        assert(l2_cost_members_.B_.innerStride() == 1);
        assert(l2_cost_members_.B_.outerStride() == SystemType::NUM_STATES);

        assert(l2_cost_members_.R_.innerStride() == 1);
        assert(l2_cost_members_.R_.outerStride() == SystemType::NUM_INPUTS);
        assert(l2_cost_members_.Q_.innerStride() == 1);
        assert(l2_cost_members_.Q_.outerStride() == SystemType::NUM_STATES);
        assert(l2_cost_members_.Qf_.innerStride() == 1);
        assert(l2_cost_members_.Qf_.outerStride() == SystemType::NUM_STATES);

        assert(l1_cost_members_.A_.innerStride() == 1);
        assert(l1_cost_members_.A_.outerStride() == SystemType::NUM_STATES);
        // TODO:.... check all

        // Solver planned to use
        solver_plan_.qp_solver = solver;
        // Create dimensions of QP
        ocp_dims_ = ocp_qp_dims_create(N);
        solver_config_ = ocp_qp_xcond_solver_config_create(solver_plan_);

        if (cost_type_ == COST_TYPE::L2Quadratic)
        {

            l2_cost_members_.A_ = Eigen::Matrix<double, SystemType::NUM_STATES, SystemType::NUM_STATES>::Identity() +
                                  (system_dt_ * A);
            l2_cost_members_.B_ = system_dt_ * B;
            l2_cost_members_.Q_ = state_cost_weights_.asDiagonal();
            l2_cost_members_.R_ = input_cost_weights_.asDiagonal();
            l2_cost_members_.Qf_ = final_state_cost_weights_.asDiagonal();

            l2_cost_members_.q_ = -(l2_cost_members_.Q_ * set_point);
            l2_cost_members_.qf_ = -(l2_cost_members_.Qf_ * set_point);

            l2_cost_members_.lbu_.setConstant(-ACADOS_INFTY);
            l2_cost_members_.ubu_.setConstant(ACADOS_INFTY);
            l2_cost_members_.lbuf_.setConstant(-ACADOS_INFTY);
            l2_cost_members_.ubuf_.setConstant(ACADOS_INFTY);
            l2_cost_members_.lbx_.setConstant(-ACADOS_INFTY);
            l2_cost_members_.ubx_.setConstant(ACADOS_INFTY);
            l2_cost_members_.lbxf_.setConstant(-ACADOS_INFTY);
            l2_cost_members_.ubxf_.setConstant(ACADOS_INFTY);

            l2_cost_members_.inputIdentity_.setIdentity();
            l2_cost_members_.stateIdentity_.setIdentity();

            int nx = SystemType::NUM_STATES; // This is necessary in order to not lose const qualifiers
            int nu = SystemType::NUM_INPUTS; // TODO: do they have to be in object?
            int nbu = SystemType::NUM_INPUTS;
            int nbx = SystemType::NUM_STATES;
            // inital value constraint for x
            ocp_qp_dims_set(solver_config_, ocp_dims_, 0, "nbx", &nx);
            for (int k = 0; k < N; k++)
            {
                // State size
                ocp_qp_dims_set(solver_config_, ocp_dims_, k, "nx", &nx);
                // Input size
                ocp_qp_dims_set(solver_config_, ocp_dims_, k, "nu", &nu);
                // Input constraints
                ocp_qp_dims_set(solver_config_, ocp_dims_, k, "nbu", &nbu);
                ocp_qp_dims_set(solver_config_, ocp_dims_, k, "nbx", &nbx);
            }

            // State size
            ocp_qp_dims_set(solver_config_, ocp_dims_, N, "nx", &nx);
            ocp_qp_dims_set(solver_config_, ocp_dims_, N, "nbx", &nbx);
            // create qp in based on the dimensions
            qp_in_ = ocp_qp_in_create(ocp_dims_);
            for (unsigned int i = SystemType::NUM_CONT_INPUTS; i < SystemType::NUM_INPUTS; i++)
            {
                l2_cost_members_.lbu_[i] = 0;
                l2_cost_members_.ubu_[i] = 1;
            }

            // set the values that won't be changed
            for (int k = 0; k < N; k++)
            {
                ocp_qp_in_set(solver_config_, qp_in_, k, const_cast<char *>("A"), l2_cost_members_.A_.data());
                ocp_qp_in_set(solver_config_, qp_in_, k, const_cast<char *>("B"), l2_cost_members_.B_.data());
                ocp_qp_in_set(solver_config_, qp_in_, k, const_cast<char *>("R"), l2_cost_members_.R_.data());
                ocp_qp_in_set(solver_config_, qp_in_, k, const_cast<char *>("Q"), l2_cost_members_.Q_.data());
                ocp_qp_in_set(solver_config_, qp_in_, k, const_cast<char *>("Jbu"), l2_cost_members_.inputIdentity_.data());
                ocp_qp_in_set(solver_config_, qp_in_, k, const_cast<char *>("Jbx"), l2_cost_members_.stateIdentity_.data());
                ocp_qp_in_set(solver_config_, qp_in_, k, const_cast<char *>("lbu"), l2_cost_members_.lbu_.data());
                ocp_qp_in_set(solver_config_, qp_in_, k, const_cast<char *>("ubu"), l2_cost_members_.ubu_.data());
                ocp_qp_in_set(solver_config_, qp_in_, k, const_cast<char *>("lbx"), l2_cost_members_.lbx_.data());
                ocp_qp_in_set(solver_config_, qp_in_, k, const_cast<char *>("ubx"), l2_cost_members_.ubx_.data());
                ocp_qp_in_set(solver_config_, qp_in_, k, const_cast<char *>("q"), l2_cost_members_.q_.data());
            }
            ocp_qp_in_set(solver_config_, qp_in_, N, const_cast<char *>("Q"), l2_cost_members_.Qf_.data());
            ocp_qp_in_set(solver_config_, qp_in_, N, const_cast<char *>("q"), l2_cost_members_.qf_.data());

            ocp_qp_in_set(solver_config_, qp_in_, N, const_cast<char *>("Jbx"), l2_cost_members_.stateIdentity_.data());
            ocp_qp_in_set(solver_config_, qp_in_, N, const_cast<char *>("lbx"), l2_cost_members_.lbxf_.data());
            ocp_qp_in_set(solver_config_, qp_in_, N, const_cast<char *>("ubx"), l2_cost_members_.ubxf_.data());
        }
        else if (cost_type_ == COST_TYPE::L1Linear)
        {

            l1_cost_members_.A_.setZero();
            l1_cost_members_.A_.template block<SystemType::NUM_STATES, SystemType::NUM_STATES>(0, 0) = Eigen::Matrix<double, SystemType::NUM_STATES, SystemType::NUM_STATES>::Identity() +
                                                                                                       (system_dt_ * A);
            l1_cost_members_.B_.setZero();
            l1_cost_members_.B_.template block<SystemType::NUM_STATES, SystemType::NUM_INPUTS>(0, 0) = system_dt_ * B;

            l1_cost_members_.C_.setZero();
            l1_cost_members_.C_.template block<SystemType::NUM_STATES, SystemType::NUM_STATES>(0, 0) = state_weights.asDiagonal();
            l1_cost_members_.C_.template block<SystemType::NUM_STATES, SystemType::NUM_STATES>(SystemType::NUM_STATES, 0) = state_weights.asDiagonal();

            l1_cost_members_.Cf_.setZero();
            l1_cost_members_.Cf_.template block<SystemType::NUM_STATES, SystemType::NUM_STATES>(0, 0) = final_weights.asDiagonal();
            l1_cost_members_.Cf_.template block<SystemType::NUM_STATES, SystemType::NUM_STATES>(SystemType::NUM_STATES, 0) = final_weights.asDiagonal();

            l1_cost_members_.D_.setZero();
            l1_cost_members_.D_.template block<SystemType::NUM_STATES, SystemType::NUM_STATES>(0, SystemType::NUM_INPUTS).setIdentity();
            l1_cost_members_.D_.template block<SystemType::NUM_STATES, SystemType::NUM_STATES>(SystemType::NUM_STATES, SystemType::NUM_INPUTS).diagonal().setConstant(-1);
            l1_cost_members_.D_.template block<SystemType::NUM_INPUTS, SystemType::NUM_INPUTS>(2 * SystemType::NUM_STATES, 0) = input_weights.asDiagonal();
            l1_cost_members_.D_.template block<SystemType::NUM_INPUTS, SystemType::NUM_INPUTS>(2 * SystemType::NUM_STATES + SystemType::NUM_INPUTS, 0) = input_weights.asDiagonal();
            l1_cost_members_.D_.template block<SystemType::NUM_INPUTS, SystemType::NUM_INPUTS>(2 * SystemType::NUM_STATES, SystemType::NUM_STATES + SystemType::NUM_INPUTS).setIdentity();
            l1_cost_members_.D_.template block<SystemType::NUM_INPUTS, SystemType::NUM_INPUTS>(2 * SystemType::NUM_STATES + SystemType::NUM_INPUTS, SystemType::NUM_STATES + SystemType::NUM_INPUTS).diagonal().setConstant(-1);

            l1_cost_members_.Df_.setZero();
            l1_cost_members_.Df_.template block<SystemType::NUM_STATES, SystemType::NUM_STATES>(0, 0).setIdentity();
            l1_cost_members_.Df_.template block<SystemType::NUM_STATES, SystemType::NUM_STATES>(SystemType::NUM_STATES, 0).diagonal().setConstant(-1);

            l1_cost_members_.lbu_.setConstant(-ACADOS_INFTY);
            l1_cost_members_.ubu_.setConstant(ACADOS_INFTY);
            l1_cost_members_.lbuf_.setConstant(-ACADOS_INFTY);
            l1_cost_members_.ubuf_.setConstant(ACADOS_INFTY);
            l1_cost_members_.lbx_.setConstant(-ACADOS_INFTY);
            l1_cost_members_.ubx_.setConstant(ACADOS_INFTY);
            l1_cost_members_.lbxf_.setConstant(-ACADOS_INFTY);
            l1_cost_members_.ubxf_.setConstant(ACADOS_INFTY);

            l1_cost_members_.lg_.setConstant(-ACADOS_INFTY);
            l1_cost_members_.ug_.setConstant(ACADOS_INFTY);
            l1_cost_members_.lgf_.setConstant(-ACADOS_INFTY);
            l1_cost_members_.ugf_.setConstant(ACADOS_INFTY);

            l1_cost_members_.lg_.template segment<SystemType::NUM_STATES>(0) = state_weights.asDiagonal() * set_point;
            l1_cost_members_.lg_.template segment<SystemType::NUM_INPUTS>(2 * SystemType::NUM_STATES).setZero();

            l1_cost_members_.lgf_.template segment<SystemType::NUM_STATES>(0) = final_weights.asDiagonal() * set_point;

            l1_cost_members_.ug_.template segment<SystemType::NUM_INPUTS>(2 * SystemType::NUM_STATES + SystemType::NUM_INPUTS).setZero();
            l1_cost_members_.ug_.template segment<SystemType::NUM_STATES>(SystemType::NUM_STATES) = state_weights.asDiagonal() * set_point;

            l1_cost_members_.ugf_.template segment<SystemType::NUM_STATES>(SystemType::NUM_STATES) = final_weights.asDiagonal() * set_point;

            l1_cost_members_.q_.setZero();
            l1_cost_members_.r_.template segment<SystemType::NUM_STATES + SystemType::NUM_INPUTS>(SystemType::NUM_INPUTS).setOnes();
            l1_cost_members_.rf_.setOnes();

            // ---- tiny Tikhonov regularization for L1 to make the QP strictly convex ----
            constexpr double eps_reg_slack = 1e-4; // 1e-8..1e-6 is typical
            constexpr double eps_reg_u = 1e-4;     // <<< was 0.0; make it > 0

            l1_cost_members_.R_.setZero();

            // indices in u = [u, s_x, t_u]
            constexpr int NUu = SystemType::NUM_INPUTS;
            constexpr int NUsx = SystemType::NUM_STATES;
            constexpr int NUtu = SystemType::NUM_INPUTS;

            // put small eps on s_x block
            // reg on real inputs u
            l1_cost_members_.R_
                .template block<NUu, NUu>(0, 0)
                .diagonal()
                .setConstant(eps_reg_u);

            // reg on s_x
            l1_cost_members_.R_
                .template block<NUsx, NUsx>(NUu, NUu)
                .diagonal()
                .setConstant(eps_reg_slack);

            // reg on t_u
            l1_cost_members_.R_
                .template block<NUtu, NUtu>(NUu + NUsx, NUu + NUsx)
                .diagonal()
                .setConstant(eps_reg_slack);

            // terminal stage: only s_x(N)
            l1_cost_members_.Rf_.setZero();
            l1_cost_members_.Rf_.diagonal().setConstant(eps_reg_slack);

            Eigen::Vector<int, SystemType::NUM_STATES> idxbx;
            for (unsigned int i = 0; i < SystemType::NUM_STATES; i++)
            {
                idxbx(i) = i;
            }
            Eigen::Vector<int, SystemType::NUM_INPUTS> idxbu;
            for (unsigned int i = 0; i < SystemType::NUM_INPUTS; i++)
            {
                idxbu(i) = i;
            }

            int nx = SystemType::NUM_STATES;                              // This is necessary in order to not lose const qualifiers
            int nu = SystemType::NUM_STATES + SystemType::NUM_INPUTS * 2; // TODO: do they have to be in object?
            int nuf = SystemType::NUM_STATES;
            int nbu = SystemType::NUM_INPUTS;
            int nbx = SystemType::NUM_STATES;

            int ng = 2 * (SystemType::NUM_STATES + SystemType::NUM_INPUTS);
            int ngf = 2 * (SystemType::NUM_STATES);

            for (int k = 0; k < N; k++)
            {
                // State size
                ocp_qp_dims_set(solver_config_, ocp_dims_, k, "nx", &nx);
                // Input size
                ocp_qp_dims_set(solver_config_, ocp_dims_, k, "nu", &nu);
                // Input constraints
                ocp_qp_dims_set(solver_config_, ocp_dims_, k, "nbu", &nbu);
                ocp_qp_dims_set(solver_config_, ocp_dims_, k, "nbx", &nbx);
                // Further constraints
                ocp_qp_dims_set(solver_config_, ocp_dims_, k, "ng", &ng);
            }

            // State size
            ocp_qp_dims_set(solver_config_, ocp_dims_, N, "nx", &nx);
            ocp_qp_dims_set(solver_config_, ocp_dims_, N, "nu", &nuf);
            ocp_qp_dims_set(solver_config_, ocp_dims_, N, "nbx", &nbx);
            ocp_qp_dims_set(solver_config_, ocp_dims_, N, "ng", &ngf);
            // create qp in based on the dimensions
            qp_in_ = ocp_qp_in_create(ocp_dims_);
            for (unsigned int i = SystemType::NUM_CONT_INPUTS; i < SystemType::NUM_INPUTS; i++)
            {
                l1_cost_members_.lbu_[i] = 0;
                l1_cost_members_.ubu_[i] = 1;
            }

            // set the values that won't be changed
            for (int k = 0; k < N; k++)
            {
                ocp_qp_in_set(solver_config_, qp_in_, k, const_cast<char *>("A"), l1_cost_members_.A_.data());
                ocp_qp_in_set(solver_config_, qp_in_, k, const_cast<char *>("B"), l1_cost_members_.B_.data());
                ocp_qp_in_set(solver_config_, qp_in_, k, const_cast<char *>("D"), l1_cost_members_.D_.data());
                ocp_qp_in_set(solver_config_, qp_in_, k, const_cast<char *>("C"), l1_cost_members_.C_.data());
                ocp_qp_in_set(solver_config_, qp_in_, k, const_cast<char *>("idxbu"), idxbu.data());
                ocp_qp_in_set(solver_config_, qp_in_, k, const_cast<char *>("idxbx"), idxbx.data());
                ocp_qp_in_set(solver_config_, qp_in_, k, const_cast<char *>("lbu"), l1_cost_members_.lbu_.data());
                ocp_qp_in_set(solver_config_, qp_in_, k, const_cast<char *>("ubu"), l1_cost_members_.ubu_.data());
                ocp_qp_in_set(solver_config_, qp_in_, k, const_cast<char *>("lbx"), l1_cost_members_.lbx_.data());
                ocp_qp_in_set(solver_config_, qp_in_, k, const_cast<char *>("ubx"), l1_cost_members_.ubx_.data());
                ocp_qp_in_set(solver_config_, qp_in_, k, const_cast<char *>("lg"), l1_cost_members_.lg_.data());
                ocp_qp_in_set(solver_config_, qp_in_, k, const_cast<char *>("ug"), l1_cost_members_.ug_.data());
                ocp_qp_in_set(solver_config_, qp_in_, k, const_cast<char *>("q"), l1_cost_members_.q_.data());
                ocp_qp_in_set(solver_config_, qp_in_, k, const_cast<char *>("r"), l1_cost_members_.r_.data());
                ocp_qp_in_set(solver_config_, qp_in_, k, const_cast<char *>("R"), l1_cost_members_.R_.data());
            }
            ocp_qp_in_set(solver_config_, qp_in_, N, const_cast<char *>("D"), l1_cost_members_.Df_.data());
            ocp_qp_in_set(solver_config_, qp_in_, N, const_cast<char *>("C"), l1_cost_members_.Cf_.data());
            ocp_qp_in_set(solver_config_, qp_in_, N, const_cast<char *>("q"), l1_cost_members_.q_.data());
            ocp_qp_in_set(solver_config_, qp_in_, N, const_cast<char *>("r"), l1_cost_members_.rf_.data());

            ocp_qp_in_set(solver_config_, qp_in_, N, const_cast<char *>("idxbx"), idxbx.data());
            ocp_qp_in_set(solver_config_, qp_in_, N, const_cast<char *>("lbx"), l1_cost_members_.lbxf_.data());
            ocp_qp_in_set(solver_config_, qp_in_, N, const_cast<char *>("ubx"), l1_cost_members_.ubxf_.data());

            ocp_qp_in_set(solver_config_, qp_in_, N, const_cast<char *>("lg"), l1_cost_members_.lgf_.data());
            ocp_qp_in_set(solver_config_, qp_in_, N, const_cast<char *>("ug"), l1_cost_members_.ugf_.data());
            ocp_qp_in_set(solver_config_, qp_in_, N, const_cast<char *>("R"), l1_cost_members_.Rf_.data());
        }

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

    template <class SystemType, int N, int min_steps_on, int min_steps_off, int max_steps_on, int num_steps_solver_delay, INTEGRATION_SCHEME integration_scheme>
        requires std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>>
    AcadosSolver<SystemType, N, min_steps_on, min_steps_off, max_steps_on, num_steps_solver_delay, integration_scheme>::AcadosSolver(const AcadosSolver &other) : l2_cost_members_(other.l2_cost_members_),
                                                                                                                                                                  l1_cost_members_(other.l1_cost_members_),
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

        switch (cost_type_)
        {
        case L2Quadratic:
        {
            int nx = SystemType::NUM_STATES; // This is necessary in order to not lose const qualifiers
            int nu = SystemType::NUM_INPUTS; // TODO: do they have to be in object?
            int nbu = SystemType::NUM_INPUTS;
            int nbx = SystemType::NUM_STATES;
            ocp_qp_dims_set(solver_config_, ocp_dims_, 0, "nbx", &nx);
            for (int k = 0; k < N; k++)
            {
                // State size
                ocp_qp_dims_set(solver_config_, ocp_dims_, k, "nx", &nx);
                // Input size
                ocp_qp_dims_set(solver_config_, ocp_dims_, k, "nu", &nu);
                // Input constraints
                ocp_qp_dims_set(solver_config_, ocp_dims_, k, "nbu", &nbu);
                ocp_qp_dims_set(solver_config_, ocp_dims_, k, "nbx", &nbx);
            }
            ocp_qp_dims_set(solver_config_, ocp_dims_, N, "nbx", &nbx);
            ocp_qp_dims_set(solver_config_, ocp_dims_, N, "nx", &nx);
        }
        break;
        case L1Linear:
        {
            int nx = SystemType::NUM_STATES;                              // This is necessary in order to not lose const qualifiers
            int nu = SystemType::NUM_STATES + SystemType::NUM_INPUTS * 2; // TODO: do they have to be in object?
            int nuf = SystemType::NUM_STATES;
            int nbu = SystemType::NUM_INPUTS;
            int nbx = SystemType::NUM_STATES;

            int ng = 2 * (SystemType::NUM_STATES + SystemType::NUM_INPUTS);
            int ngf = 2 * (SystemType::NUM_STATES);

            // inital value constraint for x
            ocp_qp_dims_set(solver_config_, ocp_dims_, 0, "nbx", &nbx);
            for (int k = 0; k < N; k++)
            {
                // State size
                ocp_qp_dims_set(solver_config_, ocp_dims_, k, "nx", &nx);
                // Input size
                ocp_qp_dims_set(solver_config_, ocp_dims_, k, "nu", &nu);
                // Input constraints
                ocp_qp_dims_set(solver_config_, ocp_dims_, k, "nbu", &nbu);
                ocp_qp_dims_set(solver_config_, ocp_dims_, k, "nbx", &nbx);
                // Further constraints
                ocp_qp_dims_set(solver_config_, ocp_dims_, k, "ng", &ng);
            }

            // State size
            ocp_qp_dims_set(solver_config_, ocp_dims_, N, "nx", &nx);
            ocp_qp_dims_set(solver_config_, ocp_dims_, N, "nu", &nuf);
            ocp_qp_dims_set(solver_config_, ocp_dims_, N, "nbx", &nbx);
            ocp_qp_dims_set(solver_config_, ocp_dims_, N, "ng", &ngf);
        }
        break;
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

        // TODO: diff weights
    }

    template <class SystemType, int N, int min_steps_on, int min_steps_off, int max_steps_on, int num_steps_solver_delay, INTEGRATION_SCHEME integration_scheme>
        requires std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>>
    void
    AcadosSolver<SystemType, N, min_steps_on, min_steps_off, max_steps_on, num_steps_solver_delay, integration_scheme>::addInputConstraintOnIndex(
        int index, double lb, double ub)
    {
        switch (cost_type_)
        {
        case L2Quadratic:
            l2_cost_members_.lbu_[index] = lb;
            l2_cost_members_.ubu_[index] = ub;
            for (int n = 0; n < (N); n++)
            {
                ocp_qp_in_set(solver_config_, qp_in_, n, const_cast<char *>("lbu"), l2_cost_members_.lbu_.data());
                ocp_qp_in_set(solver_config_, qp_in_, n, const_cast<char *>("ubu"), l2_cost_members_.ubu_.data());
            }
            break;
        case L1Linear:
            l1_cost_members_.lbu_[index] = lb;
            l1_cost_members_.ubu_[index] = ub;
            for (int n = 0; n < (N); n++)
            {
                ocp_qp_in_set(solver_config_, qp_in_, n, const_cast<char *>("lbu"), l1_cost_members_.lbu_.data());
                ocp_qp_in_set(solver_config_, qp_in_, n, const_cast<char *>("ubu"), l1_cost_members_.ubu_.data());
            }
            break;
        }
    }

    template <class SystemType, int N, int min_steps_on, int min_steps_off, int max_steps_on, int num_steps_solver_delay, INTEGRATION_SCHEME integration_scheme>
        requires std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>>
    void
    AcadosSolver<SystemType, N, min_steps_on, min_steps_off, max_steps_on, num_steps_solver_delay, integration_scheme>::addStateConstraintOnIndex(
        int index, double lb, double ub)
    {
        switch (cost_type_)
        {
        case L2Quadratic:
            l2_cost_members_.lbx_[index] = lb;
            l2_cost_members_.ubx_[index] = ub;
            l2_cost_members_.lbxf_[index] = lb;
            l2_cost_members_.ubxf_[index] = ub;

            for (int n = 0; n < (N); n++)
            {
                ocp_qp_in_set(solver_config_, qp_in_, n, const_cast<char *>("lbx"), l2_cost_members_.lbx_.data());
                ocp_qp_in_set(solver_config_, qp_in_, n, const_cast<char *>("ubx"), l2_cost_members_.ubx_.data());
            }
            ocp_qp_in_set(solver_config_, qp_in_, N, const_cast<char *>("lbx"), l2_cost_members_.lbx_.data());
            ocp_qp_in_set(solver_config_, qp_in_, N, const_cast<char *>("ubx"), l2_cost_members_.ubx_.data());
            break;
        case L1Linear:
            l1_cost_members_.lbx_[index] = lb;
            l1_cost_members_.ubx_[index] = ub;
            l1_cost_members_.lbxf_[index] = lb;
            l1_cost_members_.ubxf_[index] = ub;

            for (int n = 0; n < (N); n++)
            {
                ocp_qp_in_set(solver_config_, qp_in_, n, const_cast<char *>("lbx"), l1_cost_members_.lbx_.data());
                ocp_qp_in_set(solver_config_, qp_in_, n, const_cast<char *>("ubx"), l1_cost_members_.ubx_.data());
            }
            ocp_qp_in_set(solver_config_, qp_in_, N, const_cast<char *>("lbx"), l1_cost_members_.lbx_.data());
            ocp_qp_in_set(solver_config_, qp_in_, N, const_cast<char *>("ubx"), l1_cost_members_.ubx_.data());
            break;
        }
    }

    template <class SystemType, int N, int min_steps_on, int min_steps_off, int max_steps_on, int num_steps_solver_delay, INTEGRATION_SCHEME integration_scheme>
        requires std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>>
    void
    AcadosSolver<SystemType, N, min_steps_on, min_steps_off, max_steps_on, num_steps_solver_delay, integration_scheme>::addInputConstraintOnStep(
        int step, const typename SystemType::InputVec &lb, const typename SystemType::InputVec &ub)
    {
        switch (cost_type_)
        {
        case L2Quadratic:
            l2_cost_members_.lbu_ = lb;
            l2_cost_members_.ubu_ = ub;
            ocp_qp_in_set(solver_config_, qp_in_, step, const_cast<char *>("lbu"), l2_cost_members_.lbu_.data());
            ocp_qp_in_set(solver_config_, qp_in_, step, const_cast<char *>("ubu"), l2_cost_members_.ubu_.data());
            break;
        case L1Linear:
            l1_cost_members_.lbu_ = lb;
            l1_cost_members_.ubu_ = ub;
            ocp_qp_in_set(solver_config_, qp_in_, step, const_cast<char *>("lbu"), l1_cost_members_.lbu_.data());
            ocp_qp_in_set(solver_config_, qp_in_, step, const_cast<char *>("ubu"), l1_cost_members_.ubu_.data());
            break;
        }
    }

    template <class SystemType, int N, int min_steps_on, int min_steps_off, int max_steps_on, int num_steps_solver_delay, INTEGRATION_SCHEME integration_scheme>
        requires std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>>
    void
    AcadosSolver<SystemType, N, min_steps_on, min_steps_off, max_steps_on, num_steps_solver_delay, integration_scheme>::addStateConstraintOnStep(
        int step, const typename SystemType::StateVec &lb, const typename SystemType::StateVec &ub)
    {
        switch (cost_type_)
        {
        case L2Quadratic:
            if (step < N)
            {

                l2_cost_members_.lbx_ = lb;
                l2_cost_members_.ubx_ = ub;
                ocp_qp_in_set(solver_config_, qp_in_, step, const_cast<char *>("lbx"), l2_cost_members_.lbx_.data());
                ocp_qp_in_set(solver_config_, qp_in_, step, const_cast<char *>("ubx"), l2_cost_members_.ubx_.data());
            }
            else
            {

                l2_cost_members_.lbxf_ = lb;
                l2_cost_members_.ubxf_ = ub;
                ocp_qp_in_set(solver_config_, qp_in_, step, const_cast<char *>("lbx"), l2_cost_members_.lbxf_.data());
                ocp_qp_in_set(solver_config_, qp_in_, step, const_cast<char *>("ubx"), l2_cost_members_.ubxf_.data());
            }
            break;
        case L1Linear:
            if (step < N)
            {

                l1_cost_members_.lbx_ = lb;
                l1_cost_members_.ubx_ = ub;
                ocp_qp_in_set(solver_config_, qp_in_, step, const_cast<char *>("lbx"), l1_cost_members_.lbx_.data());
                ocp_qp_in_set(solver_config_, qp_in_, step, const_cast<char *>("ubx"), l1_cost_members_.ubx_.data());
            }
            else
            {

                l1_cost_members_.lbxf_ = lb;
                l1_cost_members_.ubxf_ = ub;
                ocp_qp_in_set(solver_config_, qp_in_, step, const_cast<char *>("lbx"), l1_cost_members_.lbxf_.data());
                ocp_qp_in_set(solver_config_, qp_in_, step, const_cast<char *>("ubx"), l1_cost_members_.ubxf_.data());
            }
            break;
        }
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
            l1_cost_members_.lgf_.template segment<SystemType::NUM_STATES>(0) = final_weights.asDiagonal() * set_point_;
            l1_cost_members_.ugf_.template segment<SystemType::NUM_STATES>(SystemType::NUM_STATES) = final_weights.asDiagonal() * set_point_;
            l1_cost_members_.Df_.template block<SystemType::NUM_STATES, SystemType::NUM_STATES>(0, 0) = final_weights.asDiagonal();
            l1_cost_members_.Df_.template block<SystemType::NUM_STATES, SystemType::NUM_STATES>(SystemType::NUM_STATES, 0) = final_weights.asDiagonal();
            ocp_qp_in_set(solver_config_, qp_in_, N, const_cast<char *>("lg"), l1_cost_members_.lgf_.data());
            ocp_qp_in_set(solver_config_, qp_in_, N, const_cast<char *>("ug"), l1_cost_members_.ugf_.data());
            ocp_qp_in_set(solver_config_, qp_in_, N, const_cast<char *>("D"), l1_cost_members_.Df_.data());
            break;
        case L2Quadratic:
            l2_cost_members_.Qf_ = final_weights.asDiagonal();
            ocp_qp_in_set(solver_config_, qp_in_, N, const_cast<char *>("Q"), l2_cost_members_.Qf_.data());
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
            l1_cost_members_.lg_.template segment<SystemType::NUM_STATES>(0) = state_cost_weights_.asDiagonal() * set_point_;
            l1_cost_members_.ug_.template segment<SystemType::NUM_STATES>(SystemType::NUM_STATES) = state_cost_weights_.asDiagonal() * set_point_;
            l1_cost_members_.C_.template block<SystemType::NUM_STATES, SystemType::NUM_STATES>(0, 0) = state_cost_weights_.asDiagonal();
            l1_cost_members_.C_.template block<SystemType::NUM_STATES, SystemType::NUM_STATES>(SystemType::NUM_STATES, 0) = state_cost_weights_.asDiagonal();
            for (unsigned int i = 0; i < N; i++)
            {
                ocp_qp_in_set(solver_config_, qp_in_, i, const_cast<char *>("lg"), l1_cost_members_.lg_.data());
                ocp_qp_in_set(solver_config_, qp_in_, i, const_cast<char *>("ug"), l1_cost_members_.ug_.data());
                ocp_qp_in_set(solver_config_, qp_in_, i, const_cast<char *>("C"), l1_cost_members_.C_.data());
            }
            break;
        case L2Quadratic:
            l2_cost_members_.Q_ = state_weights.asDiagonal();
            for (unsigned int i = 0; i < N; i++)
            {
                ocp_qp_in_set(solver_config_, qp_in_, i, const_cast<char *>("Q"), l2_cost_members_.Q_.data());
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
            l1_cost_members_.D_.template block<SystemType::NUM_INPUTS, SystemType::NUM_INPUTS>(2 * SystemType::NUM_STATES, 0) = input_weights.asDiagonal();
            l1_cost_members_.D_.template block<SystemType::NUM_INPUTS, SystemType::NUM_INPUTS>(2 * SystemType::NUM_STATES + SystemType::NUM_INPUTS, 0) = input_weights.asDiagonal();
            for (unsigned int i = 0; i < N; i++)
            {
                ocp_qp_in_set(solver_config_, qp_in_, i, const_cast<char *>("D"), l1_cost_members_.D_.data());
            }
            break;
        case L2Quadratic:
            l2_cost_members_.R_ = input_weights.asDiagonal();
            for (unsigned int i = 0; i < N; i++)
            {
                ocp_qp_in_set(solver_config_, qp_in_, i, const_cast<char *>("R"), l2_cost_members_.R_.data());
            }
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
            l1_cost_members_.lg_.template segment<SystemType::NUM_STATES>(0) = state_cost_weights_.asDiagonal() * set_point;
            l1_cost_members_.ug_.template segment<SystemType::NUM_STATES>(SystemType::NUM_STATES) = state_cost_weights_.asDiagonal() * set_point;
            l1_cost_members_.lgf_.template segment<SystemType::NUM_STATES>(0) = final_state_cost_weights_.asDiagonal() * set_point;
            l1_cost_members_.ugf_.template segment<SystemType::NUM_STATES>(SystemType::NUM_STATES) = final_state_cost_weights_.asDiagonal() * set_point;

            for (unsigned int i = 0; i < N; i++)
            {
                ocp_qp_in_set(solver_config_, qp_in_, i, const_cast<char *>("lg"), l1_cost_members_.lg_.data());
                ocp_qp_in_set(solver_config_, qp_in_, i, const_cast<char *>("ug"), l1_cost_members_.ug_.data());
            }
            ocp_qp_in_set(solver_config_, qp_in_, N, const_cast<char *>("lg"), l1_cost_members_.lgf_.data());
            ocp_qp_in_set(solver_config_, qp_in_, N, const_cast<char *>("ug"), l1_cost_members_.ugf_.data());
            break;
        case L2Quadratic:
            l2_cost_members_.q_ = -(l2_cost_members_.Q_ * set_point);
            l2_cost_members_.qf_ = -(l2_cost_members_.Qf_ * set_point);
            for (unsigned int i = 0; i < N; i++)
            {
                ocp_qp_in_set(solver_config_, qp_in_, i, const_cast<char *>("q"), l2_cost_members_.q_.data());
            }
            ocp_qp_in_set(solver_config_, qp_in_, N, const_cast<char *>("q"), l2_cost_members_.qf_.data());
            break;
        }
        sigma_delta_modulator_.reset();
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
            switch (cost_type_)
            {
            case L2Quadratic:
            {
                std::function<void(unsigned int, unsigned int, double)> updateAfun = [this](unsigned int row,
                                                                                            unsigned int col, double val)
                {
                    if (row == col)
                    {
                        l2_cost_members_.A_(row, col) = 1.0 + system_dt_ * val;
                    }
                    else
                    {
                        l2_cost_members_.A_(row, col) = system_dt_ * val;
                    }
                };
                std::function<void(unsigned int, unsigned int, double)> updateBfun = [this](unsigned int row,
                                                                                            unsigned int col, double val)
                {
                    l2_cost_members_.B_(row, col) = system_dt_ * val;
                };

                system_.updateA(state, updateAfun);
                system_.updateB(state, updateBfun);

                Eigen::Matrix<double, SystemType::NUM_INPUTS, N, Eigen::ColMajor> future_firings;
                Eigen::Matrix<double, SystemType::NUM_BIN_INPUTS, N, Eigen::ColMajor> limits;
                future_firings.setZero();
                limits.setOnes();
                sigma_delta_modulator_.template GetFutureFirings<N>(future_firings.template block<SystemType::NUM_BIN_INPUTS, N>(SystemType::NUM_CONT_INPUTS, 0), limits, system_dt_);

                Eigen::Vector<double, SystemType::NUM_INPUTS> ubu;
                ubu.template segment<SystemType::NUM_CONT_INPUTS>(0) = l2_cost_members_.ubu_.template segment<SystemType::NUM_CONT_INPUTS>(0);

                for (unsigned int n = 0; n < N; n++)
                {
                    Eigen::Vector<double, SystemType::NUM_STATES> b = l2_cost_members_.B_ * future_firings.col(n);
                    ubu.template segment<SystemType::NUM_BIN_INPUTS>(SystemType::NUM_CONT_INPUTS) = limits.col(n);

                    ocp_qp_in_set(solver_config_, qp_in_, n, const_cast<char *>("A"), l2_cost_members_.A_.data());
                    ocp_qp_in_set(solver_config_, qp_in_, n, const_cast<char *>("B"), l2_cost_members_.B_.data());
                    ocp_qp_in_set(solver_config_, qp_in_, n, const_cast<char *>("b"), b.data());
                    ocp_qp_in_set(solver_config_, qp_in_, n, const_cast<char *>("ubu"), ubu.data());
                }
            }
            break;
            case L1Linear:
            {
                std::function<void(unsigned int, unsigned int, double)> updateAfun = [this](unsigned int row,
                                                                                            unsigned int col, double val)
                {
                    assert(row < SystemType::NUM_STATES);
                    assert(col < SystemType::NUM_STATES);
                    if (row == col)
                    {
                        l1_cost_members_.A_(row, col) = 1.0 + system_dt_ * val;
                    }
                    else
                    {
                        l1_cost_members_.A_(row, col) = system_dt_ * val;
                    }
                };
                std::function<void(unsigned int, unsigned int, double)> updateBfun = [this](unsigned int row,
                                                                                            unsigned int col, double val)
                {
                    assert(row < SystemType::NUM_STATES);
                    assert(col < SystemType::NUM_INPUTS);
                    l1_cost_members_.B_(row, col) = system_dt_ * val;
                };

                system_.updateA(state, updateAfun);
                system_.updateB(state, updateBfun);

                for (unsigned int n = 0; n < N; n++)
                {
                    ocp_qp_in_set(solver_config_, qp_in_, n, const_cast<char *>("A"), l1_cost_members_.A_.data());
                    ocp_qp_in_set(solver_config_, qp_in_, n, const_cast<char *>("B"), l1_cost_members_.B_.data());
                }
            }
            break;
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
            switch (cost_type_)
            {
            case L2Quadratic:
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
            break;
            case L1Linear:
            {
                Eigen::Vector<double, 2 * SystemType::NUM_STATES> state;
                Eigen::Vector<double, SystemType::NUM_STATES + 2 * SystemType::NUM_INPUTS> input;
                for (int k = 0; k < N; k++)
                {
                    d_ocp_qp_sol_get_u(k, qp_out_, input.data());
                    d_ocp_qp_sol_get_x(k, qp_out_, state.data());
                    open_loop_state(Eigen::all, k) = state.template segment<SystemType::NUM_STATES>(0);
                    open_loop_input(Eigen::all, k) = input.template segment<SystemType::NUM_INPUTS>(0);
                }
                d_ocp_qp_sol_get_x(N, qp_out_, state.data());
                open_loop_state(Eigen::all, N) = state.template segment<SystemType::NUM_STATES>(0);
            }
            break;
            }

            open_loop_input.template block<SystemType::NUM_BIN_INPUTS, 1>(SystemType::NUM_CONT_INPUTS, 0) = const_cast<LimetingSigmaDeltaModulators<SystemType::NUM_BIN_INPUTS> &>(sigma_delta_modulator_).modulate_continuous_force(open_loop_input.template block<SystemType::NUM_BIN_INPUTS, 1>(SystemType::NUM_CONT_INPUTS, 0), controller_dt_);
        }
        else
        {
            std::cout << "acados failed: " << acados_return << std::endl;
            // print_ocp_qp_in(qp_in_);
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
