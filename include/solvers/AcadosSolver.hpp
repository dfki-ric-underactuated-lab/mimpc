#pragma once

#include <acados_c/ocp_qp_interface.h>
#include <array>
#include <type_traits>
#include <concepts>
#include <iostream>

#include "SimpleSigmaDeltaModulator.hpp"
#include "LimetingSigmaDeltaModulator.hpp"

#include "../Solver.hpp"

namespace mimpc
{

    /**
     * Provides a solver based on the open source SCIP Solver (https://scipopt.org/)
     */
    template <class SystemType, int N, int min_steps_on, int min_steps_off, int max_steps_on, int num_steps_solver_delay, INTEGRATION_SCHEME integration_scheme>
        requires std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>>
    class AcadosSolver
        : public Solver<SystemType, N, min_steps_on, min_steps_off, max_steps_on, num_steps_solver_delay, integration_scheme>
    {
    public:
        using SolverBase = Solver<SystemType, N, min_steps_on, min_steps_off, max_steps_on, num_steps_solver_delay, integration_scheme>;

    private:
        struct
        {
            typedef Eigen::Matrix<double, SystemType::NUM_STATES, SystemType::NUM_STATES, Eigen::ColMajor> QMatrixT;
            typedef Eigen::Matrix<double, SystemType::NUM_INPUTS, SystemType::NUM_INPUTS, Eigen::ColMajor> RMatrixT;
            typedef Eigen::Matrix<double, SystemType::NUM_STATES, SystemType::NUM_STATES, Eigen::ColMajor> AMatrixT;
            typedef Eigen::Matrix<double, SystemType::NUM_STATES, SystemType::NUM_INPUTS, Eigen::ColMajor> BMatrixT;
            typedef Eigen::Matrix<double, SystemType::NUM_STATES, 1, Eigen::ColMajor> stateVecT;
            typedef Eigen::Matrix<double, SystemType::NUM_INPUTS, 1, Eigen::ColMajor> inputVecT;

            /*
             * data fields
             */
            AMatrixT A_;
            BMatrixT B_;

            inputVecT lbu_;
            inputVecT ubu_;
            inputVecT ubuf_;
            inputVecT lbuf_;
            stateVecT lbx_;
            stateVecT ubx_;
            stateVecT ubxf_;
            stateVecT lbxf_;

            QMatrixT Q_;
            RMatrixT R_;
            QMatrixT Qf_;
            stateVecT q_;
            stateVecT qf_;

            AMatrixT stateIdentity_;
            RMatrixT inputIdentity_;
        } l2_cost_members_;

        struct
        {

            typedef Eigen::Matrix<double, SystemType::NUM_STATES, SystemType::NUM_STATES, Eigen::ColMajor> AMatrixT;
            typedef Eigen::Matrix<double, SystemType::NUM_STATES, SystemType::NUM_STATES + SystemType::NUM_INPUTS * 2, Eigen::ColMajor> BMatrixT;
            typedef Eigen::Matrix<double, SystemType::NUM_STATES, 1, Eigen::ColMajor> stateVecT;
            typedef Eigen::Matrix<double, SystemType::NUM_STATES + SystemType::NUM_INPUTS * 2, 1, Eigen::ColMajor> inputVecT;
            typedef Eigen::Matrix<double, SystemType::NUM_STATES, 1, Eigen::ColMajor> stateBVecT;
            typedef Eigen::Matrix<double, SystemType::NUM_INPUTS, 1, Eigen::ColMajor> inputBVecT;
            typedef Eigen::Matrix<double, SystemType::NUM_STATES * 2 + SystemType::NUM_INPUTS * 2, 1, Eigen::ColMajor> gVecT;
            typedef Eigen::Matrix<double, SystemType::NUM_STATES * 2 + SystemType::NUM_INPUTS * 2, SystemType::NUM_STATES, Eigen::ColMajor> CMatrixT;
            typedef Eigen::Matrix<double, SystemType::NUM_STATES * 2, SystemType::NUM_STATES, Eigen::ColMajor> DfMatrixT;
            typedef Eigen::Matrix<double, SystemType::NUM_STATES * 2, SystemType::NUM_STATES, Eigen::ColMajor> CfMatrixT;
            typedef Eigen::Matrix<double, SystemType::NUM_STATES * 2 + SystemType::NUM_INPUTS * 2, SystemType::NUM_STATES + SystemType::NUM_INPUTS * 2, Eigen::ColMajor> DMatrixT;
            Eigen::Matrix<double, SystemType::NUM_STATES + 2 * SystemType::NUM_INPUTS,
                          SystemType::NUM_STATES + 2 * SystemType::NUM_INPUTS>
                R_;
            Eigen::Matrix<double, SystemType::NUM_STATES, SystemType::NUM_STATES> Rf_;
            /*
             * data fields
             */
            AMatrixT A_;
            BMatrixT B_;

            inputBVecT lbu_;
            inputBVecT ubu_;
            inputBVecT ubuf_;
            inputBVecT lbuf_;
            stateBVecT lbx_;
            stateBVecT ubx_;
            stateBVecT ubxf_;
            stateBVecT lbxf_;

            gVecT ug_;
            gVecT lg_;
            gVecT ugf_;
            gVecT lgf_;

            DMatrixT D_;
            CMatrixT C_;
            DfMatrixT Df_;
            CfMatrixT Cf_;

            stateVecT q_;
            inputVecT r_;
            stateVecT rf_;

        } l1_cost_members_;

        /*
         * acados stuff
         */
        ocp_qp_solver_plan_t solver_plan_;
        ocp_qp_dims *ocp_dims_;

        ocp_qp_xcond_solver_config *solver_config_;
        ocp_qp_xcond_solver_dims *solver_dims_;
        void *solver_opts_;
        ocp_qp_solver *qp_solver_;

        ocp_qp_in *qp_in_;
        ocp_qp_out *qp_out_;

        int condensing_N_;
        std::string hpipm_mode_;
        int warm_start_;

        /*
         * generel stuff
         */

        COST_TYPE cost_type_;
        bool compensate_controller_delay_;

        const SystemType &system_;
        double system_dt_;
        const double controller_dt_;

        LimetingSigmaDeltaModulators<SystemType::NUM_BIN_INPUTS> sigma_delta_modulator_;

        Eigen::Vector<double, SystemType::NUM_STATES> state_;
        Eigen::Vector<double, SystemType::NUM_STATES> state_cost_weights_;
        Eigen::Vector<double, SystemType::NUM_STATES> final_state_cost_weights_;
        Eigen::Vector<double, SystemType::NUM_INPUTS> input_cost_weights_;
        Eigen::Vector<double, SystemType::NUM_STATES> set_point_;

        void setCost(COST_TYPE cost_type) override;

    public:
        AcadosSolver(const typename SystemType::StateVec &state_weights,
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
                     LimetingSigmaDeltaModulators<SystemType::NUM_BIN_INPUTS> &sdm);

        virtual ~AcadosSolver();

        AcadosSolver(const AcadosSolver &); // disable copy constructor

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
    };
};

#include "AcadosSolver.tpp"