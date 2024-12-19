#pragma once

#include "SCIPSolver.hpp"
#include <fmt/format.h>

#define SCIP(x)            {SCIP_RETCODE _restat_ = x; /*lint -e{506,774}*/                                         \
                          if(_restat_ != SCIP_OKAY )                                                 \
                          {                                                                                   \
                             throw std::runtime_error(fmt::format("Error in SCIP_CALL: {}", _restat_).c_str());       \
                             }    }                                                                              \


namespace mimpc {
    template<class SystemType, int N, int min_steps_on, int min_steps_off, int max_steps_on, int num_steps_solver_delay, INTEGRATION_SCHEME integration_scheme>
    requires
    std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>>
    SCIPSolver<SystemType, N, min_steps_on, min_steps_off, max_steps_on, num_steps_solver_delay, integration_scheme>::SCIPSolver(
            const SystemType::StateVec &state_weights, const SystemType::StateVec &final_weights,
            const SystemType::InputVec &input_weights, const SystemType::StateVec &set_point, COST_TYPE cost_type,
            const SystemType &system, double system_dt):
            scip_problem_(nullptr),
            cost_type_(cost_type),
            system_(system),
            system_dt_(system_dt),
            state_cost_weights_(state_weights),
            final_state_cost_weights_(final_weights),
            input_cost_weights_(input_weights),
            set_point_(set_point) {
        static_assert(min_steps_on == 1); //This implementation is very specific to our case
        static_assert(min_steps_off == 2);
        // Create SCIP problem
        SCIP(SCIPcreate(&scip_problem_));
        SCIP(SCIPcreateProbBasic(scip_problem_, "FFMIMPC"));
        SCIP(SCIPincludeDefaultPlugins(scip_problem_));
        // Create state variables
        for (unsigned int n = 0; n < (N + 1); n++) {
            for (unsigned int k = 0; k < SystemType::NUM_STATES; k++) {
                SCIP(SCIPcreateVarBasic(scip_problem_, &(scip_state_vars_[k][n]),
                                        fmt::format("state[{},{}]", k, n).c_str(),
                                        -SCIPinfinity(scip_problem_), +SCIPinfinity(scip_problem_), 0.0,
                                        SCIP_Vartype::SCIP_VARTYPE_CONTINUOUS));
                SCIP(SCIPaddVar(scip_problem_, (scip_state_vars_[k][n])));
            }
        }
        // Create input variables
        for (unsigned int n = 0; n < N; n++) {
            for (unsigned int k = 0; k < SystemType::NUM_CONT_INPUTS; k++) {
                SCIP(SCIPcreateVarBasic(scip_problem_, &(scip_input_vars_[k][n]),
                                        fmt::format("input[{},{}]", k, n).c_str(),
                                        -SCIPinfinity(scip_problem_), +SCIPinfinity(scip_problem_), 0.0,
                                        SCIP_Vartype::SCIP_VARTYPE_CONTINUOUS));
                SCIP(SCIPaddVar(scip_problem_, scip_input_vars_[k][n]));
            }
            for (unsigned int k = SystemType::NUM_CONT_INPUTS; k < (SystemType::NUM_INPUTS); k++) {
                SCIP(SCIPcreateVarBasic(scip_problem_, &(scip_input_vars_[k][n]),
                                        fmt::format("input[{},{}]", k, n).c_str(),
                                        0, 1, 0, SCIP_Vartype::SCIP_VARTYPE_BINARY));
                SCIP(SCIPaddVar(scip_problem_, scip_input_vars_[k][n]));
            }
        }
        static_assert(integration_scheme == BACKWARD_EULER or integration_scheme == FORWARD_EULER);
        if (integration_scheme == BACKWARD_EULER) {
            // Create backward euler dynamics constraint
            // get matrices
            auto A = system_.getA(state_);
            auto B = system_.getB(state_);
            auto Ab = Eigen::Matrix<double, SystemType::NUM_STATES, SystemType::NUM_STATES>::Identity() -
                      (system_dt_ * A);
            auto mAb = -Ab;
            auto Bb = system_dt_ * B;
            for (unsigned int n = 0; n < N; n++) { //for each prediction step x_k -> x_k+1 til x_k+(N-1) -> x_l+(N)
                for (unsigned int m_row = 0; m_row < SystemType::NUM_STATES; m_row++) {
                    //each row has its own constraint, since scip does not come with matrix constraints
                    //create part of z (variables array)
                    std::array<SCIP_VAR *, 1 + SystemType::NUM_STATES + (SystemType::NUM_INPUTS)> vars;
                    vars[0] = scip_state_vars_[m_row][n]; // simplified, since identity at beginning
                    for (unsigned int k = 0; k < SystemType::NUM_STATES; k++) {
                        vars[k + SystemType::NUM_INPUTS + 1] = scip_state_vars_[k][n + 1];
                    }
                    for (unsigned int k = 0; k < (SystemType::NUM_INPUTS); k++) {
                        vars[k + 1] = scip_input_vars_[k][n];
                    }
                    //create coefficients
                    //create coefficients
                    std::array<double, 1 + SystemType::NUM_STATES + (SystemType::NUM_INPUTS)> coefs;
                    coefs[0] = 1; //for first value

                    std::copy_n(Bb.row(m_row).begin(), SystemType::NUM_INPUTS, (coefs.begin() +
                                                                                1)); //TODO: this could cause issues, since i dont now how the iterators behaves
                    std::copy_n(mAb.row(m_row).begin(), SystemType::NUM_STATES,
                                (coefs.begin() + 1 + SystemType::NUM_INPUTS));
                    //create the actual constraint
                    SCIP(SCIPcreateConsBasicLinear(scip_problem_,
                                                   &(scip_dynamic_const_[n][m_row]),
                                                   fmt::format("dynamic_cons[{},{}]", n, m_row).c_str(),
                                                   1 + SystemType::NUM_STATES + (SystemType::NUM_INPUTS),
                                                   vars.data(), coefs.data(), 0, 0));
                    SCIP(SCIPaddCons(scip_problem_,
                                     scip_dynamic_const_[n][m_row]));
                }
            }
        } else if (integration_scheme == FORWARD_EULER) {
            // Create forward euler dynamics constraint
            // get matrices
            auto A = system_.getA(state_);
            auto B = system_.getB(state_);
            auto Ab = Eigen::Matrix<double, SystemType::NUM_STATES, SystemType::NUM_STATES>::Identity() +
                      (system_dt_ * A);
            auto Bb = system_dt_ * B;
            for (unsigned int n = 0; n < N; n++) { //for each prediction step x_k -> x_k+1 til x_k+(N-1) -> x_l+(N)
                for (unsigned int m_row = 0; m_row < SystemType::NUM_STATES; m_row++) {
                    //each row has its own constraint, since scip does not come with matrix constraints
                    //create part of z (variables array)
                    std::array<SCIP_VAR *, 1 + SystemType::NUM_STATES + (SystemType::NUM_INPUTS)> vars;
                    for (unsigned int k = 0; k < SystemType::NUM_STATES; k++) {
                        vars[k] = scip_state_vars_[k][n];
                    }
                    for (unsigned int k = 0; k < (SystemType::NUM_INPUTS); k++) {
                        vars[k + SystemType::NUM_STATES] = scip_input_vars_[k][n];
                    }
                    vars[SystemType::NUM_INPUTS + SystemType::NUM_STATES] = scip_state_vars_[m_row][n +
                                                                                                    1]; // simplified, since identity at end
                    //create coefficients
                    std::array<double, 1 + SystemType::NUM_STATES + (SystemType::NUM_INPUTS)> coefs;
                    coefs[SystemType::NUM_INPUTS + SystemType::NUM_STATES] = -1.; //for last value

                    std::copy_n(Ab.row(m_row).begin(), SystemType::NUM_STATES, (coefs.begin()));
                    std::copy_n(Bb.row(m_row).begin(), SystemType::NUM_INPUTS,
                                (coefs.begin() + SystemType::NUM_STATES));

                    //create the actual constraint
                    SCIP(SCIPcreateConsBasicLinear(scip_problem_,
                                                   &(scip_dynamic_const_[n][m_row]),
                                                   fmt::format("dynamic_cons[{},{}]", n, m_row).c_str(),
                                                   1 + SystemType::NUM_STATES + (SystemType::NUM_INPUTS),
                                                   vars.data(), coefs.data(), 0, 0));
                    SCIP(SCIPaddCons(scip_problem_,
                                     scip_dynamic_const_[n][m_row]));
                }
            }
        }


        // Create timing constraints
        static_assert(min_steps_off == 2);
        static_assert(min_steps_on == 1);
        static_assert(max_steps_on > min_steps_on);

        std::array<double, max_steps_on + 1> ones;
        ones.fill(1.0);

        // max constraints
        for (int n = -SolverBase::history_depth;
             n < (N - (max_steps_on)); n++) {
            for (unsigned int k = SystemType::NUM_CONT_INPUTS; k < (SystemType::NUM_INPUTS); k++) {
                auto var_start_indx = std::max(0, n);
                auto var_end_indx = n + max_steps_on + 1;
                SCIP(SCIPcreateConsBasicLinear(scip_problem_,
                                               &(scip_pattern_timing_max_const_[k - SystemType::NUM_CONT_INPUTS][n +
                                                                                                                 SolverBase::history_depth]),
                                               fmt::format("pattern_timing_max_const[{}{}]", k, n).c_str(),
                                               var_end_indx - var_start_indx,
                                               &(scip_input_vars_[k][var_start_indx]),
                                               ones.data(),
                                               0,
                                               max_steps_on));
                SCIP(SCIPaddCons(scip_problem_,
                                 (scip_pattern_timing_max_const_[k - SystemType::NUM_CONT_INPUTS][n +
                                                                                                  SolverBase::history_depth])));
            }
        }

        //min constraints (fixed to min_steps_on ) 2:
        std::array<double, 3> min_pattern = {1., -1., 1.};
        int to_n; //skip history if we have controller compensation
        if (num_steps_solver_delay == 0) {
            to_n = -1;
        } else if (num_steps_solver_delay == 1) {
            to_n = 0;
        } else if (num_steps_solver_delay >= 2) {
            to_n = 1;
        }

        for (int n = to_n; n < (N - 2); n++) {
            for (unsigned int k = SystemType::NUM_CONT_INPUTS; k < (SystemType::NUM_INPUTS); k++) {
                int num_vars = 0;
                double *coeff = nullptr;
                SCIP_VAR **vars;
                switch (n) {
                    case -1:
                        num_vars = 1;
                        coeff = &min_pattern[2];
                        vars = &(scip_input_vars_[k][0]);
                        break;
                    case 0:
                        num_vars = 2;
                        coeff = &min_pattern[1];
                        vars = &(scip_input_vars_[k][0]);
                        break;
                    default:
                        num_vars = 3;
                        coeff = min_pattern.data();
                        vars = &(scip_input_vars_[k][n - 1]);
                        break;
                }


                SCIP(SCIPcreateConsBasicLinear(scip_problem_,
                                               &(scip_pattern_timing_min_const_[k - SystemType::NUM_CONT_INPUTS][n +
                                                                                                                 1]),
                                               fmt::format("pattern_timing_min_constraint[{},{}]", k, n).c_str(),
                                               num_vars,
                                               vars,
                                               coeff,
                                               -SCIPinfinity(scip_problem_),
                                               1));
                SCIP(SCIPaddCons(scip_problem_,
                                 (scip_pattern_timing_min_const_[k - SystemType::NUM_CONT_INPUTS][n + 1])));
            }
        }

        setCost(cost_type_);

        SCIP(SCIPsetIntParam(scip_problem_, "display/verblevel", 0));
        //SCIP multithread params

    }

    template<class SystemType, int N, int min_steps_on, int min_steps_off, int max_steps_on, int num_steps_solver_delay, INTEGRATION_SCHEME integration_scheme>
    requires
    std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>>
    void
    SCIPSolver<SystemType, N, min_steps_on, min_steps_off, max_steps_on, num_steps_solver_delay, integration_scheme>::addInputConstraintOnIndex(
            int index, double lb, double ub) {
        for (int n = 0; n < (N - 1); n++) {
            SCIP(SCIPchgVarLb(scip_problem_, scip_input_vars_[index][n], lb));
            SCIP(SCIPchgVarUb(scip_problem_, scip_input_vars_[index][n], ub));
        }
    }

    template<class SystemType, int N, int min_steps_on, int min_steps_off, int max_steps_on, int num_steps_solver_delay, INTEGRATION_SCHEME integration_scheme>
    requires
    std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>>
    void
    SCIPSolver<SystemType, N, min_steps_on, min_steps_off, max_steps_on, num_steps_solver_delay, integration_scheme>::addStateConstraintOnIndex(
            int index, double lb, double ub) {
        for (int n = 0; n < (N + 1); n++) {
            SCIP(SCIPchgVarLb(scip_problem_, scip_state_vars_[index][n], lb));
            SCIP(SCIPchgVarUb(scip_problem_, scip_state_vars_[index][n], ub));
        }
    }

    template<class SystemType, int N, int min_steps_on, int min_steps_off, int max_steps_on, int num_steps_solver_delay, INTEGRATION_SCHEME integration_scheme>
    requires
    std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>>
    void
    SCIPSolver<SystemType, N, min_steps_on, min_steps_off, max_steps_on, num_steps_solver_delay, integration_scheme>::addInputConstraintOnStep(
            int step, const typename SystemType::InputVec &lb, const typename SystemType::InputVec &ub) {
        for (int k = 0; k < (SystemType::NUM_INPUTS); k++) {
            SCIP(SCIPchgVarLb(scip_problem_, scip_input_vars_[k][step], lb[k]));
            SCIP(SCIPchgVarUb(scip_problem_, scip_input_vars_[k][step], ub[k]));
        }
    }

    template<class SystemType, int N, int min_steps_on, int min_steps_off, int max_steps_on, int num_steps_solver_delay, INTEGRATION_SCHEME integration_scheme>
    requires
    std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>>
    void
    SCIPSolver<SystemType, N, min_steps_on, min_steps_off, max_steps_on, num_steps_solver_delay, integration_scheme>::addStateConstraintOnStep(
            int step, const typename SystemType::StateVec &lb, const typename SystemType::StateVec &ub) {
        for (int k = 0; k < (SystemType::NUM_STATES); k++) {
            SCIP(SCIPchgVarLb(scip_problem_, scip_state_vars_[k][step], lb[k]));
            SCIP(SCIPchgVarUb(scip_problem_, scip_state_vars_[k][step], ub[k]));
        }
    }

    template<class SystemType, int N, int min_steps_on, int min_steps_off, int max_steps_on, int num_steps_solver_delay, INTEGRATION_SCHEME integration_scheme>
    requires
    std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>>
    void
    SCIPSolver<SystemType, N, min_steps_on, min_steps_off, max_steps_on, num_steps_solver_delay, integration_scheme>::setFinalWeights(
            const typename SystemType::StateVec &final_weights) {
        final_state_cost_weights_ = final_weights;
        switch (cost_type_) {
            case L1Linear:
                for (unsigned int k = 0; k < SystemType::NUM_STATES; k++) {
                    SCIP(SCIPchgVarObj(scip_problem_,
                                       scip_T_state_[k][N],
                                       final_state_cost_weights_[k]));
                }
                break;
            case L2Quadratic:
                throw std::logic_error("Function not yet implemented");
                break;
        }

    }

    template<class SystemType, int N, int min_steps_on, int min_steps_off, int max_steps_on, int num_steps_solver_delay, INTEGRATION_SCHEME integration_scheme>
    requires
    std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>>
    void
    SCIPSolver<SystemType, N, min_steps_on, min_steps_off, max_steps_on, num_steps_solver_delay, integration_scheme>::setStateWeights(
            const typename SystemType::StateVec &state_weights) {
        state_cost_weights_ = state_weights;
        switch (cost_type_) {
            case L1Linear:
                for (unsigned int n = 0; n < (N); n++) {
                    for (unsigned int k = 0; k < SystemType::NUM_STATES; k++) {
                        SCIP(SCIPchgVarObj(scip_problem_,
                                           scip_T_state_[k][n],
                                           state_cost_weights_[k]));
                    }
                }
                break;
            case L2Quadratic:
                throw std::logic_error("Function not yet implemented");
                break;
        }

    }

    template<class SystemType, int N, int min_steps_on, int min_steps_off, int max_steps_on, int num_steps_solver_delay, INTEGRATION_SCHEME integration_scheme>
    requires
    std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>>
    void
    SCIPSolver<SystemType, N, min_steps_on, min_steps_off, max_steps_on, num_steps_solver_delay, integration_scheme>::setInputWeights(
            const typename SystemType::InputVec &input_weights) {
        input_cost_weights_ = input_weights;
        switch (cost_type_) {
            case L1Linear:
                for (unsigned int n = 0; n < (N); n++) {
                    for (unsigned int k = 0; k < (SystemType::NUM_CONT_INPUTS + SystemType::NUM_BIN_INPUTS); k++) {
                        //TODO: check N+1 / N
                        SCIP(SCIPchgVarObj(scip_problem_,
                                           scip_T_input_[k][n],
                                           input_cost_weights_[k]));
                    }
                }
                break;
            case L2Quadratic:
                throw std::logic_error("Function not yet implemented");
                break;
        }
    }

    template<class SystemType, int N, int min_steps_on, int min_steps_off, int max_steps_on, int num_steps_solver_delay, INTEGRATION_SCHEME integration_scheme>
    requires
    std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>>
    void
    SCIPSolver<SystemType, N, min_steps_on, min_steps_off, max_steps_on, num_steps_solver_delay, integration_scheme>::setCost(
            COST_TYPE cost_type) {
        cost_type_ = cost_type;
        switch (cost_type) {
            case L1Linear:
                for (unsigned int n = 0; n < (N + 1); n++) {
                    for (unsigned int k = 0; k < SystemType::NUM_STATES; k++) {
                        double weight = state_cost_weights_[k];
                        if (n == N) {
                            weight = final_state_cost_weights_[k];
                        }
                        SCIP(SCIPcreateVarBasic(
                                scip_problem_,
                                &(scip_T_state_[k][n]),
                                fmt::format("T_state[{},{}]", k, n).c_str(),
                                0,
                                SCIPinfinity(scip_problem_),
                                weight,
                                SCIP_Vartype::SCIP_VARTYPE_CONTINUOUS
                        ));
                        SCIP(SCIPaddVar(scip_problem_,
                                        (scip_T_state_[k][n])));
                        // min: abs(err)
                        //=> min scip_T_state,  with scip_T_state >= abs(err)
                        //=> with scip_T_state >= err && scip_T_state >= -err
                        //=> with scip_T_state >= err && -scip_T_state <= err
                        // state_var - set_point = err
                        // scip_T_state >= -err
                        // scip_T_state >= err
                        // therefore:
                        // scip_T_state >= -state_var + set_point
                        // scip_T_state >= state_var - set_point
                        // therefore:
                        // scip_T_state + state_var >= set_point
                        // scip_T_state - state_var >= -set_point
                        // therefore:
                        // scip_T_state + state_var >= set_point
                        // -scip_T_state + state_var <= set_point
                        // therefore:
                        // state_var + scip_T_state >= set_point
                        // state_var - scip_T_state <= set_point
                        std::array<SCIP_VAR *, 2> vars = {scip_state_vars_[k][n], scip_T_state_[k][n]};
                        std::array<double, 2> coefs1 = {1, -1};
                        std::array<double, 2> coefs2 = {1, 1};
                        // linear const is  lhs <= var <= rhs
                        SCIP(SCIPcreateConsBasicLinear(
                                scip_problem_,
                                &(scip_Tconsp_state_[k][n]),
                                fmt::format("scip_Tstaconsp[{},{}]", k, n).c_str(),
                                2,
                                vars.data(),
                                coefs1.data(),
                                -SCIPinfinity(scip_problem_),
                                set_point_[k]
                        ));

                        SCIP(SCIPcreateConsBasicLinear(
                                scip_problem_,
                                &(scip_Tconsm_state_[k][n]),
                                fmt::format("scip_Tstaconsm[{},{}]", k, n).c_str(),
                                2,
                                vars.data(),
                                coefs2.data(),
                                set_point_[k],
                                SCIPinfinity(scip_problem_)
                        ));
                        SCIP(SCIPaddCons(scip_problem_, (scip_Tconsp_state_[k][n])))
                        SCIP(SCIPaddCons(scip_problem_, scip_Tconsm_state_[k][n]));
                    }
                }
                for (unsigned int n = 0; n < N; n++) {
                    for (unsigned int k = 0; k < (SystemType::NUM_INPUTS); k++) {
                        double weight = input_cost_weights_[k];
                        SCIP(SCIPcreateVarBasic(
                                scip_problem_,
                                &(scip_T_input_[k][n]),
                                fmt::format("T_input[{},{}]", k, n).c_str(),
                                0,
                                SCIPinfinity(scip_problem_),
                                weight,
                                SCIP_Vartype::SCIP_VARTYPE_CONTINUOUS
                        ));
                        SCIP(SCIPaddVar(scip_problem_, scip_T_input_[k][n]));
                        std::array<SCIP_VAR *, 2> vars = {scip_input_vars_[k][n], scip_T_input_[k][n]};
                        std::array<double, 2> coefs1 = {1, -1};
                        std::array<double, 2> coefs2 = {1, 1};
                        SCIP(SCIPcreateConsBasicLinear(
                                scip_problem_,
                                &(scip_Tconsp_input_[k][n]),
                                fmt::format("scip_Tinconsp[{},{}]", k, n).c_str(),
                                2,
                                vars.data(),
                                coefs1.data(),
                                -SCIPinfinity(scip_problem_),
                                0
                        ));
                        SCIP(SCIPaddCons(scip_problem_,
                                         (scip_Tconsp_input_[k][n])));
                        SCIP(SCIPcreateConsBasicLinear(
                                scip_problem_,
                                &(scip_Tconsm_input_[k][n]),
                                fmt::format("scip_Tinconsm[{},{}]", k, n).c_str(),
                                2,
                                vars.data(),
                                coefs2.data(),
                                0,
                                SCIPinfinity(scip_problem_)
                        ));
                        SCIP(SCIPaddCons(scip_problem_,
                                         (scip_Tconsm_input_[k][n])));
                    }
                }

                break;
            case L2Quadratic:
                // For the states we need to define error variables that take the value of the error
                for (unsigned int n = 0; n < (N + 1); n++) {
                    for (unsigned int k = 0; k < SystemType::NUM_STATES; k++) {
                        SCIP(SCIPcreateVarBasic(
                                scip_problem_,
                                &(scip_QuadCostErr_states_[k][n]),
                                fmt::format("scip_QuadCostErr_states_[{},{}]", k, n).c_str(),
                                -SCIPinfinity(scip_problem_),
                                SCIPinfinity(scip_problem_),
                                0,
                                SCIP_Vartype::SCIP_VARTYPE_CONTINUOUS
                        ));
                        std::array<double, 2> coefs = {-1.0, 1.0};
                        std::array<SCIP_VAR *, 2> vars = {scip_QuadCostErr_states_[k][n], scip_state_vars_[k][n]};

                        SCIP(SCIPcreateConsBasicLinear(
                                scip_problem_,
                                &(scip_QuadCostErrCons_states_[k][n]),
                                fmt::format("scip_QuadCostErrCons_input_[{},{}]", k, n).c_str(),
                                2,
                                vars.data(),
                                coefs.data(),
                                set_point_[k],
                                set_point_[k]
                        ));

                        SCIP(SCIPaddVar(scip_problem_,
                                        scip_QuadCostErr_states_[k][n]));
                        SCIP(SCIPaddCons(scip_problem_,
                                         scip_QuadCostErrCons_states_[k][n]));
                    }
                }
                // For input this is not nececarry as there are no errors

                constexpr int num_all_vars = (N + 1) * (SystemType::NUM_STATES) + (N * (SystemType::NUM_INPUTS));
                std::array<SCIP_VAR *, num_all_vars> vars;
                std::array<double, num_all_vars> weights;
                unsigned int i = 0;
                for (unsigned int n = 0; n < (N + 1); n++) {
                    for (unsigned int k = 0; k < SystemType::NUM_STATES; k++) {
                        vars[i] = scip_state_vars_[k][n];
                        if (n == N) {
                            weights[i] = final_state_cost_weights_[k];
                        } else {
                            weights[i] = state_cost_weights_[k];
                        }
                        i++;
                    }
                }
                for (unsigned int n = 0; n < (N); n++) {
                    for (unsigned int k = 0; k < (SystemType::NUM_INPUTS); k++) {
                        vars[i] = scip_input_vars_[k][n];
                        weights[i] = input_cost_weights_[k];
                        i++;
                    }
                }
                SCIP(SCIPcreateVarBasic(scip_problem_,
                                        &(scip_QuadCostSum_),
                                        "scip_QuadCostVar",
                                        -SCIPinfinity(scip_problem_),
                                        SCIPinfinity(scip_problem_),
                                        1.0, // weighting via the cons
                                        SCIP_Vartype::SCIP_VARTYPE_CONTINUOUS))
                double coeff = -1.0; // to be equal to constraint
                SCIP(SCIPcreateConsBasicQuadraticNonlinear(
                        scip_problem_, &scip_QuadCostCons_,
                        "quad cost const",
                        1, &scip_QuadCostSum_, &coeff, //linear
                        num_all_vars, vars.data(), vars.data(), weights.data(), //quad
                        0, 0
                ));

                SCIP(SCIPaddVar(scip_problem_,
                                scip_QuadCostSum_));
                SCIP(SCIPaddCons(scip_problem_,
                                 scip_QuadCostCons_));

                break;
        }
    }

    template<class SystemType, int N, int min_steps_on, int min_steps_off, int max_steps_on, int num_steps_solver_delay, INTEGRATION_SCHEME integration_scheme>
    requires
    std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>>
    void
    SCIPSolver<SystemType, N, min_steps_on, min_steps_off, max_steps_on, num_steps_solver_delay, integration_scheme>::setSetPoint(
            const typename SystemType::StateVec &set_point) {
        set_point_ = set_point;
        switch (cost_type_) {
            case L1Linear:
                for (unsigned int n = 0; n < (N + 1); n++) {
                    for (unsigned int k = 0; k < SystemType::NUM_STATES; k++) {
                        SCIP(SCIPchgRhsLinear(scip_problem_, (scip_Tconsp_state_[k][n]), set_point_[k]));
                        SCIP(SCIPchgLhsLinear(scip_problem_, (scip_Tconsm_state_[k][n]), set_point_[k]));
                    }
                }
                break;
            case L2Quadratic:
                for (unsigned int n = 0; n < (N + 1); n++) {
                    for (unsigned int k = 0; k < SystemType::NUM_STATES; k++) {
                        SCIP(SCIPchgRhsLinear(scip_problem_, (scip_QuadCostErrCons_states_[k][n]), set_point_[k]));
                        SCIP(SCIPchgLhsLinear(scip_problem_, (scip_QuadCostErrCons_states_[k][n]), set_point_[k]));
                    }
                }
                break;
        }

    }

    template<class SystemType, int N, int min_steps_on, int min_steps_off, int max_steps_on, int num_steps_solver_delay, INTEGRATION_SCHEME integration_scheme>
    requires
    std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>>
    unsigned int
    SCIPSolver<SystemType, N, min_steps_on, min_steps_off, max_steps_on, num_steps_solver_delay, integration_scheme>::getStepsToCompensateControllerDelay() {
        return num_steps_solver_delay;
    }

    template<class SystemType, int N, int min_steps_on, int min_steps_off, int max_steps_on, int num_steps_solver_delay, INTEGRATION_SCHEME integration_scheme>
    requires
    std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>>
    void
    SCIPSolver<SystemType, N, min_steps_on, min_steps_off, max_steps_on, num_steps_solver_delay, integration_scheme>::setState(
            const typename SystemType::StateVec &state) {
        // update state
        state_ = state;
        // update dynamics accordingly
        if (integration_scheme == BACKWARD_EULER) {
            std::function<void(unsigned int, unsigned int, double)> updateAfun = [this](unsigned int row,
                                                                                        unsigned int col, double val) {
                double Ab;
                if (row == col) {
                    Ab = 1.0 - system_dt_ * val;
                } else {
                    Ab = system_dt_ * val;
                }
                double mAb = -Ab;
                for (unsigned int n = 0; n < N; n++) {
                    SCIP(SCIPchgCoefLinear(scip_problem_, scip_dynamic_const_[n][row], scip_state_vars_[col][n + 1],
                                           mAb));
                    //TODO: use faster method here, that doesnt have to search for the variable, since we know the index
                }
            };
            std::function<void(unsigned int, unsigned int, double)> updateBfun = [this](unsigned int row,
                                                                                        unsigned int col, double val) {
                double Bb = system_dt_ * val;
                for (unsigned int n = 0; n < N; n++) {
                    SCIP(SCIPchgCoefLinear(scip_problem_, scip_dynamic_const_[n][row], scip_input_vars_[col][n], Bb));
                    //TODO: use faster method here, that doesnt have to search for the variable, since we know the index
                }
            };

            system_.updateA(state, updateAfun);
            system_.updateB(state, updateBfun);
        } else if (integration_scheme == FORWARD_EULER) {
            std::function<void(unsigned int, unsigned int, double)> updateAfun = [this](unsigned int row,
                                                                                        unsigned int col, double val) {
                double Ab;
                if (row == col) {
                    Ab = 1.0 + system_dt_ * val;
                } else {
                    Ab = system_dt_ * val;
                }
                for (unsigned int n = 0; n < N; n++) {
                    SCIP(SCIPchgCoefLinear(scip_problem_, scip_dynamic_const_[n][row], scip_state_vars_[col][n], Ab));
                    //TODO: use faster method here, that doesnt have to search for the variable, since we know the index
                }
            };
            std::function<void(unsigned int, unsigned int, double)> updateBfun = [this](unsigned int row,
                                                                                        unsigned int col, double val) {
                double Bb = system_dt_ * val;
                for (unsigned int n = 0; n < N; n++) {
                    SCIP(SCIPchgCoefLinear(scip_problem_, scip_dynamic_const_[n][row], scip_input_vars_[col][n], Bb));
                    //TODO: use faster method here, that doesnt have to search for the variable, since we know the index
                }
            };
            system_.updateA(state, updateAfun);
            system_.updateB(state, updateBfun);
        }
    }

    template<class SystemType, int N, int min_steps_on, int min_steps_off, int max_steps_on, int num_steps_solver_delay, INTEGRATION_SCHEME integration_scheme>
    requires
    std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>>
    void
    SCIPSolver<SystemType, N, min_steps_on, min_steps_off, max_steps_on, num_steps_solver_delay, integration_scheme>::setInputHistory(
            const Eigen::Matrix<double, SystemType::NUM_INPUTS, SolverBase::history_depth> &bin_input_hist) {
        for (unsigned k = SystemType::NUM_CONT_INPUTS; k < (SystemType::NUM_INPUTS); k++) {
            // min const
            if (num_steps_solver_delay == 0) {
                SCIP(SCIPchgRhsLinear(scip_problem_,
                                      (scip_pattern_timing_min_const_[k - SystemType::NUM_CONT_INPUTS][0]),
                                      (1 - bin_input_hist(k,
                                                          SolverBase::history_depth -
                                                          2) +
                                       bin_input_hist(k,
                                                      SolverBase::history_depth -
                                                      1))));
            }
            if (num_steps_solver_delay <= 1) {
                SCIP(SCIPchgRhsLinear(scip_problem_,
                                      (scip_pattern_timing_min_const_[k - SystemType::NUM_CONT_INPUTS][1]),
                                      (1 - bin_input_hist(k,
                                                          SolverBase::history_depth -
                                                          1))));
            }
            // max const
            for (unsigned int n = 0; n <
                                     SolverBase::history_depth; n++) {
                SCIP(SCIPchgRhsLinear(scip_problem_,
                                      (scip_pattern_timing_max_const_[k - SystemType::NUM_CONT_INPUTS][n]),
                                      max_steps_on -
                                      bin_input_hist(k, Eigen::seq(n, Eigen::last)).sum())); //TODO: check
            }
        }
    }

    template<class SystemType, int N, int min_steps_on, int min_steps_off, int max_steps_on, int num_steps_solver_delay, INTEGRATION_SCHEME integration_scheme>
    requires
    std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>>
    void
    SCIPSolver<SystemType, N, min_steps_on, min_steps_off, max_steps_on, num_steps_solver_delay, integration_scheme>::setSolverTimeLimit(
            double max_seconds) {
        SCIP(SCIPsetRealParam(scip_problem_, "limits/time", max_seconds));
    }

    template<class SystemType, int N, int min_steps_on, int min_steps_off, int max_steps_on, int num_steps_solver_delay, INTEGRATION_SCHEME integration_scheme>
    requires
    std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>>
    void
    SCIPSolver<SystemType, N, min_steps_on, min_steps_off, max_steps_on, num_steps_solver_delay, integration_scheme>::setNextInputs(
            const Eigen::Matrix<double, SystemType::NUM_INPUTS, num_steps_solver_delay> &next_inputs) {
        for (unsigned int n = 0; n < num_steps_solver_delay; n++) {
            for (unsigned int k = 0; k < (SystemType::NUM_INPUTS); k++) {
                SCIPchgVarLb(scip_problem_, scip_input_vars_[k][n], next_inputs(k, n));
                SCIPchgVarUb(scip_problem_, scip_input_vars_[k][n], next_inputs(k, n));
            }
        }
    }

    template<class SystemType, int N, int min_steps_on, int min_steps_off, int max_steps_on, int num_steps_solver_delay, INTEGRATION_SCHEME integration_scheme>
    requires
    std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>>
    void
    SCIPSolver<SystemType, N, min_steps_on, min_steps_off, max_steps_on, num_steps_solver_delay, integration_scheme>::writeProblemToFile(const std::string & file_path) {
        SCIPwriteOrigProblem(scip_problem_, file_path.c_str(), NULL, true);
    }

    template<class SystemType, int N, int min_steps_on, int min_steps_off, int max_steps_on, int num_steps_solver_delay, INTEGRATION_SCHEME integration_scheme>
    requires
    std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>>
    SCIPSolver<SystemType, N, min_steps_on, min_steps_off, max_steps_on, num_steps_solver_delay, integration_scheme>::~SCIPSolver() {

        unsigned int deleted;

        for (unsigned int n = 0; n < N; n++) {
            for (unsigned int k = 0; k < (SystemType::NUM_INPUTS); k++) {
                SCIPdelVar(scip_problem_, scip_input_vars_[k][n], &deleted);
                if (cost_type_ == L1Linear) {
                    (SCIPreleaseVar(scip_problem_, &(scip_T_input_[k][n])));
                    (SCIPreleaseCons(scip_problem_, &(scip_Tconsp_input_[k][n])));
                    (SCIPreleaseCons(scip_problem_, &(scip_Tconsm_input_[k][n])));
                }
            }
        }

        for (unsigned int n = 0; n < (N + 1); n++) {
            for (unsigned int k = 0; k < SystemType::NUM_STATES; k++) {
                SCIPdelVar(scip_problem_, scip_state_vars_[k][n], &deleted);
                if (cost_type_ == L1Linear) {
                    (SCIPreleaseVar(scip_problem_, &(scip_T_state_[k][n])));
                    (SCIPreleaseCons(scip_problem_, &(scip_Tconsp_state_[k][n])));
                    (SCIPreleaseCons(scip_problem_, &(scip_Tconsm_state_[k][n])));
                }
            }
        }

        for (unsigned int n = 0; n < SolverBase::history_depth + N - max_steps_on; n++) {
            for (unsigned int k = 0; k < SystemType::NUM_BIN_INPUTS; k++) {
                (SCIPreleaseCons(scip_problem_, &(scip_pattern_timing_max_const_[k][n])));
            }
        }

        int to_n; //skip history if we have controller compensation
        if (num_steps_solver_delay == 0) {
            to_n = 0;
        } else if (num_steps_solver_delay == 1) {
            to_n = 1;
        } else if (num_steps_solver_delay >= 2) {
            to_n = 2;
        }

        for (unsigned int n = to_n; n < 1 + N - 2; n++) {
            for (unsigned int k = 0; k < SystemType::NUM_BIN_INPUTS; k++) {
                (SCIPreleaseCons(scip_problem_, &(scip_pattern_timing_min_const_[k][n])));
            }
        }

        (SCIPfree(&scip_problem_));
    }

    template<class SystemType, int N, int min_steps_on, int min_steps_off, int max_steps_on, int num_steps_solver_delay, INTEGRATION_SCHEME integration_scheme>
    requires
    std::derived_from<SystemType, System<SystemType::NUM_STATES, SystemType::NUM_CONT_INPUTS, SystemType::NUM_BIN_INPUTS>>
    SOLVER_RETURN
    SCIPSolver<SystemType, N, min_steps_on, min_steps_off, max_steps_on, num_steps_solver_delay, integration_scheme>::solve(
            const typename Eigen::Matrix<double, SystemType::NUM_INPUTS, N, Eigen::RowMajor> &last_open_loop_input,
            const typename Eigen::Matrix<double, SystemType::NUM_STATES, N + 1, Eigen::RowMajor> &last_open_loop_state,
            typename Eigen::Matrix<double, SystemType::NUM_INPUTS, N, Eigen::RowMajor> &open_loop_input,
            typename Eigen::Matrix<double, SystemType::NUM_STATES, N + 1, Eigen::RowMajor> &open_loop_state) const {

        //TODO: measure time for statistics

        //Set init guess
        SCIP_SOL *scip_init_guess;
        SCIP(SCIPcreatePartialSol(scip_problem_, &scip_init_guess, nullptr));

        for (unsigned int k = 0; k < SystemType::NUM_STATES; k++) {
            SCIP(SCIPsetSolVals(scip_problem_,
                                scip_init_guess, (N + 1),
                                const_cast<SCIP_VAR **>(scip_state_vars_[k].data()),
                                const_cast<double *>(last_open_loop_state.row(k).data())
            ));
        }

        for (unsigned int k = 0; k < (SystemType::NUM_INPUTS); k++) {
            SCIP(SCIPsetSolVals(scip_problem_,
                                scip_init_guess, (N),
                                const_cast<SCIP_VAR **>(scip_input_vars_[k].data()),
                                const_cast<double *>(last_open_loop_input.row(k).data())
            ));
        }
        bool accepted;
        //SCIP(SCIPaddSolFree(scip_problem_, &scip_init_guess, reinterpret_cast<unsigned int *>(&accepted)));
        // TODO: shift solution and check if it actually worked here
        (void) (accepted);

        SCIP(SCIPsolve(scip_problem_));
        auto solution_status = SCIPgetStatus(scip_problem_);
        auto n_sols = SCIPgetNSols(scip_problem_);
        SOLVER_RETURN result;
        if(solution_status == SCIP_STATUS_USERINTERRUPT){
          result = USER_INTERRUPT;
        } else if (solution_status == SCIP_STATUS_OPTIMAL) {
            result = OPTIMAL;
        } else if (n_sols >= 1) {
          result = TIME_LIMIT_WITH_SOLUTION; // TODO: simplifies other termination resons to time limit
        } else {
            result = NO_SOLUTION;
        }
        //SCIP(SCIPsolveConcurrent(scip_problem_));

        //TODO: check, if memory was acclocated previouously correct
        if (result != NO_SOLUTION) {
            SCIP_SOL *best_solution = SCIPgetBestSol(scip_problem_);


            for (unsigned int k = 0; k < (SystemType::NUM_INPUTS); k++) {
                for (unsigned int n = 0; n < N; n++) {
                    double i = SCIPgetSolVal(scip_problem_, best_solution, scip_input_vars_[k][n]);
                    open_loop_input(k, n) = i;
                }
//            SCIP(SCIPgetSolVals(scip_problem_,
//                                best_solution, (N),
//                                const_cast<SCIP_VAR**>(scip_input_vars_[k].data()),
//                                open_loop_input.row(k).data()
//            ));
            }

            for (unsigned int k = 0; k < SystemType::NUM_STATES; k++) {
                for (unsigned int n = 0; n < (N + 1); n++) {
                    open_loop_state(k, n) = SCIPgetSolVal(scip_problem_, best_solution, scip_state_vars_[k][n]);
                }
//            SCIP(SCIPgetSolVals(scip_problem_,
//                                best_solution, (N+1),
//                         const_cast<SCIP_VAR**>(scip_state_vars_[k].data()),
//                                open_loop_state.row(k).data()
//            ));
            }


        }
        SCIP(SCIPfreeTransform(scip_problem_));

        return result;
    }
};