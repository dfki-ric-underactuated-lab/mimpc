#pragma once

#include "sim/MPCLeafSystem.hpp"
#include "systems/REACSA.hpp"
#include <iostream>

namespace mimpc::simulation {
    template<class MPCType>
    requires std::derived_from<MPCType, MPC<typename MPCType::SYSTEM_TYPE, typename MPCType::SOLVER_TYPE>>
    MPCLeafSystem<MPCType>::~MPCLeafSystem() {

    }

    template<class MPCType>
    requires std::derived_from<MPCType, MPC<typename MPCType::SYSTEM_TYPE, typename MPCType::SOLVER_TYPE>>
    MPCLeafSystem<MPCType>::MPCLeafSystem(MPCType &reacsaSolver, double control_dt,
                                          const MPCType::SYSTEM_TYPE::StateVec &state_constraints_lb,
                                          const MPCType::SYSTEM_TYPE::StateVec &state_constraints_ub):
            reacsa_solver_(reacsaSolver),
            control_dt_(control_dt),
            state_constraints_lb_(state_constraints_lb),
            state_constraints_ub_(state_constraints_ub) {
        plant_out_port_ = &DeclareVectorInputPort("plant_output_port", MPCType::SYSTEM_TYPE::NUM_STATES);
        control_out_state_ = DeclareDiscreteState(
                MPCType::SYSTEM_TYPE::NUM_BIN_INPUTS + MPCType::SYSTEM_TYPE::NUM_CONT_INPUTS);
        mpc_time_out_state_ = DeclareDiscreteState(1);
        control_out_port_ = &DeclareStateOutputPort("control_input_port", control_out_state_);
        mpc_time_port_ = &DeclareStateOutputPort("mpc_time_port", mpc_time_out_state_);
        DeclarePeriodicDiscreteUpdateEvent(control_dt_, 0.0, &MPCLeafSystem<MPCType>::discreteUpdate);
    }

    template<class MPCType>
    requires std::derived_from<MPCType, MPC<typename MPCType::SYSTEM_TYPE, typename MPCType::SOLVER_TYPE>>
    drake::systems::EventStatus MPCLeafSystem<MPCType>::discreteUpdate(const drake::systems::Context<double> &context,
                                                drake::systems::DiscreteValues<double> *discrete_state) const {
        auto system_state = plant_out_port_->Eval(context);
        Eigen::Vector<double,
                MPCType::SYSTEM_TYPE::NUM_CONT_INPUTS + MPCType::SYSTEM_TYPE::NUM_BIN_INPUTS> control_input;
        std::string stats;
        //clamp state to stay feasible:
        for (unsigned int state_idx = 0; state_idx < MPCType::SYSTEM_TYPE::NUM_STATES; state_idx++) {
            if (system_state[state_idx] < state_constraints_lb_[state_idx]) {
                std::cout << "clamp up state " << state_idx << "from [" << system_state[state_idx] << "] to ["
                          << state_constraints_lb_[state_idx] << "]" << std::endl;
                system_state[state_idx] = state_constraints_lb_[state_idx];
            } else if (system_state[state_idx] > state_constraints_ub_[state_idx]) {
                std::cout << "clamp down state " << state_idx << "from [" << system_state[state_idx] << "] to ["
                          << state_constraints_ub_[state_idx] << "]" << std::endl;
                system_state[state_idx] = state_constraints_ub_[state_idx];
            }
        }
        auto ret = reacsa_solver_.control(system_state, control_input, stats);
        discrete_state->get_mutable_vector(0).SetFromVector(control_input);
        //TODO: get and set time
        if(ret == USER_INTERRUPT){
          return drake::systems::EventStatus::ReachedTermination(this, "User interrupt in solver");
        }
        return drake::systems::EventStatus::Succeeded();
    }
}

