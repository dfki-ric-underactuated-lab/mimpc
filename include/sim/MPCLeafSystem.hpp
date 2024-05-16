#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "MPC.hpp"
#include "System.hpp"

namespace mimpc::simulation {
    template<class MPCType> requires std::derived_from<MPCType, MPC<typename MPCType::SYSTEM_TYPE, typename MPCType::SOLVER_TYPE>>
    class MPCLeafSystem : public drake::systems::LeafSystem<double> {
    public:

        MPCLeafSystem(MPCType &reacsaSolver, double control_dt,
                      const MPCType::SYSTEM_TYPE::StateVec &state_constraints_lb,
                      const MPCType::SYSTEM_TYPE::StateVec &state_constraints_ub);

        ~MPCLeafSystem() override;

    private:
        MPCType &reacsa_solver_;
        double control_dt_;
        MPCType::SYSTEM_TYPE::StateVec state_constraints_lb_;
        MPCType::SYSTEM_TYPE::StateVec state_constraints_ub_;

        drake::systems::InputPort<double> *plant_out_port_;
        drake::systems::OutputPort<double> *mpc_time_port_;
        drake::systems::OutputPort<double> *control_out_port_;
        drake::systems::DiscreteStateIndex control_out_state_;
        drake::systems::DiscreteStateIndex mpc_time_out_state_;

        drake::systems::EventStatus discreteUpdate(const drake::systems::Context<double> &context,
                            drake::systems::DiscreteValues<double> *discrete_state) const;

    };
}
#include "MPCLeafSystem.tpp"