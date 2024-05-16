//
// Created by fstark on 06.08.23.
//

#pragma once

#include <cnpy.h>

#include <type_traits>
#include <concepts>
#include "System.hpp"
#include "MPC.hpp"

#include "drake/systems/framework/diagram_builder.h"
#include "drake/common/drake_assert.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/externally_applied_spatial_force.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/plant/propeller.h"
#include "drake/multibody/plant/externally_applied_spatial_force_multiplexer.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/pid_controller.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/systems/primitives/matrix_gain.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/visualization/visualization_config_functions.h"
#include "drake/systems/primitives/discrete_time_delay.h"
#include "drake/systems/primitives/vector_log_sink.h"
#include "MPCLeafSystem.hpp"
#include "VizForces.hpp"

namespace mimpc::simulation {
    template<class MPCType>
    class Simulation {
        using StateVec = MPCType::SYSTEM_TYPE::StateVec;
    private:
        MPCType::SYSTEM_TYPE::StateVec init_state_;
        MPCType::SYSTEM_TYPE::StateVec target_state_;
        MPCType::SYSTEM_TYPE::StateVec target_threshold_;

        MPCType::SYSTEM_TYPE::StateVec state_constraints_lb_;
        MPCType::SYSTEM_TYPE::StateVec state_constraints_ub_;

        MPCType &mpc_;

        // drake
        drake::systems::DiagramBuilder<double> *builder_;
        drake::multibody::MultibodyPlant<double> *plant_;
        drake::geometry::SceneGraph<double> *scene_graph_;

        drake::multibody::ExternallyAppliedSpatialForceMultiplexer<double> *forces_mux_;
        drake::systems::Demultiplexer<double> *input_demux_;
        drake::systems::DiscreteTimeDelay<double> *controller_delay_;

        drake::systems::MatrixGain<double> *state_output_matrix_;
        MPCLeafSystem<MPCType> *mpc_leaf_system_;

        std::unique_ptr<drake::systems::Diagram<double>> diagram_;


        // viz
        std::shared_ptr<drake::geometry::Meshcat> meshcat_;
        VizForces *forces_viz_;

        // logger
        drake::systems::VectorLogSink<double> *state_logger_;
        drake::systems::VectorLogSink<double> *input_logger_;

        drake::systems::Simulator<double> *simulator_;

        // TODO: check the coordinates here!
        Eigen::Matrix<double, 7, 8> matrix_;


    public:
        Simulation(double delay_to_simulate, const std::string &plant_urdf_file, MPCType &mpc,
                   const StateVec &initState, const StateVec &targetState,
                   const StateVec &targetThreshold, const StateVec &stateConstraintsLb,
                   const StateVec &stateConstraintsUb, bool withViz, double binary_force);

        bool simulateToTarget(float time_out_seconds);

        int simulate(float for_seconds);

        void saveData(const std::string &filename);
    };
};

#include "Simulation.tpp"