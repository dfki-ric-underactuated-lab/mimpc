//
// Created by fstark on 06.08.23.
//
#pragma once

#include "sim/Simulation.hpp"
#include "systems/REACSA_constants.h"

namespace mimpc::simulation {
    template<class MPCType>
    Simulation<MPCType>::Simulation(double delay_to_simulate, const std::string &plant_urdf_file, MPCType &mpc,
                                    const StateVec &initState, const StateVec &targetState,
                                    const StateVec &targetThreshold, const StateVec &stateConstraintsLb,
                                    const StateVec &stateConstraintsUb, bool withViz, double binary_force):
            init_state_(initState),
            target_state_(targetState),
            target_threshold_(targetThreshold),
            state_constraints_lb_(stateConstraintsLb),
            state_constraints_ub_(stateConstraintsUb),
            mpc_(mpc) {
        builder_ = new drake::systems::DiagramBuilder<double>;
        std::tie(plant_, scene_graph_) = drake::multibody::AddMultibodyPlantSceneGraph(builder_, 0.0);
        drake::multibody::Parser(plant_, scene_graph_).AddModels(plant_urdf_file);
        plant_->mutable_gravity_field().set_gravity_vector(Eigen::Vector3d::Zero());
        plant_->Finalize();

        forces_mux_ = builder_->AddSystem<drake::multibody::ExternallyAppliedSpatialForceMultiplexer>(8);
        input_demux_ = builder_->AddSystem<drake::systems::Demultiplexer>(9, 1);
        for (unsigned int i = 0; i < 8; i++) {
            auto &thruster_link = plant_->GetBodyByName(fmt::format("f{}", i));
            drake::math::RigidTransformd transform(drake::math::RollPitchYawd(0, M_PI_2, 0), Eigen::Vector3d::Zero());
            auto thruster = builder_->AddSystem<drake::multibody::Propeller>(thruster_link.index(), transform,
                                                                             binary_force, 0.0);
            builder_->Connect(thruster->get_spatial_forces_output_port(), forces_mux_->get_input_port(i));
            builder_->Connect(input_demux_->get_output_port(i + 1), thruster->get_command_input_port());
            builder_->Connect(plant_->get_body_poses_output_port(), thruster->get_body_poses_input_port());
        }
        controller_delay_ = builder_->AddSystem<drake::systems::DiscreteTimeDelay>(delay_to_simulate, 1, 9);

        builder_->Connect(input_demux_->get_output_port(0), plant_->GetInputPort("REACSA_actuation"));
        builder_->Connect(forces_mux_->get_output_port(0), plant_->get_applied_spatial_force_input_port());

        matrix_ << 1, 0, 0, 0, 0, 0, 0, 0,
                    0, 1, 0, 0, 0, 0, 0, 0,
                    0, 0, 1, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 1, 0, 0, 0,
                    0, 0, 0, 0, 0, 1, 0, 0,
                    0, 0, 0, 0, 0, 0, 1, 0,
                    0, 0, 0, 0, 0, 0, 0, 1;

        state_output_matrix_ = builder_->AddSystem<drake::systems::MatrixGain>(matrix_);
        builder_->Connect(plant_->get_state_output_port(), state_output_matrix_->get_input_port());
        mpc_leaf_system_ = builder_->AddSystem<MPCLeafSystem<MPCType>>(mpc_, 0.1, state_constraints_lb_,
                                                                       state_constraints_ub_);

        builder_->Connect(state_output_matrix_->get_output_port(), mpc_leaf_system_->get_input_port());
        builder_->Connect(mpc_leaf_system_->get_output_port(0), controller_delay_->get_input_port());
        builder_->Connect(controller_delay_->get_output_port(), input_demux_->get_input_port());

        if (withViz) {
            meshcat_ = std::make_shared<drake::geometry::Meshcat>(); //TODO: make meshcat a shared pointer, or maybe make all a sahred pointer
            forces_viz_ = builder_->AddSystem<VizForces>(*plant_, *meshcat_.get(), 8, "base_link", 1.0);
            builder_->Connect(plant_->get_state_output_port(), forces_viz_->get_input_port(0));
            builder_->Connect(forces_mux_->get_output_port(), forces_viz_->get_input_port(1));
            drake::visualization::AddDefaultVisualization(builder_, meshcat_);
        }

        state_logger_ = drake::systems::LogVectorOutput(state_output_matrix_->get_output_port(), builder_);
        input_logger_ = drake::systems::LogVectorOutput(mpc_leaf_system_->get_output_port(0), builder_);


        diagram_ = builder_->Build();
        simulator_ = new drake::systems::Simulator<double>(*diagram_);

        auto &context = simulator_->get_mutable_context();

        Eigen::Vector<double, 8> init_state_sized = {init_state_(0), init_state_(1), init_state_(2), 0.0,
                                                     init_state_(3), init_state_(4), init_state_(5), init_state_(6)};
        diagram_->GetMutableSubsystemState(*plant_, &context).get_mutable_continuous_state().SetFromVector(
                init_state_sized);
        simulator_->set_target_realtime_rate(1.0);
    }

    template<class MPCType>
    bool
    Simulation<MPCType>::simulateToTarget(float time_out_seconds) {
        simulator_->set_monitor([this](const drake::systems::Context<double> &context) -> drake::systems::EventStatus {
            auto &sub_context = state_output_matrix_->GetMyContextFromRoot(context);
            auto stats_abs = state_output_matrix_->get_output_port().Eval(sub_context).cwiseAbs();
            for (unsigned int i = 0; i < 3; i++) {
                if (stats_abs(i) > target_threshold_(i)) {
                    return drake::systems::EventStatus::Succeeded();
                }
            }
            std::cout << "Reached target" << std::endl;
            return drake::systems::EventStatus::ReachedTermination(mpc_leaf_system_, "Alle States below threshold");
        });
        auto status = simulator_->AdvanceTo(time_out_seconds);
        simulator_->clear_monitor();
        return status.return_time() < status.boundary_time();
    }

    template<class MPCType>
    int Simulation<MPCType>::simulate(float for_seconds) {
        auto sim_status = simulator_->AdvanceTo(simulator_->get_context().get_time() + for_seconds);
        if(sim_status.reason() == drake::systems::SimulatorStatus::kReachedTerminationCondition){
          return -1;
        }else{
          return 0;
        }
    }

    template<class MPCType>
    void
    Simulation<MPCType>::saveData(const std::string &filename) {
        auto x_log = state_logger_->FindLog(simulator_->get_context());
        auto u_log = input_logger_->FindLog(simulator_->get_context());


        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> xnp = Eigen::Map<const Eigen::MatrixXd>(
                x_log.data().data(), x_log.data().rows(), x_log.data().cols());
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> unp = Eigen::Map<const Eigen::MatrixXd>(
                u_log.data().data(), u_log.data().rows(), u_log.data().cols());
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> xtimenp = Eigen::Map<const Eigen::MatrixXd>(
                x_log.sample_times().data(), x_log.sample_times().rows(), x_log.sample_times().cols());
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> utimenp = Eigen::Map<const Eigen::MatrixXd>(
                u_log.sample_times().data(), u_log.sample_times().rows(), u_log.sample_times().cols());

        cnpy::npz_save(filename, "x_log", xnp.data(), {(size_t) xnp.rows(), (size_t) xnp.cols()}, "w");
        cnpy::npz_save(filename, "u_log", unp.data(), {(size_t) unp.rows(), (size_t) unp.cols()}, "a");
        cnpy::npz_save(filename, "x_time", xtimenp.data(), {(size_t) xtimenp.rows(), (size_t) xtimenp.cols()}, "a");
        cnpy::npz_save(filename, "u_time", utimenp.data(), {(size_t) utimenp.rows(), (size_t) utimenp.cols()}, "a");

    }
}

