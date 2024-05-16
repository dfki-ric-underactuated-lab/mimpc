#include "sim/VizForces.hpp"

namespace mimpc::simulation {
    VizForces::VizForces(drake::multibody::MultibodyPlant<double> &plant, drake::geometry::Meshcat &meshcat,
                         const int num_forces,
                         const std::string &bodyLinkName, const double arrowLenMultiplier) : plant_(plant),
                                                                                             plant_context_(
                                                                                                     plant_.CreateDefaultContext()),
                                                                                             meshcat_(meshcat),
                                                                                             num_forces_(num_forces),
                                                                                             body_link_name_(
                                                                                                     bodyLinkName),
                                                                                             arrow_len_multiplier_(
                                                                                                     arrowLenMultiplier),
                                                                                             arrow_width_(
                                                                                                     arrow_len_multiplier_ /
                                                                                                     20.0) {

        // create all arrows in meshcat
        drake::geometry::Cylinder arrow_neck(arrow_width_, arrow_len_multiplier_);
        drake::geometry::MeshcatCone arrow_head(2 * arrow_width_, 2 * arrow_width_, 2 * arrow_width_);
        drake::geometry::Rgba color_red(1, 0, 0, 1);

        drake::math::RigidTransform arrow_neck_transform(drake::math::RollPitchYaw(0., M_PI_2, 0.),
                                                         {arrow_len_multiplier_ / 2.0, 0, 0});
        drake::math::RigidTransform arrow_head_transform(drake::math::RollPitchYaw(0., -M_PI_2, 0.),
                                                         {arrow_len_multiplier_ + arrow_width_, 0, 0});

        for (int i = 0; i < num_forces_; i++) {
            meshcat_.SetObject(fmt::format("arrow_{}/neck", i), arrow_neck, color_red);
            meshcat_.SetObject(fmt::format("arrow_{}/head", i), arrow_head, color_red);

            meshcat_.SetTransform(fmt::format("arrow_{}/neck", i), arrow_neck_transform);
            meshcat_.SetTransform(fmt::format("arrow_{}/head", i), arrow_head_transform);
        }

        // create ports
        body_state_input_port_ = &DeclareVectorInputPort("body_state_input_port", plant_.GetStateNames().size());
        forces_input_port_ = DeclareAbstractInputPort("forces_input_port_",
                                                      drake::Value<std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>>>()).get_index();
        DeclarePeriodicPublishEvent(0.1, 0.0, &VizForces::updateArrows);

    }

    void VizForces::updateArrows(const drake::systems::Context<double> &context) const {
        auto forces = static_cast<const drake::Value<std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>>> *>(EvalAbstractInput(
                context, forces_input_port_));
        auto state = body_state_input_port_->Eval(context);

        plant_.SetPositionsAndVelocities(plant_context_.get(), state);

        for (int i = 0; i < num_forces_; i++) {
            auto force_W = forces->get_value()[i].F_Bq_W.get_coeffs()(Eigen::seq(3, Eigen::last));

            if (force_W.norm() <= 0.01) {
                meshcat_.SetProperty(fmt::format("arrow_{}", i), "visible", false);
            } else {
                auto transl_B_F_B = forces->get_value()[i].p_BoBq_B;
                auto transF_W_B = plant_.EvalBodyPoseInWorld(*(plant_context_.get()),
                                                             plant_.get_body(forces->get_value()[i].body_index));
                auto ROT_X1_F_W = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d({1, 0, 0}), force_W);
                auto transl_B_F_W = transF_W_B * transl_B_F_B;

                meshcat_.SetProperty(fmt::format("arrow_{}/neck", i), "length",
                                     arrow_len_multiplier_ * force_W.norm()); //TODO: placce arrow head according to len
                //meshcat_.SetTransform(fmt::format("arrow_{}/head", i), drake::math::RigidTransformd(drake::math::RollPitchYaw(0.,-M_PI_2,0.), {(arrow_len_multiplier_*force_W.norm()) + arrow_width_,0,0}));
                meshcat_.SetTransform(fmt::format("arrow_{}", i),
                                      drake::math::RigidTransform(ROT_X1_F_W, transl_B_F_W));
                meshcat_.SetProperty(fmt::format("arrow_{}", i), "visible", true);
            }

        }
    }
}
