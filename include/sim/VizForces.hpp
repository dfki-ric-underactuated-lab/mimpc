#pragma once

#include <math.h>
#include <fmt/format.h>

#include "drake/systems/framework/leaf_system.h"

#include "drake/multibody/plant/externally_applied_spatial_force.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/math/spatial_force.h"

#include "drake/geometry/meshcat.h"
#include "drake/geometry/shape_specification.h"

#include "drake/common/value.h"




namespace mimpc::simulation {
    class VizForces : public drake::systems::LeafSystem<double> {

    private:
        drake::multibody::MultibodyPlant<double> &plant_;
        std::unique_ptr<drake::systems::Context<double>> plant_context_;
        drake::geometry::Meshcat &meshcat_;

        const int num_forces_;
        const std::string body_link_name_;

        const double arrow_len_multiplier_;
        const double arrow_width_;

        drake::systems::InputPort<double> *body_state_input_port_;
        drake::TypeSafeIndex<drake::systems::InputPortTag> forces_input_port_;


    public:

        VizForces(drake::multibody::MultibodyPlant<double> &plant, drake::geometry::Meshcat &meshcat,
                  const int num_forces,
                  const std::string &bodyLinkName, const double arrowLenMultiplier);

        void updateArrows(const drake::systems::Context<double> &context) const;

    };
}

