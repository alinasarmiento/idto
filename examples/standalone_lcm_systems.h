#pragma once

#include <string>
#include <vector>

#include "dairlib/lcmt_trajectory_block.hpp"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"

#include "mpc_controller.h"

namespace idto {
namespace examples {

class LcmStateReceiver : public drake::systems::LeafSystem<double> {
 public:
  LcmStateReceiver(int state_size);

  const drake::systems::InputPort<double>& get_robot_input_port() const {
    return this->get_input_port(robot_input_port_);
  }
  const drake::systems::InputPort<double>& get_obj_input_port() const {
    return this->get_input_port(obj_input_port_);
  }

 private:
  void CopyOutput(const drake::systems::Context<double>& context,
                  drake::systems::BasicVector<double>* output) const;

  int n_x_{0};
  drake::systems::InputPortIndex robot_input_port_{};
  drake::systems::InputPortIndex obj_input_port_{};
};

class LcmTrajAdapter : public drake::systems::LeafSystem<double> {
 public:
  LcmTrajAdapter();

 private:
  void ConvertTraj(const drake::systems::Context<double>& context,
                   dairlib::lcmt_trajectory_block* output) const;

  drake::systems::InputPortIndex traj_input_port_{};
};

}  // namespace examples
}  // namespace idto
