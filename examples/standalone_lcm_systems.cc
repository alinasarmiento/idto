#include "standalone_lcm_systems.h"

#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_object_state.hpp"
#include "dairlib/lcmt_trajectory_block.hpp"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include <drake/systems/framework/basic_vector.h>

#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

namespace idto {
namespace examples {

using drake::multibody::MultibodyPlant;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using Eigen::VectorXd;

// -----------------------------------------------------------------------------
// LcmStateReceiver
// -----------------------------------------------------------------------------

LcmStateReceiver::LcmStateReceiver(int state_size){
  n_x_ = state_size;
  // state_names_ = state_names;
  
  robot_input_port_ = this->DeclareAbstractInputPort("lcmt_robot_output",
						     drake::Value<dairlib::lcmt_robot_output>{}).get_index();
  obj_input_port_ = this->DeclareAbstractInputPort("lcmt_object_state",
						   drake::Value<dairlib::lcmt_object_state>{}).get_index();
  this->DeclareVectorOutputPort("x, t",
				BasicVector<double>(n_x_),
				&LcmStateReceiver::CopyOutput);
}

void LcmStateReceiver::CopyOutput(const Context<double>& context,
			   BasicVector<double>* output) const {
  const drake::AbstractValue* robot_input = this->EvalAbstractInput(context, robot_input_port_);
  const drake::AbstractValue* obj_input = this->EvalAbstractInput(context, obj_input_port_);
  const auto& robot_state_msg = robot_input->get_value<dairlib::lcmt_robot_output>();
  const auto& obj_state_msg = obj_input->get_value<dairlib::lcmt_object_state>();

  int n_x_robot = robot_state_msg.num_positions;
  int n_x_obj = obj_state_msg.num_positions;
  int n_v_robot = robot_state_msg.num_velocities;
  int n_v_obj = obj_state_msg.num_velocities;

  int idx_xr = 0;
  int idx_xo = n_x_robot;
  int idx_vr = n_x_robot + n_x_obj;
  int idx_vo = n_x_robot + n_x_obj + n_v_robot;

  auto state = output->get_mutable_value();
  // Copy positions
  for (int i = idx_xr; i < idx_xr + n_x_robot; ++i) {
    state(i) = robot_state_msg.position[i - idx_xr];
  }
  for (int i = idx_xo; i < idx_xo + n_x_obj; ++i) {
    state(i) = obj_state_msg.position[i - idx_xo];
  }
  
  // Copy velocities
  for (int i = idx_vr; i < idx_vr + n_v_robot; ++i) {
    state(i) = robot_state_msg.velocity[i - idx_vr];
  }
  for (int i = idx_vo; i < idx_vo + n_v_obj; ++i) {
    state(i) = obj_state_msg.velocity[i - idx_vo];
  }
  std::cout << "outputting state: " << state << std::endl;
}

// -----------------------------------------------------------------------------
// LcmTrajAdapter
// -----------------------------------------------------------------------------

LcmTrajAdapter::LcmTrajAdapter() {
  traj_input_port_ = this->DeclareAbstractInputPort(
      "stored_trajectory",
      drake::Value<mpc::StoredTrajectory>{}).get_index();

  this->DeclareAbstractOutputPort("lcmt_trajectory_block",
				  dairlib::lcmt_trajectory_block{},
				  &LcmTrajAdapter::ConvertTraj);
}

namespace {

std::vector<double> MakeSampleTimes(const mpc::StoredTrajectory& traj) {
  std::vector<double> times;
  if (traj.q.get_segment_times().empty()) {
    times.push_back(traj.start_time);
    return times;
  }

  for (double t : traj.q.get_segment_times()) {
    if (t > 0) {
      const auto it = std::find(times.begin(), times.end(), t);
      if (it == times.end()) {
        times.push_back(t);
      }
    }
  }

  std::vector<double> with_midpoints;
  with_midpoints.push_back(times.front());
  for (size_t i = 0; i + 1 < times.size(); ++i) {
    const double t0 = times[i];
    const double t1 = times[i + 1];
    if (t1 - t0 > 1e-12) {
      with_midpoints.push_back(0.5 * (t0 + t1));
    }
    with_midpoints.push_back(t1);
  }

  return with_midpoints;
}

}  // namespace

  
void LcmTrajAdapter::ConvertTraj(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_trajectory_block* output) const {
  const drake::AbstractValue* traj_input =
      this->EvalAbstractInput(context, traj_input_port_);
  const auto& traj = traj_input->get_value<mpc::StoredTrajectory>();

  const std::vector<double> sample_times = MakeSampleTimes(traj);
  const int num_points = static_cast<int>(sample_times.size());

  if (num_points == 0) {
    output->trajectory_name = "idto_traj";
    output->num_points = 0;
    output->num_datatypes = 0;
    output->time_vec.clear();
    output->datatypes.clear();
    output->datapoints.clear();
    return;
  }

  const auto q0 = traj.q.value(sample_times.front());
  const auto v0 = traj.v.value(sample_times.front());
  const auto u0 = traj.u.value(sample_times.front());

  const int num_q = static_cast<int>(u0.size());
  const int num_v = static_cast<int>(u0.size());
  const int num_u = static_cast<int>(u0.size());

  output->trajectory_name = "idto_traj";
  output->num_points = num_points;
  output->num_datatypes = num_q + num_v + num_u;

  output->time_vec.clear();
  output->time_vec.resize(num_points);
  for (int i = 0; i < num_points; ++i) {
    output->time_vec[i] = sample_times[i] + traj.start_time;
  }

  output->datatypes.clear();
  output->datatypes.resize(output->num_datatypes);
  output->datapoints.clear();
  output->datapoints.resize(output->num_datatypes, std::vector<double>(num_points));

  // std::cout << "WRITING NEW MSG" << std::endl;
  // for (int i=0; i<num_points; ++i){
  //   std::cout << output->time_vec[i] << std::endl;
  // }
  for (int row = 0; row < num_u; ++row) {
    const int out_row = row;
    output->datatypes[out_row] = "u_" + std::to_string(row);
    for (int i = 0; i < num_points; ++i) {
      const auto u_val = traj.u.value(sample_times[i]);
      output->datapoints[out_row][i] = u_val(row);
    }
  }
  
  for (int row = 0; row < num_q; ++row) {
    const int out_row = num_u + row;
    output->datatypes[out_row] = "q_" + std::to_string(row);
    for (int i = 0; i < num_points; ++i) {
      const auto q_val = traj.q.value(sample_times[i]);
      output->datapoints[out_row][i] = q_val(row);
    }
  }

  for (int row = 0; row < num_v; ++row) {
    const int out_row = num_u + num_q + row;
    output->datatypes[out_row] = "v_" + std::to_string(row);
    for (int i = 0; i < num_points; ++i) {
      const auto v_val = traj.v.value(sample_times[i]);
      output->datapoints[out_row][i] = v_val(row);
    }
  }

}

}  // namespace examples
}  // namespace idto
