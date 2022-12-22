// Copyright (c) 2022 Stogl Robotics Consulting UG (haftungsbeschränkt)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "joint_trajectory_controller/cartesian_trajectory_generator.hpp"

#include "tf2/transform_datatypes.h"

#include "controller_interface/helpers.hpp"
#include "joint_trajectory_controller/trajectory.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"


namespace
{  // utility

void reset_twist_msg(geometry_msgs::msg::Twist & msg)
{
  msg.linear.x = std::numeric_limits<double>::quiet_NaN();
  msg.linear.y = std::numeric_limits<double>::quiet_NaN();
  msg.linear.z = std::numeric_limits<double>::quiet_NaN();
  msg.angular.x = std::numeric_limits<double>::quiet_NaN();
  msg.angular.y = std::numeric_limits<double>::quiet_NaN();
  msg.angular.z = std::numeric_limits<double>::quiet_NaN();
}

using ControllerReferenceMsg =
  cartesian_trajectory_generator::CartesianTrajectoryGenerator::ControllerReferenceMsg;

// called from RT control loop
void reset_controller_reference_msg(ControllerReferenceMsg & msg)
{
  msg.transforms.resize(1);
  msg.transforms[0].translation.x = std::numeric_limits<double>::quiet_NaN();
  msg.transforms[0].translation.y = std::numeric_limits<double>::quiet_NaN();
  msg.transforms[0].translation.z = std::numeric_limits<double>::quiet_NaN();
  msg.transforms[0].rotation.x = std::numeric_limits<double>::quiet_NaN();
  msg.transforms[0].rotation.y = std::numeric_limits<double>::quiet_NaN();
  msg.transforms[0].rotation.z = std::numeric_limits<double>::quiet_NaN();
  msg.transforms[0].rotation.w = std::numeric_limits<double>::quiet_NaN();

  msg.velocities.resize(1);
  reset_twist_msg(msg.velocities[0]);

  msg.accelerations.resize(1);
  reset_twist_msg(msg.accelerations[0]);
}

void reset_controller_reference_msg(const std::shared_ptr<ControllerReferenceMsg> & msg)
{
  reset_controller_reference_msg(*msg);
}

using ControllerFeedbackMsg =
  cartesian_trajectory_generator::CartesianTrajectoryGenerator::ControllerFeedbackMsg;

// called from RT control loop
void reset_controller_feedback_msg(ControllerFeedbackMsg & msg)
{
  msg.pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  msg.pose.pose.position.y = std::numeric_limits<double>::quiet_NaN();
  msg.pose.pose.position.z = std::numeric_limits<double>::quiet_NaN();
  msg.pose.pose.orientation.x = std::numeric_limits<double>::quiet_NaN();
  msg.pose.pose.orientation.y = std::numeric_limits<double>::quiet_NaN();
  msg.pose.pose.orientation.z = std::numeric_limits<double>::quiet_NaN();
  msg.pose.pose.orientation.w = std::numeric_limits<double>::quiet_NaN();
  msg.pose.covariance.fill(std::numeric_limits<double>::quiet_NaN());

  reset_twist_msg(msg.twist.twist);
  msg.twist.covariance.fill(std::numeric_limits<double>::quiet_NaN());
}
void reset_controller_feedback_msg(const std::shared_ptr<ControllerFeedbackMsg> & msg)
{
  reset_controller_feedback_msg(*msg);
}
}  // namespace

namespace cartesian_trajectory_generator
{
CartesianTrajectoryGenerator::CartesianTrajectoryGenerator()
: joint_trajectory_controller::JointTrajectoryController()
{
}

controller_interface::InterfaceConfiguration
CartesianTrajectoryGenerator::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::NONE;
  return conf;
}

controller_interface::CallbackReturn CartesianTrajectoryGenerator::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  auto ret = joint_trajectory_controller::JointTrajectoryController::on_configure(previous_state);
  if (ret != CallbackReturn::SUCCESS)
  {
    return ret;
  }

  // set all position per default to not use positions
  for (const auto & joint_name : command_joint_names_)
  {
    use_position_input_[joint_name] = realtime_tools::RealtimeBuffer(false);
  }

  // topics QoS
  auto subscribers_qos = rclcpp::SystemDefaultsQoS();
  subscribers_qos.keep_last(1);
  subscribers_qos.best_effort();

  // Reference Subscriber
  ref_subscriber_ = get_node()->create_subscription<ControllerReferenceMsg>(
    "~/reference", subscribers_qos,
    std::bind(&CartesianTrajectoryGenerator::reference_callback, this, std::placeholders::_1));

  std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();
  reset_controller_reference_msg(msg);
  input_ref_.writeFromNonRT(msg);

  // Odometry feedback
  auto feedback_callback = [&](const std::shared_ptr<ControllerFeedbackMsg> feedback_msg) -> void
  { feedback_.writeFromNonRT(feedback_msg); };
  feedback_subscriber_ = get_node()->create_subscription<ControllerFeedbackMsg>(
    "~/feedback", subscribers_qos, feedback_callback);
  std::shared_ptr<ControllerFeedbackMsg> feedback_msg = std::make_shared<ControllerFeedbackMsg>();
  reset_controller_feedback_msg(feedback_msg);
  feedback_.writeFromNonRT(feedback_msg);

  // NOTE(rebase): this used to also create two runtime-reconfiguration services:
  //  - `~/reset_axes` (control_msgs::srv::ResetAxis): let a caller switch a named axis from
  //    velocity-streaming back to position-hold at the current feedback pose (a "let go and
  //    freeze in place" control for Cartesian jogging).
  //  - `~/set_joint_limits` (control_msgs::srv::SetDOFLimits): let a caller retune the per-axis
  //    position/velocity/acceleration/jerk/effort limits feeding the Ruckig smoothing at runtime,
  //    without a controller reconfigure.
  // Both service types were custom additions to a private control_msgs fork from 2022 that were
  // never upstreamed and don't exist in jazzy's control_msgs. Neither service is required for the
  // core feature (reference subscription -> position/velocity blending -> trajectory generation
  // feeding the existing Ruckig-smoothed JTC pipeline), so both were dropped here rather than
  // reinventing custom .srv types. Impact: axes can only be velocity-streamed, never explicitly
  // released back to position-hold via service (see the note on use_position_input_ in the
  // header); and joint_limits_ can only be set at on_configure() time, not retuned live.

  return CallbackReturn::SUCCESS;
}

void CartesianTrajectoryGenerator::reference_callback(
  const std::shared_ptr<ControllerReferenceMsg> msg)
{
  // store input ref for later use
  input_ref_.writeFromNonRT(msg);

  trajectory_msgs::msg::JointTrajectoryPoint state;
  resize_joint_trajectory_point(state, dof_);
  read_state_from_state_interfaces(state);

  // assume for now that we are working with trajectories with one point - we don't know exactly
  // where we are in the trajectory before sampling - nevertheless this should work for the use case
  auto new_traj_msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
  new_traj_msg->joint_names = params_.joints;
  new_traj_msg->points.resize(1);
  new_traj_msg->points[0].positions.resize(
    params_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  new_traj_msg->points[0].velocities.resize(
    params_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  new_traj_msg->points[0].time_from_start = rclcpp::Duration::from_seconds(0.01);

  // check all axes for "type" of messages coming in. If there are velocity values in a filed then
  // we switch away from position mode and set position to NaN
  auto assign_value_depending_on_input = [&](
                                           const double pos_from_msg, const double vel_from_msg,
                                           const std::string & joint_name, const size_t index,
                                           const double pos_feedback) {
    if (!std::isnan(vel_from_msg))
    {
      if (*(use_position_input_[joint_name].readFromNonRT()))
      {
        RCLCPP_INFO(
          get_node()->get_logger(), "Disabling position mode on dof '%s'.", joint_name.c_str());
      }
      use_position_input_[joint_name].writeFromNonRT(false);
      new_traj_msg->points[0].velocities[index] = vel_from_msg;
    }
    else if (*(use_position_input_[joint_name].readFromNonRT()))
    {
      if (!std::isnan(pos_from_msg))
      {
        new_traj_msg->points[0].positions[index] = pos_from_msg;
        new_traj_msg->points[0].velocities[index] = 0.0;
      }
      else
      {
        new_traj_msg->points[0].positions[index] = pos_feedback;
        new_traj_msg->points[0].velocities[index] = 0.0;
      }
    }
    else
    {
      RCLCPP_DEBUG(
        get_node()->get_logger(),
        "Input velocity is NaN, but not using position mode. Ignoring input message.");
    }
  };

  assign_value_depending_on_input(
    msg->transforms[0].translation.x, msg->velocities[0].linear.x, params_.joints[0], 0,
    state.positions[0]);
  assign_value_depending_on_input(
    msg->transforms[0].translation.y, msg->velocities[0].linear.y, params_.joints[1], 1,
    state.positions[1]);
  assign_value_depending_on_input(
    msg->transforms[0].translation.z, msg->velocities[0].linear.z, params_.joints[2], 2,
    state.positions[2]);
  assign_value_depending_on_input(
    msg->transforms[0].rotation.x, msg->velocities[0].angular.x, params_.joints[3], 3,
    state.positions[3]);
  assign_value_depending_on_input(
    msg->transforms[0].rotation.y, msg->velocities[0].angular.y, params_.joints[4], 4,
    state.positions[4]);
  assign_value_depending_on_input(
    msg->transforms[0].rotation.z, msg->velocities[0].angular.z, params_.joints[5], 5,
    state.positions[5]);

  add_new_trajectory_msg(new_traj_msg);
}

controller_interface::CallbackReturn CartesianTrajectoryGenerator::on_activate(
  const rclcpp_lifecycle::State &)
{
  // order all joints in the storage
  // NOTE(rebase): command_interface_types_ was superseded by params_.command_interfaces when
  // jazzy migrated parameter declarations to the generate_parameter_library params_ struct.
  for (const auto & interface : params_.command_interfaces)
  {
    auto it =
      std::find(allowed_interface_types_.begin(), allowed_interface_types_.end(), interface);
    auto index = std::distance(allowed_interface_types_.begin(), it);
    if (!controller_interface::get_ordered_interfaces(
          command_interfaces_, command_joint_names_, interface, joint_command_interface_[index]))
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Expected %zu '%s' command interfaces, got %zu.", dof_,
        interface.c_str(), joint_command_interface_[index].size());
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  // NOTE(rebase): no state-interface ordering here (this was already commented out in the
  // original commit) -- state_interface_configuration() returns NONE for this controller;
  // Cartesian state comes from the tf2/Odometry feedback subscriber instead, via
  // read_state_from_state_interfaces() below.

  // NOTE(rebase): dropped the original "Store 'home' pose" block (traj_msg_home_ptr_,
  // traj_home_point_ptr_) -- jazzy removed the whole go-home concept independently of this
  // branch (see commits 1-14 of this rebase); nothing in the current update()/on_deactivate()
  // pipeline ever triggers a return-to-home anymore, so it was dead weight even before the
  // rename below. Replaced traj_external_point_ptr_/traj_point_active_ptr_/
  // traj_msg_external_point_ptr_ with jazzy's collapsed current_trajectory_/new_trajectory_msg_
  // (mirrors JointTrajectoryController::on_activate() exactly). Also dropped
  // last_state_publish_time_, which is unused dead vestigial state (see commit 14 of this
  // rebase for the same fix in the base class).
  current_trajectory_ = std::make_shared<joint_trajectory_controller::Trajectory>();
  new_trajectory_msg_.writeFromNonRT(std::shared_ptr<trajectory_msgs::msg::JointTrajectory>());

  subscriber_is_active_ = true;

  // Initialize current state storage if hardware state has tracking offset
  read_state_from_state_interfaces(state_current_);
  read_state_from_state_interfaces(state_desired_);
  read_state_from_state_interfaces(last_commanded_state_);
  // Handle restart of controller by reading from commands if
  // those are not nan
  trajectory_msgs::msg::JointTrajectoryPoint state;
  resize_joint_trajectory_point(state, dof_);
  if (read_state_from_command_interfaces(state))
  {
    state_current_ = state;
    state_desired_ = state;
    last_commanded_state_ = state;
  }

  // NOTE(rebase): added to match JointTrajectoryController::on_activate()'s current behavior.
  // Without this, current_trajectory_ has no trajectory message and has_active_trajectory()
  // stays false until the first ~/reference message arrives, so update() writes nothing to the
  // command interfaces in the meantime. For position command interfaces that's harmless (the
  // hardware just holds its last position), but for velocity/effort command interfaces it would
  // leave them uncommanded (driver-dependent fallback, often but not guaranteed to be zero) from
  // activation until the first Cartesian reference. Holding at the current position immediately
  // is the safer default and is what the rest of the codebase now assumes.
  add_new_trajectory_msg(set_hold_position());
  rt_is_holding_ = true;

  return CallbackReturn::SUCCESS;
}

void CartesianTrajectoryGenerator::read_state_from_state_interfaces(JointTrajectoryPoint & state)
{
  std::array<double, 3> orientation_angles;
  const auto measured_state = *(feedback_.readFromRT());
  tf2::Quaternion measured_q;
  tf2::fromMsg(measured_state->pose.pose.orientation, measured_q);
  tf2::Matrix3x3 m(measured_q);
  m.getRPY(orientation_angles[0], orientation_angles[1], orientation_angles[2]);

  // Assign values from the hardware
  // Position states always exist
  state.positions[0] = measured_state->pose.pose.position.x;
  state.positions[1] = measured_state->pose.pose.position.y;
  state.positions[2] = measured_state->pose.pose.position.z;
  state.positions[3] = orientation_angles[0];
  state.positions[4] = orientation_angles[1];
  state.positions[5] = orientation_angles[2];

  state.velocities[0] = measured_state->twist.twist.linear.x;
  state.velocities[1] = measured_state->twist.twist.linear.y;
  state.velocities[2] = measured_state->twist.twist.linear.z;
  state.velocities[3] = measured_state->twist.twist.angular.x;
  state.velocities[4] = measured_state->twist.twist.angular.y;
  state.velocities[5] = measured_state->twist.twist.angular.z;

  state.accelerations.clear();
}

}  // namespace cartesian_trajectory_generator

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  cartesian_trajectory_generator::CartesianTrajectoryGenerator,
  controller_interface::ControllerInterface)
