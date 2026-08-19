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

#include "angles/angles.h"
#include "controller_interface/helpers.hpp"
#include "joint_limits/joint_limits_rosparam.hpp"
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

// NOTE(rebase): commented out (not deleted) -- params_.joints is now the real robot joint list,
// so this override is no longer needed; the inherited state_interface_configuration() correctly
// claims real joint state interfaces from it. Kept here for review before the next step.
// controller_interface::InterfaceConfiguration
// CartesianTrajectoryGenerator::state_interface_configuration() const
// {
//   controller_interface::InterfaceConfiguration conf;
//   conf.type = controller_interface::interface_configuration_type::NONE;
//   return conf;
// }

controller_interface::CallbackReturn CartesianTrajectoryGenerator::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  // kinematics is mandatory for IK conversion, fail if no param is declared.
  if (params_.kinematics.plugin_name.empty())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "kinematics parameters are empty. Please declare valid parameters in the controllers YAML!");
    return CallbackReturn::FAILURE;
  }

  // Call the base class on_configure
  auto ret = joint_trajectory_controller::JointTrajectoryController::on_configure(previous_state);
  if (ret != CallbackReturn::SUCCESS)
  {
    return ret;
  }

  // Load the differential IK plugin
  try
  {
    // Make sure we destroy the interface first. Otherwise we might run into a segfault
    if (kinematics_loader_)
    {
      kinematics_.reset();
    }
    kinematics_loader_ =
      std::make_shared<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>>(
        params_.kinematics.plugin_package, "kinematics_interface::KinematicsInterface");
    kinematics_ = std::unique_ptr<kinematics_interface::KinematicsInterface>(
      kinematics_loader_->createUnmanagedInstance(params_.kinematics.plugin_name));

    if (!kinematics_->initialize(
          get_robot_description(), get_node()->get_node_parameters_interface(), "kinematics"))
    {
      return CallbackReturn::FAILURE;
    }
  }
  catch (pluginlib::PluginlibException & ex)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Exception while loading the IK plugin '%s': '%s'",
      params_.kinematics.plugin_name.c_str(), ex.what());
    return CallbackReturn::FAILURE;
  }

  // This controller only supports writing position commands for now
  if (!has_position_command_interface_)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "CartesianTrajectoryGenerator requires 'position' in command_interfaces. IK-converted "
      "targets are only ever written as position commands.");
    return CallbackReturn::FAILURE;
  }

  // The IK-delta safety check inside update() can only succeed if that joint has a
  // declared max_velocity. We add a warning at startup for any joint missing one
  for (size_t i = 0; i < dof_; ++i)
  {
    if (!joint_limits_[i].has_velocity_limits)
    {
      RCLCPP_WARN(
        get_node()->get_logger(),
        "Joint '%s' has no declared velocity limit (joint_limits.%s.max_velocity). The "
        "IK-delta safety check in update() will not be able to catch an "
        "oversized command for this joint.",
        params_.joints[i].c_str(), params_.joints[i].c_str());
    }
  }

  // Check if the cartesian axes parameter is correctly populated
  if (params_.kinematics.cartesian_axes.size() != 6)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "params_.kinematics.cartesian_axes must list exactly 6 entries (x, y, z, rx, ry, rz order), "
      "got %zu.",
      params_.kinematics.cartesian_axes.size());
    return CallbackReturn::FAILURE;
  }

  // set all position per default to not use positions
  for (const auto & axis_name : params_.kinematics.cartesian_axes)
  {
    use_position_input_[axis_name] = realtime_tools::RealtimeBuffer(false);
  }

  // Load the cartesian joints limits of type JointLimits. We only use this structure even in
  // cartesian space as it contains the necessary information for Trajectory sample/update methods
  // and matches their signature.
  // FLAG: we can maybe  use a custom message later on? Need to check in the future version of the
  // code first.
  cartesian_joint_limits_.resize(params_.kinematics.cartesian_axes.size());
  for (size_t i = 0; i < cartesian_joint_limits_.size(); ++i)
  {
    const auto & axis_name = params_.kinematics.cartesian_axes[i];
    if (joint_limits::declare_parameters(axis_name, get_node()))
    {
      joint_limits::get_joint_limits(axis_name, get_node(), cartesian_joint_limits_[i]);
      RCLCPP_INFO(
        get_node()->get_logger(), "Limits for Cartesian axis %zu (%s) are: \n%s", i,
        axis_name.c_str(), cartesian_joint_limits_[i].to_string().c_str());
    }
    // The Cartesian-delta safety check can only succeed if it has
    // a declared max_velocity. We add a warning at startup for any axis missing one
    if (!cartesian_joint_limits_[i].has_velocity_limits)
    {
      RCLCPP_WARN(
        get_node()->get_logger(),
        "Cartesian axis '%s' has no declared velocity limit. The "
        "Cartesian-delta safety check in update() will not be able to catch an oversized command "
        "for this axis.",
        axis_name.c_str());
    }
  }

  // Instantiate the cartesian trajectory object
  current_cartesian_trajectory_ = std::make_shared<joint_trajectory_controller::Trajectory>();

  // Size the dedicated real-joint-space state
  resize_joint_trajectory_point(joint_state_current_, dof_);
  resize_joint_trajectory_point(joint_state_desired_, dof_);

  // resize to the number of cartesian axes
  cartesian_state_current_.positions.resize(params_.kinematics.cartesian_axes.size());
  cartesian_state_current_.velocities.resize(params_.kinematics.cartesian_axes.size());

  // this message will be lock-free
  new_cartesian_trajectory_msg_.writeFromNonRT(
    std::shared_ptr<trajectory_msgs::msg::JointTrajectory>());

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

  // NOTE(rebase): the original's second service, `~/set_joint_limits`
  // (control_msgs::srv::SetDOFLimits, retuning per-axis Ruckig limits live without a reconfigure),
  // stays dropped -- also a private-fork-only message type, and not required for the core feature.
  // Impact: cartesian_joint_limits_ can only be set at on_configure() time, not retuned live.

  // service QoS
  auto services_qos = rclcpp::SystemDefaultsQoS();  // message queue depth
  services_qos.keep_all();
  services_qos.reliable();
  services_qos.durability_volatile();

  // Control mode service
  auto reset_axes_service_callback =
    [&](
      const std::shared_ptr<ControllerModeSrvType::Request> request,
      std::shared_ptr<ControllerModeSrvType::Response> response)
  {
    response->ok = true;
    for (size_t i = 0; i < request->names.size(); ++i)
    {
      auto it = std::find(
        params_.kinematics.cartesian_axes.begin(), params_.kinematics.cartesian_axes.end(),
        request->names[i]);
      if (it != params_.kinematics.cartesian_axes.end())
      {
        use_position_input_[request->names[i]].writeFromNonRT(true);
        RCLCPP_INFO(
          get_node()->get_logger(), "Enabling position mode on dof '%s'.",
          request->names[i].c_str());
        // TODO(destogl): use RealtimeBox or similar with readFromNonRT
        // reset data in the last reference to read values from feedback
        auto current_ref = std::make_shared<ControllerReferenceMsg>(**(input_ref_.readFromNonRT()));
        auto cmd_itf_index = std::distance(params_.kinematics.cartesian_axes.begin(), it);
        if (cmd_itf_index == 0)
        {
          current_ref->transforms[0].translation.x = std::numeric_limits<double>::quiet_NaN();
          current_ref->velocities[0].linear.x = std::numeric_limits<double>::quiet_NaN();
        }
        if (cmd_itf_index == 1)
        {
          current_ref->transforms[0].translation.y = std::numeric_limits<double>::quiet_NaN();
          current_ref->velocities[0].linear.y = std::numeric_limits<double>::quiet_NaN();
        }
        if (cmd_itf_index == 2)
        {
          current_ref->transforms[0].translation.z = std::numeric_limits<double>::quiet_NaN();
          current_ref->velocities[0].linear.z = std::numeric_limits<double>::quiet_NaN();
        }
        if (cmd_itf_index == 3)
        {
          current_ref->transforms[0].rotation.x = std::numeric_limits<double>::quiet_NaN();
          current_ref->velocities[0].angular.x = std::numeric_limits<double>::quiet_NaN();
        }
        if (cmd_itf_index == 4)
        {
          current_ref->transforms[0].rotation.y = std::numeric_limits<double>::quiet_NaN();
          current_ref->velocities[0].angular.y = std::numeric_limits<double>::quiet_NaN();
        }
        if (cmd_itf_index == 5)
        {
          current_ref->transforms[0].rotation.z = std::numeric_limits<double>::quiet_NaN();
          current_ref->velocities[0].angular.z = std::numeric_limits<double>::quiet_NaN();
        }
        reference_callback(current_ref);
      }
      else
      {
        RCLCPP_WARN(
          get_node()->get_logger(), "Name '%s' is not command interface. Ignoring this entry.",
          request->names[i].c_str());
        response->ok = false;
      }
    }
  };

  reset_axes_service_ = get_node()->create_service<ControllerModeSrvType>(
    "~/reset_axes", reset_axes_service_callback, services_qos);

  return CallbackReturn::SUCCESS;
}

void CartesianTrajectoryGenerator::query_state_service(
  const std::shared_ptr<control_msgs::srv::QueryTrajectoryState::Request> /*request*/,
  std::shared_ptr<control_msgs::srv::QueryTrajectoryState::Response> response)
{
  response->success = false;
  response->message =
    "query_state is not supported by CartesianTrajectoryGenerator -- there is no discrete "
    "joint trajectory to query; this controller streams Cartesian references converted to "
    "joint commands every cycle via differential IK.";
}

void CartesianTrajectoryGenerator::reference_callback(
  const std::shared_ptr<ControllerReferenceMsg> msg)
{
  // store input ref for later use
  input_ref_.writeFromNonRT(msg);

  // populate the current cartesian state with values from feedback
  read_cartesian_state_from_feedback(cartesian_state_current_, *(feedback_.readFromNonRT()));

  // assume for now that we are working with trajectories with one point - we don't know exactly
  // where we are in the trajectory before sampling - nevertheless this should work for the use case
  auto new_traj_msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
  new_traj_msg->joint_names = params_.kinematics.cartesian_axes;
  new_traj_msg->points.resize(1);
  new_traj_msg->points[0].positions.resize(
    params_.kinematics.cartesian_axes.size(), std::numeric_limits<double>::quiet_NaN());
  new_traj_msg->points[0].velocities.resize(
    params_.kinematics.cartesian_axes.size(), std::numeric_limits<double>::quiet_NaN());
  new_traj_msg->points[0].time_from_start = rclcpp::Duration::from_seconds(0.01);

  // check all axes for "type" of messages coming in. If there are velocity values in a field then
  // we switch away from position mode and set position to NaN
  auto assign_value_depending_on_input = [&](
                                           const double pos_from_msg, const double vel_from_msg,
                                           const std::string & joint_name, const size_t index,
                                           const double pos_feedback)
  {
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
    msg->transforms[0].translation.x, msg->velocities[0].linear.x,
    params_.kinematics.cartesian_axes[0], 0, cartesian_state_current_.positions[0]);
  assign_value_depending_on_input(
    msg->transforms[0].translation.y, msg->velocities[0].linear.y,
    params_.kinematics.cartesian_axes[1], 1, cartesian_state_current_.positions[1]);
  assign_value_depending_on_input(
    msg->transforms[0].translation.z, msg->velocities[0].linear.z,
    params_.kinematics.cartesian_axes[2], 2, cartesian_state_current_.positions[2]);
  assign_value_depending_on_input(
    msg->transforms[0].rotation.x, msg->velocities[0].angular.x,
    params_.kinematics.cartesian_axes[3], 3, cartesian_state_current_.positions[3]);
  assign_value_depending_on_input(
    msg->transforms[0].rotation.y, msg->velocities[0].angular.y,
    params_.kinematics.cartesian_axes[4], 4, cartesian_state_current_.positions[4]);
  assign_value_depending_on_input(
    msg->transforms[0].rotation.z, msg->velocities[0].angular.z,
    params_.kinematics.cartesian_axes[5], 5, cartesian_state_current_.positions[5]);

  // Store the new trajectory message for later use
  new_cartesian_trajectory_msg_.writeFromNonRT(new_traj_msg);
}

controller_interface::CallbackReturn CartesianTrajectoryGenerator::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  // call the base class on_activate()
  auto ret = joint_trajectory_controller::JointTrajectoryController::on_activate(previous_state);
  if (ret != CallbackReturn::SUCCESS)
  {
    return ret;
  }

  current_cartesian_trajectory_ = std::make_shared<joint_trajectory_controller::Trajectory>();

  // Store the current state as new trajectory message, so the first update() cycle will hold the
  // same pose as the current one
  read_cartesian_state_from_feedback(cartesian_state_current_, *(feedback_.readFromNonRT()));
  auto hold_traj_msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
  hold_traj_msg->joint_names = params_.kinematics.cartesian_axes;
  hold_traj_msg->points.resize(1);
  hold_traj_msg->points[0].positions = cartesian_state_current_.positions;
  hold_traj_msg->points[0].velocities.assign(params_.kinematics.cartesian_axes.size(), 0.0);
  hold_traj_msg->points[0].time_from_start = rclcpp::Duration::from_seconds(0.0);
  new_cartesian_trajectory_msg_.writeFromNonRT(hold_traj_msg);

  return CallbackReturn::SUCCESS;
}

void CartesianTrajectoryGenerator::read_cartesian_state_from_feedback(
  JointTrajectoryPoint & cartesian_state,
  const std::shared_ptr<ControllerFeedbackMsg> & measured_state)
{
  std::array<double, 3> orientation_angles;
  tf2::Quaternion measured_q;
  tf2::fromMsg(measured_state->pose.pose.orientation, measured_q);
  tf2::Matrix3x3 m(measured_q);
  m.getRPY(orientation_angles[0], orientation_angles[1], orientation_angles[2]);

  // Assign values from the hardware
  // Position states always exist
  cartesian_state.positions[0] = measured_state->pose.pose.position.x;
  cartesian_state.positions[1] = measured_state->pose.pose.position.y;
  cartesian_state.positions[2] = measured_state->pose.pose.position.z;
  cartesian_state.positions[3] = orientation_angles[0];
  cartesian_state.positions[4] = orientation_angles[1];
  cartesian_state.positions[5] = orientation_angles[2];

  cartesian_state.velocities[0] = measured_state->twist.twist.linear.x;
  cartesian_state.velocities[1] = measured_state->twist.twist.linear.y;
  cartesian_state.velocities[2] = measured_state->twist.twist.linear.z;
  cartesian_state.velocities[3] = measured_state->twist.twist.angular.x;
  cartesian_state.velocities[4] = measured_state->twist.twist.angular.y;
  cartesian_state.velocities[5] = measured_state->twist.twist.angular.z;

  cartesian_state.accelerations.clear();
}

controller_interface::return_type CartesianTrajectoryGenerator::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // Check if a new trajectory message has been received from Non-RT threads
  const auto current_cartesian_trajectory_msg = current_cartesian_trajectory_->get_trajectory_msg();
  auto new_cartesian_external_msg = new_cartesian_trajectory_msg_.readFromRT();

  // Update the current cartesian trajectory with the new message, but only if it actually
  // changed. Trajectory::update() unconditionally resets the Ruckig smoother state and sampling
  if (current_cartesian_trajectory_msg != *new_cartesian_external_msg)
  {
    current_cartesian_trajectory_->update(
      *new_cartesian_external_msg, cartesian_joint_limits_, period);
  }

  // current joints state update (joint space)
  joint_state_current_.time_from_start.sec = 0;
  joint_state_current_.time_from_start.nanosec = 0;
  read_state_from_state_interfaces(joint_state_current_);

  // Current cartesian update from the odometry feedback
  read_cartesian_state_from_feedback(cartesian_state_current_, *(feedback_.readFromRT()));

  // Guards against empty trajectory messages
  if (has_active_cartesian_trajectory())
  {
    // if sampling the first time, set the point before you sample. Also (re)sync the paused
    // clock to real time
    if (!current_cartesian_trajectory_->is_sampled_already())
    {
      cartesian_trajectory_time_ = time;
      current_cartesian_trajectory_->set_point_before_trajectory_msg(
        cartesian_trajectory_time_, cartesian_state_current_, {});
    }

    // Sample expected state from the trajectory. Uses cartesian_trajectory_time_, not the real
    // time, so that a held cycle doesn't advance the sampled target
    joint_trajectory_controller::TrajectoryPointConstIter start_it, end_it;
    current_cartesian_trajectory_->sample(
      cartesian_trajectory_time_, interpolation_method_, cartesian_target_, start_it, end_it,
      period, cartesian_joint_limits_, cartesian_splines_state_, cartesian_ruckig_state_,
      cartesian_ruckig_input_state_);

    // Cartesian delta between the smoothed target and the actual current Cartesian pose
    std::vector<double> delta_x(6);
    for (size_t i = 0; i < 3; ++i)
    {
      // Simple substraction for translations
      delta_x[i] = cartesian_target_.positions[i] - cartesian_state_current_.positions[i];
    }
    for (size_t i = 3; i < 6; ++i)
    {
      // compute shortest path between angles for rotations
      delta_x[i] = angles::shortest_angular_distance(
        cartesian_state_current_.positions[i], cartesian_target_.positions[i]);
    }

    // Safety check: if the raw Cartesian step itself is unreasonable before converting to joint
    // We hold the same position in that case
    for (size_t i = 0; i < 6; ++i)
    {
      if (!cartesian_joint_limits_[i].has_velocity_limits)
      {
        continue;
      }
      const double max_delta = cartesian_joint_limits_[i].max_velocity * period.seconds();
      if (std::abs(delta_x[i]) > max_delta)
      {
        RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(), *(get_node()->get_clock()), 1000,
          "Cartesian delta for axis '%s' (%f) exceeds the max step allowed this cycle (%f). "
          "Holding position.",
          params_.kinematics.cartesian_axes[i].c_str(), delta_x[i], max_delta);
        return controller_interface::return_type::OK;
      }
    }

    // Convert to a joint-space delta via the kinematics plugin
    std::vector<double> joint_delta(dof_, 0.0);
    if (!kinematics_->convert_cartesian_deltas_to_joint_deltas(
          joint_state_current_.positions, delta_x, params_.kinematics.tip, joint_delta))
    {
      RCLCPP_WARN(
        get_node()->get_logger(),
        "Failed to convert Cartesian reference to joint deltas via IK this cycle.");
      return controller_interface::return_type::OK;
    }

    // Safety check: differential IK is only accurate for small steps, a legitimate, small
    // Cartesian delta can still map to a large joint delta near a kinematic singularity
    // We hold position in that case
    for (size_t i = 0; i < dof_; ++i)
    {
      if (!joint_limits_[i].has_velocity_limits)
      {
        continue;
      }
      const double max_delta = joint_limits_[i].max_velocity * period.seconds();
      if (std::abs(joint_delta[i]) > max_delta)
      {
        RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 1000,
          "IK-converted delta for joint '%s' (%f) exceeds the max step allowed this cycle (%f). "
          "Holding position.",
          params_.joints[i].c_str(), joint_delta[i], max_delta);
        return controller_interface::return_type::OK;
      }
    }

    // Both safety checks passed so the joint command will be written this cycle,
    // so advance the paused clock by one period now
    cartesian_trajectory_time_ += period;

    // Target real joint positions = current real joint positions + the IK-converted delta.
    std::vector<double> target_real_joint_positions(dof_);
    for (size_t i = 0; i < dof_; ++i)
    {
      target_real_joint_positions[i] = joint_state_current_.positions[i] + joint_delta[i];
    }

    // NOTE: joint_state_desired_.velocities (and therefore state_error_.velocities) is temporarily
    // left unset here as it's only ever consumed by the state_publisher, nothing in the actual
    // IK/safety-check/hardware-write pipeline above reads it. Will be added later when needed.

    // Fill joint_state_desired_/state_error_ for accurate publish_state() reporting.
    joint_state_desired_.positions = target_real_joint_positions;
    for (size_t i = 0; i < dof_; ++i)
    {
      compute_error_for_joint(state_error_, i, joint_state_current_, joint_state_desired_);
    }

    // Write to hardware
    for (size_t i = 0; i < num_cmd_joints_; ++i)
    {
      joint_command_interface_[0][i].get().set_value(
        target_real_joint_positions[map_cmd_to_joints_[i]]);
    }
    last_commanded_state_ = joint_state_desired_;
    last_commanded_time_ = time;

    publish_state(
      time, joint_state_desired_, joint_state_current_, state_error_, splines_state_, ruckig_state_,
      ruckig_input_state_);
  }

  return controller_interface::return_type::OK;
}

bool CartesianTrajectoryGenerator::has_active_cartesian_trajectory() const
{
  return current_cartesian_trajectory_ != nullptr &&
         current_cartesian_trajectory_->has_trajectory_msg();
}

}  // namespace cartesian_trajectory_generator

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  cartesian_trajectory_generator::CartesianTrajectoryGenerator,
  controller_interface::ControllerInterface)
