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

#ifndef JOINT_TRAJECTORY_CONTROLLER__CARTESIAN_TRAJECTORY_GENERATOR_HPP_
#define JOINT_TRAJECTORY_CONTROLLER__CARTESIAN_TRAJECTORY_GENERATOR_HPP_

#include "joint_trajectory_controller/joint_trajectory_controller.hpp"

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp"

using namespace std::chrono_literals;  // NOLINT

namespace cartesian_trajectory_generator
{
class CartesianTrajectoryGenerator : public joint_trajectory_controller::JointTrajectoryController
{
public:
  CartesianTrajectoryGenerator();

  /**
   * @brief command_interface_configuration This controller requires the position and velocity
   * state interfaces for the controlled joints
   */
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  using ControllerReferenceMsg = trajectory_msgs::msg::MultiDOFJointTrajectoryPoint;
  using ControllerFeedbackMsg = nav_msgs::msg::Odometry;

protected:
  void read_state_from_state_interfaces(JointTrajectoryPoint & state) override;

  // Command subscribers and Controller State publisher
  rclcpp::Subscription<ControllerReferenceMsg>::SharedPtr ref_subscriber_ = nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerReferenceMsg>> input_ref_;

  rclcpp::Subscription<ControllerFeedbackMsg>::SharedPtr feedback_subscriber_ = nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerFeedbackMsg>> feedback_;

  // NOTE(rebase): originally toggled by a `~/reset_axes` service (control_msgs::srv::ResetAxis)
  // that let a caller switch a named axis from velocity-streaming back to position-hold at the
  // current feedback pose. That service type doesn't exist in jazzy's control_msgs and was never
  // upstreamed, so the service was dropped (see on_configure()). Without it, nothing ever sets an
  // entry of use_position_input_ back to true after on_configure() initializes it to false, so
  // the position-hold branch in reference_callback() is currently unreachable: only velocity-
  // streaming input is functional. Kept here (rather than deleted) since reference_callback()
  // still reads it and re-adding the service later is the natural way to restore position-hold.
  std::unordered_map<std::string, realtime_tools::RealtimeBuffer<bool>> use_position_input_;

private:
  void reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg);
};

}  // namespace cartesian_trajectory_generator

#endif  // JOINT_TRAJECTORY_CONTROLLER__CARTESIAN_TRAJECTORY_GENERATOR_HPP_
