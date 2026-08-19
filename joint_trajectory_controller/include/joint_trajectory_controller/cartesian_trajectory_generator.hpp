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

#include "control_msgs/srv/reset_axis.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp"

using namespace std::chrono_literals;  // NOLINT

namespace cartesian_trajectory_generator
{

// FLAG(review): kinematics_/kinematics_loader_ are declared on the base
// JointTrajectoryController, not here, even though this generator is their only real consumer --
// see the matching FLAG(review) note next to their declaration in joint_trajectory_controller.hpp.
// Moving them onto this class instead would fully isolate the Cartesian/IK feature from the base
// class; deferred for now.
class CartesianTrajectoryGenerator : public joint_trajectory_controller::JointTrajectoryController
{
public:
  CartesianTrajectoryGenerator();

  // controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  using ControllerReferenceMsg = trajectory_msgs::msg::MultiDOFJointTrajectoryPoint;
  using ControllerFeedbackMsg = nav_msgs::msg::Odometry;
  using ControllerModeSrvType = control_msgs::srv::ResetAxis;

protected:
  // Command subscribers and Controller State publisher
  rclcpp::Subscription<ControllerReferenceMsg>::SharedPtr ref_subscriber_ = nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerReferenceMsg>> input_ref_;

  rclcpp::Subscription<ControllerFeedbackMsg>::SharedPtr feedback_subscriber_ = nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerFeedbackMsg>> feedback_;

  std::unordered_map<std::string, realtime_tools::RealtimeBuffer<bool>> use_position_input_;
  rclcpp::Service<ControllerModeSrvType>::SharedPtr reset_axes_service_;

  std::shared_ptr<joint_trajectory_controller::Trajectory> current_cartesian_trajectory_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<trajectory_msgs::msg::JointTrajectory>>
    new_cartesian_trajectory_msg_;
  std::vector<joint_limits::JointLimits> cartesian_joint_limits_;

  //  Separate time variable for cartesian trajectory. Used to pause time on cycles where the robot
  //  does not move. Advances by one period before writing to the hardware
  rclcpp::Time cartesian_trajectory_time_;

  // Dedicated Cartesian-space debug/working points (NOT the inherited splines_state_/
  // ruckig_state_/ruckig_input_state_, which are real-joint-space elsewhere in the base class —
  // reusing them here would be a confusing naming collision even though it'd technically compile).

  // FLAG: check for any cartesian smoothing logic from the JTC later
  trajectory_msgs::msg::JointTrajectoryPoint cartesian_state_current_;
  trajectory_msgs::msg::JointTrajectoryPoint cartesian_target_;
  trajectory_msgs::msg::JointTrajectoryPoint cartesian_splines_state_;
  trajectory_msgs::msg::JointTrajectoryPoint cartesian_ruckig_state_;
  trajectory_msgs::msg::JointTrajectoryPoint cartesian_ruckig_input_state_;

  // Separate joint state variables from the inherited ones. Renamed with joint_ prefix for improved
  // readability
  trajectory_msgs::msg::JointTrajectoryPoint joint_state_current_;
  trajectory_msgs::msg::JointTrajectoryPoint joint_state_desired_;

  // Override the service callback as current_trajectory_ is not modified in this controller
  void query_state_service(
    const std::shared_ptr<control_msgs::srv::QueryTrajectoryState::Request> request,
    std::shared_ptr<control_msgs::srv::QueryTrajectoryState::Response> response) override;

private:
  void reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg);

  // updates the cartesian state with the values received from the feedback
  void read_cartesian_state_from_feedback(
    JointTrajectoryPoint & cartesian_state,
    const std::shared_ptr<ControllerFeedbackMsg> & measured_state);

  bool has_active_cartesian_trajectory() const;
};

}  // namespace cartesian_trajectory_generator

#endif  // JOINT_TRAJECTORY_CONTROLLER__CARTESIAN_TRAJECTORY_GENERATOR_HPP_
