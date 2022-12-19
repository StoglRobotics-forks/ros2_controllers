// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt)
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

#ifndef STEERING_CONTROLLERS__STEERING_CONTROLLERS_HPP_
#define STEERING_CONTROLLERS__STEERING_CONTROLLERS_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "std_srvs/srv/set_bool.hpp"
#include "steering_controllers/steering_odometry.hpp"
#include "steering_controllers/visibility_control.h"
#include "steering_controllers_parameters.hpp"

// TODO(anyone): Replace with controller specific messages
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "control_msgs/msg/steering_controller_status.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace steering_controllers
{
class SteeringControllers : public controller_interface::ChainableControllerInterface
{
public:
  STEERING_CONTROLLERS__VISIBILITY_PUBLIC SteeringControllers();

  STEERING_CONTROLLERS__VISIBILITY_PUBLIC controller_interface::CallbackReturn on_init() override;

  STEERING_CONTROLLERS__VISIBILITY_PUBLIC controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  STEERING_CONTROLLERS__VISIBILITY_PUBLIC controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  virtual STEERING_CONTROLLERS__VISIBILITY_PUBLIC controller_interface::CallbackReturn
  configure_odometry();

  virtual STEERING_CONTROLLERS__VISIBILITY_PUBLIC bool update_odometry(
    const rclcpp::Duration & period);

  STEERING_CONTROLLERS__VISIBILITY_PUBLIC controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  STEERING_CONTROLLERS__VISIBILITY_PUBLIC controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  STEERING_CONTROLLERS__VISIBILITY_PUBLIC controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  STEERING_CONTROLLERS__VISIBILITY_PUBLIC controller_interface::return_type
  update_reference_from_subscribers() override;

  STEERING_CONTROLLERS__VISIBILITY_PUBLIC controller_interface::return_type
  update_and_write_commands(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // TODO(anyone): replace the state and command message types
  using ControllerReferenceMsg = geometry_msgs::msg::TwistStamped;
  using ControllerStateMsgOdom = nav_msgs::msg::Odometry;
  using ControllerStateMsgTf = tf2_msgs::msg::TFMessage;

protected:
  std::shared_ptr<steering_controllers::ParamListener> param_listener_;
  steering_controllers::Params params_;

  // Command subscribers and Controller State publisher
  rclcpp::Subscription<ControllerReferenceMsg>::SharedPtr ref_subscriber_ = nullptr;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr ref_subscriber_unstamped_ = nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerReferenceMsg>> input_ref_;
  rclcpp::Duration ref_timeout_ = rclcpp::Duration::from_seconds(0.0);  // 0ms

  using ControllerStatePublisherOdom = realtime_tools::RealtimePublisher<ControllerStateMsgOdom>;
  using ControllerStatePublisherTf = realtime_tools::RealtimePublisher<ControllerStateMsgTf>;

  rclcpp::Publisher<ControllerStateMsgOdom>::SharedPtr odom_s_publisher_;
  rclcpp::Publisher<ControllerStateMsgTf>::SharedPtr tf_odom_s_publisher_;

  std::unique_ptr<ControllerStatePublisherOdom> rt_odom_state_publisher_;
  std::unique_ptr<ControllerStatePublisherTf> rt_tf_odom_state_publisher_;

  // override methods from ChainableControllerInterface
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  bool on_set_chained_mode(bool chained_mode) override;

  /// Odometry:
  steering_odometry::SteeringOdometry odometry_;

  using AckermanControllerState = control_msgs::msg::SteeringControllerStatus;
  AckermanControllerState published_state_;

  using ControllerStatePublisher = realtime_tools::RealtimePublisher<AckermanControllerState>;
  rclcpp::Publisher<AckermanControllerState>::SharedPtr controller_s_publisher_;
  std::unique_ptr<ControllerStatePublisher> controller_state_publisher_;

  // name constants for state interfaces
  size_t nr_state_itfs_;
  // name constants for command interfaces
  size_t nr_cmd_itfs_;
  // name constants for reference interfaces
  size_t nr_ref_itfs_;

  STEERING_CONTROLLERS__VISIBILITY_PROTECTED controller_interface::CallbackReturn
  set_interface_numbers(size_t nr_state_itfs, size_t nr_cmd_itfs, size_t nr_ref_itfs);

  // store last velocity
  double last_linear_velocity_ = 0.0;
  double last_angular_velocity_ = 0.0;

private:
  // callback for topic interface
  STEERING_CONTROLLERS__VISIBILITY_LOCAL void reference_callback(
    const std::shared_ptr<ControllerReferenceMsg> msg);
  void reference_callback_unstamped(const std::shared_ptr<geometry_msgs::msg::Twist> msg);
};

}  // namespace steering_controllers

#endif  // STEERING_CONTROLLERS__steering_controllers_HPP_