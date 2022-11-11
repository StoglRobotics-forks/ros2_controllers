// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
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

#ifndef ROS2_ACKERMANN_CONT__ACKERMANN_STEERING_CONTROLLER_ROS2_HPP_
#define ROS2_ACKERMANN_CONT__ACKERMANN_STEERING_CONTROLLER_ROS2_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
// #include "diff_drive_controller/speed_limiter.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "hardware_interface/handle.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_box.h"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"

#include "ros2_ackermann_cont/odometry.h"
// #include "ros2_ackermann_cont/odometry.hpp"
#include "ackermann_steering_controller_ros2_parameters.hpp"
#include "ros2_ackermann_cont/visibility_control.h"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "std_srvs/srv/set_bool.hpp"

#include "tf2_msgs/msg/tf_message.hpp"
#include "nav_msgs/msg/odometry.hpp"



namespace ros2_ackermann_cont{

static constexpr size_t NR_CMD_ITFS = 2;
static constexpr size_t NR_STATE_ITFS = 2;
static constexpr size_t NR_REF_ITFS = 2;

class AckermannSteeringControllerRos2 : public controller_interface::ChainableControllerInterface
{
  using Twist = geometry_msgs::msg::TwistStamped;

public:
  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  AckermannSteeringControllerRos2();

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  controller_interface::return_type update_reference_from_subscribers() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // TODO(anyone): replace the state and command message types
  using ControllerReferenceMsg = geometry_msgs::msg::TwistStamped;
  //using ControllerModeSrvType = std_srvs::srv::SetBool;
  using ControllerStateMsgOdom = nav_msgs::msg::Odometry;
  using ControllerStateMsgTf = tf2_msgs::msg::TFMessage;

  std::shared_ptr<ackermann_steering_controller_ros2::ParamListener> param_listener_;
  ackermann_steering_controller_ros2::Params params_;

protected:
  // std::shared_ptr<ackermann_steering_controller_ros2::ParamListener> param_listener_;
  // ackermann_steering_controller_ros2::Params params_;

  // Command subscribers and Controller State publisher
  rclcpp::Subscription<ControllerReferenceMsg>::SharedPtr velocity_command_subscriber_ = nullptr;
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

  struct WheelHandle
  {
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> feedback;
    // std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity;
  };

  std::vector<WheelHandle> registered_rear_wheel_handles_;
  std::vector<WheelHandle> registered_front_wheel_handles_;

  /// Odometry related:
  rclcpp::Duration publish_period_ = rclcpp::Duration::from_nanoseconds(0);
  bool open_loop_;
  rclcpp::Time previous_publish_timestamp_{0};

  /// Velocity command related:
  struct Commands
  {
    double lin;
    double ang;
    rclcpp::Time stamp;

    Commands() : lin(0.0), ang(0.0), stamp(0.0) {}
  };

  // Odometry related:
  Odometry odometry_;

private:
  // callback for topic interface
  TEMPLATES__ROS2_CONTROL__VISIBILITY_LOCAL
  void reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg);

  /// Frame to use for the robot base:
  std::string base_frame_id_;

  /// Frame to use for odometry and odom tf:
  std::string odom_frame_id_;

  /// Whether to publish odometry to tf or not:
  bool enable_odom_tf_;

  // store last velocity
  double last_linear_velocity_;
  double last_angular_velocity_;

};  // namespace ros2_ackermann_cont
}
#endif  // ROS2_ACKERMANN_CONT__ACKERMANN_STEERING_CONTROLLER_ROS2_HPP_
