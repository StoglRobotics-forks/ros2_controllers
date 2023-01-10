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

#ifndef TEST_MECANUM_DRIVE_CONTROLLER_HPP_
#define TEST_MECANUM_DRIVE_CONTROLLER_HPP_

#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "gmock/gmock.h"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "mecanum_drive_controller/mecanum_drive_controller.hpp"
#include "rclcpp/parameter_value.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

using ControllerStateMsg = mecanum_drive_controller::MecanumDriveController::ControllerStateMsg;
using ControllerReferenceMsg =
  mecanum_drive_controller::MecanumDriveController::ControllerReferenceMsg;
using TfStateMsg = mecanum_drive_controller::MecanumDriveController::TfStateMsg;
using OdomStateMsg = mecanum_drive_controller::MecanumDriveController::OdomStateMsg;

namespace
{
constexpr auto NODE_SUCCESS = controller_interface::CallbackReturn::SUCCESS;
constexpr auto NODE_ERROR = controller_interface::CallbackReturn::ERROR;
}  // namespace
// namespace

// subclassing and friending so we can access member variables
class TestableMecanumDriveController : public mecanum_drive_controller::MecanumDriveController
{
  FRIEND_TEST(MecanumDriveControllerTest, all_parameters_set_configure_success);
  FRIEND_TEST(MecanumDriveControllerTest, check_exported_intefaces);
  FRIEND_TEST(MecanumDriveControllerTest, activate_success);
  FRIEND_TEST(MecanumDriveControllerTest, update_success);
  FRIEND_TEST(MecanumDriveControllerTest, deactivate_success);
  FRIEND_TEST(MecanumDriveControllerTest, reactivate_success);
  FRIEND_TEST(MecanumDriveControllerTest, publish_status_success);
  FRIEND_TEST(MecanumDriveControllerTest, receive_message_and_publish_updated_status);
  FRIEND_TEST(MecanumDriveControllerTest, test_sending_too_old_message);
  FRIEND_TEST(MecanumDriveControllerTest, test_time_stamp_zero);
  FRIEND_TEST(MecanumDriveControllerTest, test_message_accepted);
  FRIEND_TEST(MecanumDriveControllerTest, test_update_logic_chainable);
  FRIEND_TEST(MecanumDriveControllerTest, test_update_logic);
  FRIEND_TEST(MecanumDriveControllerTest, test_ref_timeout_zero_for_update);
  FRIEND_TEST(MecanumDriveControllerTest, test_ref_timeout_zero_for_reference_callback);

public:
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override
  {
    auto ret = mecanum_drive_controller::MecanumDriveController::on_configure(previous_state);
    // Only if on_configure is successful create subscription
    if (ret == CallbackReturn::SUCCESS)
    {
      ref_subscriber_wait_set_.add_subscription(ref_subscriber_);
    }
    return ret;
  }

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override
  {
    auto ref_itfs = on_export_reference_interfaces();
    return mecanum_drive_controller::MecanumDriveController::on_activate(previous_state);
  }

  /**
   * @brief wait_for_command blocks until a new ControllerReferenceMsg is received.
   * Requires that the executor is not spinned elsewhere between the
   *  message publication and the call to this function.
   *
   * @return true if new ControllerReferenceMsg msg was received, false if timeout.
   */
  bool wait_for_command(
    rclcpp::Executor & executor, rclcpp::WaitSet & subscriber_wait_set,
    const std::chrono::milliseconds & timeout = std::chrono::milliseconds{500})
  {
    bool success = subscriber_wait_set.wait(timeout).kind() == rclcpp::WaitResultKind::Ready;
    if (success)
    {
      executor.spin_some();
    }
    return success;
  }

  bool wait_for_commands(
    rclcpp::Executor & executor,
    const std::chrono::milliseconds & timeout = std::chrono::milliseconds{500})
  {
    return wait_for_command(executor, ref_subscriber_wait_set_, timeout);
  }

private:
  rclcpp::WaitSet ref_subscriber_wait_set_;
};

// We are using template class here for easier reuse of Fixture in specializations of controllers
template <typename CtrlType>
class MecanumDriveControllerFixture : public ::testing::Test
{
public:
  static void SetUpTestCase() {}

  void SetUp()
  {
    // initialize controller
    controller_ = std::make_unique<CtrlType>();

    command_publisher_node_ = std::make_shared<rclcpp::Node>("command_publisher");
    command_publisher_ = command_publisher_node_->create_publisher<ControllerReferenceMsg>(
      "/test_mecanum_drive_controller/reference", rclcpp::SystemDefaultsQoS());

    odom_s_publisher_node_ = std::make_shared<rclcpp::Node>("odom_s_publisher");
    odom_s_publisher_ = odom_s_publisher_node_->create_publisher<OdomStateMsg>(
      "/test_mecanum_drive_controller/odometry", rclcpp::SystemDefaultsQoS());

    tf_odom_s_publisher_node_ = std::make_shared<rclcpp::Node>("tf_odom_s_publisher");
    tf_odom_s_publisher_ = tf_odom_s_publisher_node_->create_publisher<TfStateMsg>(
      "/test_mecanum_drive_controller/tf_odometry", rclcpp::SystemDefaultsQoS());
  }

  static void TearDownTestCase() {}

  void TearDown() { controller_.reset(nullptr); }

protected:
  void SetUpController(const std::string controller_name = "test_mecanum_drive_controller")
  {
    ASSERT_EQ(controller_->init(controller_name), controller_interface::return_type::OK);

    std::vector<hardware_interface::LoanedCommandInterface> command_ifs;
    command_itfs_.reserve(joint_command_values_.size());
    command_ifs.reserve(joint_command_values_.size());

    for (size_t i = 0; i < joint_command_values_.size(); ++i)
    {
      command_itfs_.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], interface_name_, &joint_command_values_[i]));
      command_ifs.emplace_back(command_itfs_.back());
    }

    std::vector<hardware_interface::LoanedStateInterface> state_ifs;
    state_itfs_.reserve(joint_state_values_.size());
    state_ifs.reserve(joint_state_values_.size());

    for (size_t i = 0; i < joint_state_values_.size(); ++i)
    {
      state_itfs_.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], interface_name_, &joint_state_values_[i]));
      state_ifs.emplace_back(state_itfs_.back());
    }

    controller_->assign_interfaces(std::move(command_ifs), std::move(state_ifs));
  }

  void subscribe_and_get_messages(ControllerStateMsg & msg)
  {
    // create a new subscriber
    rclcpp::Node test_subscription_node("test_subscription_node");
    auto subs_callback = [&](const ControllerStateMsg::SharedPtr) {};
    auto subscription = test_subscription_node.create_subscription<ControllerStateMsg>(
      "/test_mecanum_drive_controller/controller_state", 10, subs_callback);
    // call update to publish the test value
    ASSERT_EQ(
      controller_->update_reference_from_subscribers(
        controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
      controller_interface::return_type::OK);
    ASSERT_EQ(
      controller_->update_and_write_commands(
        controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
      controller_interface::return_type::OK);
    // call update to publish the test value
    // since update doesn't guarantee a published message, republish until received
    int max_sub_check_loop_count = 5;  // max number of tries for pub/sub loop
    rclcpp::WaitSet wait_set;          // block used to wait on message
    wait_set.add_subscription(subscription);
    while (max_sub_check_loop_count--)
    {
      controller_->update_reference_from_subscribers(
        controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01));
      controller_->update_and_write_commands(
        controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01));
      // check if message has been received
      if (wait_set.wait(std::chrono::milliseconds(2)).kind() == rclcpp::WaitResultKind::Ready)
      {
        break;
      }
    }
    ASSERT_GE(max_sub_check_loop_count, 0) << "Test was unable to publish a message through "
                                              "controller/broadcaster update loop";

    // take message from subscription
    rclcpp::MessageInfo msg_info;
    ASSERT_TRUE(subscription->take(msg, msg_info));
  }

  void publish_commands(
    const rclcpp::Time & stamp, const double & twist_linear_x = 1.5,
    const double & twist_linear_y = 0.0, const double & twist_angular_z = 0.0)
  {
    auto wait_for_topic = [&](const auto topic_name)
    {
      size_t wait_count = 0;
      while (command_publisher_node_->count_subscribers(topic_name) == 0)
      {
        if (wait_count >= 5)
        {
          auto error_msg =
            std::string("publishing to ") + topic_name + " but no node subscribes to it";
          throw std::runtime_error(error_msg);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        ++wait_count;
      }
    };

    wait_for_topic(command_publisher_->get_topic_name());

    ControllerReferenceMsg msg;
    msg.header.stamp = stamp;
    msg.twist.linear.x = twist_linear_x;
    msg.twist.linear.y = twist_linear_y;
    msg.twist.linear.z = std::numeric_limits<double>::quiet_NaN();
    msg.twist.angular.x = std::numeric_limits<double>::quiet_NaN();
    msg.twist.angular.y = std::numeric_limits<double>::quiet_NaN();
    msg.twist.angular.z = twist_angular_z;

    command_publisher_->publish(msg);
  }

protected:
  std::vector<std::string> joint_names_ = {
    "front_left_wheel_joint", "back_left_wheel_joint", "back_right_wheel_joint",
    "front_right_wheel_joint"};
  std::vector<std::string> state_joint_names_ = {
    "state_front_left_wheel_joint", "state_back_left_wheel_joint", "state_back_right_wheel_joint",
    "state_front_right_wheel_joint"};
  std::string interface_name_ = "velocity";

  // Controller-related parameters

  std::array<double, 4> joint_state_values_ = {0.1, 0.1, 0.1, 0.1};
  std::array<double, 4> joint_command_values_ = {101.101, 101.101, 101.101, 101.101};

  std::vector<hardware_interface::StateInterface> state_itfs_;
  std::vector<hardware_interface::CommandInterface> command_itfs_;

  double ref_timeout_ = 0.2;

  // Test related parameters
  std::unique_ptr<TestableMecanumDriveController> controller_;
  rclcpp::Node::SharedPtr command_publisher_node_;
  rclcpp::Publisher<ControllerReferenceMsg>::SharedPtr command_publisher_;
  rclcpp::Node::SharedPtr odom_s_publisher_node_;
  rclcpp::Publisher<OdomStateMsg>::SharedPtr odom_s_publisher_;
  rclcpp::Node::SharedPtr tf_odom_s_publisher_node_;
  rclcpp::Publisher<TfStateMsg>::SharedPtr tf_odom_s_publisher_;
};

#endif  // TEST_MECANUM_DRIVE_CONTROLLER_HPP_
