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

//
// Source of this file are templates in
// [RosTeamWorkspace](https://github.com/StoglRobotics/ros_team_workspace) repository.
//

#include "test_io_gripper_controller.hpp"
#include "rclcpp/rclcpp.hpp"

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>


// Test open gripper service sets command its as expected and publishes msg
TEST_F(IOGripperControllerTest, OpenCloseGripperAction)
{
  SetUpController();

  setup_parameters();

  controller_->get_node()->set_parameter({"use_action", true});

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());
  executor.add_node(subscription_caller_node_->get_node_base_interface());
  executor.add_node(action_caller_node_->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  auto goal = std::make_shared<GripperAction::Goal>();

  bool wait_for_action_ret = gripper_action_client_->wait_for_action_server(std::chrono::milliseconds(500));
  EXPECT_TRUE(wait_for_action_ret);
  if (!wait_for_action_ret)
  {
    throw std::runtime_error("Action server is not available!");
  }

  goal->open = true;

  gripper_action_client_->async_send_goal(*goal);

  executor.spin_some(std::chrono::milliseconds(5000));

  ASSERT_EQ(
  controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
  controller_interface::return_type::OK); 

  ASSERT_EQ(*(controller_->service_buffer_.readFromRT()), io_gripper_controller::service_mode_type::OPEN);
  ASSERT_EQ(*(controller_->gripper_state_buffer_.readFromRT()), io_gripper_controller::gripper_state_type::OPEN_GRIPPER);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK); 

  ASSERT_EQ(*(controller_->service_buffer_.readFromRT()), io_gripper_controller::service_mode_type::OPEN);
  ASSERT_EQ(*(controller_->gripper_state_buffer_.readFromRT()), io_gripper_controller::gripper_state_type::CHECK_GRIPPER_STATE);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK); 

  ASSERT_EQ(*(controller_->service_buffer_.readFromRT()), io_gripper_controller::service_mode_type::OPEN);
  ASSERT_EQ(*(controller_->gripper_state_buffer_.readFromRT()), io_gripper_controller::gripper_state_type::SET_AFTER_COMMAND);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK); 

  ASSERT_EQ(*(controller_->service_buffer_.readFromRT()), io_gripper_controller::service_mode_type::IDLE);
  ASSERT_EQ(*(controller_->gripper_state_buffer_.readFromRT()), io_gripper_controller::gripper_state_type::IDLE);

  // update to make sure the publisher value is updated
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK); 
  executor.spin_some(std::chrono::milliseconds(1000)); // this solve the issue related to subscriber not able to get the message
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  // since update doesn't guarantee a published message, republish until received
  int max_sub_check_loop_count = 5;  // max number of tries for pub/sub loop
  while (max_sub_check_loop_count--)
  {
    controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01));
    const auto timeout = std::chrono::milliseconds{5};
    const auto until = subscription_caller_node_->get_clock()->now() + timeout;
    while (!joint_state_sub_msg_ && subscription_caller_node_->get_clock()->now() < until)
    {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
    // check if message has been received
    if (max_sub_check_loop_count == 0)
    {
      break;
    }
  }
  executor.spin_some(std::chrono::milliseconds(1000)); // this solve the issue related to subscriber not able to get the message
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  ASSERT_TRUE(joint_state_sub_msg_);

  ASSERT_EQ(joint_state_sub_msg_->position.size(), 2);
  EXPECT_EQ(joint_state_sub_msg_->position[0], 0.0);

  // now sending action goal to close the gripper
  goal->open = false;

  gripper_action_client_->async_send_goal(*goal);

  executor.spin_some(std::chrono::milliseconds(5000));

  ASSERT_EQ(
  controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
  controller_interface::return_type::OK); 

  ASSERT_EQ(*(controller_->service_buffer_.readFromRT()), io_gripper_controller::service_mode_type::CLOSE);
  ASSERT_EQ(*(controller_->gripper_state_buffer_.readFromRT()), io_gripper_controller::gripper_state_type::CLOSE_GRIPPER);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK); 

  ASSERT_EQ(*(controller_->service_buffer_.readFromRT()), io_gripper_controller::service_mode_type::CLOSE);
  ASSERT_EQ(*(controller_->gripper_state_buffer_.readFromRT()), io_gripper_controller::gripper_state_type::CHECK_GRIPPER_STATE);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK); 

  ASSERT_EQ(*(controller_->service_buffer_.readFromRT()), io_gripper_controller::service_mode_type::CLOSE);
  ASSERT_EQ(*(controller_->gripper_state_buffer_.readFromRT()), io_gripper_controller::gripper_state_type::SET_AFTER_COMMAND);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK); 

  ASSERT_EQ(*(controller_->service_buffer_.readFromRT()), io_gripper_controller::service_mode_type::IDLE);
  ASSERT_EQ(*(controller_->gripper_state_buffer_.readFromRT()), io_gripper_controller::gripper_state_type::IDLE);
  // update to make sure the publisher value is updated
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK); 

  executor.spin_some(std::chrono::milliseconds(1000)); // this solve the issue related to subscriber not able to get the message
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  // since update doesn't guarantee a published message, republish until received
  max_sub_check_loop_count = 5;  // max number of tries for pub/sub loop
  while (max_sub_check_loop_count--)
  {
    controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.1));
    const auto timeout = std::chrono::milliseconds{500};
    const auto until = subscription_caller_node_->get_clock()->now() + timeout;
    while (!joint_state_sub_msg_ && subscription_caller_node_->get_clock()->now() < until)
    {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
    // check if message has been received
    if (max_sub_check_loop_count == 0)
    {
      break;
    }
  }
  executor.spin_some(std::chrono::milliseconds(2000)); // this solve the issue related to subscriber not able to get the message
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  ASSERT_TRUE(joint_state_sub_msg_);

  ASSERT_EQ(joint_state_sub_msg_->position.size(), 2);
  EXPECT_EQ(joint_state_sub_msg_->position[0], 0.08);
}