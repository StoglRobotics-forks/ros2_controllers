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

//
// Source of this file are templates in
// [RosTeamWorkspace](https://github.com/StoglRobotics/ros_team_workspace) repository.
//

#include "test_gripper_io_controller.hpp"
#include "rclcpp/rclcpp.hpp"

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>


class GripperIOControllerTest : public GripperIOControllerFixture<TestableGripperIOController>
{
};

// Test setting all params and getting success
TEST_F(GripperIOControllerTest, AllParamsSetSuccess)
{
  SetUpController();

  controller_->get_node()->set_parameter({"open_close_joints", open_close_joints});
  controller_->get_node()->set_parameter({"open.joint_states", open_joint_states});
  controller_->get_node()->set_parameter({"open.set_before_command.high", open_set_before_command_high});
  controller_->get_node()->set_parameter({"open.set_before_command.low", open_set_before_command_low});
  controller_->get_node()->set_parameter({"open.command.high", open_command_high});
  controller_->get_node()->set_parameter({"open.command.low", open_command_low});
  controller_->get_node()->set_parameter({"open.state.high", open_state_high});
  controller_->get_node()->set_parameter({"open.state.low", open_state_low});
  controller_->get_node()->set_parameter({"close.joint_states", close_joint_states});
  controller_->get_node()->set_parameter({"close.set_before_command.high", close_set_before_command_high});
  controller_->get_node()->set_parameter({"close.set_before_command.low", close_set_before_command_low});
  controller_->get_node()->set_parameter({"close.command.high", close_command_high});
  controller_->get_node()->set_parameter({"close.command.low", close_command_low});
  controller_->get_node()->set_parameter({"close.state.high", close_state_high});
  controller_->get_node()->set_parameter({"close.state.low", close_state_low});


  controller_->get_node()->set_parameter({"configurations", configurations_list});
  controller_->get_node()->set_parameter({"configuration_joints", configuration_joints});
  controller_->get_node()->set_parameter({"configuration_setup.stichmass_125.joint_states", stichmass_joint_states});
  controller_->get_node()->set_parameter({"configuration_setup.stickmass_125.command_high", stichmass_command_high});
  controller_->get_node()->set_parameter({"configuration_setup.stickmass_125.command_low", stichmass_command_low});
  controller_->get_node()->set_parameter({"configuration_setup.stickmass_125.state_high", stichmass_state_high});
  controller_->get_node()->set_parameter({"configuration_setup.stick_mass_125.state_low", stichmass_state_low});

  controller_->get_node()->set_parameter({"gripper_specific_sensors", gripper_specific_sensors});
  controller_->get_node()->set_parameter({"sensors_interfaces.hohenabfrage.input", gripper_interfaces_input});


  RCLCPP_INFO(rclcpp::get_logger("test_gripper_io_controller"), "Running test: AllParamSetSuccess");

  // configure success.
  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
}


// Test not setting the one param and getting failure
TEST_F(GripperIOControllerTest, AllParamNotSetFailure)
{
  SetUpController();

  controller_->get_node()->set_parameter({"open_close_joints", ""}); // this will make the test fail
  controller_->get_node()->set_parameter({"open.joint_states", open_joint_states});
  controller_->get_node()->set_parameter({"open.set_before_command.high", open_set_before_command_high});
  controller_->get_node()->set_parameter({"open.set_before_command.low", open_set_before_command_low});
  controller_->get_node()->set_parameter({"open.command.high", open_command_high});
  controller_->get_node()->set_parameter({"open.command.low", open_command_low});
  controller_->get_node()->set_parameter({"open.state.high", open_state_high});
  controller_->get_node()->set_parameter({"open.state.low", open_state_low});
  controller_->get_node()->set_parameter({"close.joint_states", close_joint_states});
  controller_->get_node()->set_parameter({"close.set_before_command.high", close_set_before_command_high});
  controller_->get_node()->set_parameter({"close.set_before_command.low", close_set_before_command_low});
  controller_->get_node()->set_parameter({"close.command.high", close_command_high});
  controller_->get_node()->set_parameter({"close.command.low", close_command_low});
  controller_->get_node()->set_parameter({"close.state.high", close_state_high});
  controller_->get_node()->set_parameter({"close.state.low", close_state_low});


  controller_->get_node()->set_parameter({"configurations", configurations_list});
  controller_->get_node()->set_parameter({"configuration_joints", configuration_joints});
  controller_->get_node()->set_parameter({"configuration_setup.stichmass_125.joint_states", stichmass_joint_states});
  controller_->get_node()->set_parameter({"configuration_setup.stickmass_125.command_high", stichmass_command_high});
  controller_->get_node()->set_parameter({"configuration_setup.stickmass_125.command_low", stichmass_command_low});
  controller_->get_node()->set_parameter({"configuration_setup.stickmass_125.state_high", stichmass_state_high});
  controller_->get_node()->set_parameter({"configuration_setup.stick_mass_125.state_low", stichmass_state_low});

  controller_->get_node()->set_parameter({"gripper_specific_sensors", gripper_specific_sensors});
  controller_->get_node()->set_parameter({"sensors_interfaces.hohenabfrage.input", gripper_interfaces_input});


  RCLCPP_INFO(rclcpp::get_logger("test_gripper_io_controller"), "Running test: AllParamNotSetFailure");


  // configure success. remaining parameters are default
  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::FAILURE);
}


// Test open gripper service sets command its as expected and publishes msg
TEST_F(GripperIOControllerTest, OpenGripperService)
{
  SetUpController();

  controller_->get_node()->set_parameter({"open_close_joints", open_close_joints});
  controller_->get_node()->set_parameter({"open.joint_states", open_joint_states});
  controller_->get_node()->set_parameter({"open.set_before_command.high", open_set_before_command_high});
  controller_->get_node()->set_parameter({"open.set_before_command.low", open_set_before_command_low});
  controller_->get_node()->set_parameter({"open.command.high", open_command_high});
  controller_->get_node()->set_parameter({"open.command.low", open_command_low});
  controller_->get_node()->set_parameter({"open.state.high", open_state_high});
  controller_->get_node()->set_parameter({"open.state.low", open_state_low});
  controller_->get_node()->set_parameter({"close.joint_states", close_joint_states});
  controller_->get_node()->set_parameter({"close.set_before_command.high", close_set_before_command_high});
  controller_->get_node()->set_parameter({"close.set_before_command.low", close_set_before_command_low});
  controller_->get_node()->set_parameter({"close.command.high", close_command_high});
  controller_->get_node()->set_parameter({"close.command.low", close_command_low});
  controller_->get_node()->set_parameter({"close.state.high", close_state_high});
  controller_->get_node()->set_parameter({"close.state.low", close_state_low});


  controller_->get_node()->set_parameter({"configurations", configurations_list});
  controller_->get_node()->set_parameter({"configuration_joints", configuration_joints});
  controller_->get_node()->set_parameter({"configuration_setup.stichmass_125.joint_states", stichmass_joint_states});
  controller_->get_node()->set_parameter({"configuration_setup.stickmass_125.command_high", stichmass_command_high});
  controller_->get_node()->set_parameter({"configuration_setup.stickmass_125.command_low", stichmass_command_low});
  controller_->get_node()->set_parameter({"configuration_setup.stickmass_125.state_high", stichmass_state_high});
  controller_->get_node()->set_parameter({"configuration_setup.stick_mass_125.state_low", stichmass_state_low});

  controller_->get_node()->set_parameter({"gripper_specific_sensors", gripper_specific_sensors});
  controller_->get_node()->set_parameter({"sensors_interfaces.hohenabfrage.input", gripper_interfaces_input});
  

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());
  executor.add_node(service_caller_node_->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  RCLCPP_INFO(rclcpp::get_logger("test_gripper_io_controller"), "Calling open service");
  // std::thread service_thread([this, &executor]() {
  //   call_open_service(executor);
  // });

  // service_thread.detach();

  controller_->service_buffer_.writeFromNonRT(gripper_io_controller::service_mode_type::OPEN);
  controller_->gripper_state_buffer_.writeFromNonRT(gripper_io_controller::gripper_state_type::OPEN_GRIPPER);



  RCLCPP_INFO(rclcpp::get_logger("test_gripper_io_controller"), "done open service");


  RCLCPP_INFO(rclcpp::get_logger("test_gripper_io_controller"), "calling udpate");
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  // get the value of the command itfs and match their values
  ASSERT_EQ(greif_oeffen_wqg1_cmd_.get_value(), 1.0);
  ASSERT_EQ(greif_schliess_wqg2_cmd_.get_value(), 0.0);

  // get the value of the state itfs and match their values
  ASSERT_EQ(greif_geoff_bg01_state_.get_value(), 1.0);
  ASSERT_EQ(greif_geschl_bg02_state_.get_value(), 0.0);
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  
}


// Test close gripper service sets command its as expected and publishes msg
TEST_F(GripperIOControllerTest, CloseGripperService)
{
  SetUpController();

  controller_->get_node()->set_parameter({"open_close_joints", open_close_joints});
  controller_->get_node()->set_parameter({"open.joint_states", open_joint_states});
  controller_->get_node()->set_parameter({"open.set_before_command.high", open_set_before_command_high});
  controller_->get_node()->set_parameter({"open.set_before_command.low", open_set_before_command_low});
  controller_->get_node()->set_parameter({"open.command.high", open_command_high});
  controller_->get_node()->set_parameter({"open.command.low", open_command_low});
  controller_->get_node()->set_parameter({"open.state.high", open_state_high});
  controller_->get_node()->set_parameter({"open.state.low", open_state_low});
  controller_->get_node()->set_parameter({"close.joint_states", close_joint_states});
  controller_->get_node()->set_parameter({"close.set_before_command.high", close_set_before_command_high});
  controller_->get_node()->set_parameter({"close.set_before_command.low", close_set_before_command_low});
  controller_->get_node()->set_parameter({"close.command.high", close_command_high});
  controller_->get_node()->set_parameter({"close.command.low", close_command_low});
  controller_->get_node()->set_parameter({"close.state.high", close_state_high});
  controller_->get_node()->set_parameter({"close.state.low", close_state_low});


  controller_->get_node()->set_parameter({"configurations", configurations_list});
  controller_->get_node()->set_parameter({"configuration_joints", configuration_joints});
  controller_->get_node()->set_parameter({"configuration_setup.stichmass_125.joint_states", stichmass_joint_states});
  controller_->get_node()->set_parameter({"configuration_setup.stickmass_125.command_high", stichmass_command_high});
  controller_->get_node()->set_parameter({"configuration_setup.stickmass_125.command_low", stichmass_command_low});
  controller_->get_node()->set_parameter({"configuration_setup.stickmass_125.state_high", stichmass_state_high});
  controller_->get_node()->set_parameter({"configuration_setup.stick_mass_125.state_low", stichmass_state_low});

  controller_->get_node()->set_parameter({"gripper_specific_sensors", gripper_specific_sensors});
  controller_->get_node()->set_parameter({"sensors_interfaces.hohenabfrage.input", gripper_interfaces_input});
  

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());
  executor.add_node(service_caller_node_->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  RCLCPP_INFO(rclcpp::get_logger("test_gripper_io_controller"), "Calling open service");
  // std::thread service_thread([this, &executor]() {
  //   call_open_service(executor);
  // });

  // service_thread.detach();

  controller_->service_buffer_.writeFromNonRT(gripper_io_controller::service_mode_type::CLOSE);
  controller_->gripper_state_buffer_.writeFromNonRT(gripper_io_controller::gripper_state_type::CLOSE_GRIPPER);



  RCLCPP_INFO(rclcpp::get_logger("test_gripper_io_controller"), "done open service");


  RCLCPP_INFO(rclcpp::get_logger("test_gripper_io_controller"), "calling udpate");
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  // get the value of the command itfs and match their values
  ASSERT_EQ(greif_oeffen_wqg1_cmd_.get_value(), 0.0);
  ASSERT_EQ(greif_schliess_wqg2_cmd_.get_value(), 1.0);

  // get the value of the state itfs and match their values
  ASSERT_EQ(greif_geoff_bg01_state_.get_value(), 0.0);
  ASSERT_EQ(greif_geschl_bg02_state_.get_value(), 1.0);
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  
}


// Test reconfigure gripper service sets command its as expected and publishes msg
TEST_F(GripperIOControllerTest, ReconfigureGripperService)
{
  SetUpController();

  controller_->get_node()->set_parameter({"open_close_joints", open_close_joints});
  controller_->get_node()->set_parameter({"open.joint_states", open_joint_states});
  controller_->get_node()->set_parameter({"open.set_before_command.high", open_set_before_command_high});
  controller_->get_node()->set_parameter({"open.set_before_command.low", open_set_before_command_low});
  controller_->get_node()->set_parameter({"open.command.high", open_command_high});
  controller_->get_node()->set_parameter({"open.command.low", open_command_low});
  controller_->get_node()->set_parameter({"open.state.high", open_state_high});
  controller_->get_node()->set_parameter({"open.state.low", open_state_low});
  controller_->get_node()->set_parameter({"close.joint_states", close_joint_states});
  controller_->get_node()->set_parameter({"close.set_before_command.high", close_set_before_command_high});
  controller_->get_node()->set_parameter({"close.set_before_command.low", close_set_before_command_low});
  controller_->get_node()->set_parameter({"close.command.high", close_command_high});
  controller_->get_node()->set_parameter({"close.command.low", close_command_low});
  controller_->get_node()->set_parameter({"close.state.high", close_state_high});
  controller_->get_node()->set_parameter({"close.state.low", close_state_low});


  controller_->get_node()->set_parameter({"configurations", configurations_list});
  controller_->get_node()->set_parameter({"configuration_joints", configuration_joints});
  controller_->get_node()->set_parameter({"configuration_setup.stichmass_125.joint_states", stichmass_joint_states});
  controller_->get_node()->set_parameter({"configuration_setup.stickmass_125.command_high", stichmass_command_high});
  controller_->get_node()->set_parameter({"configuration_setup.stickmass_125.command_low", stichmass_command_low});
  controller_->get_node()->set_parameter({"configuration_setup.stickmass_125.state_high", stichmass_state_high});
  controller_->get_node()->set_parameter({"configuration_setup.stick_mass_125.state_low", stichmass_state_low});

  controller_->get_node()->set_parameter({"gripper_specific_sensors", gripper_specific_sensors});
  controller_->get_node()->set_parameter({"sensors_interfaces.hohenabfrage.input", gripper_interfaces_input});

  // configure success.
  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());
  executor.add_node(service_caller_node_->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  RCLCPP_INFO(rclcpp::get_logger("test_gripper_io_controller"), "Calling open service");
  // std::thread service_thread([this, &executor]() {
  //   call_open_service(executor);
  // });

  // service_thread.detach();

  controller_->reconfigure_state_buffer_.writeFromNonRT(gripper_io_controller::reconfigure_state_type::RECONFIGURE);
  controller_->configure_gripper_buffer_.writeFromNonRT("stichmass_125");
  controller_->reconfigureFlag_ = true;




  RCLCPP_INFO(rclcpp::get_logger("test_gripper_io_controller"), "done open service");


  RCLCPP_INFO(rclcpp::get_logger("test_gripper_io_controller"), "calling udpate");
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  // get the value of the command itfs and match their values
  ASSERT_EQ(stich_125_wqg5_cmd_.get_value(), 1.0);
  ASSERT_EQ(stich_250_wqg6_cmd_.get_value(), 0.0);

  // get the value of the state itfs and match their values
  ASSERT_EQ(stich_125_bg03_state_.get_value(), 1.0);
  ASSERT_EQ(stich_250_bg04_state_.get_value(), 0.0);
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  
}


// Another tests

// 1. Opening the Gripper with service call
// 2. Closing the Gripper with service call
// 3. Reconfiguring the Gripper with service call

// // Test not setting the closing command ios and getting failure
// TEST_F(GripperIOControllerTest, ClosingCommandsParametersNotSet)
// {
//   SetUpController();

//   std::vector<std::string> opening_high = {"gripper_joint"};
//   std::vector<std::string> opening_low = {"finger_joint"};
//   std::vector<std::string> closing_high = {};
//   std::vector<std::string> closing_low = {};
//   std::vector<std::string> opened_high = {"gripper_joint"};
//   std::vector<std::string> opened_low = {};
//   std::vector<std::string> closed_high = {};
//   std::vector<std::string> closed_low = {"gripper_joint"};

//   controller_->get_node()->set_parameter({"opening_ios.high",  opening_high});
//   controller_->get_node()->set_parameter({"opening_ios.low",  opening_low});
//   controller_->get_node()->set_parameter({"closing_ios.high",  closing_high});
//   controller_->get_node()->set_parameter({"closing_ios.low",  closing_low});
//   controller_->get_node()->set_parameter({"opened_states.high",  opened_high});
//   controller_->get_node()->set_parameter({"opened_states.low",  opened_low});
//   controller_->get_node()->set_parameter({"closed_states.high",  closed_high});
//   controller_->get_node()->set_parameter({"closed_states.low",  closed_low});


//   // configure success. remaining parameters are default
//   ASSERT_EQ(
//     controller_->on_configure(rclcpp_lifecycle::State()),
//     controller_interface::CallbackReturn::FAILURE);
// }

// // Test setting the closing and opening command ios to different things
// TEST_F(GripperIOControllerTest, DifferentCommandsParametersSet)
// {
//   SetUpController();

//   std::vector<std::string> opening_high = {"joint1"};
//   std::vector<std::string> opening_low = {"finger_joint"};
//   std::vector<std::string> closing_high = {};
//   std::vector<std::string> closing_low = {"gripper_joint", "finger_joint"};
//   std::vector<std::string> opened_high = {"gripper_joint"};
//   std::vector<std::string> opened_low = {};
//   std::vector<std::string> closed_high = {};
//   std::vector<std::string> closed_low = {"gripper_joint"};

//   controller_->get_node()->set_parameter({"opening_ios.high",  opening_high});
//   controller_->get_node()->set_parameter({"opening_ios.low",  opening_low});
//   controller_->get_node()->set_parameter({"closing_ios.high",  closing_high});
//   controller_->get_node()->set_parameter({"closing_ios.low",  closing_low});
//   controller_->get_node()->set_parameter({"opened_states.high",  opened_high});
//   controller_->get_node()->set_parameter({"opened_states.low",  opened_low});
//   controller_->get_node()->set_parameter({"closed_states.high",  closed_high});
//   controller_->get_node()->set_parameter({"closed_states.low",  closed_low});


//   // configure success. remaining parameters are default
//   ASSERT_EQ(
//     controller_->on_configure(rclcpp_lifecycle::State()),
//     controller_interface::CallbackReturn::FAILURE);
// }

// // Test not setting the opened states
// TEST_F(GripperIOControllerTest, OpenedStatesParametersNotSet)
// {
//   SetUpController();

//   std::vector<std::string> opening_high = {"gripper_joint"};
//   std::vector<std::string> opening_low = {"finger_joint"};
//   std::vector<std::string> closing_high = {};
//   std::vector<std::string> closing_low = {"gripper_joint", "finger_joint"};
//   std::vector<std::string> opened_high = {};
//   std::vector<std::string> opened_low = {};
//   std::vector<std::string> closed_high = {};
//   std::vector<std::string> closed_low = {"gripper_joint"};

//   controller_->get_node()->set_parameter({"opening_ios.high",  opening_high});
//   controller_->get_node()->set_parameter({"opening_ios.low",  opening_low});
//   controller_->get_node()->set_parameter({"closing_ios.high",  closing_high});
//   controller_->get_node()->set_parameter({"closing_ios.low",  closing_low});
//   controller_->get_node()->set_parameter({"opened_states.high",  opened_high});
//   controller_->get_node()->set_parameter({"opened_states.low",  opened_low});
//   controller_->get_node()->set_parameter({"closed_states.high",  closed_high});
//   controller_->get_node()->set_parameter({"closed_states.low",  closed_low});


//   // configure success. remaining parameters are default
//   ASSERT_EQ(
//     controller_->on_configure(rclcpp_lifecycle::State()),
//     controller_interface::CallbackReturn::FAILURE);
// }

// // Test not setting the closed states
// TEST_F(GripperIOControllerTest, ClosedStatesParametersNotSet)
// {
//   SetUpController();

//   std::vector<std::string> opening_high = {"gripper_joint"};
//   std::vector<std::string> opening_low = {"finger_joint"};
//   std::vector<std::string> closing_high = {};
//   std::vector<std::string> closing_low = {"gripper_joint", "finger_joint"};
//   std::vector<std::string> opened_high = {"gripper_joint"};
//   std::vector<std::string> opened_low = {};
//   std::vector<std::string> closed_high = {};
//   std::vector<std::string> closed_low = {};

//   controller_->get_node()->set_parameter({"opening_ios.high",  opening_high});
//   controller_->get_node()->set_parameter({"opening_ios.low",  opening_low});
//   controller_->get_node()->set_parameter({"closing_ios.high",  closing_high});
//   controller_->get_node()->set_parameter({"closing_ios.low",  closing_low});
//   controller_->get_node()->set_parameter({"opened_states.high",  opened_high});
//   controller_->get_node()->set_parameter({"opened_states.low",  opened_low});
//   controller_->get_node()->set_parameter({"closed_states.high",  closed_high});
//   controller_->get_node()->set_parameter({"closed_states.low",  closed_low});


//   // configure success. remaining parameters are default
//   ASSERT_EQ(
//     controller_->on_configure(rclcpp_lifecycle::State()),
//     controller_interface::CallbackReturn::FAILURE);
// }

// // Test setting the opened and closed states to different things
// TEST_F(GripperIOControllerTest, DifferentStatesParametersNotSet)
// {
//   SetUpController();

//   std::vector<std::string> opening_high = {"gripper_joint"};
//   std::vector<std::string> opening_low = {"finger_joint"};
//   std::vector<std::string> closing_high = {};
//   std::vector<std::string> closing_low = {"gripper_joint", "finger_joint"};
//   std::vector<std::string> opened_high = {"gripper_joint"};
//   std::vector<std::string> opened_low = {};
//   std::vector<std::string> closed_high = {};
//   std::vector<std::string> closed_low = {"joint1"};

//   controller_->get_node()->set_parameter({"opening_ios.high",  opening_high});
//   controller_->get_node()->set_parameter({"opening_ios.low",  opening_low});
//   controller_->get_node()->set_parameter({"closing_ios.high",  closing_high});
//   controller_->get_node()->set_parameter({"closing_ios.low",  closing_low});
//   controller_->get_node()->set_parameter({"opened_states.high",  opened_high});
//   controller_->get_node()->set_parameter({"opened_states.low",  opened_low});
//   controller_->get_node()->set_parameter({"closed_states.high",  closed_high});
//   controller_->get_node()->set_parameter({"closed_states.low",  closed_low});


//   // configure success. remaining parameters are default
//   ASSERT_EQ(
//     controller_->on_configure(rclcpp_lifecycle::State()),
//     controller_interface::CallbackReturn::FAILURE);
// }

// TEST_F(GripperIOControllerTest, all_parameters_set_configure_success)
// {
//   SetUpController();

//   ASSERT_TRUE(controller_->params_.opening_ios.high.empty());
//   ASSERT_TRUE(controller_->params_.opening_ios.low.empty());
//   ASSERT_TRUE(controller_->params_.closing_ios.high.empty());
//   ASSERT_TRUE(controller_->params_.closing_ios.low.empty());
//   ASSERT_TRUE(controller_->params_.opened_states.high.empty());
//   ASSERT_TRUE(controller_->params_.opened_states.low.empty());
//   ASSERT_TRUE(controller_->params_.closed_states.high.empty());
//   ASSERT_TRUE(controller_->params_.closed_states.low.empty());
//   ASSERT_TRUE(controller_->params_.status_joint_name.empty());

//   std::vector<std::string> opening_high = {"gripper_joint"};
//   std::vector<std::string> opening_low = {"finger_joint"};
//   std::vector<std::string> closing_high = {};
//   std::vector<std::string> closing_low = {"gripper_joint", "finger_joint"};
//   std::vector<std::string> opened_high = {"gripper_joint"};
//   std::vector<std::string> opened_low = {};
//   std::vector<std::string> closed_high = {};
//   std::vector<std::string> closed_low = {"gripper_joint"};

//   controller_->get_node()->set_parameter({"opening_ios.high",  opening_high});
//   controller_->get_node()->set_parameter({"opening_ios.low",  opening_low});
//   controller_->get_node()->set_parameter({"closing_ios.high",  closing_high});
//   controller_->get_node()->set_parameter({"closing_ios.low",  closing_low});
//   controller_->get_node()->set_parameter({"opened_states.high",  opened_high});
//   controller_->get_node()->set_parameter({"opened_states.low",  opened_low});
//   controller_->get_node()->set_parameter({"closed_states.high",  closed_high});
//   controller_->get_node()->set_parameter({"closed_states.low",  closed_low});


//   // configure success. remaining parameters are default
//   ASSERT_EQ(
//     controller_->on_configure(rclcpp_lifecycle::State()),
//     controller_interface::CallbackReturn::SUCCESS);
//   ASSERT_EQ(controller_->params_.interface_name, interface_name_);
//   ASSERT_EQ(controller_->params_.joint_value_opened, joint_value_opened_);
//   ASSERT_EQ(controller_->params_.joint_value_closed, joint_value_closed_);

//   ASSERT_EQ(controller_->params_.opening_ios.high.size() + controller_->params_.opening_ios.low.size(), 
//     controller_->params_.closing_ios.high.size() + controller_->params_.closing_ios.low.size());
//   ASSERT_EQ(controller_->command_ios.size(), controller_->command_opening.size());
//   ASSERT_EQ(controller_->command_ios.size(), controller_->command_closing.size());

//   ASSERT_EQ(controller_->params_.opened_states.high.size() + controller_->params_.opened_states.low.size(), 
//     controller_->params_.closed_states.high.size() + controller_->params_.closed_states.low.size());
//   ASSERT_EQ(controller_->state_ios.size(), controller_->state_opened.size());
//   ASSERT_EQ(controller_->state_ios.size(), controller_->state_closed.size());
// }

// // Test open gripper service sets command its as expected and publishes msg
// TEST_F(GripperIOControllerTest, OpenGripperService)
// {
//   SetUpController();

//   std::vector<std::string> opening_high = {"gripper_joint"};
//   std::vector<std::string> opening_low = {"finger_joint"};
//   std::vector<std::string> closing_high = {};
//   std::vector<std::string> closing_low = {"gripper_joint", "finger_joint"};
//   std::vector<std::string> opened_high = {"gripper_joint"};
//   std::vector<std::string> opened_low = {};
//   std::vector<std::string> closed_high = {};
//   std::vector<std::string> closed_low = {"gripper_joint"};

//   controller_->get_node()->set_parameter({"opening_ios.high",  opening_high});
//   controller_->get_node()->set_parameter({"opening_ios.low",  opening_low});
//   controller_->get_node()->set_parameter({"closing_ios.high",  closing_high});
//   controller_->get_node()->set_parameter({"closing_ios.low",  closing_low});
//   controller_->get_node()->set_parameter({"opened_states.high",  opened_high});
//   controller_->get_node()->set_parameter({"opened_states.low",  opened_low});
//   controller_->get_node()->set_parameter({"closed_states.high",  closed_high});
//   controller_->get_node()->set_parameter({"closed_states.low",  closed_low});

//   rclcpp::executors::MultiThreadedExecutor executor;
//   executor.add_node(controller_->get_node()->get_node_base_interface());
//   executor.add_node(service_caller_node_->get_node_base_interface());

//   ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
//   ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

//   ASSERT_NO_THROW(call_open_service(executor));

//   ASSERT_EQ(
//     controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
//     controller_interface::return_type::OK);

//   // get the value of the command itfs and match their values
//   ASSERT_EQ(joint_1_gpio_cmd_.get_value(), 1.0);
//   ASSERT_EQ(joint_2_gpio_cmd_.get_value(), 0.0);

// }

// // Test close gripper service sets command its as expected and publishes msg
// TEST_F(GripperIOControllerTest, CloseGripperService)
// {
//   SetUpController();

//   std::vector<std::string> opening_high = {"gripper_joint"};
//   std::vector<std::string> opening_low = {"finger_joint"};
//   std::vector<std::string> closing_high = {};
//   std::vector<std::string> closing_low = {"gripper_joint", "finger_joint"};
//   std::vector<std::string> opened_high = {"gripper_joint"};
//   std::vector<std::string> opened_low = {};
//   std::vector<std::string> closed_high = {};
//   std::vector<std::string> closed_low = {"gripper_joint"};

//   controller_->get_node()->set_parameter({"opening_ios.high",  opening_high});
//   controller_->get_node()->set_parameter({"opening_ios.low",  opening_low});
//   controller_->get_node()->set_parameter({"closing_ios.high",  closing_high});
//   controller_->get_node()->set_parameter({"closing_ios.low",  closing_low});
//   controller_->get_node()->set_parameter({"opened_states.high",  opened_high});
//   controller_->get_node()->set_parameter({"opened_states.low",  opened_low});
//   controller_->get_node()->set_parameter({"closed_states.high",  closed_high});
//   controller_->get_node()->set_parameter({"closed_states.low",  closed_low});

//   rclcpp::executors::MultiThreadedExecutor executor;
//   executor.add_node(controller_->get_node()->get_node_base_interface());
//   executor.add_node(service_caller_node_->get_node_base_interface());

//   ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
//   ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

//   ASSERT_NO_THROW(call_close_service(executor));

//   ASSERT_EQ(
//     controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
//     controller_interface::return_type::OK);

//   // get the value of the command itfs and match their values
//   ASSERT_EQ(joint_1_gpio_cmd_.get_value(), 0.0);
//   ASSERT_EQ(joint_2_gpio_cmd_.get_value(), 0.0);
// }

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}

