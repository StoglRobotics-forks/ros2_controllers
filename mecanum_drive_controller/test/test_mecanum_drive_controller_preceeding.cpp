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

#include "test_mecanum_drive_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

using mecanum_drive_controller::NR_CMD_ITFS;
using mecanum_drive_controller::NR_REF_ITFS;
using mecanum_drive_controller::NR_STATE_ITFS;

class MecanumDriveControllerTest
: public MecanumDriveControllerFixture<TestableMecanumDriveController>
{
};

// checking if all parameters are initialized and set as expected
TEST_F(MecanumDriveControllerTest, all_parameters_set_configure_success)
{
  SetUpController();

  ASSERT_EQ(controller_->params_.reference_timeout, 0.0);
  ASSERT_TRUE(controller_->params_.joint_names.empty());
  ASSERT_TRUE(controller_->params_.state_joint_names.empty());
  ASSERT_TRUE(controller_->params_.interface_name.empty());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_EQ(controller_->params_.reference_timeout, 0.1);
  ASSERT_THAT(controller_->params_.joint_names, testing::ElementsAreArray(joint_names_));
  ASSERT_THAT(
    controller_->params_.state_joint_names, testing::ElementsAreArray(state_joint_names_));
  ASSERT_THAT(controller_->state_joint_names_, testing::ElementsAreArray(state_joint_names_));
  ASSERT_EQ(controller_->params_.interface_name, interface_name_);
}

// checking if all interfaces, command and state interfaces are exported as expected
TEST_F(MecanumDriveControllerTest, check_exported_intefaces)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  auto command_intefaces = controller_->command_interface_configuration();
  ASSERT_EQ(command_intefaces.names.size(), joint_command_values_.size());
  for (size_t i = 0; i < command_intefaces.names.size(); ++i)
  {
    EXPECT_EQ(command_intefaces.names[i], joint_names_[i] + "/" + interface_name_);
  }

  auto state_intefaces = controller_->state_interface_configuration();
  ASSERT_EQ(state_intefaces.names.size(), joint_state_values_.size());
  for (size_t i = 0; i < state_intefaces.names.size(); ++i)
  {
    EXPECT_EQ(state_intefaces.names[i], state_joint_names_[i] + "/" + interface_name_);
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
