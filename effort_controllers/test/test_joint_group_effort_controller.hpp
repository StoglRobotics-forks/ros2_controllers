// Copyright 2020 PAL Robotics SL.
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

#ifndef TEST_JOINT_GROUP_EFFORT_CONTROLLER_HPP_
#define TEST_JOINT_GROUP_EFFORT_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "gmock/gmock.h"

#include "effort_controllers/joint_group_effort_controller.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

using hardware_interface::CommandInterface;
using hardware_interface::HW_IF_EFFORT;
using hardware_interface::InterfaceDescription;
using hardware_interface::InterfaceInfo;
// subclassing and friending so we can access member variables
class FriendJointGroupEffortController : public effort_controllers::JointGroupEffortController
{
  FRIEND_TEST(JointGroupEffortControllerTest, CommandSuccessTest);
  FRIEND_TEST(JointGroupEffortControllerTest, WrongCommandCheckTest);
  FRIEND_TEST(JointGroupEffortControllerTest, CommandCallbackTest);
  FRIEND_TEST(JointGroupEffortControllerTest, StopJointsOnDeactivateTest);
};

class JointGroupEffortControllerTest : public ::testing::Test
{
public:
  static void SetUpTestCase();
  static void TearDownTestCase();

  void SetUp();
  void TearDown();

  void SetUpController();

protected:
  std::unique_ptr<FriendJointGroupEffortController> controller_;

  // dummy joint state values used for tests
  const std::vector<std::string> joint_names_ = {"joint1", "joint2", "joint3"};

  CommandInterface joint_1_cmd_{
    InterfaceDescription(joint_names_[0], InterfaceInfo(HW_IF_EFFORT, "1.1", "double"))};
  CommandInterface joint_2_cmd_{
    InterfaceDescription(joint_names_[1], InterfaceInfo(HW_IF_EFFORT, "2.1", "double"))};
  CommandInterface joint_3_cmd_{
    InterfaceDescription(joint_names_[2], InterfaceInfo(HW_IF_EFFORT, "3.1", "double"))};
};

#endif  // TEST_JOINT_GROUP_EFFORT_CONTROLLER_HPP_
