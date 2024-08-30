// Copyright 2021 PAL Robotics SL.
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

/*
 * Authors: Subhas Das, Denis Stogl, Victor Lopez
 */

#ifndef TEST_IMU_SENSOR_BROADCASTER_HPP_
#define TEST_IMU_SENSOR_BROADCASTER_HPP_

#include <gmock/gmock.h>

#include <memory>
#include <string>

#include "hardware_interface/hardware_info.hpp"
#include "imu_sensor_broadcaster/imu_sensor_broadcaster.hpp"

using hardware_interface::InterfaceDescription;
using hardware_interface::InterfaceInfo;
// subclassing and friending so we can access member variables
class FriendIMUSensorBroadcaster : public imu_sensor_broadcaster::IMUSensorBroadcaster
{
  FRIEND_TEST(IMUSensorBroadcasterTest, SensorNameParameterNotSet);
  FRIEND_TEST(IMUSensorBroadcasterTest, InterfaceNamesParameterNotSet);
  FRIEND_TEST(IMUSensorBroadcasterTest, FrameIdParameterNotSet);
  FRIEND_TEST(IMUSensorBroadcasterTest, SensorNameParameterIsEmpty);
  FRIEND_TEST(IMUSensorBroadcasterTest, InterfaceNameParameterIsEmpty);

  FRIEND_TEST(IMUSensorBroadcasterTest, ActivateSuccess);
  FRIEND_TEST(IMUSensorBroadcasterTest, UpdateTest);
  FRIEND_TEST(IMUSensorBroadcasterTest, SensorStatePublishTest);
};

class IMUSensorBroadcasterTest : public ::testing::Test
{
public:
  static void SetUpTestCase();
  static void TearDownTestCase();

  void SetUp();
  void TearDown();

  void SetUpIMUBroadcaster();

protected:
  const std::string sensor_name_ = "imu_sensor";
  const std::string frame_id_ = "imu_sensor_frame";
  std::array<double, 10> sensor_values_ = {1.1, 2.2, 3.3, 4.4, 5.5, 6.6, 7.7, 8.8, 9.9, 10.10};
  hardware_interface::StateInterface imu_orientation_x_{
    InterfaceDescription(sensor_name_, InterfaceInfo("orientation.x", "1.1", "double"))};
  hardware_interface::StateInterface imu_orientation_y_{
    InterfaceDescription(sensor_name_, InterfaceInfo("orientation.y", "2.2", "double"))};
  hardware_interface::StateInterface imu_orientation_z_{
    InterfaceDescription(sensor_name_, InterfaceInfo("orientation.z", "3.3", "double"))};
  hardware_interface::StateInterface imu_orientation_w_{
    InterfaceDescription(sensor_name_, InterfaceInfo("orientation.w", "4.4", "double"))};
  hardware_interface::StateInterface imu_angular_velocity_x_{
    InterfaceDescription(sensor_name_, InterfaceInfo("angular_velocity.x", "5.5", "double"))};
  hardware_interface::StateInterface imu_angular_velocity_y_{
    InterfaceDescription(sensor_name_, InterfaceInfo("angular_velocity.y", "6.6", "double"))};
  hardware_interface::StateInterface imu_angular_velocity_z_{
    InterfaceDescription(sensor_name_, InterfaceInfo("angular_velocity.z", "7.7", "double"))};
  hardware_interface::StateInterface imu_linear_acceleration_x_{
    InterfaceDescription(sensor_name_, InterfaceInfo("linear_acceleration.x", "8.8", "double"))};
  hardware_interface::StateInterface imu_linear_acceleration_y_{
    InterfaceDescription(sensor_name_, InterfaceInfo("linear_acceleration.y", "9.9", "double"))};
  hardware_interface::StateInterface imu_linear_acceleration_z_{
    InterfaceDescription(sensor_name_, InterfaceInfo("linear_acceleration.z", "10.10", "double"))};

  std::unique_ptr<FriendIMUSensorBroadcaster> imu_broadcaster_;

  void subscribe_and_get_message(sensor_msgs::msg::Imu & imu_msg);
};

#endif  // TEST_IMU_SENSOR_BROADCASTER_HPP_
