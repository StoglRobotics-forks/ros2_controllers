// Copyright 2020 PAL Robotics S.L.
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
 * Author: Luca Marchionni
 * Author: Bence Magyar
 * Author: Enrique Fernández
 * Author: Paul Mathieu
 */

#ifndef ROS2_ACKERMANN_CONT__ODOMETRY_HPP_
#define ROS2_ACKERMANN_CONT__ODOMETRY_HPP_

#include <cmath>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/function.hpp>
#include "rclcpp/time.hpp"
#include "rcppmath/rolling_mean_accumulator.hpp"

namespace ros2_ackermann_cont
{
class Odometry
{
public:
    /// Integration function, used to integrate the odometry:
  typedef boost::function<void(double, double)> IntegrationFunction;
  explicit Odometry(P params);

  void init(const rclcpp::Time & time);
  bool update(double rear_wheel_pos, double front_steer_pos, const rclcpp::Time & time);
  void updateOpenLoop(double linear, double angular, const rclcpp::Time & time);

  double getHeading() const { return heading_; }
  double getX() const { return x_; }
  double getY() const { return y_; }
  double getLinear() const { return linear_; }
  double getAngular() const { return angular_; }

  void setWheelParams(double wheel_reparation_h, double wheel_radius);
  void setVelocityRollingWindowSize(size_t velocity_rolling_window_size);

private:
  using RollingMeanAcc = rcppmath::RollingMeanAccumulator<double>;

  void integrateRungeKutta2(double linear, double angular);
  void integrateExact(double linear, double angular);
  void resetAccumulators();

  // Current timestamp:
  rclcpp::Time timestamp_;

  // Current pose:
  double x_;        //   [m]
  double y_;        //   [m]
  double heading_;  // [rad]

  // Current velocity:
  double linear_;   //   [m/s]
  double angular_;  // [rad/s]

  // Wheel kinematic parameters [m]:
  double wheel_separation_;
  double wheel_radius_;

  /// Previous wheel position/state [rad]:
  double rear_wheel_old_pos_;


  // Rolling mean accumulators for the linear and angular velocities:
  int velocity_rolling_window_size_;
  RollingMeanAcc linear_acc_;
  RollingMeanAcc angular_acc_;

  // Integration funcion, used to integrate the odometry:
  IntegrationFunction integrate_fun_;
};

}  // namespace ros2_ackermann_cont

#endif  // DIFF_DRIVE_CONTROLLER__ODOMETRY_HPP_