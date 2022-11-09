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

#include "ros2_ackermann_cont/ackermann_steering_controller_ros2.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "tf2/LinearMath/Quaternion.h"


#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <queue>



#include "controller_interface/helpers.hpp"

namespace
{

// TODO(destogl): remove this when merged upstream
// Changed services history QoS to keep all so we don't lose any client service calls
static constexpr rmw_qos_profile_t rmw_qos_profile_services_hist_keep_all = {
  RMW_QOS_POLICY_HISTORY_KEEP_ALL,
  1,  // message queue depth
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  RMW_QOS_POLICY_DURABILITY_VOLATILE,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false};

using ControllerReferenceMsg = ros2_ackermann_cont::AckermannSteeringControllerRos2::ControllerReferenceMsg;

// called from RT control loop
void reset_controller_reference_msg(
  const std::shared_ptr<ControllerReferenceMsg> & msg, 
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node)
{

  msg->header.stamp = node->now();
  msg->twist.linear.x  = std::numeric_limits<double>::quiet_NaN();
  msg->twist.angular.z = std::numeric_limits<double>::quiet_NaN();  

}

}  // namespace

namespace ros2_ackermann_cont
{

using namespace std::chrono_literals;
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using lifecycle_msgs::msg::State;

AckermannSteeringControllerRos2::AckermannSteeringControllerRos2() : controller_interface::ChainableControllerInterface() 
{}

controller_interface::CallbackReturn AckermannSteeringControllerRos2::on_init()
{

  try {
    param_listener_ = std::make_shared<ackermann_steering_controller_ros2::ParamListener>(get_node());
    // with the lifecycle node being initialized, we can declare parameters
/*
    auto_declare<bool>("linear.x.has_velocity_limits", false);
    auto_declare<bool>("linear.x.has_acceleration_limits", false);
    auto_declare<bool>("linear.x.has_jerk_limits", false);
    auto_declare<double>("linear.x.max_velocity", NAN);
    auto_declare<double>("linear.x.min_velocity", NAN);
    auto_declare<double>("linear.x.max_acceleration", NAN);
    auto_declare<double>("linear.x.min_acceleration", NAN);
    auto_declare<double>("linear.x.max_jerk", NAN);
    auto_declare<double>("linear.x.min_jerk", NAN);

    auto_declare<bool>("angular.z.has_velocity_limits", false);
    auto_declare<bool>("angular.z.has_acceleration_limits", false);
    auto_declare<bool>("angular.z.has_jerk_limits", false);
    auto_declare<double>("angular.z.max_velocity", NAN);
    auto_declare<double>("angular.z.min_velocity", NAN);
    auto_declare<double>("angular.z.max_acceleration", NAN);
    auto_declare<double>("angular.z.min_acceleration", NAN);
    auto_declare<double>("angular.z.max_jerk", NAN);
    auto_declare<double>("angular.z.min_jerk", NAN);
    publish_rate_ = auto_declare<double>("publish_rate", publish_rate_);

    */

  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Regardless of how we got the separation and radius, use them
    // to set the odometry parameters
    const double ws_h = params_.wheel_separation_multiplier * params_.wheel_separation;
    const double wr = params_.wheel_radius_multiplier * params_.wheel_radius;
    odometry_().setWheelParams(ws_h, wr);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AckermannSteeringControllerRos2::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();

  // if (params_.joints.size() != state_joints_.size()) {
  //   RCLCPP_FATAL(
  //     get_node()->get_logger(),
  //     "Size of 'joints' (%d) and 'state_joints' (%d) parameters has to be the same!",
    //denis why should cmd joints and state joints be same in num?
  //     params_.joints.size(), state_joints_.size());
  //   return CallbackReturn::FAILURE;
  // }

  // topics QoS
  auto subscribers_qos = rclcpp::SystemDefaultsQoS();
  subscribers_qos.keep_last(1);
  subscribers_qos.best_effort();

  // Reference Subscriber
  ref_timeout_ = rclcpp::Duration::from_seconds(params_.reference_timeout);
  velocity_command_subscriber_ = get_node()->create_subscription<ControllerReferenceMsg>(
    "/reference_vel", subscribers_qos,
    std::bind(&AckermannSteeringControllerRos2::reference_callback, this, std::placeholders::_1));

  std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();
  reset_controller_reference_msg(msg, get_node());
  input_ref_.writeFromNonRT(msg);


  try {
    // Odom State publisher
    odom_s_publisher_ =
      get_node()->create_publisher<ControllerStateMsgOdom>("~/odom_state", rclcpp::SystemDefaultsQoS());
    rt_odom_state_publisher_ = std::make_unique<ControllerStatePublisherOdom>(odom_s_publisher_);
  } catch (const std::exception & e) {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  auto & odometry_message = rt_odom_state_publisher_->msg_;

    // TODO(anyone): Reserve memory in state publisher depending on the message type
  rt_odom_state_publisher_->lock();
  odometry_message.header.frame_id = params_.rear_wheel_name;
  rt_odom_state_publisher_->unlock();

  
  odometry_message.header.frame_id = params_.odom_frame_id;
  odometry_message.child_frame_id =  params_.base_frame_id;
  odometry_message.pose.pose.position.z = 0;
  // limit the publication on the topics /odom and /tf
  publish_period_ = rclcpp::Duration::from_seconds(1.0 / params_.publish_rate);
  previous_publish_timestamp_ = get_node()->get_clock()->now(); //issuse iitialize with zero and update it in first update1 call
  //ros2 multiple clocks-how odom is initialoied in diffdrive

  // odometry_().init(time);

  // initialize odom values zeros
  odometry_message.twist =
    geometry_msgs::msg::TwistWithCovariance(rosidl_runtime_cpp::MessageInitialization::ALL);

  constexpr size_t NUM_DIMENSIONS = 6;
  for (size_t index = 0; index < 6; ++index)
  {
    // 0, 7, 14, 21, 28, 35
    const size_t diagonal_index = NUM_DIMENSIONS * index + index;
    odometry_message.pose.covariance[diagonal_index] = params_.pose_covariance_diagonal[index];
    odometry_message.twist.covariance[diagonal_index] =
      params_.twist_covariance_diagonal[index];
  }

  try {
    // Tf State publisher
    tf_odom_s_publisher_ =
      get_node()->create_publisher<ControllerStateMsgTf>("~/tf_odom_state", rclcpp::SystemDefaultsQoS());
    rt_tf_odom_state_publisher_ = std::make_unique<ControllerStatePublisherTf>(tf_odom_s_publisher_);
  } catch (const std::exception & e) {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  auto & odometry_transform_message = rt_tf_odom_state_publisher_->msg_;
    // TODO(anyone): Reserve memory in state publisher depending on the message type
  rt_tf_odom_state_publisher_->lock();
  odometry_transform_message.transforms.front().header.frame_id = params_.rear_wheel_name;
  rt_tf_odom_state_publisher_->unlock();

  // keeping track of odom and base_link transforms only
  odometry_transform_message.transforms.resize(1);
  odometry_transform_message.transforms[0].transform.translation.z = 0.0;
  odometry_transform_message.transforms.front().header.frame_id = params_.odom_frame_id;
  odometry_transform_message.transforms.front().child_frame_id = params_.base_frame_id;
    
  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

void AckermannSteeringControllerRos2::reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg)
{
  const auto age_of_last_command = get_node()->now() - msg->header.stamp;
  // if no timestamp provided use current time for command timestamp
  if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0u) {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Timestamp in header is missing, using current time as command timestamp.");
    msg->header.stamp = get_node()->now();
  }
  if (ref_timeout_ == rclcpp::Duration::from_seconds(0) || age_of_last_command <= ref_timeout_) {
    input_ref_.writeFromNonRT(msg);
  }else {
    RCLCPP_ERROR(
      get_node()->get_logger(),
        "Received message has timestamp %.10f older then allowed timeout at timestamp %.10f",
         rclcpp::Time(msg->header.stamp).seconds(), get_node()->now().seconds());
    }
}

controller_interface::InterfaceConfiguration AckermannSteeringControllerRos2::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  
  command_interfaces_config.names.push_back(params_.rear_wheel_name + "/" + HW_IF_VELOCITY);
  command_interfaces_config.names.push_back(params_.front_steer_name + "/" + HW_IF_POSITION);
  

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration AckermannSteeringControllerRos2::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interfaces_config.names.reserve(STATE_MY_ITFS);
  state_interfaces_config.names.push_back(params_.rear_wheel_name + "/" + HW_IF_POSITION);
  state_interfaces_config.names.push_back(params_.front_steer_name + "/" + HW_IF_POSITION);

  return state_interfaces_config;
}



std::vector<hardware_interface::CommandInterface> AckermannSteeringControllerRos2::on_export_reference_interfaces()
{
  reference_interfaces_.resize(CMD_MY_ITFS, std::numeric_limits<double>::quiet_NaN());

  std::vector<hardware_interface::CommandInterface> reference_interfaces;
  reference_interfaces.reserve(reference_interfaces_.size());


  reference_interfaces.push_back(hardware_interface::CommandInterface(
    get_node()->get_name(), params_.rear_wheel_name + "/" + HW_IF_VELOCITY,
    &reference_interfaces_[0]));

  reference_interfaces.push_back(hardware_interface::CommandInterface(
    get_node()->get_name(), params_.rear_wheel_name + "/" + HW_IF_POSITION,
    &reference_interfaces_[1]));


  return reference_interfaces;
}

bool AckermannSteeringControllerRos2::on_set_chained_mode(bool chained_mode)
{
  // Always accept switch to/from chained mode
  return true || chained_mode;
}

controller_interface::CallbackReturn AckermannSteeringControllerRos2::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Set default value in command
  reset_controller_reference_msg(*(input_ref_.readFromRT()), get_node());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AckermannSteeringControllerRos2::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): depending on number of interfaces, use definitions, e.g., `CMD_MY_ITFS`,
  // instead of a loop
  for (size_t i = 0; i < CMD_MY_ITFS; ++i) {
    command_interfaces_[i].set_value(std::numeric_limits<double>::quiet_NaN());
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type AckermannSteeringControllerRos2::update_reference_from_subscribers()
{
  auto current_ref = input_ref_.readFromRT();
  const auto age_of_last_command = get_node()->now() - (*current_ref)->header.stamp;

  if (params_.open_loop)
  {
    odometry_().updateOpenLoop(last0_cmd_.lin, last0_cmd_.ang, get_node()->now());
  }else{

      // double left_feedback_mean = 0.0;
      // double right_feedback_mean = 0.0;

      // double wheel_pos  = rear_wheel_joint_.getPosition();
      // double steer_pos = front_steer_joint_.getPosition();

      const double rear_wheel_pos = registered_rear_wheel_handles_[0].feedback.get().get_value();
      const double front_steer_pos =
        registered_front_wheel_handles_[0].feedback.get().get_value();

      if (std::isnan(rear_wheel_pos) || std::isnan(front_steer_pos))
        {
        RCLCPP_ERROR(get_node()->get_logger(),
        "Either the rear or front wheel %s is invalid", HW_IF_POSITION);
      }

      // Estimate linear and angular velocity using joint information
      odometry_().update(rear_wheel_pos, front_steer_pos, get_node()->now());
    }

    // Publish odometry message
    if (previous_publish_timestamp_ + publish_period_ < get_node()->now())
    {
      previous_publish_timestamp_ += publish_period_;
      // Compute and store orientation info
      tf2::Quaternion orientation;
      orientation.setRPY(0.0, 0.0, odometry_().getHeading());

      // Populate odom message and publish
      if (rt_odom_state_publisher_->trylock())
      {
        auto & odometry_message = rt_odom_state_publisher_->msg_;
        odometry_message.header.stamp = get_node()->now();
        odometry_message.pose.pose.position.x = odometry_().getX();
        odometry_message.pose.pose.position.y = odometry_().getY();
        odometry_message.pose.pose.orientation.x = orientation.x();
        odometry_message.pose.pose.orientation.y = orientation.y();
        odometry_message.pose.pose.orientation.z = orientation.z();
        odometry_message.pose.pose.orientation.w = orientation.w();
        odometry_message.twist.twist.linear.x = odometry_().getLinear();
        odometry_message.twist.twist.angular.z = odometry_().getAngular();
        rt_odom_state_publisher_->unlockAndPublish();

      }

      // Publish tf /odom frame
      if (params_.enable_odom_tf && rt_tf_odom_state_publisher_->trylock())
      {
        auto & transform = rt_tf_odom_state_publisher_->msg_.transforms.front();
        transform.header.stamp = get_node()->now();
        transform.transform.translation.x = odometry_().getX();
        transform.transform.translation.y = odometry_().getY();
        transform.transform.rotation.x = orientation.x();
        transform.transform.rotation.y = orientation.y();
        transform.transform.rotation.z = orientation.z();
        transform.transform.rotation.w = orientation.w();
        rt_tf_odom_state_publisher_->unlockAndPublish();
      }
    }

    // MOVE ROBOT
    // Retreive current velocity command and time step:
    auto & curr_cmd = (*current_ref);
    const auto dt = (get_node()->now() - curr_cmd->header.stamp);

    // Brake if cmd_vel has timeout:
    if (dt > ref_timeout_)
    {
      curr_cmd->twist.linear.x = 0.0;
      curr_cmd->twist.angular.z = 0.0;
    }

    // Limit velocities and accelerations:
    // const double cmd_dt(period.toSec());

    // limiter_lin_.limit(curr_cmd.lin, last0_cmd_.lin, last1_cmd_.lin, cmd_dt);
    // limiter_ang_.limit(curr_cmd.ang, last0_cmd_.ang, last1_cmd_.ang, cmd_dt);

    //last1_cmd_ = last0_cmd_;
    last0_cmd_.lin = curr_cmd->twist.linear.x;
    last0_cmd_.ang = curr_cmd->twist.angular.z;
    last0_cmd_.stamp = curr_cmd->header.stamp;

    // Set Command
    const double r_wheel_vel = curr_cmd->twist.linear.x/params_.wheel_radius; // omega = linear_vel / radius
    const auto f_wheel_pos = curr_cmd->twist.angular.z;
    // rear_wheel_joint_.setCommand(rear_wheel_vel);
    // front_steer_joint_.setCommand(front_wheel_pos);


  // TODO(anyone): depending on number of interfaces, use definitions, e.g., `CMD_MY_ITFS`,
  // instead of a loop
    // send message only if there is no timeout
  if (age_of_last_command <= ref_timeout_ || ref_timeout_ == rclcpp::Duration::from_seconds(0)) {
    if (!std::isnan(r_wheel_vel)) {
      reference_interfaces_[0] = r_wheel_vel;
      reference_interfaces_[1] = f_wheel_pos;
      if (ref_timeout_ == rclcpp::Duration::from_seconds(0)){
        reference_interfaces_[0] = std::numeric_limits<double>::quiet_NaN();
        reference_interfaces_[1] = std::numeric_limits<double>::quiet_NaN();
      }
    }
  } else {
        reference_interfaces_[0] = std::numeric_limits<double>::quiet_NaN();
        reference_interfaces_[1] = std::numeric_limits<double>::quiet_NaN();
  }
  return controller_interface::return_type::OK;
}

controller_interface::return_type AckermannSteeringControllerRos2::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  auto current_ref = input_ref_.readFromRT();
  const auto age_of_last_command = get_node()->now() - (*current_ref)->header.stamp;

  // TODO(anyone): depending on number of interfaces, use definitions, e.g., `CMD_MY_ITFS`,
  // instead of a loop
  for (size_t i = 0; i < CMD_MY_ITFS; ++i) {
    // send message only if there is no timeout
    if (age_of_last_command <= ref_timeout_ || ref_timeout_ == rclcpp::Duration::from_seconds(0)) {
      if (!std::isnan(reference_interfaces_[i])) {
        command_interfaces_[i].set_value(reference_interfaces_[i]);
        if (ref_timeout_ == rclcpp::Duration::from_seconds(0)){
          reference_interfaces_[i] = std::numeric_limits<double>::quiet_NaN();
        }
      }
    } else {
      reference_interfaces_[i] = std::numeric_limits<double>::quiet_NaN();
    }
  }

  return controller_interface::return_type::OK;
}

}  // namespace ros2_ackermann_cont

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_ackermann_cont::AckermannSteeringControllerRos2, controller_interface::ChainableControllerInterface)
