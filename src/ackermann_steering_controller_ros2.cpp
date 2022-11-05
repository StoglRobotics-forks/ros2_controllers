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

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"


#include <urdf_parser/urdf_parser.h>

static double euclideanOfVectors(const urdf::Vector3& vec1, const urdf::Vector3& vec2)
{
  return std::sqrt(std::pow(vec1.x-vec2.x,2) +
                   std::pow(vec1.y-vec2.y,2) +
                   std::pow(vec1.z-vec2.z,2));
}

/*
 * \brief Check if the link is modeled as a cylinder
 * \param link Link
 * \return true if the link is modeled as a Cylinder; false otherwise
 */
static bool isCylinder(const urdf::LinkConstSharedPtr& link)
{
  if (!link)
  {
    ROS_ERROR("Link pointer is null.");
    return false;
  }

  if (!link->collision)
  {
    ROS_ERROR_STREAM("Link " << link->name << " does not have collision description. Add collision description for link to urdf.");
    return false;
  }

  if (!link->collision->geometry)
  {
    ROS_ERROR_STREAM("Link " << link->name << " does not have collision geometry description. Add collision geometry description for link to urdf.");
    return false;
  }

  if (link->collision->geometry->type != urdf::Geometry::CYLINDER)
  {
    ROS_ERROR_STREAM("Link " << link->name << " does not have cylinder geometry");
    return false;
  }

  return true;
}

/*
 * \brief Get the wheel radius
 * \param [in]  wheel_link   Wheel link
 * \param [out] wheel_radius Wheel radius [m]
 * \return true if the wheel radius was found; false other
wise
 */
static bool getWheelRadius(const urdf::LinkConstSharedPtr& wheel_link, double& wheel_radius)
{
  if (!isCylinder(wheel_link))
  {
    ROS_ERROR_STREAM("Wheel link " << wheel_link->name << " is NOT modeled as a cylinder!");
    return false;
  }

  wheel_radius = (static_cast<urdf::Cylinder*>(wheel_link->collision->geometry.get()))->radius;
  return true;
}
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
  /*
  msg->header.stamp = node->now();
  msg->joint_names = joint_names;
  msg->displacements.resize(joint_names.size(), std::numeric_limits<double>::quiet_NaN());
  msg->velocities.resize(joint_names.size(), std::numeric_limits<double>::quiet_NaN());
  msg->duration = std::numeric_limits<double>::quiet_NaN();
  */
  msg->header.stamp = node->now();
  msg->twist.linear.x  = odometry_.getLinear();
  msg->twist.angular.z = odometry_.getAngular();

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

const char * DiffDriveController::feedback_type() const
{
  return odom_params_.position_feedback ? HW_IF_POSITION : HW_IF_VELOCITY;
}

controller_interface::CallbackReturn AckermannSteeringControllerRos2::on_init()
{

  try {
    param_listener_ = std::make_shared<ackermann_steering_controller_ros2::ParamListener>(get_node());
    auto_declare<std::vector<double>>("pose_covariance_diagonal", std::vector<double>());
    auto_declare<std::vector<double>>("twist_covariance_diagonal", std::vector<double>());
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
    const double ws_h = params_.wheel_separation_h_multiplier * params_.wheel_separation_h;
    const double wr = params_.wheel_radius_multiplier * params_.wheel_radius;
    odometry_.setWheelParams(ws_h, wr);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AckermannSteeringControllerRos2::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();

  if (!params_.state_joints.empty()) {
    state_joints_ = params_.state_joints;
  } else {
    state_joints_ = params_.joints;
  }

  if (params_.joints.size() != state_joints_.size()) {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "Size of 'joints' (%d) and 'state_joints' (%d) parameters has to be the same!",
      params_.joints.size(), state_joints_.size());
    return CallbackReturn::FAILURE;
  }

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
  reset_controller_reference_msg(msg, params_.joints, get_node());
  input_ref_.writeFromNonRT(msg);


  try {
    // Odom State publisher
    odom_s_publisher_ =
      get_node()->create_publisher<ControllerStateMsg>("~/odom_state", rclcpp::SystemDefaultsQoS());
    rt_odom_state_publisher_ = std::make_unique<ControllerStatePublisher>(odom_s_publisher_);
  } catch (const std::exception & e) {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

    // TODO(anyone): Reserve memory in state publisher depending on the message type
  rt_odom_state_publisher_->lock();
  rt_odom_state_publisher_->msg_.header.frame_id = params_.rear_wheel_name;
  rt_odom_state_publisher_->unlock();

  auto & odometry_message = rt_odom_state_publisher_->msg_;
  odometry_message.header.frame_id = params_.odom_frame_id;
  odometry_message.child_frame_id =  params_.base_frame_id;
  odometry_message.pose.pose.position.z = 0;
  // limit the publication on the topics /odom and /tf
  publish_period_ = rclcpp::Duration::from_seconds(1.0 / params_.publish_rate);
  previous_publish_timestamp_ = get_node()->get_clock()->now();

  odometry_.init(time);

  // initialize odom values zeros
  odometry_message.twist =
    geometry_msgs::msg::TwistWithCovariance(rosidl_runtime_cpp::MessageInitialization::ALL);

  constexpr size_t NUM_DIMENSIONS = 6;
  for (size_t index = 0; index < 6; ++index)
  {
    // 0, 7, 14, 21, 28, 35
    const size_t diagonal_index = NUM_DIMENSIONS * index + index;
    odometry_message.pose.covariance[diagonal_index] = odom_params_.pose_covariance_diagonal[index];
    odometry_message.twist.covariance[diagonal_index] =
      params_.twist_covariance_diagonal[index];
  }

  try {
    // Tf State publisher
    tf_odom_s_publisher_ =
      get_node()->create_publisher<ControllerStateMsg>("~/tf_odom_state", rclcpp::SystemDefaultsQoS());
    rt_tf_odom_state_publisher_ = std::make_unique<ControllerStatePublisher>(tf_odom_s_publisher_);
  } catch (const std::exception & e) {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

    // TODO(anyone): Reserve memory in state publisher depending on the message type
  rt_tf_odom_state_publisher_->lock();
  rt_tf_odom_state_publisher_->msg_.header.frame_id = params_.rear_wheel_name;
  rt_tf_odom_state_publisher_->unlock();

  // keeping track of odom and base_link transforms only
  auto & odometry_transform_message = rt_tf_odom_state_publisher_->msg_;
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

  auto logger = get_node()->get_logger();
  if (get_state().id() == State::PRIMARY_STATE_INACTIVE)
  {
    if (!is_halted)
    {
      halt();
      is_halted = true;
    }
    return controller_interface::return_type::OK;
  }

  if (params_.open_loop)
  {
    odometry_.updateOpenLoop(last0_cmd_.lin, last0_cmd_.ang, time);
  }else{

      // double left_feedback_mean = 0.0;
      // double right_feedback_mean = 0.0;

      // double wheel_pos  = rear_wheel_joint_.getPosition();
      // double steer_pos = front_steer_joint_.getPosition();

      const double params_.rear_wheel_pos = registered_rear_wheel_handles_[index].feedback.get().get_value();
      const double params_.front_steer_pos =
        registered_front_wheel_handles_[index].feedback.get().get_value();

      if (std::isnan(params_.rear_wheel_pos) || std::isnan(params_.front_steer_pos))
        return;

      // Estimate linear and angular velocity using joint information
      params_.front_steer_pos = params_.front_steer_pos * params_.steer_pos_multiplier;
      odometry_.update(params_.rear_wheel_pos, params_.front_steer_pos, time);
    }

    // Publish odometry message
    if (previous_publish_timestamp_ + publish_period_ < time)
    {
      previous_publish_timestamp_ += publish_period_;
      // Compute and store orientation info
      tf2::Quaternion orientation;
      orientation.setRPY(0.0, 0.0, odometry_.getHeading());

      // Populate odom message and publish
      if (rt_odom_state_publisher_->trylock())
      {
        auto & odometry_message = realtime_odometry_publisher_->msg_;
        odometry_message.header.stamp = time;
        odometry_message.pose.pose.position.x = odometry_.getX();
        odometry_message.pose.pose.position.y = odometry_.getY();
        odometry_message.pose.pose.orientation = orientation;
        odometry_message.twist.twist.linear.x = odometry_.getLinear();
        odometry_message.twist.twist.angular.z = odometry_.getAngular();
        realtime_odometry_publisher_->unlockAndPublish();

      }

      // Publish tf /odom frame
      if (params_.enable_odom_tf && rt_tf_odom_state_publisher_->trylock())
      {
        auto & transform = realtime_odometry_transform_publisher_->msg_.transforms.front();
        transform.header.stamp = time;
        transform.transform.translation.x = odometry_.getX();
        transform.transform.translation.y = odometry_.getY();
        transform.transform.rotation = orientation;
        rt_tf_odom_state_publisher_->unlockAndPublish();
      }
    }

    // MOVE ROBOT
    // Retreive current velocity command and time step:
    auto & curr_cmd = (*current_ref)->msg;
    const double dt = (time - curr_cmd.header.stamp).toSec();

    // Brake if cmd_vel has timeout:
    if (dt > cmd_vel_timeout_)
    {
      curr_cmd.twist.twist.linear.x = 0.0;
      curr_cmd.twist.twist.angular.z = 0.0;
    }

    // Limit velocities and accelerations:
    const double cmd_dt(period.toSec());

    // limiter_lin_.limit(curr_cmd.lin, last0_cmd_.lin, last1_cmd_.lin, cmd_dt);
    // limiter_ang_.limit(curr_cmd.ang, last0_cmd_.ang, last1_cmd_.ang, cmd_dt);

    last1_cmd_ = last0_cmd_;
    last0_cmd_ = curr_cmd;

    // Set Command
    const double rear_wheel_vel = curr_cmd.twist.twist.linear.x/wheel_radius_; // omega = linear_vel / radius
    front_wheel_pos = curr_cmd.twist.twist.angular.z
    // rear_wheel_joint_.setCommand(rear_wheel_vel);
    // front_steer_joint_.setCommand(front_wheel_pos);


  // TODO(anyone): depending on number of interfaces, use definitions, e.g., `CMD_MY_ITFS`,
  // instead of a loop
    // send message only if there is no timeout
  if (age_of_last_command <= ref_timeout_ || ref_timeout_ == rclcpp::Duration::from_seconds(0)) {
    if (!std::isnan(rear_wheel_vel)) {
      reference_interfaces_[0] = rear_wheel_vel;
      reference_interfaces_[1] = front_wheel_pos;
      if (ref_timeout_ == rclcpp::Duration::from_seconds(0)){
        rear_wheel_vel = std::numeric_limits<double>::quiet_NaN();
        front_wheel_pos = std::numeric_limits<double>::quiet_NaN();
      }
    }
  } else {
        rear_wheel_vel = std::numeric_limits<double>::quiet_NaN();
        front_wheel_pos = std::numeric_limits<double>::quiet_NaN();
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
  for (size_t i = 0; i < command_interfaces_.size(); ++i) {
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

  if (odom_state_publisher_ && odom_state_publisher_->trylock()) {
    odom_state_publisher_->msg_.header.stamp = time;
    odom_state_publisher_->msg_.set_point = command_interfaces_[CMD_MY_ITFS].get_value();
    odom_state_publisher_->unlockAndPublish();
  }

  if (tf_odom_state_publisher_ && tf_odom_state_publisher_->trylock()) { 
    tf_odom_state_publisher_->msg_.header.stamp = time;
    tf_odom_state_publisher_->msg_.set_point = command_interfaces_[CMD_MY_ITFS].get_value();
    tf_odom_state_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace ros2_ackermann_cont

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_ackermann_cont::AckermannSteeringControllerRos2, controller_interface::ChainableControllerInterface)
