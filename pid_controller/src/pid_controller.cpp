// Copyright (c) 2023, Stogl Robotics Consulting UG (haftungsbeschränkt)
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
// Authors: Daniel Azanov, Dr. Denis
//

#include "pid_controller/pid_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "angles/angles.h"
#include "control_msgs/msg/single_dof_state.hpp"
#include "controller_interface/helpers.hpp"
#include "hardware_interface/hardware_info.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/version.h"

namespace
{  // utility

// Changed services history QoS to keep all so we don't lose any client service calls
// \note The versions conditioning is added here to support the source-compatibility with Humble
#if RCLCPP_VERSION_MAJOR >= 17
rclcpp::QoS qos_services =
  rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_ALL, 1))
    .reliable()
    .durability_volatile();
#else
static const rmw_qos_profile_t qos_services = {
  RMW_QOS_POLICY_HISTORY_KEEP_ALL,
  1,  // message queue depth
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  RMW_QOS_POLICY_DURABILITY_VOLATILE,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false};
#endif

using ControllerCommandMsg = pid_controller::PidController::ControllerReferenceMsg;
using hardware_interface::InterfaceDescription;
using hardware_interface::InterfaceInfo;
// called from RT control loop
void reset_controller_reference_msg(
  const std::shared_ptr<ControllerCommandMsg> & msg, const std::vector<std::string> & dof_names)
{
  msg->dof_names = dof_names;
  msg->values.resize(dof_names.size(), std::numeric_limits<double>::quiet_NaN());
  msg->values_dot.resize(dof_names.size(), std::numeric_limits<double>::quiet_NaN());
}

void reset_controller_measured_state_msg(
  const std::shared_ptr<ControllerCommandMsg> & msg, const std::vector<std::string> & dof_names)
{
  reset_controller_reference_msg(msg, dof_names);
}

}  // namespace

namespace pid_controller
{
PidController::PidController() : controller_interface::ChainableControllerInterface() {}

controller_interface::CallbackReturn PidController::on_init()
{
  control_mode_.initRT(feedforward_mode_type::OFF);

  try
  {
    param_listener_ = std::make_shared<pid_controller::ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

void PidController::update_parameters()
{
  if (!param_listener_->is_old(params_))
  {
    return;
  }
  params_ = param_listener_->get_params();
}

controller_interface::CallbackReturn PidController::configure_parameters()
{
  update_parameters();

  if (!params_.reference_and_state_dof_names.empty())
  {
    reference_and_state_dof_names_ = params_.reference_and_state_dof_names;
  }
  else
  {
    reference_and_state_dof_names_ = params_.dof_names;
  }

  if (params_.dof_names.size() != reference_and_state_dof_names_.size())
  {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "Size of 'dof_names' (%zu) and 'reference_and_state_dof_names' (%zu) parameters has to be "
      "the same!",
      params_.dof_names.size(), reference_and_state_dof_names_.size());
    return CallbackReturn::FAILURE;
  }

  dof_names_ = params_.dof_names;

  // TODO(destogl): is this even possible? Test it...
  if (params_.gains.dof_names_map.size() != dof_names_.size())
  {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "Size of 'gains' (%zu) map and number or 'dof_names' (%zu) have to be the same!",
      params_.gains.dof_names_map.size(), dof_names_.size());
    return CallbackReturn::FAILURE;
  }

  pids_.resize(dof_names_.size());

  for (size_t i = 0; i < dof_names_.size(); ++i)
  {
    // prefix should be interpreted as parameters prefix
    pids_[i] =
      std::make_shared<control_toolbox::PidROS>(get_node(), "gains." + params_.dof_names[i], true);
    if (!pids_[i]->initPid())
    {
      return CallbackReturn::FAILURE;
    }
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PidController::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  reference_and_state_dof_names_.clear();
  pids_.clear();

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PidController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto ret = configure_parameters();
  if (ret != CallbackReturn::SUCCESS)
  {
    return ret;
  }

  // topics QoS
  auto subscribers_qos = rclcpp::SystemDefaultsQoS();
  subscribers_qos.keep_last(1);
  subscribers_qos.best_effort();

  // Reference Subscriber
  ref_subscriber_ = get_node()->create_subscription<ControllerReferenceMsg>(
    "~/reference", subscribers_qos,
    std::bind(&PidController::reference_callback, this, std::placeholders::_1));

  std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();
  reset_controller_reference_msg(msg, reference_and_state_dof_names_);
  input_ref_.writeFromNonRT(msg);

  // input state Subscriber and callback
  if (params_.use_external_measured_states)
  {
    auto measured_state_callback =
      [&](const std::shared_ptr<ControllerMeasuredStateMsg> state_msg) -> void
    {
      // TODO(destogl): Sort the input values based on joint and interface names
      measured_state_.writeFromNonRT(state_msg);
    };
    measured_state_subscriber_ = get_node()->create_subscription<ControllerMeasuredStateMsg>(
      "~/measured_state", subscribers_qos, measured_state_callback);
  }
  std::shared_ptr<ControllerMeasuredStateMsg> measured_state_msg =
    std::make_shared<ControllerMeasuredStateMsg>();
  reset_controller_measured_state_msg(measured_state_msg, reference_and_state_dof_names_);
  measured_state_.writeFromNonRT(measured_state_msg);

  for (const auto & dof_name : params_.reference_and_state_dof_names)
  {
    for (const auto & itf_name : params_.reference_and_state_interfaces)
    {
      measured_state_values_[dof_name + "/" + itf_name] = std::numeric_limits<double>::quiet_NaN();
    }
  }

  auto set_feedforward_control_callback =
    [&](
      const std::shared_ptr<ControllerModeSrvType::Request> request,
      std::shared_ptr<ControllerModeSrvType::Response> response)
  {
    if (request->data)
    {
      control_mode_.writeFromNonRT(feedforward_mode_type::ON);
    }
    else
    {
      control_mode_.writeFromNonRT(feedforward_mode_type::OFF);
    }
    response->success = true;
  };

  set_feedforward_control_service_ = get_node()->create_service<ControllerModeSrvType>(
    "~/set_feedforward_control", set_feedforward_control_callback, qos_services);

  try
  {
    // State publisher
    s_publisher_ = get_node()->create_publisher<ControllerStateMsg>(
      "~/controller_state", rclcpp::SystemDefaultsQoS());
    state_publisher_ = std::make_unique<ControllerStatePublisher>(s_publisher_);
  }
  catch (const std::exception & e)
  {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (
    params_.reference_and_state_interfaces.size() == 0 ||
    params_.reference_and_state_interfaces[0].empty())
  {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "No reference_and_state_interface name given. Need to give at least one.");
    return controller_interface::CallbackReturn::ERROR;
  }
  itf_ = "/" + params_.reference_and_state_interfaces[0];

  if (
    params_.reference_and_state_interfaces.size() == 2 &&
    !params_.reference_and_state_interfaces[1].empty())
  {
    itf_dot_ = "/" + params_.reference_and_state_interfaces[1];
  }
  node_name = std::string(get_node()->get_name()) + "/";

  // Reserve memory in state publisher
  state_publisher_->lock();
  state_publisher_->msg_.dof_states.resize(reference_and_state_dof_names_.size());
  for (size_t i = 0; i < reference_and_state_dof_names_.size(); ++i)
  {
    state_publisher_->msg_.dof_states[i].name = reference_and_state_dof_names_[i];
  }
  state_publisher_->unlock();

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

void PidController::reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg)
{
  if (msg->dof_names.empty() && msg->values.size() == reference_and_state_dof_names_.size())
  {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Reference massage does not have DoF names defined. "
      "Assuming that value have order as defined state DoFs");
    auto ref_msg = msg;
    ref_msg->dof_names = reference_and_state_dof_names_;
    input_ref_.writeFromNonRT(ref_msg);
  }
  else if (
    msg->dof_names.size() == reference_and_state_dof_names_.size() &&
    msg->values.size() == reference_and_state_dof_names_.size())
  {
    auto ref_msg = msg;  // simple initialization

    // sort values in the ref_msg
    reset_controller_reference_msg(msg, reference_and_state_dof_names_);

    bool all_found = true;
    for (size_t i = 0; i < msg->dof_names.size(); ++i)
    {
      auto found_it =
        std::find(ref_msg->dof_names.begin(), ref_msg->dof_names.end(), msg->dof_names[i]);
      if (found_it == msg->dof_names.end())
      {
        all_found = false;
        RCLCPP_WARN(
          get_node()->get_logger(), "DoF name '%s' not found in the defined list of state DoFs.",
          msg->dof_names[i].c_str());
        break;
      }

      auto position = std::distance(ref_msg->dof_names.begin(), found_it);
      ref_msg->values[position] = msg->values[i];
      ref_msg->values_dot[position] = msg->values_dot[i];
    }

    if (all_found)
    {
      input_ref_.writeFromNonRT(ref_msg);
    }
  }
  else
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Size of input data names (%zu) and/or values (%zu) is not matching the expected size (%zu).",
      msg->dof_names.size(), msg->values.size(), reference_and_state_dof_names_.size());
  }
}

controller_interface::InterfaceConfiguration PidController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interfaces_config.names.reserve(params_.dof_names.size());
  for (const auto & dof_name : params_.dof_names)
  {
    command_interfaces_config.names.push_back(dof_name + "/" + params_.command_interface);
  }

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration PidController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;

  if (params_.use_external_measured_states)
  {
    state_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
  }
  else
  {
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    state_interfaces_config.names.reserve(
      dof_names_.size() * params_.reference_and_state_interfaces.size());
    for (const auto & interface : params_.reference_and_state_interfaces)
    {
      for (const auto & dof_name : reference_and_state_dof_names_)
      {
        state_interfaces_config.names.push_back(dof_name + "/" + interface);
      }
    }
  }

  return state_interfaces_config;
}

std::vector<hardware_interface::InterfaceDescription>
PidController::export_state_interface_descriptions()
{
  // does not export any StateInterfaces
  return {};
}

std::vector<hardware_interface::InterfaceDescription>
PidController::export_reference_interface_descriptions()
{
  reference_interfaces_.reserve(dof_names_.size() * params_.reference_and_state_interfaces.size());

  std::vector<hardware_interface::InterfaceDescription> reference_interfaces_descr;
  reference_interfaces_descr.reserve(reference_interfaces_.size());

  for (const auto & interface : params_.reference_and_state_interfaces)
  {
    for (const auto & dof_name : reference_and_state_dof_names_)
    {
      reference_interfaces_descr.push_back(hardware_interface::InterfaceDescription(
        get_node()->get_name(),
        hardware_interface::InterfaceInfo(dof_name + "/" + interface, std::string("double"))));
    }
  }

  return reference_interfaces_descr;
}

bool PidController::on_set_chained_mode(bool chained_mode)
{
  // Always accept switch to/from chained mode
  return true || chained_mode;
}

controller_interface::CallbackReturn PidController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Set default value in command (the same number as state interfaces)
  reset_controller_reference_msg(*(input_ref_.readFromRT()), reference_and_state_dof_names_);
  reset_controller_measured_state_msg(
    *(measured_state_.readFromRT()), reference_and_state_dof_names_);

  for (auto & [name, interface] : reference_interfaces_)
  {
    interface->set_value(std::numeric_limits<double>::quiet_NaN());
  }

  for (const auto & [name, value] : measured_state_values_)
  {
    measured_state_values_[name] = std::numeric_limits<double>::quiet_NaN();
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PidController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type PidController::update_reference_from_subscribers(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  auto current_ref = *input_ref_.readFromRT();

  for (size_t i = 0; i < reference_and_state_dof_names_.size(); ++i)
  {
    if (!std::isnan(current_ref->values[i]))
    {
      const auto ref_itf = node_name + current_ref->dof_names[i] + itf_;
      reference_interfaces_[ref_itf]->set_value(current_ref->values[i]);
      if (
        reference_interfaces_.size() == 2 * dof_names_.size() && !itf_dot_.empty() &&
        !std::isnan(current_ref->values_dot[i]))
      {
        const auto ref_itf_dot = node_name + current_ref->dof_names[i] + itf_dot_;
        reference_interfaces_[ref_itf_dot]->set_value(current_ref->values_dot[i]);
      }

      current_ref->values[i] = std::numeric_limits<double>::quiet_NaN();
    }
  }
  return controller_interface::return_type::OK;
}

controller_interface::return_type PidController::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // check for any parameter updates
  update_parameters();

  if (params_.use_external_measured_states)
  {
    const auto measured_state = *(measured_state_.readFromRT());
    for (size_t i = 0; i < reference_and_state_dof_names_.size(); ++i)
    {
      const auto itf_name = measured_state->dof_names[i] + itf_;
      measured_state_values_[itf_name] = measured_state->values[i];
      if (
        measured_state_values_.size() == 2 * dof_names_.size() && !itf_dot_.empty() &&
        !std::isnan(measured_state->values_dot[i]))
      {
        const auto itf_dot_name = measured_state->dof_names[i] + itf_dot_;
        measured_state_values_[itf_dot_name] = measured_state->values_dot[i];
      }
    }
  }
  else
  {
    for (size_t i = 0; i < measured_state_values_.size(); ++i)
    {
      measured_state_values_[state_interfaces_[i].get_name()] =
        state_interfaces_[i].get_value<double>();
    }
  }

  for (size_t i = 0; i < dof_names_.size(); ++i)
  {
    double tmp_command = std::numeric_limits<double>::quiet_NaN();
    const auto itf = reference_and_state_dof_names_[i] + itf_;
    const auto itf_dot = reference_and_state_dof_names_[i] + itf_dot_;
    const auto ref_itf = node_name + itf;
    const auto ref_itf_dot = node_name + itf_dot;
    // Using feedforward
    if (
      !std::isnan(reference_interfaces_[ref_itf]->get_value<double>()) &&
      !std::isnan(measured_state_values_[itf]))
    {
      // calculate feed-forward
      if (*(control_mode_.readFromRT()) == feedforward_mode_type::ON)
      {
        tmp_command = reference_interfaces_[ref_itf_dot]->get_value<double>() *
                      params_.gains.dof_names_map[params_.dof_names[i]].feedforward_gain;
      }
      else
      {
        tmp_command = 0.0;
      }

      double error =
        reference_interfaces_[ref_itf]->get_value<double>() - measured_state_values_[itf];
      if (params_.gains.dof_names_map[params_.dof_names[i]].angle_wraparound)
      {
        // for continuous angles the error is normalized between -pi<error<pi
        error = angles::shortest_angular_distance(
          measured_state_values_[itf], reference_interfaces_[ref_itf]->get_value<double>());
      }

      // checking if there are two interfaces
      if (
        reference_interfaces_.size() == 2 * dof_names_.size() &&
        measured_state_values_.size() == 2 * dof_names_.size())
      {
        if (
          !std::isnan(reference_interfaces_[ref_itf_dot]->get_value<double>()) &&
          !std::isnan(measured_state_values_[itf_dot]))
        {
          // use calculation with 'error' and 'error_dot'
          tmp_command += pids_[i]->computeCommand(
            error,
            reference_interfaces_[ref_itf_dot]->get_value<double>() -
              measured_state_values_[itf_dot],
            period);
        }
        else
        {
          // Fallback to calculation with 'error' only
          tmp_command += pids_[i]->computeCommand(error, period);
        }
      }
      else
      {
        // use calculation with 'error' only
        tmp_command += pids_[i]->computeCommand(error, period);
      }

      // write calculated values
      command_interfaces_[i].set_value(tmp_command);
    }
  }

  if (state_publisher_ && state_publisher_->trylock())
  {
    state_publisher_->msg_.header.stamp = time;
    for (size_t i = 0; i < dof_names_.size(); ++i)
    {
      const auto itf = reference_and_state_dof_names_[i] + itf_;
      const auto itf_dot = reference_and_state_dof_names_[i] + itf_dot_;
      const auto ref_itf = node_name + itf;
      const auto ref_itf_dot = node_name + itf_dot;
      state_publisher_->msg_.dof_states[i].reference =
        reference_interfaces_[ref_itf]->get_value<double>();
      state_publisher_->msg_.dof_states[i].feedback = measured_state_values_[itf];
      if (
        reference_interfaces_.size() == 2 * dof_names_.size() &&
        measured_state_values_.size() == 2 * dof_names_.size())
      {
        state_publisher_->msg_.dof_states[i].feedback_dot = measured_state_values_[itf_dot];
      }
      state_publisher_->msg_.dof_states[i].error =
        reference_interfaces_[ref_itf]->get_value<double>() - measured_state_values_[itf];
      if (params_.gains.dof_names_map[params_.dof_names[i]].angle_wraparound)
      {
        // for continuous angles the error is normalized between -pi<error<pi
        state_publisher_->msg_.dof_states[i].error = angles::shortest_angular_distance(
          measured_state_values_[itf], reference_interfaces_[ref_itf]->get_value<double>());
      }
      if (
        reference_interfaces_.size() == 2 * dof_names_.size() &&
        measured_state_values_.size() == 2 * dof_names_.size())
      {
        state_publisher_->msg_.dof_states[i].error_dot =
          reference_interfaces_[ref_itf_dot]->get_value<double>() - measured_state_values_[itf_dot];
      }
      state_publisher_->msg_.dof_states[i].time_step = period.seconds();
      // Command can store the old calculated values. This should be obvious because at least one
      // another value is NaN.
      state_publisher_->msg_.dof_states[i].output = command_interfaces_[i].get_value<double>();
    }
    state_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace pid_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  pid_controller::PidController, controller_interface::ChainableControllerInterface)
