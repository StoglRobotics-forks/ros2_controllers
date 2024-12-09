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

#include "gripper_io_controller/gripper_io_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"

namespace
{  // utility

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

}  // namespace

namespace gripper_io_controller
{
GripperIOController::GripperIOController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn GripperIOController::on_init()
{
  control_mode_.initRT(control_mode_type::FAST);
  service_buffer_.initRT(service_mode_type::UNDEFINED);
  configuration_key_ = "stichmass_250"; // setting this as default
  configure_gripper_buffer_.initRT(configuration_key_);
  gripper_state_buffer_.initRT(gripper_state_type::UNDEFINED);
  reconfigure_state_buffer_.initRT(reconfigure_state_type::UNDEFINED);

  try
  {
    param_listener_ = std::make_shared<gripper_io_controller::ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GripperIOController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();

  auto result = check_parameters();
  if (result != controller_interface::CallbackReturn::SUCCESS)
  {
    return result;
  }

  /**
   * realtime publisher for the gripper_specific sensors, type publishing is boolean
  */

  prepare_command_and_state_ios();

  result = prepare_publishers_and_services();
  if (result != controller_interface::CallbackReturn::SUCCESS)
  {
    return result;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration GripperIOController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interfaces_config.names.reserve(command_if_ios.size());
  for (const auto & command_io : command_if_ios)
  {
    command_interfaces_config.names.push_back(command_io);
  }

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration GripperIOController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interfaces_config.names.reserve(state_if_ios.size());
  for (const auto & state_io : state_if_ios)
  {
    state_interfaces_config.names.push_back(state_io);
  }
  
  return state_interfaces_config;
}

controller_interface::CallbackReturn GripperIOController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GripperIOController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type GripperIOController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  /// open and close
  /// use state machine with flags to open and close the gripper

  /// state for the configuration
  /// stop the open/close when 
  /// store where gripper, enum class, open, close, undefined
  /// state, changing configuration, store in realtime buffer and check it in the callback.
  /// if it is in configuration then I can't open or close the gripper

  /// set the unit test later

  /// reconfigure flag will be enabled when the service is called with the name of the configuration
  if (reconfigureFlag_)
  {
    configuration_key_ = *(configure_gripper_buffer_.readFromRT());
    RCLCPP_INFO(get_node()->get_logger(), "Reconfiguring to configuration: %s", configuration_key_.c_str());

    handle_reconfigure_state_transition(*(reconfigure_state_buffer_.readFromRT()));
  }

  // get the current command
  auto current_command = service_buffer_.readFromRT();
  switch (*(service_buffer_.readFromRT()))
  {
  case service_mode_type::UNDEFINED:
    // do nothing
    break;
  case service_mode_type::OPEN:
    handle_gripper_state_transition_open(*(gripper_state_buffer_.readFromRT()));
    break;
  case service_mode_type::CLOSE:
    handle_gripper_state_transition_close(*(gripper_state_buffer_.readFromRT()));
    break;
  
  default:
    break;
  }
  
  // this is publishing the joint states for the gripper and reconfigure
  publish_gripper_joint_states();
  publish_dynamic_interface_values();

  return controller_interface::return_type::OK;
}

bool GripperIOController::find_and_set_command(const std::string & name, const double value)
{
  auto it = std::find_if(
    command_interfaces_.begin(), command_interfaces_.end(),
    [&](const hardware_interface::LoanedCommandInterface & command_interface)
    {
      return command_interface.get_name() == name;
    });

  if (it != command_interfaces_.end())
  {
    it->set_value(value);
    return true;
  }
  return false;
}

bool GripperIOController::find_and_get_state(const std::string & name, double& value)
{
  auto it = std::find_if(
    state_interfaces_.begin(), state_interfaces_.end(),
    [&](const hardware_interface::LoanedStateInterface & state_interface)
    {
      return state_interface.get_name() == name;
    });

  if (it != state_interfaces_.end())
  {
    value = it->get_value();
    return true;
  }
  value = 0.0f;
  return false;
}

bool GripperIOController::find_and_get_command(const std::string & name, double& value)
{
  auto it = std::find_if(
    command_interfaces_.begin(), command_interfaces_.end(),
    [&](const hardware_interface::LoanedCommandInterface & command_interface)
    {
      return command_interface.get_name() == name;
    });

  if (it != command_interfaces_.end())
  {
    value = it->get_value();
    return true;
  }
  value = 0.0f;
  return false;
}


void GripperIOController::handle_gripper_state_transition_close(const gripper_state_type & state)
{
      switch (state)
      {
      case gripper_state_type::UNDEFINED:
        // do nothing
        break;
      case gripper_state_type::SET_BEFORE_COMMAND:
        RCLCPP_INFO(get_node()->get_logger(), "Setting the state of the gripper before opening");

        // set before closeing
      for (size_t i = 0; i < set_before_command_close.size(); ++i)
      {
        setResult = find_and_set_command(set_before_command_close[i], set_before_command_close_values[i]);
        if (!setResult)
        {
          RCLCPP_ERROR(
            get_node()->get_logger(), "Failed to set the command state for %s", set_before_command_close[i].c_str());
        }
        else 
        {
          RCLCPP_INFO(get_node()->get_logger(), "Setting the command state for %s value to %f", set_before_command_close[i].c_str(), set_before_command_close_values[i]);
        }
      }


        gripper_state_buffer_.writeFromNonRT(gripper_state_type::CLOSE_GRIPPER);
        break;
      case gripper_state_type::CLOSE_GRIPPER:
        RCLCPP_INFO(get_node()->get_logger(), "Closing the gripper");

        for (size_t i = 0; i < command_ios_close.size(); ++i)
      {
        setResult = find_and_set_command(command_ios_close[i], command_ios_close_values[i]);
        if (!setResult)
        {
          RCLCPP_ERROR(
            get_node()->get_logger(), "Failed to set the command state for %s", command_ios_close[i].c_str());
        }
        else 
        {
          RCLCPP_INFO(get_node()->get_logger(), "Setting the command state for %s value to %f", command_ios_close[i].c_str(), command_ios_close_values[i]);
        }
      }
        
        gripper_state_buffer_.writeFromNonRT(gripper_state_type::CHECK_GRIPPER_STATE);
        break;
      case gripper_state_type::CHECK_GRIPPER_STATE:
        RCLCPP_INFO(get_node()->get_logger(), "Checking the state of the gripper");
        // check the state of the gripper
        check_state_ios_ = false;

        for (size_t i = 0; i < state_ios_close.size(); ++i)
        {
          setResult = find_and_get_state(state_ios_close[i], state_value_);
          if (!setResult)
          {
            RCLCPP_ERROR(
              get_node()->get_logger(), "Failed to get the state for %s", state_ios_close[i].c_str());
          }
          else {
            if (abs(state_value_ - state_ios_close_values[i]) < std::numeric_limits<double>::epsilon())
            {
              check_state_ios_ = true;
            }
            else {
              check_state_ios_ = false;
              break;
            }
          }
        }
        if(check_state_ios_)
        {
          RCLCPP_INFO(get_node()->get_logger(), "State values are as expected");
          gripper_state_buffer_.writeFromNonRT(gripper_state_type::SET_AFTER_COMMAND);
        }
        break;
      case gripper_state_type::SET_AFTER_COMMAND:
        RCLCPP_INFO(get_node()->get_logger(), "Setting the state of the gripper after closing");

        // set after closeing
        for (size_t i = 0; i < set_after_command_close.size(); ++i)
        {
          setResult = find_and_set_command(set_after_command_close[i], set_after_command_close_values[i]);
          if (!setResult)
          {
            RCLCPP_ERROR(
              get_node()->get_logger(), "Failed to set the command state for %s", set_after_command_close[i].c_str());
          }
          else 
          {
            RCLCPP_INFO(get_node()->get_logger(), "Setting the command state for %s value to %f", set_after_command_close[i].c_str(), set_after_command_close_values[i]);
          }
        }

        gripper_state_buffer_.writeFromNonRT(gripper_state_type::UNDEFINED);
        RCLCPP_INFO(get_node()->get_logger(), "Gripper closed");
        closeFlag_ = false;
        service_buffer_.writeFromNonRT(service_mode_type::UNDEFINED);
        break;
      default:
        break;
      }
}


void GripperIOController::handle_gripper_state_transition_open(const gripper_state_type & state)
{
      switch (state)
      {
      case gripper_state_type::UNDEFINED:
        // do nothing
        break;
      case gripper_state_type::SET_BEFORE_COMMAND:
        RCLCPP_INFO(get_node()->get_logger(), "Setting the state of the gripper before opening");

        // set before opening
        for (size_t i = 0; i < set_before_command_open.size(); ++i)
        {
          setResult = find_and_set_command(set_before_command_open[i], set_before_command_open_values[i]);
          if (!setResult)
          {
            RCLCPP_ERROR(
              get_node()->get_logger(), "Failed to set the command state for %s", set_before_command_open[i].c_str());
          }
          else 
          {
            RCLCPP_INFO(get_node()->get_logger(), "Setting the command state for %s value to %f", set_before_command_open[i].c_str(), set_before_command_open_values[i]);
          }
        }
        
        gripper_state_buffer_.writeFromNonRT(gripper_state_type::OPEN_GRIPPER); 
        break;
      case gripper_state_type::OPEN_GRIPPER:
        RCLCPP_INFO(get_node()->get_logger(), "Opening the gripper");

        // now open the gripper
        for (size_t i = 0; i < command_ios_open.size(); ++i)
        {
          setResult = find_and_set_command(command_ios_open[i], command_ios_open_values[i]);
          if (!setResult)
          {
            RCLCPP_ERROR(
              get_node()->get_logger(), "Failed to set the command state for %s", command_ios_open[i].c_str());
          }
          else 
          {
            RCLCPP_INFO(get_node()->get_logger(), "Setting the command state for %s value to %f", command_ios_close[i].c_str(), command_ios_close_values[i]);
          }
        }
        
        gripper_state_buffer_.writeFromNonRT(gripper_state_type::CHECK_GRIPPER_STATE);
        break;
      case gripper_state_type::CHECK_GRIPPER_STATE:
        RCLCPP_INFO(get_node()->get_logger(), "Checking the state of the gripper");
        // check the state of the gripper
        check_state_ios_ = false;

        for (size_t i = 0; i < state_ios_open.size(); ++i)
        {
          setResult = find_and_get_state(state_ios_open[i], state_value_);
          if (!setResult)
          {
            RCLCPP_ERROR(
              get_node()->get_logger(), "Failed to get the state for %s", state_ios_open[i].c_str());
          }
          else {
            if (abs(state_value_ - state_ios_open_values[i]) < std::numeric_limits<double>::epsilon())
            {
              check_state_ios_ = true;
            }
            else {
              check_state_ios_ = false;
              RCLCPP_INFO(get_node()->get_logger(), "State value for %s is not as expected", state_ios_open[i].c_str());
              break;
            }
          }
        }
        if(check_state_ios_)
        {
          RCLCPP_INFO(get_node()->get_logger(), "State values are as expected");
          gripper_state_buffer_.writeFromNonRT(gripper_state_type::SET_AFTER_COMMAND);
        }
        break;
      case gripper_state_type::SET_AFTER_COMMAND:
        RCLCPP_INFO(get_node()->get_logger(), "Setting the state of the gripper after opening");

        // set after opening
        for (size_t i = 0; i < set_after_command_open.size(); ++i)
        {
          setResult = find_and_set_command(set_after_command_open[i], set_after_command_open_values[i]);
          if (!setResult)
          {
            RCLCPP_ERROR(
              get_node()->get_logger(), "Failed to set the command state for %s", set_after_command_open[i].c_str());
          }
          else 
          {
            RCLCPP_INFO(get_node()->get_logger(), "Setting the command state for %s value to %f", set_after_command_open[i].c_str(), set_after_command_open_values[i]);
          }
        }
        gripper_state_buffer_.writeFromNonRT(gripper_state_type::UNDEFINED);
        RCLCPP_INFO(get_node()->get_logger(), "Gripper opened");
        openFlag_ = false;
        service_buffer_.writeFromNonRT(service_mode_type::UNDEFINED);
        break;
      default:
        break;
      }
}

void GripperIOController::handle_reconfigure_state_transition(const reconfigure_state_type & state)
{
  switch (state)
    {
    case reconfigure_state_type::UNDEFINED:
      // do nothing
      break;
    case reconfigure_state_type::FIND_CONFIG:
      // find the configuration
      RCLCPP_INFO(get_node()->get_logger(), "Finding the configuration");

      config_index_ = std::find(configurations_list_.begin(), configurations_list_.end(), configuration_key_);


      if (config_index_ == configurations_list_.end())
      {
        RCLCPP_ERROR(get_node()->get_logger(), "Configuration not found");
        reconfigure_state_buffer_.writeFromNonRT(reconfigure_state_type::UNDEFINED);
      }
      else
      {
        RCLCPP_INFO(get_node()->get_logger(), "Configuration found");
        conf_it_ = config_map_[std::distance(configurations_list_.begin(), config_index_)];
        reconfigure_state_buffer_.writeFromNonRT(reconfigure_state_type::SET_COMMAND);
      }

      break;

    case reconfigure_state_type::SET_COMMAND:

      RCLCPP_INFO(get_node()->get_logger(), "Setting the command state for the gripper");

      setResult = false;

      for (const auto & io : conf_it_.command_high)
      {
        setResult = find_and_set_command(io, 1.0);
        if (!setResult)
        {
          RCLCPP_ERROR(
            get_node()->get_logger(), "Failed to set the command state for %s", io.c_str());
        }
        else 
        {
          RCLCPP_INFO(get_node()->get_logger(), "Setting the command state for %s value to %f", io.c_str(), 1.0);
        }
      }
      for (const auto & io : conf_it_.command_low)
      {
        setResult = find_and_set_command(io, 0.0);
        if (!setResult)
        {
          RCLCPP_ERROR(
            get_node()->get_logger(), "Failed to set the command state for %s", io.c_str());
        }
        else 
        {
          RCLCPP_INFO(get_node()->get_logger(), "Setting the command state for %s value to %f", io.c_str(), 0.0);
        }
      }

      reconfigure_state_buffer_.writeFromNonRT(reconfigure_state_type::CHECK_STATE);

      break;

    case reconfigure_state_type::CHECK_STATE:

      RCLCPP_INFO(get_node()->get_logger(), "Checking the state of the gripper");
      check_state_ios_ = false;

      // implement the code read the state of the gripper
      for (const auto & io : conf_it_.state_high)
      {
        // read the state of the gripper
        setResult = find_and_get_state(io, state_value_);
        // if the state is not as expected, then set the state to the expected state
        if (!setResult)
        {
          check_state_ios_ = false;
          RCLCPP_ERROR(
            get_node()->get_logger(), "Failed to get the state for %s", io.c_str());
        }
        else 
        {
            if (!(std::abs(state_value_ - 1.0) < std::numeric_limits<double>::epsilon()))
          {
            check_state_ios_ = false;
            RCLCPP_ERROR(
              get_node()->get_logger(), "value for state doesn't match %s", io.c_str());
            break;
          }
          else {
            check_state_ios_ = true;
          }
        }
        // if the state is as expected, then do nothing
      }

      for (const auto & io : conf_it_.state_low)
      {
        // read the state of the gripper
        setResult = find_and_get_state(io, state_value_);
        // if the state is not as expected, then set the state to the expected state
        if (!setResult)
        {
          RCLCPP_ERROR(
            get_node()->get_logger(), "Failed to get the state for %s", io.c_str());
        }
        else
        { 
          if (!(std::abs(state_value_ - 0.0) < std::numeric_limits<double>::epsilon()))
          {
            RCLCPP_ERROR(
              get_node()->get_logger(), "value doesn't match %s", io.c_str());
            check_state_ios_ = false;
            break;
          }
          else 
          {
            check_state_ios_ = true;
          }
        }
        // if the state is as expected, then do nothing
      }

      if (check_state_ios_)
      {
        RCLCPP_INFO(get_node()->get_logger(), "Reconfigured successfully to configuration: %s", configuration_key_.c_str());

        // publish_reconfigure_gripper_joint_states(); // TODO : remove later, not required

        reconfigure_state_buffer_.writeFromNonRT(reconfigure_state_type::UNDEFINED);
        configure_gripper_buffer_.writeFromNonRT(""); // this is not required, remove later TODO (Sachin) :s
        reconfigureFlag_ = false;
      }
      break;
    
    default:
      break;
    }
}

controller_interface::CallbackReturn GripperIOController::check_parameters()
{
  /// Param validation 

  if (params_.open_close_joints.empty())
  {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "Size of open close joints parameter cannot be zero.");
    return CallbackReturn::FAILURE;
  }


  // size of open_close_joint should match with the open.joint_states and close.joint_states  
  if (params_.open_close_joints.size() != params_.open.joint_states.size() &&
      params_.open_close_joints.size() != params_.close.joint_states.size())
  {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "Size of open close joints parameter should match with the open.joint_states and close.joint_states.");
    return CallbackReturn::FAILURE;
  }


  if (params_.open.joint_states.empty())
  {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "Size of joint states parameter cannot be zero.");
    return CallbackReturn::FAILURE;
  }

  if (params_.open.set_after_command.high.empty() and params_.open.set_after_command.low.empty())
  {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "Size of set before command high and low parameters cannot be zero.");
    return CallbackReturn::FAILURE;
  }


  if (params_.open.command.high.empty() and params_.open.command.low.empty())
  {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "Size of open command high and low parameters cannot be zero.");
    return CallbackReturn::FAILURE;
  }

  if (params_.open.state.high.empty() and params_.open.state.low.empty())
  {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "Size of open state high and low parameters cannot be zero.");
    return CallbackReturn::FAILURE;
  }

  // close parameters
  if (params_.close.joint_states.empty())
  {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "Size of joint states parameter cannot be zero.");
    return CallbackReturn::FAILURE;
  }

  if (params_.close.set_before_command.high.empty() and params_.close.set_before_command.low.empty())
  {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "Size of set before command high and low parameters cannot be zero.");
    return CallbackReturn::FAILURE;
  }


  if (params_.close.command.high.empty() and params_.close.command.low.empty())
  {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "Size of close command high and low parameters cannot be zero.");
    return CallbackReturn::FAILURE;
  }

  if (params_.close.state.high.empty() and params_.close.state.low.empty())
  {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "Size of close state high and low parameters cannot be zero.");
    return CallbackReturn::FAILURE;
  }


  // configurations parameter
  if (params_.configurations.empty())
  {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "Size of configurations parameter cannot be zero.");
    return CallbackReturn::FAILURE;
  }

  // configuration joints parameter
  if (params_.configuration_joints.empty())
  {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "Size of configuration joints parameter cannot be zero.");
    return CallbackReturn::FAILURE;
  }

  // configuration setup parameter
  if (params_.configuration_setup.configurations_map.empty())
  {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "Size of configuration map parameter cannot be zero.");
    return CallbackReturn::FAILURE;
  }

  // gripper_specific_sensors parameter
  if (params_.gripper_specific_sensors.empty())
  {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "Size of gripper specific sensors parameter cannot be zero.");
    return CallbackReturn::FAILURE;
  }

  // sensors interfaces parameter
  if (params_.sensors_interfaces.gripper_specific_sensors_map.empty())
  {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "Size of sensors interfaces parameter cannot be zero.");
    return CallbackReturn::FAILURE;
  }

  // if sensor input string is empty then return failure
  for (const auto& [key, val] : params_.sensors_interfaces.gripper_specific_sensors_map)
  {
    if (val.input == "")
    {
      RCLCPP_FATAL(
        get_node()->get_logger(),
        "Size of sensor input string parameter cannot be empty ("").");
      return CallbackReturn::FAILURE;
    }
  }
  return CallbackReturn::SUCCESS;
}

void GripperIOController::prepare_command_and_state_ios()
{
  // make full command ios lists -- just once
  for (const auto& key : params_.open.set_after_command.high) {
    set_before_command_open.push_back(key);
    set_before_command_open_values.push_back(1.0);
    command_if_ios.insert(key);
    state_if_ios.insert(key);
  }

  for (const auto& key : params_.open.set_after_command.low) {
    set_before_command_open.push_back(key);
    set_before_command_open_values.push_back(0.0);
    command_if_ios.insert(key);
    state_if_ios.insert(key);
  } 
  for (const auto& key : params_.open.command.high) {
    command_ios_open.push_back(key);
    command_ios_open_values.push_back(1.0);
    // command_ios_open[key] = 1.0;
    command_if_ios.insert(key);
    state_if_ios.insert(key);
  }

  for (const auto& key : params_.open.command.low) {
    command_ios_open.push_back(key);
    command_ios_open_values.push_back(0.0);
    // command_ios_open[key] = 0.0;
    command_if_ios.insert(key);
    state_if_ios.insert(key);
  }

  for (const auto& key : params_.open.set_after_command.high) {
    set_after_command_open.push_back(key);
    set_after_command_open_values.push_back(1.0);
    command_if_ios.insert(key);
    state_if_ios.insert(key);
  }

  for (const auto& key : params_.open.set_after_command.low) {
    set_after_command_open.push_back(key);
    set_after_command_open_values.push_back(0.0);
    command_if_ios.insert(key);
    state_if_ios.insert(key);
  }

  for (const auto& key : params_.close.set_before_command.high) {
    set_before_command_close.push_back(key);
    set_before_command_close_values.push_back(1.0);
    command_if_ios.insert(key);
    state_if_ios.insert(key);
  }

  for (const auto& key : params_.close.set_before_command.low) {
    set_before_command_close.push_back(key);
    set_before_command_close_values.push_back(0.0);
    command_if_ios.insert(key);
    state_if_ios.insert(key);
  }

  for (const auto& key : params_.close.command.high) {
    command_ios_close.push_back(key);
    command_ios_close_values.push_back(1.0);
    // command_ios_close[key] = 1.0;
    command_if_ios.insert(key);
    state_if_ios.insert(key);
  }
  for (const auto& key : params_.close.command.low) {
    command_ios_close.push_back(key);
    command_ios_close_values.push_back(0.0);
    // command_ios_close[key] = 0.0;
    command_if_ios.insert(key);
    state_if_ios.insert(key);
  }


  // make full state ios lists -- just once
  for (const auto& key : params_.open.state.high) {
    state_ios_open.push_back(key);
    state_ios_open_values.push_back(1.0);
    // state_ios_open[key] = 1.0;
    state_if_ios.insert(key);
  }
  for (const auto& key : params_.open.state.low) {
    state_ios_open.push_back(key);
    state_ios_open_values.push_back(0.0);
    // state_ios_open[key] = 0.0;
    state_if_ios.insert(key);
  }
  for (const auto& key : params_.close.state.high) {
    state_ios_close.push_back(key);
    state_ios_close_values.push_back(1.0);
    // state_ios_close[key] = 1.0;
    state_if_ios.insert(key);
  }
  for (const auto& key : params_.close.state.low) {
    state_ios_close.push_back(key);
    state_ios_close_values.push_back(0.0);
    // state_ios_close[key] = 0.0;
    state_if_ios.insert(key);
  }

  for (const auto& key : params_.close.set_after_command.high) {
    set_after_command_close.push_back(key);
    set_after_command_close_values.push_back(1.0);
    command_if_ios.insert(key);
    state_if_ios.insert(key);
  }

  for (const auto& key : params_.close.set_after_command.low) {
    set_after_command_close.push_back(key);
    set_after_command_close_values.push_back(0.0);
    command_if_ios.insert(key);
    state_if_ios.insert(key);
  }
  
  // get the configurations for different io which needs to be high or low
  for (const auto & [key, val] : params_.configuration_setup.configurations_map)
  {
    config_map_.push_back(val);
  }

  // get the configurations list
  configurations_list_ = params_.configurations;

  for (const auto & config : config_map_)
  {
    for (const auto & io : config.command_high)
    {
      command_if_ios.insert(io);
      state_if_ios.insert(io);
    }
    for (const auto & io : config.command_low)
    {
      command_if_ios.insert(io);
      state_if_ios.insert(io);
    }
    for (const auto & io : config.state_high)
    {
      state_if_ios.insert(io);
    }
    for (const auto & io : config.state_low)
    {
      state_if_ios.insert(io);
    }
  }
  RCLCPP_INFO(get_node()->get_logger(), "Size of gripper_specific_sensors_map: %zu", params_.sensors_interfaces.gripper_specific_sensors_map.size());

  // get the configurations for different io which needs to be high or low
  for (const auto & [key, val] : params_.sensors_interfaces.gripper_specific_sensors_map)
  {
    sensors_map_.push_back(val);
  }


  // TODO (Sachin) : Add the gripper specific sensors to the state_if_ios, not able to access the values, discuss this with Dr. Denis
  for (size_t i = 0; i < params_.gripper_specific_sensors.size(); ++i)
  {
    state_if_ios.insert(params_.sensors_interfaces.gripper_specific_sensors_map.at(params_.gripper_specific_sensors[i]).input);
  }
}

controller_interface::CallbackReturn GripperIOController::prepare_publishers_and_services()
{

  reconfigureFlag_ = false;


  // topics QoS
  /*auto subscribers_qos = rclcpp::SystemDefaultsQoS();
  subscribers_qos.keep_last(1);
  subscribers_qos.best_effort();*/

  // slow control mode service
  auto set_slow_mode_service_callback =
    [&](
      const std::shared_ptr<ControllerModeSrvType::Request> request,
      std::shared_ptr<ControllerModeSrvType::Response> response)
  {
    if (request->data)
    {
      control_mode_.writeFromNonRT(control_mode_type::SLOW);
    }
    else
    {
      control_mode_.writeFromNonRT(control_mode_type::FAST);
    }
    response->success = true;
  };

  set_slow_control_mode_service_ = get_node()->create_service<ControllerModeSrvType>(
    "~/set_slow_control_mode", set_slow_mode_service_callback,
    rmw_qos_profile_services_hist_keep_all);

  // reset service buffer
  service_buffer_.writeFromNonRT(service_mode_type::UNDEFINED);

  // reset gripper state buffer
  gripper_state_buffer_.writeFromNonRT(gripper_state_type::UNDEFINED);

  // callback groups for each service
  open_service_callback_group_ = get_node()->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  close_service_callback_group_ = get_node()->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  reconfigure_service_callback_group_ = get_node()->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  
  // open close action server

  gripper_feedback_ = std::make_shared<GripperAction::Feedback>();
  gripper_result_ = std::make_shared<GripperAction::Result>();
  gripper_action_server_ = rclcpp_action::create_server<GripperAction>(
    get_node(), "~/gripper_action",
    std::bind(&GripperIOController::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&GripperIOController::handle_cancel, this, std::placeholders::_1),
    std::bind(&GripperIOController::handle_accepted, this, std::placeholders::_1));

  // open service
  auto open_service_callback =
    [&](
      const std::shared_ptr<OpenSrvType::Request> /*request*/,
      std::shared_ptr<OpenSrvType::Response> response)
  {
    try
    {
      RCLCPP_INFO(get_node()->get_logger(), "Service called for open");
      if (reconfigureFlag_)
      {
        RCLCPP_ERROR(get_node()->get_logger(), "Cannot close the gripper while reconfiguring");
        response->success = false;
        return;
      }
      // Check if an action is running
      if (*(gripper_state_buffer_.readFromRT()) != gripper_state_type::UNDEFINED)
      {
        RCLCPP_INFO(get_node()->get_logger(), "Canceling current action for open service");
        gripper_state_buffer_.writeFromNonRT(gripper_state_type::HALTED);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500)); // wait for 500 ms
      if (closeFlag_)
      {
        closeFlag_ = false;
      }
      openFlag_ = true;
      service_buffer_.writeFromNonRT(service_mode_type::OPEN);
      gripper_state_buffer_.writeFromNonRT(gripper_state_type::SET_BEFORE_COMMAND);
      while(openFlag_)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if ((*(gripper_state_buffer_.readFromRT()) == gripper_state_type::HALTED))
        {
          response->success = false;
          break;
        }
        else {
          response->success = true;
        }
      }
    }
    catch (const std::exception & e)
    {
      response->success = false;
    }
  };

  open_service_ = get_node()->create_service<OpenSrvType>(
    "~/gripper_open", open_service_callback,
    rmw_qos_profile_services_hist_keep_all, open_service_callback_group_);

  // close service
  auto close_service_callback =
    [&](
      const std::shared_ptr<OpenSrvType::Request> /*request*/,
      std::shared_ptr<OpenSrvType::Response> response)
  {
    try
    {
      RCLCPP_INFO(get_node()->get_logger(), "Service called for close");
      if (reconfigureFlag_)
      {
        RCLCPP_ERROR(get_node()->get_logger(), "Cannot close the gripper while reconfiguring");
        response->success = false;
        return;
      }
      // Check if an action is running
      if (*(gripper_state_buffer_.readFromRT()) != gripper_state_type::UNDEFINED)
      {
        RCLCPP_INFO(get_node()->get_logger(), "Canceling current action for close service");
        gripper_state_buffer_.writeFromNonRT(gripper_state_type::HALTED);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500)); // wait for 500 ms
      service_buffer_.writeFromNonRT(service_mode_type::CLOSE);
      gripper_state_buffer_.writeFromNonRT(gripper_state_type::SET_BEFORE_COMMAND);
      if (openFlag_)
      {
        openFlag_ = false;
      }
      closeFlag_ = true;
      while(closeFlag_)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if ((*(gripper_state_buffer_.readFromRT()) == gripper_state_type::HALTED))
        {
          response->success = false;
          break;
        }
        else {
          response->success = true;
        }
      }
    }
    catch (const std::exception & e)
    {
      response->success = false;
    }
  };

  close_service_ = get_node()->create_service<OpenSrvType>(
    "~/gripper_close", close_service_callback,
    rmw_qos_profile_services_hist_keep_all, close_service_callback_group_);


  // open close action server

  gripper_config_feedback_ = std::make_shared<GripperConfigAction::Feedback>();
  gripper_config_result_ = std::make_shared<GripperConfigAction::Result>();
  gripper_config_action_server_ = rclcpp_action::create_server<GripperConfigAction>(
    get_node(), "~/reconfigure_gripper_action",
    std::bind(&GripperIOController::config_handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&GripperIOController::config_handle_cancel, this, std::placeholders::_1),
    std::bind(&GripperIOController::config_handle_accepted, this, std::placeholders::_1));

  // configure gripper service
  // TODO (Sachin) : Change service type to string
  auto configure_gripper_service_callback =
    [&](
      const std::shared_ptr<ConfigSrvType::Request> request,
      std::shared_ptr<ConfigSrvType::Response> response)
  {
    try
    {
      std::string conf = request->config_name;
      configure_gripper_buffer_.writeFromNonRT(conf.c_str());
      reconfigure_state_buffer_.writeFromNonRT(reconfigure_state_type::FIND_CONFIG);
      reconfigureFlag_ = true;
      // wait with thread sleep, until certain flag is not set
      while(reconfigureFlag_)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      response->result = true;
      response->status = "Gripper reconfigured";
    }
    catch (const std::exception & e)
    {
      response->result = false;
      response->status = "Failed to reconfigure gripper";
    }
  };

  configure_gripper_service_ = get_node()->create_service<ConfigSrvType>(
    "~/reconfigure_to", configure_gripper_service_callback,
    rmw_qos_profile_services_hist_keep_all, reconfigure_service_callback_group_);

  try
  {
    // State publisher
    g_j_s_publisher_ =
      get_node()->create_publisher<ControllerStateMsg>("/joint_states", rclcpp::SystemDefaultsQoS());
    gripper_joint_state_publisher_ = std::make_unique<ControllerStatePublisher>(g_j_s_publisher_);
  }
  catch (const std::exception & e)
  {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  try
  {
    // Config Publisher
    c_publisher_ = get_node()->create_publisher<ConfigJointMsg>(
      "/configuration_joint_states", rclcpp::SystemDefaultsQoS());
    config_publisher_ = std::make_unique<ConfigPublisher>(c_publisher_);
  }
  catch (const std::exception & e)
  {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  try
  {
    // interface publisher
    if_publisher_ = get_node()->create_publisher<InterfaceMsg>(
      "~/dynamic_interface", rclcpp::SystemDefaultsQoS());
    interface_publisher_ = std::make_unique<InterfacePublisher>(if_publisher_);
  }
  catch(const std::exception& e)
  {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

void GripperIOController::publish_gripper_joint_states()
{
  if (gripper_joint_state_publisher_ && gripper_joint_state_publisher_->trylock())
  {
    gripper_joint_state_publisher_->msg_.header.stamp = get_node()->get_clock()->now();  // make sure this works and also discuss with Dr. Denis TODO (Sachin): time;
    for (size_t joint_name_index = 0; joint_name_index < params_.open_close_joints.size(); joint_name_index++)
    {
      gripper_joint_state_publisher_->msg_.name.clear(); // TODO (Sachin) : Change code to initialize these values once
      gripper_joint_state_publisher_->msg_.position.clear();
      gripper_joint_state_publisher_->msg_.name.push_back(params_.open_close_joints[joint_name_index]);
    
      for (size_t i = 0; i < state_ios_open.size(); ++i)
      {
        setResult = find_and_get_state(state_ios_open[i], state_value_);
        if (!setResult)
        {
          RCLCPP_ERROR(
            get_node()->get_logger(), "Failed to get the state for %s", state_ios_open[i].c_str());
        }
        else {
          RCLCPP_DEBUG(get_node()->get_logger(), "state_value_: %f, state_ios_open_values[%zu]: %f", state_value_, i, state_ios_open_values[i]);
          if (std::abs(state_value_ - state_ios_open_values[i]) < std::numeric_limits<double>::epsilon())
          {
            is_open = true;
          }
          else {
            is_open = false;
            break;
          }
        }
      }
      if (is_open)
      {
        gripper_joint_state_publisher_->msg_.position.push_back(params_.open.joint_states[joint_name_index]);

        RCLCPP_DEBUG(get_node()->get_logger(), "Gripper is open");
      }
      else
      {
        gripper_joint_state_publisher_->msg_.position.push_back(params_.close.joint_states[joint_name_index]);
        RCLCPP_DEBUG(get_node()->get_logger(), "Gripper is closed");
      }
    }

    // find the joint value for reconfigure
      for (const auto & [key, configValues] : params_.configuration_setup.configurations_map)
      {
        if (key != configuration_key_)
        {
          continue;
        }
        else
        {
          bool publishConfig = false;

        for (size_t i = 0; i < configValues.state_high.size(); ++i)
        {
          setResult = find_and_get_state(configValues.state_high[i], state_value_);
          if (!setResult)
          {
            RCLCPP_ERROR(
              get_node()->get_logger(), "Failed to get the state for %s", configValues.state_high[i].c_str());
          }
          else {
            if (abs(state_value_ - 1.0) < std::numeric_limits<double>::epsilon())
            {
              publishConfig = true;
            }
            else {
              publishConfig = false;
              RCLCPP_INFO(get_node()->get_logger(), "State value for %s is not as expected", configValues.state_high[i].c_str());
              break;
            }
          }
        }

        for (size_t i = 0; i < configValues.state_low.size(); ++i)
        {
          setResult = find_and_get_state(configValues.state_low[i], state_value_);
          if (!setResult)
          {
            RCLCPP_ERROR(
              get_node()->get_logger(), "Failed to get the state for %s", configValues.state_low[i].c_str());
          }
          else {
            if (abs(state_value_ - 0.0) < std::numeric_limits<double>::epsilon())
            {
              publishConfig = true;
            }
            else {
              publishConfig = false;
              RCLCPP_INFO(get_node()->get_logger(), "State value for %s is not as expected", configValues.state_low[i].c_str());
              break;
            }
          }
        }

        if (publishConfig)
        {
          gripper_joint_state_publisher_->msg_.name.push_back(configuration_key_);
          gripper_joint_state_publisher_->msg_.position.push_back(configValues.joint_states[0]);
        }
        else
        {
          RCLCPP_INFO(get_node()->get_logger(), "Configuration values are not as expected");
        }
      }
    }
  }
  gripper_joint_state_publisher_->unlockAndPublish();

}

void GripperIOController::publish_dynamic_interface_values()
{
  if (interface_publisher_ && interface_publisher_->trylock())
  {
    interface_publisher_->msg_.header.stamp = get_node()->get_clock()->now(); // Make sure it works and discuss with Dr. Denis
    interface_publisher_->msg_.states.interface_names.clear();
    interface_publisher_->msg_.states.values.clear();
    interface_publisher_->msg_.states.values.resize(state_interfaces_.size());
    for (size_t i = 0; i < state_interfaces_.size(); ++i)
    {
      interface_publisher_->msg_.states.interface_names.push_back(state_interfaces_.at(i).get_name()); // this can be done in a separate function one time. Change it later TODO (Sachin) :
      interface_publisher_->msg_.states.values.at(i) = static_cast<float>(state_interfaces_.at(i).get_value());
    }

    interface_publisher_->msg_.commands.interface_names.clear();
    interface_publisher_->msg_.commands.values.clear();
    interface_publisher_->msg_.commands.values.resize(command_interfaces_.size());
    for (size_t i = 0; i < command_interfaces_.size(); ++i)
    {
      interface_publisher_->msg_.commands.interface_names.push_back(command_interfaces_.at(i).get_name()); // this can be done in a separate function one time. Change it later TODO (Sachin) :
      interface_publisher_->msg_.commands.values.at(i) = static_cast<float>(command_interfaces_.at(i).get_value());
    }
    interface_publisher_->unlockAndPublish();
  }
}


void GripperIOController::publish_reconfigure_gripper_joint_states()
{
  RCLCPP_INFO(get_node()->get_logger(), "Publishing the reconfigured gripper joint states");
  if (config_publisher_ && config_publisher_->trylock())
    {
      for (const auto & [key, configValues] : params_.configuration_setup.configurations_map)
      {
        if (key != configuration_key_)
        {
          continue;
        }
        else
        {
          bool publishConfig = false;

        for (size_t i = 0; i < configValues.state_high.size(); ++i)
        {
          RCLCPP_INFO(get_node()->get_logger(), "Getting the state for %s", configValues.state_high[i].c_str());
          setResult = find_and_get_state(configValues.state_high[i], state_value_);
          if (!setResult)
          {
            RCLCPP_ERROR(
              get_node()->get_logger(), "Failed to get the state for %s", configValues.state_high[i].c_str());
          }
          else {
            if (abs(state_value_ - 1.0) < std::numeric_limits<double>::epsilon())
            {
              publishConfig = true;
            }
            else {
              publishConfig = false;
              RCLCPP_INFO(get_node()->get_logger(), "State value for %s is not as expected", configValues.state_high[i].c_str());
              break;
            }
          }
        }

        for (size_t i = 0; i < configValues.state_low.size(); ++i)
        {
          RCLCPP_INFO(get_node()->get_logger(), "Getting the state for %s", configValues.state_low[i].c_str());
          setResult = find_and_get_state(configValues.state_low[i], state_value_);
          if (!setResult)
          {
            RCLCPP_ERROR(
              get_node()->get_logger(), "Failed to get the state for %s", configValues.state_low[i].c_str());
          }
          else {
            if (abs(state_value_ - 0.0) < std::numeric_limits<double>::epsilon())
            {
              publishConfig = true;
            }
            else {
              publishConfig = false;
              RCLCPP_INFO(get_node()->get_logger(), "State value for %s is not as expected", configValues.state_low[i].c_str());
              break;
            }
          }
        }

        if (publishConfig)
        {
          config_publisher_->msg_.header.stamp = get_node()->get_clock()->now(); // TODO (Sachin): discuss with Dr. Denis time;
          config_publisher_->msg_.name.clear();
          config_publisher_->msg_.position.clear();
          config_publisher_->msg_.name.push_back(configuration_key_);
          config_publisher_->msg_.position.push_back(configValues.joint_states[0]);
        }
        else
        {
          RCLCPP_INFO(get_node()->get_logger(), "Configuration values are not as expected");
        }
        config_publisher_->unlockAndPublish();

        }
      
    }
  }
  
}



  rclcpp_action::GoalResponse GripperIOController::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const GripperAction::Goal> goal)
    {

      RCLCPP_INFO(get_node()->get_logger(), "Received request to execute goal");
      
      try
      {
        if (*(gripper_state_buffer_.readFromRT()) != gripper_state_type::UNDEFINED)
        {
          RCLCPP_INFO(get_node()->get_logger(), "Canceling current services for open/close action");
          gripper_state_buffer_.writeFromNonRT(gripper_state_type::HALTED);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500)); // wait for 500 ms
        RCLCPP_INFO(get_node()->get_logger(), "Action handle goal for open/close gripper");
        if (reconfigureFlag_)
        {
          RCLCPP_ERROR(get_node()->get_logger(), "Cannot close the gripper while reconfiguring");
          return rclcpp_action::GoalResponse::REJECT;
        }
        service_buffer_.writeFromNonRT((goal->open) ? service_mode_type::OPEN : service_mode_type::CLOSE);
        gripper_state_buffer_.writeFromNonRT(gripper_state_type::SET_BEFORE_COMMAND);
      }
      catch (const std::exception & e)
      {
        RCLCPP_ERROR(get_node()->get_logger(), "Exception thrown during action handle goal with message: %s", e.what());
        return rclcpp_action::GoalResponse::REJECT;
      }
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

  rclcpp_action::CancelResponse GripperIOController::handle_cancel(
    const std::shared_ptr<GoalHandleGripper> goal_handle)
    {
      RCLCPP_INFO(get_node()->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
      service_buffer_.writeFromNonRT(service_mode_type::UNDEFINED);
      gripper_state_buffer_.writeFromNonRT(gripper_state_type::UNDEFINED);
      return rclcpp_action::CancelResponse::ACCEPT;
    }

  void GripperIOController::handle_accepted(const std::shared_ptr<GoalHandleGripper> goal_handle)
  {
    // don't think need to do anything here as it is handled in the update function
    std::thread{std::bind(&GripperIOController::execute, this, std::placeholders::_1), goal_handle}.detach();
    
  }

  void GripperIOController::execute(const std::shared_ptr<GoalHandleGripper> goal_handle)
  {
    while (true)
    {
      if (*(gripper_state_buffer_.readFromRT()) == gripper_state_type::UNDEFINED)
      {
        gripper_result_->success = true;
        gripper_result_->message = "Gripper action executed";
        goal_handle->succeed(gripper_result_);
        break;
      }
      else if (*(gripper_state_buffer_.readFromRT()) == gripper_state_type::HALTED)
      {
        gripper_result_->success = false;
        gripper_result_->message = "Gripper action halted";
        goal_handle->abort(gripper_result_);
        break;
      }
      else 
      {
        gripper_feedback_->state = static_cast<uint8_t> (*(gripper_state_buffer_.readFromRT()));
        goal_handle->publish_feedback(gripper_feedback_);
      }
     std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
    }
  }


  rclcpp_action::GoalResponse GripperIOController::config_handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const GripperConfigAction::Goal> goal)
    {
      try
    {
      std::string conf = goal->config_name;
      configure_gripper_buffer_.writeFromNonRT(conf.c_str());
      reconfigure_state_buffer_.writeFromNonRT(reconfigure_state_type::FIND_CONFIG);
      reconfigureFlag_ = true;
      
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Exception thrown during action handle goal with message: %s", e.what());
      return rclcpp_action::GoalResponse::REJECT;
      
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

    }


  rclcpp_action::CancelResponse GripperIOController::config_handle_cancel(
    const std::shared_ptr<GoalHandleGripperConfig> goal_handle)
    {
      RCLCPP_INFO(get_node()->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    configure_gripper_buffer_.writeFromNonRT("");
    reconfigure_state_buffer_.writeFromNonRT(reconfigure_state_type::FIND_CONFIG);
    return rclcpp_action::CancelResponse::ACCEPT;

    }

  void GripperIOController::config_handle_accepted(const std::shared_ptr<GoalHandleGripperConfig> goal_handle)
  {
  std::thread{std::bind(&GripperIOController::config_execute, this, std::placeholders::_1), goal_handle}.detach();
  }
  void GripperIOController::config_execute(const std::shared_ptr<GoalHandleGripperConfig> goal_handle)
  {
    // wait with thread sleep, until certain flag is not set
    while (true)
    {
      if(*(reconfigure_state_buffer_.readFromRT()) == reconfigure_state_type::UNDEFINED)
      {
        gripper_config_result_->result = true;
        gripper_config_result_->status = "Gripper reconfigured";
        goal_handle->succeed(gripper_config_result_);
        break;
      }
      else 
      {
        gripper_config_feedback_->state = static_cast<uint8_t> (*(reconfigure_state_buffer_.readFromRT()));
        goal_handle->publish_feedback(gripper_config_feedback_);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

  }
  

// void GripperIOController::init_msgs()
// {
//   for (const auto & name : state_interfaces_)
//   {
//     interface_msg_.interface_names.push_back(name.get_name());
//   }
//   interface_msg_.values.resize(state_interfaces_.size());
// }
  
}  // namespace gripper_io_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  gripper_io_controller::GripperIOController, controller_interface::ControllerInterface)
