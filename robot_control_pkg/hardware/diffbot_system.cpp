// Copyright 2021 ros2_control Development Team
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

#include "robot_control_pkg/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot_control_pkg
{
// Initialize hardware with configuration parameters
hardware_interface::CallbackReturn RobotControlHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Load configuration parameters from hardware info
  cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);

  // Check if PID parameters are provided; if not, use default values
  if (info_.hardware_parameters.count("pid_p") > 0)
  {
    cfg_.pid_p = std::stoi(info_.hardware_parameters["pid_p"]);
    cfg_.pid_d = std::stoi(info_.hardware_parameters["pid_d"]);
    cfg_.pid_i = std::stoi(info_.hardware_parameters["pid_i"]);
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("RobotControlHardware"), "PID values not supplied, using defaults.");
  }
  
  // Set up wheels with their respective encoder counts per revolution
  wheel_l_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
  wheel_r_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);

  // Validate the joint interfaces
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // Ensure there is only one command interface per joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RobotControlHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Ensure the command interface is of type VELOCITY
    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RobotControlHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Ensure there are exactly two state interfaces per joint
    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RobotControlHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Ensure the first state interface is POSITION
    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RobotControlHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Ensure the second state interface is VELOCITY
    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RobotControlHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// Export state interfaces for wheels
std::vector<hardware_interface::StateInterface> RobotControlHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Left wheel position and velocity
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_l_.name, hardware_interface::HW_IF_POSITION, &wheel_l_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.vel));

  // Right wheel position and velocity
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_.name, hardware_interface::HW_IF_POSITION, &wheel_r_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.vel));

  return state_interfaces;
}

// Export command interfaces for wheels
std::vector<hardware_interface::CommandInterface> RobotControlHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Left wheel velocity command
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.cmd));

  // Right wheel velocity command
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.cmd));

  return command_interfaces;
}

// Configure the hardware, establish connection
hardware_interface::CallbackReturn RobotControlHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RobotControlHardware"), "Configuring ...please wait...");
  if (comms_.connected())
  {
    comms_.disconnect();
  }

  comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);


  RCLCPP_INFO(rclcpp::get_logger("RobotControlHardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

// Clean up by disconnecting the communication
hardware_interface::CallbackReturn RobotControlHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RobotControlHardware"), "Cleaning up ...please wait...");
  if (comms_.connected())
  {
    comms_.disconnect();
  }
  RCLCPP_INFO(rclcpp::get_logger("RobotControlHardware"), "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

// Activate hardware, set PID if necessary
hardware_interface::CallbackReturn RobotControlHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RobotControlHardware"), "Activating ...please wait...");
  if (!comms_.connected())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (cfg_.pid_p > 0)
  {
    comms_.set_pid_values(cfg_.pid_p,cfg_.pid_d,cfg_.pid_i);
  }
  RCLCPP_INFO(rclcpp::get_logger("RobotControlHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

// Deactivate hardware
hardware_interface::CallbackReturn RobotControlHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RobotControlHardware"), "Deactivating ...please wait...");
  RCLCPP_INFO(rclcpp::get_logger("RobotControlHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

// Read encoder data and update wheel positions and velocities
hardware_interface::return_type RobotControlHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  // Read the encoder values from the Arduino
  comms_.read_encoder_values(wheel_l_.enc, wheel_r_.enc);

  // Get time difference
  double delta_seconds = period.seconds();

  // Calculate the wheels' position and velocity in rad and rad/s
  double pos_prev = wheel_l_.pos;
  wheel_l_.pos = wheel_l_.calc_enc_angle();
  wheel_l_.vel = (wheel_l_.pos - pos_prev) / delta_seconds;

  pos_prev = wheel_r_.pos;
  wheel_r_.pos = wheel_r_.calc_enc_angle();
  wheel_r_.vel = (wheel_r_.pos - pos_prev) / delta_seconds;

  return hardware_interface::return_type::OK;
}

// Write velocity commands to the hardware
hardware_interface::return_type robot_control_pkg::RobotControlHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  // Feed the motor values in rad/s to the Arduino
  float motor_l_rad_s = wheel_l_.cmd;
  float motor_r_rad_s = wheel_r_.cmd;
    
  comms_.set_motor_values(motor_l_rad_s, motor_r_rad_s);

  return hardware_interface::return_type::OK;
}

}  // namespace robot_control_pkg

#include "pluginlib/class_list_macros.hpp"
// Export the class as a plugin
PLUGINLIB_EXPORT_CLASS(
  robot_control_pkg::RobotControlHardware, hardware_interface::SystemInterface)
