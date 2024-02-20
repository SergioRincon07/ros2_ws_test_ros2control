// Copyright 2020 ros2_control Development Team
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

#include "test2_ros2control/rrbot.hpp"
#include "test2_ros2control/hardware_constants.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace test2_ros2control
{
// Implementación de la función que se llama al inicializar el hardware
hardware_interface::CallbackReturn RRBotWheelVelocity::on_init(
  const hardware_interface::HardwareInfo & info)
{
  // Verifica si la inicialización del hardware base fue exitosa
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Inicializa parámetros del sistema RRBot
  hw_start_sec_ = stod(info_.hardware_parameters["hw_start_duration_sec"]);
  hw_stop_sec_ = stod(info_.hardware_parameters["hw_stop_duration_sec"]);
  hw_slowdown_ = stod(info_.hardware_parameters["hw_slowdown"]);
  // Inicializa vectores para estados y comandos
  hw_position_state_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocity_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  // Se verifica si cada articulación tiene interfaces válidas de comando y estado.
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // Se verifica si hay exactamente las interfaces de comando en cada articulación en ros2_control.xacro.
    if (joint.command_interfaces.size() != hardware_constants::NUM_COMMANDS)
    {
      RCLCPP_FATAL(
          rclcpp::get_logger("RRBotWheelVelocity"),
          "La articulación '%s' tiene %zu interfaces de comando. Se esperaba 1.", joint.name.c_str(),
          joint.command_interfaces.size());

      return hardware_interface::CallbackReturn::ERROR;
    }

    // Se verifica que las interfaces de comando sean del tipo (se llamen igual) al que se define en ros2_control.xaxro.
    if (joint.command_interfaces[0].name != hardware_constants::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotWheelVelocity"),
        "La articulación '%s' tiene %s interfaces de comando encontradas. Se esperaba '%s'.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_constants::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Se verifica si hay exactamente las interfaces de estado en cada articulación definidas en el ros2_control.xacro.
    if (joint.state_interfaces.size() != hardware_constants::NUM_STATE)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotWheelVelocity"),
        "La articulación '%s' tiene %zu interfaces de estado. Se esperaba 1.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Se verifica que las interfaces de estado sean del tipo (se llamen igual) al que se define en ros2_control.xaxro.
    if (joint.state_interfaces[0].name != hardware_constants::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotWheelVelocity"),
        "La articulación '%s' tiene interfaz de estado %s. Se esperaba '%s'.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_constants::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RRBotWheelVelocity::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("RRBotWheelVelocity"), "Configuring ...please wait...");

  for (int i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("RRBotWheelVelocity"), "%.1f seconds left...",
      hw_start_sec_ - i);
  }

  // reset values always when configuring hardware
  for (uint i = 0; i < hw_position_state_.size(); i++)
  {
    hw_position_state_[i] = 0;
    hw_velocity_commands_[i] = 0;
  }

  RCLCPP_INFO(rclcpp::get_logger("RRBotWheelVelocity"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
RRBotWheelVelocity::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_constants::HW_IF_POSITION, &hw_position_state_[i]));

    RCLCPP_INFO(
      rclcpp::get_logger("Export_State_Interface"), 
      "state_interfaces_1 %s\n", 
      state_interfaces[i].get_name().c_str());
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
RRBotWheelVelocity::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_constants::HW_IF_VELOCITY, &hw_velocity_commands_[i]));
    
    RCLCPP_INFO(
      rclcpp::get_logger("Export_Command_Interface"), 
      "command_interfaces_1 %s\n", 
      command_interfaces[i].get_name().c_str());

  }

  return command_interfaces;
}

hardware_interface::CallbackReturn RRBotWheelVelocity::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(
    rclcpp::get_logger("RRBotWheelVelocity"), "Activating ...please wait...");

  for (int i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("RRBotWheelVelocity"), "%.1f seconds left...",
      hw_start_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // command and state should be equal when starting
  for (uint i = 0; i < hw_position_state_.size(); i++)
  {
    hw_velocity_commands_[i] = hw_position_state_[i];
  }

  RCLCPP_INFO(rclcpp::get_logger("RRBotWheelVelocity"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RRBotWheelVelocity::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(
    rclcpp::get_logger("RRBotWheelVelocity"), "Deactivating ...please wait...");

  for (int i = 0; i < hw_stop_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("RRBotWheelVelocity"), "%.1f seconds left...",
      hw_stop_sec_ - i);
  }

  RCLCPP_INFO(rclcpp::get_logger("RRBotWheelVelocity"), "Successfully deactivated!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RRBotWheelVelocity::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("RRBotWheelVelocity"), "Reading...");

  for (std::size_t i = 0; i < hw_velocity_commands_.size(); i++)
  {
    // Simulate DiffBot wheels's movement as a first-order system
    // Update the joint status: this is a revolute joint without any limit.
    // Simply integrates

    RCLCPP_INFO(
      rclcpp::get_logger("Inputs"), 
      "hw_velocity_commands %lf\n"
      "hw_position_state %lf\n", 
      hw_velocity_commands_[i],
      hw_position_state_[i]);

    hw_position_state_[i] = hw_position_state_[i] + period.seconds() * hw_velocity_commands_[i];

    RCLCPP_INFO(
      rclcpp::get_logger("DiffBotSystemHardware"),
      "Got position state %.5f and velocity state %.5f for '%s'!", hw_position_state_[i],
      hw_velocity_commands_[i], info_.joints[i].name.c_str());
  }
  RCLCPP_INFO(rclcpp::get_logger("RRBotWheelVelocity"), "Joints successfully read!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RRBotWheelVelocity::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("RRBotWheelVelocity"), "Writing...");

  for (uint i = 0; i < hw_velocity_commands_.size(); i++)
  {
    // Simulate sending commands to the hardware
    RCLCPP_INFO(
      rclcpp::get_logger("RRBotWheelVelocity"), "Got command %.5f for joint %d!",
      hw_velocity_commands_[i], i);
  }
  RCLCPP_INFO(
    rclcpp::get_logger("RRBotWheelVelocity"), "Joints successfully written!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

}  // namespace TEST2_ROS2CONTROL

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  test2_ros2control::RRBotWheelVelocity, hardware_interface::SystemInterface)
