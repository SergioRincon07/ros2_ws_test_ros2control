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

#ifndef TEST2_ROS2CONTROL__RRBOT_HPP_
#define TEST2_ROS2CONTROL__RRBOT_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "test2_ros2control/visibility_control.h"

namespace test2_ros2control
{
// Clase que representa el hardware de un sistema RRBot con control de posición solamente
class RRBotSystemPositionOnlyHardware : public hardware_interface::SystemInterface
{
public:
  // Define las convenciones de puntero compartido para esta clase
  RCLCPP_SHARED_PTR_DEFINITIONS(RRBotSystemPositionOnlyHardware)

  // Función que se llama cuando se inicializa el hardware
  TEST2_ROS2CONTROL_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  // Función que se llama cuando se configura el hardware
  TEST2_ROS2CONTROL_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  // Función que exporta las interfaces de estado del hardware
  TEST2_ROS2CONTROL_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  // Función que exporta las interfaces de comandos del hardware
  TEST2_ROS2CONTROL_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // Función que se llama cuando se activa el hardware
  TEST2_ROS2CONTROL_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  // Función que se llama cuando se desactiva el hardware
  TEST2_ROS2CONTROL_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  // Función que lee el estado del hardware
  TEST2_ROS2CONTROL_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // Función que escribe en el hardware
  TEST2_ROS2CONTROL_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Parámetros para la simulación del sistema RRBot
  double hw_start_sec_; // Tiempo de inicio durante la activación del plugin
  double hw_stop_sec_;  // Tiempo de detención durante la desactivación del plugin
  double hw_slowdown_;  // Tiempo de desaceleración entre lectura y escritura

  // Almacena los comandos para el robot simulado
  std::vector<double> hw_commands_;
  std::vector<double> hw_position_;
};

}  // namespace TEST2_ROS2CONTROL

#endif  // TEST2_ROS2CONTROL__RRBOT_HPP_

