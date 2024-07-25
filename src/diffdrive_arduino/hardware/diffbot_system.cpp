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

#include "diffdrive_arduino/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace diffdrive_arduino
{

HardwareCommand::HardwareCommand() : Node("hardware_command")
{
  battery_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/battery", 10);
  subscription_ = this->create_subscription<std_msgs::msg::String>(
      "/led_strip", 10, std::bind(&HardwareCommand::topic_callback, this, std::placeholders::_1));
}

void HardwareCommand::publishBattery(float battery_percentage)
{
  auto message = std_msgs::msg::Float32();
  message.data = battery_percentage;
  battery_publisher_->publish(message);
}

void HardwareCommand::publishRemoteState(bool state)
{
  auto message = std_msgs::msg::Bool();
  message.data = state;
  remote_state_publisher_->publish(message);
}

void HardwareCommand::topic_callback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  led_strip_state = msg->data;
}

std::string HardwareCommand::get_led_state()
{
  return led_strip_state;
}


hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  // cfg_.device = info_.hardware_parameters["device"];
  cfg_.device = "/dev/serial/by-path/pci-0000:00:14.0-usb-0:1:1.0-port0";
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);
  if (info_.hardware_parameters.count("pid_p") > 0)
  {
    cfg_.pid_p = std::stoi(info_.hardware_parameters["pid_p"]);
    cfg_.pid_d = std::stoi(info_.hardware_parameters["pid_d"]);
    cfg_.pid_i = std::stoi(info_.hardware_parameters["pid_i"]);
    cfg_.pid_o = std::stoi(info_.hardware_parameters["pid_o"]);
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "PID values not supplied, using defaults.");
  }

  hw_cmd = std::make_shared<HardwareCommand>();  //fire up the publisher node
  executor_.add_node(hw_cmd);
  std::thread([this]() { executor_.spin(); }).detach();

  wheel_l_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
  wheel_r_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);
  pid_l.attach(0.01, 1022.0, -1022.0, 0.0, 0.1, 0);
  pid_r.attach(0.01, 1022.0, -1022.0, 0.0, 0.1, 0);

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffDriveArduinoHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_l_.name, hardware_interface::HW_IF_POSITION, &wheel_l_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_.name, hardware_interface::HW_IF_POSITION, &wheel_r_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.vel));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffDriveArduinoHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.cmd));

  return command_interfaces;
}

hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Configuring ...please wait...");
  if (comms_.connected())
  {
    comms_.disconnect();
  }
  
  // comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);
  // RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully configured!");

  try
  {
    comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully configured!");
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
    ClientSocket client_socket("localhost", 1111);
    std::string reply;

    try
    {
      RCLCPP_INFO(rclcpp::get_logger("SERIAL FAILED"), "Request to reset process");
      client_socket << "reset";
      client_socket >> reply;
    }
    catch(const std::exception& e)
    {
      std::cerr << e.what() << '\n';
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Cleaning up ...please wait...");
  if (comms_.connected())
  {
    comms_.disconnect();
  }
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Activating ...please wait...");
  if (!comms_.connected())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  // if (cfg_.pid_p > 0)
  // {
  //   comms_.set_pid_values(cfg_.pid_p,cfg_.pid_d,cfg_.pid_i,cfg_.pid_o);
  // }
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Deactivating ...please wait...");
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffDriveArduinoHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period)*/)
{
  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type diffdrive_arduino ::DiffDriveArduinoHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }
  
  // led_strip
  std::string led_state = hw_cmd->get_led_state();
  int led_strip_goal = this->led_state_2_mode(led_state);
  
  wheel_l_.cmd *= (31.2/6.2);
  wheel_r_.cmd *= (31.2/6.2);
  if      (wheel_l_.cmd > 0 && wheel_l_.cmd < 9) wheel_l_.cmd = 9;
  else if (wheel_l_.cmd < 0 && wheel_l_.cmd > -9) wheel_l_.cmd = -9;
  if      (wheel_r_.cmd > 0 && wheel_r_.cmd < 9) wheel_r_.cmd = 9;
  else if (wheel_r_.cmd < 0 && wheel_r_.cmd > -9) wheel_r_.cmd = -9;
  double delta_seconds = period.seconds();

  // // update pid param
  // if      (wheel_l_.cmd < 15) pid_l.set_kp(22.0);
  // else if (wheel_l_.cmd < 60) pid_l.set_kp(15.0);
  // else                        pid_l.set_kp(11.5);

  // if      (wheel_r_.cmd < 15) pid_r.set_kp(18.4);
  // else if (wheel_r_.cmd < 20) pid_r.set_kp(12.5);
  // else if (wheel_r_.cmd < 30) pid_r.set_kp(13.7);
  // else if (wheel_r_.cmd < 50) pid_r.set_kp(14.2);
  // else if (wheel_r_.cmd < 60) pid_r.set_kp(14.4);
  // else if (wheel_r_.cmd < 65) pid_r.set_kp(11.5);
  // else if (wheel_r_.cmd < 70) pid_r.set_kp(11.0);
  // else                        pid_r.set_kp(11.3);

    // update pid param
  if      (wheel_l_.cmd < 15) pid_l.set_kp(22.5);
  else                        pid_l.set_kp(15.0);

  if      (wheel_r_.cmd < 15) pid_r.set_kp(23.0);
  else if (wheel_r_.cmd < 30) pid_r.set_kp(16.0);
  else if (wheel_r_.cmd < 40) pid_r.set_kp(15.7);
  else if (wheel_r_.cmd < 50) pid_r.set_kp(15.65);
  else if (wheel_r_.cmd < 60) pid_r.set_kp(15.2);
  else if (wheel_r_.cmd < 70) pid_r.set_kp(15.0);
  else                        pid_r.set_kp(15.0);
  
  // set motor speed
  int pwm_l = pid_l.calculate(wheel_l_.cmd, wheel_l_.vel, delta_seconds);
  int pwm_r = pid_r.calculate(wheel_r_.cmd, wheel_r_.vel, delta_seconds);

  // command motor to move and get last enc count
  comms_.write_command(wheel_l_.enc, wheel_r_.enc, voltage, remote_activated_, pwm_l, pwm_r, led_strip_goal);


  // calculate motor speed feedback
  wheel_l_.calculate_wheel_position(delta_seconds);
  wheel_r_.calculate_wheel_position(delta_seconds);

  // publish battery
  hw_cmd->publishBattery(float(voltage));
  // hw_cmd->publishRemoteState(remote_activated_);

  RCLCPP_INFO(rclcpp::get_logger("feedback"),"d_l: %lf, d_r: %lf, pwm_l: %d, pwm_r: %d, cmd_l : %lf, cmd_r : %lf, vel_l : %lf, vel_r : %lf, voltage: %d", wheel_l_.distance, wheel_r_.distance, pwm_l, pwm_r, wheel_l_.cmd, wheel_r_.cmd, wheel_l_.vel,wheel_r_.vel, voltage);
  // RCLCPP_INFO(rclcpp::get_logger("feedback"),"battery: %d", voltage);

  return hardware_interface::return_type::OK;
}

int diffdrive_arduino::DiffDriveArduinoHardware::led_state_2_mode(std::string led_state)
{
  /**
    0 grey
    1 green
    2 yellow
    3 blue
    4 red
    5 red-blink
  */

  int state;
  if      (led_state.compare("grey") == 0) state = 0;
  else if (led_state.compare("green") == 0) state = 1;
  else if (led_state.compare("yellow") == 0) state = 2;
  else if (led_state.compare("blue") == 0) state = 3;
  else if (led_state.compare("red") == 0) state = 4;
  else if (led_state.compare("red-blink") == 0) state = 5;
  
  return state;
}

}  // namespace diffdrive_arduino

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  diffdrive_arduino::DiffDriveArduinoHardware, hardware_interface::SystemInterface)
