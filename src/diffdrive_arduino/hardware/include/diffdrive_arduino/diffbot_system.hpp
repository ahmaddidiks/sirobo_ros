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

#ifndef DIFFDRIVE_ARDUINO__DIFFBOT_SYSTEM_HPP_
#define DIFFDRIVE_ARDUINO__DIFFBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "diffdrive_arduino/visibility_control.h"
#include "diffdrive_arduino/Socket.hpp"

#include "diffdrive_arduino/arduino_comms.hpp"
#include "diffdrive_arduino/wheel.hpp"
#include "diffdrive_arduino/pid.hpp"

// led strip and battery not using ros_control, use normal topic instead
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"

namespace diffdrive_arduino
{

class HardwareCommand : public rclcpp::Node  //the node definition for the publisher to talk to micro-ROS agent
{
  public:
    HardwareCommand();
    
    void publishBattery(float battery_percentage);
    void publishRemoteState(bool state);
    
    std::string get_led_state();

  private:
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr remote_state_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    
    void topic_callback(const std_msgs::msg::String::SharedPtr msg);
    
    std::string led_strip_state = "grey"; // default to grey
};

class DiffDriveArduinoHardware : public hardware_interface::SystemInterface
{

struct Config
{
  std::string left_wheel_name = "";
  std::string right_wheel_name = "";
  float loop_rate = 0.0;
  std::string device = "";
  int baud_rate = 0;
  int timeout_ms = 0;
  int enc_counts_per_rev = 0;
  int pid_p = 0;
  int pid_d = 0;
  int pid_i = 0;
  int pid_o = 0;
};


public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DiffDriveArduinoHardware);

  DIFFDRIVE_ARDUINO_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  DIFFDRIVE_ARDUINO_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  DIFFDRIVE_ARDUINO_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  DIFFDRIVE_ARDUINO_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  DIFFDRIVE_ARDUINO_PUBLIC
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;


  DIFFDRIVE_ARDUINO_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  DIFFDRIVE_ARDUINO_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  DIFFDRIVE_ARDUINO_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  DIFFDRIVE_ARDUINO_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  
  std::shared_ptr<HardwareCommand> hw_cmd;    //make the publisher node a member

private:

  ArduinoComms comms_;
  Config cfg_;
  Wheel wheel_l_;
  Wheel wheel_r_;
  PID pid_l;
  PID pid_r;
  int voltage;
  bool remote_activated_;

  double last_time;
   // battery and led strip
  int led_state_2_mode(std::string led_state);
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_pub;
  rclcpp::executors::SingleThreadedExecutor executor_;  //Executor needed to subscriber // https://answers.ros.org/question/403073/ros2-subscriber-in-component/
};

}  // namespace diffdrive_arduino

#endif  // DIFFDRIVE_ARDUINO__DIFFBOT_SYSTEM_HPP_
