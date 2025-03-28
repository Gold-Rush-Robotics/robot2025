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

#ifndef GRR_HARDWARE__ISAAC_DRIVE_HPP_
#define GRR_HARDWARE__ISAAC_DRIVE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <map>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "grr_hardware/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "realtime_tools/realtime_box.h"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"

// using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace grr_hardware
{
class IsaacDriveHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(IsaacDriveHardware)

  GRR_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  GRR_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  GRR_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  GRR_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  GRR_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  GRR_HARDWARE_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  GRR_HARDWARE_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  double ticksToMeters(double ticks);
  // Parameters for the DiffBot simulation
  double hw_start_sec_;
  double hw_stop_sec_;

  // Store the command for the simulated robot
  std::vector<double> hw_commands_velocity_;
  std::vector<double> hw_commands_effort_;
  std::vector<double> hw_state_positions_;
  std::vector<double> hw_state_velocities_;
  std::vector<std::string> joint_names_;
  std::map<std::string, uint> joint_names_map_;


  // Pub Sub to isaac
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>> isaac_publisher_ = nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>
    realtime_isaac_publisher_ = nullptr;

  bool subscriber_is_active_ = false;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr isaac_subscriber_ = nullptr;
  realtime_tools::RealtimeBox<std::shared_ptr<sensor_msgs::msg::JointState>> received_joint_msg_ptr_{nullptr};

};

}  // namespace grr_hardware

#endif  // GRR_HARDWARE__DIFFBOT_SYSTEM_HPP_