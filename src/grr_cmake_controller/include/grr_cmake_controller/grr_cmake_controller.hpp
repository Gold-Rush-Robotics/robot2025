// Copyright 2020 PAL Robotics S.L.
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

/*
 * Author: Bence Magyar, Enrique Fernández, Manuel Meraz
 */

#ifndef GRR_CMAKE_CONTROLLER__GRR_CMAKE_CONTROLLER_HPP_
#define GRR_CMAKE_CONTROLLER__GRR_CMAKE_CONTROLLER_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "grr_cmake_controller/visibility_control.h"
#include "grr_cmake_controller/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "hardware_interface/handle.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_box.h"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include <hardware_interface/loaned_command_interface.hpp>

namespace grr_cmake_controller
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;


class Wheel {
  public:
    Wheel(std::reference_wrapper<hardware_interface::LoanedCommandInterface> command_velocity_,std::reference_wrapper< const hardware_interface::LoanedStateInterface> state_velocity_,std::string name);
    void set_velocity(double velocity);
    double get_velocity();

  private:
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> command_velocity_;
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> state_velocity_;
    std::string name;
    std::shared_ptr<std::mutex> mutex_ = std::make_shared<std::mutex>();


};

class MecanumController : public controller_interface::ControllerInterface
{
  using Twist = geometry_msgs::msg::TwistStamped;

public:
  GRR_CMAKE_CONTROLLER_PUBLIC
  MecanumController();

  GRR_CMAKE_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  GRR_CMAKE_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  GRR_CMAKE_CONTROLLER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  GRR_CMAKE_CONTROLLER_PUBLIC
  CallbackReturn on_init() override;

  GRR_CMAKE_CONTROLLER_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  GRR_CMAKE_CONTROLLER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  GRR_CMAKE_CONTROLLER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  GRR_CMAKE_CONTROLLER_PUBLIC
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  GRR_CMAKE_CONTROLLER_PUBLIC
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  GRR_CMAKE_CONTROLLER_PUBLIC
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

protected:
  std::shared_ptr<Wheel> get_wheel(const std::string & wheel_name);
  std::shared_ptr<Wheel> front_left_wheel_interface_handle_;
  std::shared_ptr<Wheel> front_right_wheel_interface_handle_;
  std::shared_ptr<Wheel> rear_left_wheel_interface_handle_;
  std::shared_ptr<Wheel> rear_right_wheel_interface_handle_;
  std::string front_left_wheel_joint_name_;
  std::string front_right_wheel_joint_name_;
  std::string rear_left_wheel_joint_name_;
  std::string rear_right_wheel_joint_name_;
  std::shared_ptr<Odometry> odom = std::make_shared<Odometry>();
  std::shared_ptr<bool> fieldOrientationEnabled = std::make_shared<bool>(true);

  struct WheelParams
  {
    double x_offset = 0.0; // Chassis Center to Axle Center
    double y_offset = 0.0; // Axle Center to Wheel Center
    double radius = 0.0;   // Assumed to be the same for all wheels
  } wheel_params_;

  // Timeout to consider cmd_vel commands old
  std::chrono::milliseconds cmd_vel_timeout_{500};
  rclcpp::Time previous_update_timestamp_{0};

  // Topic Subscription
  bool subscriber_is_active_ = false;
  rclcpp::Subscription<Twist>::SharedPtr velocity_command_subscriber_ = nullptr;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr
    pose_command_subscriber_ = nullptr;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
    velocity_command_unstamped_subscriber_ = nullptr;

  realtime_tools::RealtimeBox<std::shared_ptr<Twist>> received_velocity_msg_ptr_{nullptr};

  bool is_halted = false;
  bool use_stamped_vel_ = true;

  bool reset();
  void halt();
};
}  // namespace grr_cmake_controller
#endif  // GRR_CMAKE_CONTROLLER__GRR_CMAKE_CONTROLLER_HPP_
