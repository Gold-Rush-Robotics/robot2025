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

#include "grr_hardware/can_interface.hpp"


using std::placeholders::_1;

namespace grr_hardware
{
hardware_interface::CallbackReturn IsaacDriveHardware::on_init(const hardware_interface::HardwareInfo & info)
{

  // rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  // custom_qos_profile.depth = 7;

  node_ = rclcpp::Node::make_shared("isaac_hardware_interface");

  // CAN socket setup
  can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (can_socket_ < 0)
  {
    RCLCPP_FATAL(rclcpp::get_logger("IsaacDriveHardware"), "Failed to create CAN socket");
    return CallbackReturn::ERROR;
  }

  struct ifreq ifr;
  std::strcpy(ifr.ifr_name, "can0");  // Replace "can0" with your CAN interface name
  if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0)
  {
    RCLCPP_FATAL(rclcpp::get_logger("IsaacDriveHardware"), "Failed to get CAN interface index");
    return CallbackReturn::ERROR;
  }

  struct sockaddr_can addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.can_family = PF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
  {
    RCLCPP_FATAL(rclcpp::get_logger("IsaacDriveHardware"), "Failed to bind CAN socket");
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("IsaacDriveHardware"), "CAN socket initialized successfully");



  // INTERFACE SETUP
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  // hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  hw_state_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_state_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_effort_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    joint_names_.push_back(joint.name);
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("IsaacDriveHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY && joint.command_interfaces[0].name != hardware_interface::HW_IF_EFFORT)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("IsaacDriveHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("IsaacDriveHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("IsaacDriveHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("IsaacDriveHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }
  }

  return CallbackReturn::SUCCESS;
}



std::vector<hardware_interface::StateInterface> IsaacDriveHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_state_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_state_velocities_[i]));
  }

  return state_interfaces;
}



std::vector<hardware_interface::CommandInterface> IsaacDriveHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    if (info_.joints[i].command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY )
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
       info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocity_[i]));
    }
    else if (info_.joints[i].command_interfaces[0].name == hardware_interface::HW_IF_EFFORT )
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
       info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_effort_[i]));
    }
    else{
      RCLCPP_FATAL(
        rclcpp::get_logger("IsaacDriveHardware"),
        "Joint '%s' have '%s' as command interface. '%s' or '%s' expected.", info_.joints[i].name.c_str(),
        info_.joints[i].command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_EFFORT);
      throw std::runtime_error("Invalid command interface");
    }
    
  }

  return command_interfaces;
}



hardware_interface::CallbackReturn IsaacDriveHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("IsaacDriveHardware"), "Activating ...please wait...");

  // set some default values
  for (auto i = 0u; i < hw_state_positions_.size(); i++)
  {
    if (std::isnan(hw_state_positions_[i]))
    {
      hw_state_positions_[i] = 0;
      hw_state_velocities_[i] = 0;
      hw_commands_velocity_[i] = 0;
      hw_commands_effort_[i] = 0;
    }
    joint_names_map_[joint_names_[i]] = i + 1; // ADD 1 to differentiate null key
  }

  subscriber_is_active_ = true;

  RCLCPP_INFO(rclcpp::get_logger("IsaacDriveHardware"), "Successfully activated!");

  return CallbackReturn::SUCCESS;
}



hardware_interface::CallbackReturn IsaacDriveHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("IsaacDriveHardware"), "Deactivating ...please wait...");
  subscriber_is_active_ = false;
  RCLCPP_INFO(rclcpp::get_logger("IsaacDriveHardware"), "Successfully deactivated!");

  return CallbackReturn::SUCCESS;
}


// ||                        ||
// \/ THE STUFF THAT MATTERS \/
double IsaacDriveHardware::ticksToMeters(double ticks)
{
  return ticks;
}
hardware_interface::return_type grr_hardware::IsaacDriveHardware::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    struct can_frame frame;
    ssize_t nbytes = read(can_socket_, &frame, sizeof(struct can_frame));
  
    if (nbytes < 0)
    {
      RCLCPP_WARN(rclcpp::get_logger("IsaacDriveHardware"), "Failed to read from CAN socket");
      return hardware_interface::return_type::ERROR;
    }
  
    if (nbytes == sizeof(struct can_frame))
    {
      // Example: Update hardware state based on CAN frame data
      if (frame.can_id == 0x123)  // Replace with your CAN ID
      {
        hw_state_positions_[0] = frame.data[0];  // Example: Map CAN data to position
        hw_state_velocities_[0] = frame.data[1]; // Example: Map CAN data to velocity
      }
    }
  
    return hardware_interface::return_type::OK;
}



hardware_interface::return_type grr_hardware::IsaacDriveHardware::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // RCLCPP_INFO(rclcpp::get_logger("IsaacDriveHardware"), "Velocity: %f", hw_commands_[0]);

  struct can_frame frame;
  frame.can_id = 0x123;  // Replace with your CAN ID
  frame.can_dlc = 2;     // Data length
  frame.data[0] = static_cast<uint8_t>(hw_commands_velocity_[0]);  // Example: Map velocity command
  frame.data[1] = static_cast<uint8_t>(hw_commands_effort_[0]);    // Example: Map effort command
  frame.data[2] = static_cast<uint8_t>(hw_commands_velocity_[1]);  // Example: Map velocity command
  ssize_t nbytes = write(can_socket_, &frame, sizeof(struct can_frame));

  if (nbytes != sizeof(struct can_frame))
  {
    RCLCPP_WARN(rclcpp::get_logger("IsaacDriveHardware"), "Failed to write to CAN socket");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace grr_hardware



#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  grr_hardware::IsaacDriveHardware, hardware_interface::SystemInterface)