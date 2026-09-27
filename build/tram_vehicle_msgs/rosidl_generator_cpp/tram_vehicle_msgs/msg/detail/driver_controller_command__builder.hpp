// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from tram_vehicle_msgs:msg/DriverControllerCommand.idl
// generated code does not contain a copyright notice

#ifndef TRAM_VEHICLE_MSGS__MSG__DETAIL__DRIVER_CONTROLLER_COMMAND__BUILDER_HPP_
#define TRAM_VEHICLE_MSGS__MSG__DETAIL__DRIVER_CONTROLLER_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "tram_vehicle_msgs/msg/detail/driver_controller_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace tram_vehicle_msgs
{

namespace msg
{

namespace builder
{

class Init_DriverControllerCommand_position
{
public:
  explicit Init_DriverControllerCommand_position(::tram_vehicle_msgs::msg::DriverControllerCommand & msg)
  : msg_(msg)
  {}
  ::tram_vehicle_msgs::msg::DriverControllerCommand position(::tram_vehicle_msgs::msg::DriverControllerCommand::_position_type arg)
  {
    msg_.position = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tram_vehicle_msgs::msg::DriverControllerCommand msg_;
};

class Init_DriverControllerCommand_header
{
public:
  Init_DriverControllerCommand_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DriverControllerCommand_position header(::tram_vehicle_msgs::msg::DriverControllerCommand::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_DriverControllerCommand_position(msg_);
  }

private:
  ::tram_vehicle_msgs::msg::DriverControllerCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::tram_vehicle_msgs::msg::DriverControllerCommand>()
{
  return tram_vehicle_msgs::msg::builder::Init_DriverControllerCommand_header();
}

}  // namespace tram_vehicle_msgs

#endif  // TRAM_VEHICLE_MSGS__MSG__DETAIL__DRIVER_CONTROLLER_COMMAND__BUILDER_HPP_
