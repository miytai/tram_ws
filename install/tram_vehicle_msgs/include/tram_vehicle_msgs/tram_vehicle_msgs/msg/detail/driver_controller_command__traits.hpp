// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from tram_vehicle_msgs:msg/DriverControllerCommand.idl
// generated code does not contain a copyright notice

#ifndef TRAM_VEHICLE_MSGS__MSG__DETAIL__DRIVER_CONTROLLER_COMMAND__TRAITS_HPP_
#define TRAM_VEHICLE_MSGS__MSG__DETAIL__DRIVER_CONTROLLER_COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "tram_vehicle_msgs/msg/detail/driver_controller_command__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace tram_vehicle_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const DriverControllerCommand & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: position
  {
    out << "position: ";
    rosidl_generator_traits::value_to_yaml(msg.position, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DriverControllerCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position: ";
    rosidl_generator_traits::value_to_yaml(msg.position, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DriverControllerCommand & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace tram_vehicle_msgs

namespace rosidl_generator_traits
{

[[deprecated("use tram_vehicle_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const tram_vehicle_msgs::msg::DriverControllerCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  tram_vehicle_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use tram_vehicle_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const tram_vehicle_msgs::msg::DriverControllerCommand & msg)
{
  return tram_vehicle_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<tram_vehicle_msgs::msg::DriverControllerCommand>()
{
  return "tram_vehicle_msgs::msg::DriverControllerCommand";
}

template<>
inline const char * name<tram_vehicle_msgs::msg::DriverControllerCommand>()
{
  return "tram_vehicle_msgs/msg/DriverControllerCommand";
}

template<>
struct has_fixed_size<tram_vehicle_msgs::msg::DriverControllerCommand>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<tram_vehicle_msgs::msg::DriverControllerCommand>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<tram_vehicle_msgs::msg::DriverControllerCommand>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TRAM_VEHICLE_MSGS__MSG__DETAIL__DRIVER_CONTROLLER_COMMAND__TRAITS_HPP_
