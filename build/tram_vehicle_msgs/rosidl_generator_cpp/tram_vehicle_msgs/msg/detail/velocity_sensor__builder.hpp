// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from tram_vehicle_msgs:msg/VelocitySensor.idl
// generated code does not contain a copyright notice

#ifndef TRAM_VEHICLE_MSGS__MSG__DETAIL__VELOCITY_SENSOR__BUILDER_HPP_
#define TRAM_VEHICLE_MSGS__MSG__DETAIL__VELOCITY_SENSOR__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "tram_vehicle_msgs/msg/detail/velocity_sensor__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace tram_vehicle_msgs
{

namespace msg
{

namespace builder
{

class Init_VelocitySensor_velocity
{
public:
  explicit Init_VelocitySensor_velocity(::tram_vehicle_msgs::msg::VelocitySensor & msg)
  : msg_(msg)
  {}
  ::tram_vehicle_msgs::msg::VelocitySensor velocity(::tram_vehicle_msgs::msg::VelocitySensor::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tram_vehicle_msgs::msg::VelocitySensor msg_;
};

class Init_VelocitySensor_header
{
public:
  Init_VelocitySensor_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_VelocitySensor_velocity header(::tram_vehicle_msgs::msg::VelocitySensor::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_VelocitySensor_velocity(msg_);
  }

private:
  ::tram_vehicle_msgs::msg::VelocitySensor msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::tram_vehicle_msgs::msg::VelocitySensor>()
{
  return tram_vehicle_msgs::msg::builder::Init_VelocitySensor_header();
}

}  // namespace tram_vehicle_msgs

#endif  // TRAM_VEHICLE_MSGS__MSG__DETAIL__VELOCITY_SENSOR__BUILDER_HPP_
