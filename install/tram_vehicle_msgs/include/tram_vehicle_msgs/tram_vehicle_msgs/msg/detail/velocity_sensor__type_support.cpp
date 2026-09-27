// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from tram_vehicle_msgs:msg/VelocitySensor.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "tram_vehicle_msgs/msg/detail/velocity_sensor__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace tram_vehicle_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void VelocitySensor_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) tram_vehicle_msgs::msg::VelocitySensor(_init);
}

void VelocitySensor_fini_function(void * message_memory)
{
  auto typed_message = static_cast<tram_vehicle_msgs::msg::VelocitySensor *>(message_memory);
  typed_message->~VelocitySensor();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember VelocitySensor_message_member_array[2] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tram_vehicle_msgs::msg::VelocitySensor, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "velocity",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tram_vehicle_msgs::msg::VelocitySensor, velocity),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers VelocitySensor_message_members = {
  "tram_vehicle_msgs::msg",  // message namespace
  "VelocitySensor",  // message name
  2,  // number of fields
  sizeof(tram_vehicle_msgs::msg::VelocitySensor),
  VelocitySensor_message_member_array,  // message members
  VelocitySensor_init_function,  // function to initialize message memory (memory has to be allocated)
  VelocitySensor_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t VelocitySensor_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &VelocitySensor_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace tram_vehicle_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<tram_vehicle_msgs::msg::VelocitySensor>()
{
  return &::tram_vehicle_msgs::msg::rosidl_typesupport_introspection_cpp::VelocitySensor_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, tram_vehicle_msgs, msg, VelocitySensor)() {
  return &::tram_vehicle_msgs::msg::rosidl_typesupport_introspection_cpp::VelocitySensor_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
