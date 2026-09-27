// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from tram_vehicle_msgs:msg/VelocitySensor.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "tram_vehicle_msgs/msg/detail/velocity_sensor__rosidl_typesupport_introspection_c.h"
#include "tram_vehicle_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "tram_vehicle_msgs/msg/detail/velocity_sensor__functions.h"
#include "tram_vehicle_msgs/msg/detail/velocity_sensor__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void tram_vehicle_msgs__msg__VelocitySensor__rosidl_typesupport_introspection_c__VelocitySensor_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  tram_vehicle_msgs__msg__VelocitySensor__init(message_memory);
}

void tram_vehicle_msgs__msg__VelocitySensor__rosidl_typesupport_introspection_c__VelocitySensor_fini_function(void * message_memory)
{
  tram_vehicle_msgs__msg__VelocitySensor__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember tram_vehicle_msgs__msg__VelocitySensor__rosidl_typesupport_introspection_c__VelocitySensor_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tram_vehicle_msgs__msg__VelocitySensor, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tram_vehicle_msgs__msg__VelocitySensor, velocity),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers tram_vehicle_msgs__msg__VelocitySensor__rosidl_typesupport_introspection_c__VelocitySensor_message_members = {
  "tram_vehicle_msgs__msg",  // message namespace
  "VelocitySensor",  // message name
  2,  // number of fields
  sizeof(tram_vehicle_msgs__msg__VelocitySensor),
  tram_vehicle_msgs__msg__VelocitySensor__rosidl_typesupport_introspection_c__VelocitySensor_message_member_array,  // message members
  tram_vehicle_msgs__msg__VelocitySensor__rosidl_typesupport_introspection_c__VelocitySensor_init_function,  // function to initialize message memory (memory has to be allocated)
  tram_vehicle_msgs__msg__VelocitySensor__rosidl_typesupport_introspection_c__VelocitySensor_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t tram_vehicle_msgs__msg__VelocitySensor__rosidl_typesupport_introspection_c__VelocitySensor_message_type_support_handle = {
  0,
  &tram_vehicle_msgs__msg__VelocitySensor__rosidl_typesupport_introspection_c__VelocitySensor_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_tram_vehicle_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, tram_vehicle_msgs, msg, VelocitySensor)() {
  tram_vehicle_msgs__msg__VelocitySensor__rosidl_typesupport_introspection_c__VelocitySensor_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!tram_vehicle_msgs__msg__VelocitySensor__rosidl_typesupport_introspection_c__VelocitySensor_message_type_support_handle.typesupport_identifier) {
    tram_vehicle_msgs__msg__VelocitySensor__rosidl_typesupport_introspection_c__VelocitySensor_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &tram_vehicle_msgs__msg__VelocitySensor__rosidl_typesupport_introspection_c__VelocitySensor_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
