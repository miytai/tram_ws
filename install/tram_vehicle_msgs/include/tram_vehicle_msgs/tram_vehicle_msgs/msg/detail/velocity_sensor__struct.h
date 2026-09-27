// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from tram_vehicle_msgs:msg/VelocitySensor.idl
// generated code does not contain a copyright notice

#ifndef TRAM_VEHICLE_MSGS__MSG__DETAIL__VELOCITY_SENSOR__STRUCT_H_
#define TRAM_VEHICLE_MSGS__MSG__DETAIL__VELOCITY_SENSOR__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/VelocitySensor in the package tram_vehicle_msgs.
/**
  * Message
 */
typedef struct tram_vehicle_msgs__msg__VelocitySensor
{
  std_msgs__msg__Header header;
  double velocity;
} tram_vehicle_msgs__msg__VelocitySensor;

// Struct for a sequence of tram_vehicle_msgs__msg__VelocitySensor.
typedef struct tram_vehicle_msgs__msg__VelocitySensor__Sequence
{
  tram_vehicle_msgs__msg__VelocitySensor * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} tram_vehicle_msgs__msg__VelocitySensor__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TRAM_VEHICLE_MSGS__MSG__DETAIL__VELOCITY_SENSOR__STRUCT_H_
