// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from tram_vehicle_msgs:msg/DriverControllerCommand.idl
// generated code does not contain a copyright notice

#ifndef TRAM_VEHICLE_MSGS__MSG__DETAIL__DRIVER_CONTROLLER_COMMAND__STRUCT_H_
#define TRAM_VEHICLE_MSGS__MSG__DETAIL__DRIVER_CONTROLLER_COMMAND__STRUCT_H_

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

/// Struct defined in msg/DriverControllerCommand in the package tram_vehicle_msgs.
/**
  * Driver controller command
  * Reconstructed from the recorded rosbag wire format (verified byte-for-byte
  * against /vehicle/driver_position_cmd across multiple bags and both vehicles).
 */
typedef struct tram_vehicle_msgs__msg__DriverControllerCommand
{
  std_msgs__msg__Header header;
  /// Driver controller position (notch):
  ///   0        -> neutral
  ///   +1..+15  -> traction (acceleration)
  ///   -1..-15  -> braking
  int8_t position;
} tram_vehicle_msgs__msg__DriverControllerCommand;

// Struct for a sequence of tram_vehicle_msgs__msg__DriverControllerCommand.
typedef struct tram_vehicle_msgs__msg__DriverControllerCommand__Sequence
{
  tram_vehicle_msgs__msg__DriverControllerCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} tram_vehicle_msgs__msg__DriverControllerCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TRAM_VEHICLE_MSGS__MSG__DETAIL__DRIVER_CONTROLLER_COMMAND__STRUCT_H_
