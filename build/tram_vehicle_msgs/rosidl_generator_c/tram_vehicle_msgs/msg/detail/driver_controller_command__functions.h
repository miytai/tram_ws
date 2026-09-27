// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from tram_vehicle_msgs:msg/DriverControllerCommand.idl
// generated code does not contain a copyright notice

#ifndef TRAM_VEHICLE_MSGS__MSG__DETAIL__DRIVER_CONTROLLER_COMMAND__FUNCTIONS_H_
#define TRAM_VEHICLE_MSGS__MSG__DETAIL__DRIVER_CONTROLLER_COMMAND__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "tram_vehicle_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "tram_vehicle_msgs/msg/detail/driver_controller_command__struct.h"

/// Initialize msg/DriverControllerCommand message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * tram_vehicle_msgs__msg__DriverControllerCommand
 * )) before or use
 * tram_vehicle_msgs__msg__DriverControllerCommand__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_tram_vehicle_msgs
bool
tram_vehicle_msgs__msg__DriverControllerCommand__init(tram_vehicle_msgs__msg__DriverControllerCommand * msg);

/// Finalize msg/DriverControllerCommand message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tram_vehicle_msgs
void
tram_vehicle_msgs__msg__DriverControllerCommand__fini(tram_vehicle_msgs__msg__DriverControllerCommand * msg);

/// Create msg/DriverControllerCommand message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * tram_vehicle_msgs__msg__DriverControllerCommand__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_tram_vehicle_msgs
tram_vehicle_msgs__msg__DriverControllerCommand *
tram_vehicle_msgs__msg__DriverControllerCommand__create();

/// Destroy msg/DriverControllerCommand message.
/**
 * It calls
 * tram_vehicle_msgs__msg__DriverControllerCommand__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tram_vehicle_msgs
void
tram_vehicle_msgs__msg__DriverControllerCommand__destroy(tram_vehicle_msgs__msg__DriverControllerCommand * msg);

/// Check for msg/DriverControllerCommand message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_tram_vehicle_msgs
bool
tram_vehicle_msgs__msg__DriverControllerCommand__are_equal(const tram_vehicle_msgs__msg__DriverControllerCommand * lhs, const tram_vehicle_msgs__msg__DriverControllerCommand * rhs);

/// Copy a msg/DriverControllerCommand message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_tram_vehicle_msgs
bool
tram_vehicle_msgs__msg__DriverControllerCommand__copy(
  const tram_vehicle_msgs__msg__DriverControllerCommand * input,
  tram_vehicle_msgs__msg__DriverControllerCommand * output);

/// Initialize array of msg/DriverControllerCommand messages.
/**
 * It allocates the memory for the number of elements and calls
 * tram_vehicle_msgs__msg__DriverControllerCommand__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_tram_vehicle_msgs
bool
tram_vehicle_msgs__msg__DriverControllerCommand__Sequence__init(tram_vehicle_msgs__msg__DriverControllerCommand__Sequence * array, size_t size);

/// Finalize array of msg/DriverControllerCommand messages.
/**
 * It calls
 * tram_vehicle_msgs__msg__DriverControllerCommand__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tram_vehicle_msgs
void
tram_vehicle_msgs__msg__DriverControllerCommand__Sequence__fini(tram_vehicle_msgs__msg__DriverControllerCommand__Sequence * array);

/// Create array of msg/DriverControllerCommand messages.
/**
 * It allocates the memory for the array and calls
 * tram_vehicle_msgs__msg__DriverControllerCommand__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_tram_vehicle_msgs
tram_vehicle_msgs__msg__DriverControllerCommand__Sequence *
tram_vehicle_msgs__msg__DriverControllerCommand__Sequence__create(size_t size);

/// Destroy array of msg/DriverControllerCommand messages.
/**
 * It calls
 * tram_vehicle_msgs__msg__DriverControllerCommand__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tram_vehicle_msgs
void
tram_vehicle_msgs__msg__DriverControllerCommand__Sequence__destroy(tram_vehicle_msgs__msg__DriverControllerCommand__Sequence * array);

/// Check for msg/DriverControllerCommand message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_tram_vehicle_msgs
bool
tram_vehicle_msgs__msg__DriverControllerCommand__Sequence__are_equal(const tram_vehicle_msgs__msg__DriverControllerCommand__Sequence * lhs, const tram_vehicle_msgs__msg__DriverControllerCommand__Sequence * rhs);

/// Copy an array of msg/DriverControllerCommand messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_tram_vehicle_msgs
bool
tram_vehicle_msgs__msg__DriverControllerCommand__Sequence__copy(
  const tram_vehicle_msgs__msg__DriverControllerCommand__Sequence * input,
  tram_vehicle_msgs__msg__DriverControllerCommand__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // TRAM_VEHICLE_MSGS__MSG__DETAIL__DRIVER_CONTROLLER_COMMAND__FUNCTIONS_H_
