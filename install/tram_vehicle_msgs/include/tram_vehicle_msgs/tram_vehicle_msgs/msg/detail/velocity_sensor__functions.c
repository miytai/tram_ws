// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from tram_vehicle_msgs:msg/VelocitySensor.idl
// generated code does not contain a copyright notice
#include "tram_vehicle_msgs/msg/detail/velocity_sensor__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
tram_vehicle_msgs__msg__VelocitySensor__init(tram_vehicle_msgs__msg__VelocitySensor * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    tram_vehicle_msgs__msg__VelocitySensor__fini(msg);
    return false;
  }
  // velocity
  return true;
}

void
tram_vehicle_msgs__msg__VelocitySensor__fini(tram_vehicle_msgs__msg__VelocitySensor * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // velocity
}

bool
tram_vehicle_msgs__msg__VelocitySensor__are_equal(const tram_vehicle_msgs__msg__VelocitySensor * lhs, const tram_vehicle_msgs__msg__VelocitySensor * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // velocity
  if (lhs->velocity != rhs->velocity) {
    return false;
  }
  return true;
}

bool
tram_vehicle_msgs__msg__VelocitySensor__copy(
  const tram_vehicle_msgs__msg__VelocitySensor * input,
  tram_vehicle_msgs__msg__VelocitySensor * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // velocity
  output->velocity = input->velocity;
  return true;
}

tram_vehicle_msgs__msg__VelocitySensor *
tram_vehicle_msgs__msg__VelocitySensor__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tram_vehicle_msgs__msg__VelocitySensor * msg = (tram_vehicle_msgs__msg__VelocitySensor *)allocator.allocate(sizeof(tram_vehicle_msgs__msg__VelocitySensor), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(tram_vehicle_msgs__msg__VelocitySensor));
  bool success = tram_vehicle_msgs__msg__VelocitySensor__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
tram_vehicle_msgs__msg__VelocitySensor__destroy(tram_vehicle_msgs__msg__VelocitySensor * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    tram_vehicle_msgs__msg__VelocitySensor__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
tram_vehicle_msgs__msg__VelocitySensor__Sequence__init(tram_vehicle_msgs__msg__VelocitySensor__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tram_vehicle_msgs__msg__VelocitySensor * data = NULL;

  if (size) {
    data = (tram_vehicle_msgs__msg__VelocitySensor *)allocator.zero_allocate(size, sizeof(tram_vehicle_msgs__msg__VelocitySensor), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = tram_vehicle_msgs__msg__VelocitySensor__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        tram_vehicle_msgs__msg__VelocitySensor__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
tram_vehicle_msgs__msg__VelocitySensor__Sequence__fini(tram_vehicle_msgs__msg__VelocitySensor__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      tram_vehicle_msgs__msg__VelocitySensor__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

tram_vehicle_msgs__msg__VelocitySensor__Sequence *
tram_vehicle_msgs__msg__VelocitySensor__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tram_vehicle_msgs__msg__VelocitySensor__Sequence * array = (tram_vehicle_msgs__msg__VelocitySensor__Sequence *)allocator.allocate(sizeof(tram_vehicle_msgs__msg__VelocitySensor__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = tram_vehicle_msgs__msg__VelocitySensor__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
tram_vehicle_msgs__msg__VelocitySensor__Sequence__destroy(tram_vehicle_msgs__msg__VelocitySensor__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    tram_vehicle_msgs__msg__VelocitySensor__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
tram_vehicle_msgs__msg__VelocitySensor__Sequence__are_equal(const tram_vehicle_msgs__msg__VelocitySensor__Sequence * lhs, const tram_vehicle_msgs__msg__VelocitySensor__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!tram_vehicle_msgs__msg__VelocitySensor__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
tram_vehicle_msgs__msg__VelocitySensor__Sequence__copy(
  const tram_vehicle_msgs__msg__VelocitySensor__Sequence * input,
  tram_vehicle_msgs__msg__VelocitySensor__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(tram_vehicle_msgs__msg__VelocitySensor);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    tram_vehicle_msgs__msg__VelocitySensor * data =
      (tram_vehicle_msgs__msg__VelocitySensor *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!tram_vehicle_msgs__msg__VelocitySensor__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          tram_vehicle_msgs__msg__VelocitySensor__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!tram_vehicle_msgs__msg__VelocitySensor__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
