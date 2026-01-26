// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from bno055_imu:msg/BNO055Data.idl
// generated code does not contain a copyright notice
#include "bno055_imu/msg/detail/bno055_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `orientation`
#include "geometry_msgs/msg/detail/quaternion__functions.h"
// Member `linear_acceleration`
// Member `angular_velocity`
// Member `magnetic_field`
#include "geometry_msgs/msg/detail/vector3__functions.h"

bool
bno055_imu__msg__BNO055Data__init(bno055_imu__msg__BNO055Data * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    bno055_imu__msg__BNO055Data__fini(msg);
    return false;
  }
  // orientation
  if (!geometry_msgs__msg__Quaternion__init(&msg->orientation)) {
    bno055_imu__msg__BNO055Data__fini(msg);
    return false;
  }
  // euler_heading
  // euler_roll
  // euler_pitch
  // linear_acceleration
  if (!geometry_msgs__msg__Vector3__init(&msg->linear_acceleration)) {
    bno055_imu__msg__BNO055Data__fini(msg);
    return false;
  }
  // angular_velocity
  if (!geometry_msgs__msg__Vector3__init(&msg->angular_velocity)) {
    bno055_imu__msg__BNO055Data__fini(msg);
    return false;
  }
  // magnetic_field
  if (!geometry_msgs__msg__Vector3__init(&msg->magnetic_field)) {
    bno055_imu__msg__BNO055Data__fini(msg);
    return false;
  }
  // temperature
  // calibration_system
  // calibration_gyro
  // calibration_accel
  // calibration_mag
  return true;
}

void
bno055_imu__msg__BNO055Data__fini(bno055_imu__msg__BNO055Data * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // orientation
  geometry_msgs__msg__Quaternion__fini(&msg->orientation);
  // euler_heading
  // euler_roll
  // euler_pitch
  // linear_acceleration
  geometry_msgs__msg__Vector3__fini(&msg->linear_acceleration);
  // angular_velocity
  geometry_msgs__msg__Vector3__fini(&msg->angular_velocity);
  // magnetic_field
  geometry_msgs__msg__Vector3__fini(&msg->magnetic_field);
  // temperature
  // calibration_system
  // calibration_gyro
  // calibration_accel
  // calibration_mag
}

bool
bno055_imu__msg__BNO055Data__are_equal(const bno055_imu__msg__BNO055Data * lhs, const bno055_imu__msg__BNO055Data * rhs)
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
  // orientation
  if (!geometry_msgs__msg__Quaternion__are_equal(
      &(lhs->orientation), &(rhs->orientation)))
  {
    return false;
  }
  // euler_heading
  if (lhs->euler_heading != rhs->euler_heading) {
    return false;
  }
  // euler_roll
  if (lhs->euler_roll != rhs->euler_roll) {
    return false;
  }
  // euler_pitch
  if (lhs->euler_pitch != rhs->euler_pitch) {
    return false;
  }
  // linear_acceleration
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->linear_acceleration), &(rhs->linear_acceleration)))
  {
    return false;
  }
  // angular_velocity
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->angular_velocity), &(rhs->angular_velocity)))
  {
    return false;
  }
  // magnetic_field
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->magnetic_field), &(rhs->magnetic_field)))
  {
    return false;
  }
  // temperature
  if (lhs->temperature != rhs->temperature) {
    return false;
  }
  // calibration_system
  if (lhs->calibration_system != rhs->calibration_system) {
    return false;
  }
  // calibration_gyro
  if (lhs->calibration_gyro != rhs->calibration_gyro) {
    return false;
  }
  // calibration_accel
  if (lhs->calibration_accel != rhs->calibration_accel) {
    return false;
  }
  // calibration_mag
  if (lhs->calibration_mag != rhs->calibration_mag) {
    return false;
  }
  return true;
}

bool
bno055_imu__msg__BNO055Data__copy(
  const bno055_imu__msg__BNO055Data * input,
  bno055_imu__msg__BNO055Data * output)
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
  // orientation
  if (!geometry_msgs__msg__Quaternion__copy(
      &(input->orientation), &(output->orientation)))
  {
    return false;
  }
  // euler_heading
  output->euler_heading = input->euler_heading;
  // euler_roll
  output->euler_roll = input->euler_roll;
  // euler_pitch
  output->euler_pitch = input->euler_pitch;
  // linear_acceleration
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->linear_acceleration), &(output->linear_acceleration)))
  {
    return false;
  }
  // angular_velocity
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->angular_velocity), &(output->angular_velocity)))
  {
    return false;
  }
  // magnetic_field
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->magnetic_field), &(output->magnetic_field)))
  {
    return false;
  }
  // temperature
  output->temperature = input->temperature;
  // calibration_system
  output->calibration_system = input->calibration_system;
  // calibration_gyro
  output->calibration_gyro = input->calibration_gyro;
  // calibration_accel
  output->calibration_accel = input->calibration_accel;
  // calibration_mag
  output->calibration_mag = input->calibration_mag;
  return true;
}

bno055_imu__msg__BNO055Data *
bno055_imu__msg__BNO055Data__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  bno055_imu__msg__BNO055Data * msg = (bno055_imu__msg__BNO055Data *)allocator.allocate(sizeof(bno055_imu__msg__BNO055Data), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(bno055_imu__msg__BNO055Data));
  bool success = bno055_imu__msg__BNO055Data__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
bno055_imu__msg__BNO055Data__destroy(bno055_imu__msg__BNO055Data * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    bno055_imu__msg__BNO055Data__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
bno055_imu__msg__BNO055Data__Sequence__init(bno055_imu__msg__BNO055Data__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  bno055_imu__msg__BNO055Data * data = NULL;

  if (size) {
    data = (bno055_imu__msg__BNO055Data *)allocator.zero_allocate(size, sizeof(bno055_imu__msg__BNO055Data), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = bno055_imu__msg__BNO055Data__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        bno055_imu__msg__BNO055Data__fini(&data[i - 1]);
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
bno055_imu__msg__BNO055Data__Sequence__fini(bno055_imu__msg__BNO055Data__Sequence * array)
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
      bno055_imu__msg__BNO055Data__fini(&array->data[i]);
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

bno055_imu__msg__BNO055Data__Sequence *
bno055_imu__msg__BNO055Data__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  bno055_imu__msg__BNO055Data__Sequence * array = (bno055_imu__msg__BNO055Data__Sequence *)allocator.allocate(sizeof(bno055_imu__msg__BNO055Data__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = bno055_imu__msg__BNO055Data__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
bno055_imu__msg__BNO055Data__Sequence__destroy(bno055_imu__msg__BNO055Data__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    bno055_imu__msg__BNO055Data__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
bno055_imu__msg__BNO055Data__Sequence__are_equal(const bno055_imu__msg__BNO055Data__Sequence * lhs, const bno055_imu__msg__BNO055Data__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!bno055_imu__msg__BNO055Data__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
bno055_imu__msg__BNO055Data__Sequence__copy(
  const bno055_imu__msg__BNO055Data__Sequence * input,
  bno055_imu__msg__BNO055Data__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(bno055_imu__msg__BNO055Data);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    bno055_imu__msg__BNO055Data * data =
      (bno055_imu__msg__BNO055Data *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!bno055_imu__msg__BNO055Data__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          bno055_imu__msg__BNO055Data__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!bno055_imu__msg__BNO055Data__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
