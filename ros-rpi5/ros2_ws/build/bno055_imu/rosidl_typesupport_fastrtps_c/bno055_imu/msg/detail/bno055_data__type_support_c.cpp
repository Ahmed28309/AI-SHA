// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from bno055_imu:msg/BNO055Data.idl
// generated code does not contain a copyright notice
#include "bno055_imu/msg/detail/bno055_data__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "bno055_imu/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "bno055_imu/msg/detail/bno055_data__struct.h"
#include "bno055_imu/msg/detail/bno055_data__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "geometry_msgs/msg/detail/quaternion__functions.h"  // orientation
#include "geometry_msgs/msg/detail/vector3__functions.h"  // angular_velocity, linear_acceleration, magnetic_field
#include "std_msgs/msg/detail/header__functions.h"  // header

// forward declare type support functions

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_bno055_imu
bool cdr_serialize_geometry_msgs__msg__Quaternion(
  const geometry_msgs__msg__Quaternion * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_bno055_imu
bool cdr_deserialize_geometry_msgs__msg__Quaternion(
  eprosima::fastcdr::Cdr & cdr,
  geometry_msgs__msg__Quaternion * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_bno055_imu
size_t get_serialized_size_geometry_msgs__msg__Quaternion(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_bno055_imu
size_t max_serialized_size_geometry_msgs__msg__Quaternion(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_bno055_imu
bool cdr_serialize_key_geometry_msgs__msg__Quaternion(
  const geometry_msgs__msg__Quaternion * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_bno055_imu
size_t get_serialized_size_key_geometry_msgs__msg__Quaternion(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_bno055_imu
size_t max_serialized_size_key_geometry_msgs__msg__Quaternion(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_bno055_imu
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Quaternion)();

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_bno055_imu
bool cdr_serialize_geometry_msgs__msg__Vector3(
  const geometry_msgs__msg__Vector3 * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_bno055_imu
bool cdr_deserialize_geometry_msgs__msg__Vector3(
  eprosima::fastcdr::Cdr & cdr,
  geometry_msgs__msg__Vector3 * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_bno055_imu
size_t get_serialized_size_geometry_msgs__msg__Vector3(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_bno055_imu
size_t max_serialized_size_geometry_msgs__msg__Vector3(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_bno055_imu
bool cdr_serialize_key_geometry_msgs__msg__Vector3(
  const geometry_msgs__msg__Vector3 * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_bno055_imu
size_t get_serialized_size_key_geometry_msgs__msg__Vector3(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_bno055_imu
size_t max_serialized_size_key_geometry_msgs__msg__Vector3(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_bno055_imu
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Vector3)();

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_bno055_imu
bool cdr_serialize_std_msgs__msg__Header(
  const std_msgs__msg__Header * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_bno055_imu
bool cdr_deserialize_std_msgs__msg__Header(
  eprosima::fastcdr::Cdr & cdr,
  std_msgs__msg__Header * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_bno055_imu
size_t get_serialized_size_std_msgs__msg__Header(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_bno055_imu
size_t max_serialized_size_std_msgs__msg__Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_bno055_imu
bool cdr_serialize_key_std_msgs__msg__Header(
  const std_msgs__msg__Header * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_bno055_imu
size_t get_serialized_size_key_std_msgs__msg__Header(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_bno055_imu
size_t max_serialized_size_key_std_msgs__msg__Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_bno055_imu
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, std_msgs, msg, Header)();


using _BNO055Data__ros_msg_type = bno055_imu__msg__BNO055Data;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_bno055_imu
bool cdr_serialize_bno055_imu__msg__BNO055Data(
  const bno055_imu__msg__BNO055Data * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: header
  {
    cdr_serialize_std_msgs__msg__Header(
      &ros_message->header, cdr);
  }

  // Field name: orientation
  {
    cdr_serialize_geometry_msgs__msg__Quaternion(
      &ros_message->orientation, cdr);
  }

  // Field name: euler_heading
  {
    cdr << ros_message->euler_heading;
  }

  // Field name: euler_roll
  {
    cdr << ros_message->euler_roll;
  }

  // Field name: euler_pitch
  {
    cdr << ros_message->euler_pitch;
  }

  // Field name: linear_acceleration
  {
    cdr_serialize_geometry_msgs__msg__Vector3(
      &ros_message->linear_acceleration, cdr);
  }

  // Field name: angular_velocity
  {
    cdr_serialize_geometry_msgs__msg__Vector3(
      &ros_message->angular_velocity, cdr);
  }

  // Field name: magnetic_field
  {
    cdr_serialize_geometry_msgs__msg__Vector3(
      &ros_message->magnetic_field, cdr);
  }

  // Field name: temperature
  {
    cdr << ros_message->temperature;
  }

  // Field name: calibration_system
  {
    cdr << ros_message->calibration_system;
  }

  // Field name: calibration_gyro
  {
    cdr << ros_message->calibration_gyro;
  }

  // Field name: calibration_accel
  {
    cdr << ros_message->calibration_accel;
  }

  // Field name: calibration_mag
  {
    cdr << ros_message->calibration_mag;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_bno055_imu
bool cdr_deserialize_bno055_imu__msg__BNO055Data(
  eprosima::fastcdr::Cdr & cdr,
  bno055_imu__msg__BNO055Data * ros_message)
{
  // Field name: header
  {
    cdr_deserialize_std_msgs__msg__Header(cdr, &ros_message->header);
  }

  // Field name: orientation
  {
    cdr_deserialize_geometry_msgs__msg__Quaternion(cdr, &ros_message->orientation);
  }

  // Field name: euler_heading
  {
    cdr >> ros_message->euler_heading;
  }

  // Field name: euler_roll
  {
    cdr >> ros_message->euler_roll;
  }

  // Field name: euler_pitch
  {
    cdr >> ros_message->euler_pitch;
  }

  // Field name: linear_acceleration
  {
    cdr_deserialize_geometry_msgs__msg__Vector3(cdr, &ros_message->linear_acceleration);
  }

  // Field name: angular_velocity
  {
    cdr_deserialize_geometry_msgs__msg__Vector3(cdr, &ros_message->angular_velocity);
  }

  // Field name: magnetic_field
  {
    cdr_deserialize_geometry_msgs__msg__Vector3(cdr, &ros_message->magnetic_field);
  }

  // Field name: temperature
  {
    cdr >> ros_message->temperature;
  }

  // Field name: calibration_system
  {
    cdr >> ros_message->calibration_system;
  }

  // Field name: calibration_gyro
  {
    cdr >> ros_message->calibration_gyro;
  }

  // Field name: calibration_accel
  {
    cdr >> ros_message->calibration_accel;
  }

  // Field name: calibration_mag
  {
    cdr >> ros_message->calibration_mag;
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_bno055_imu
size_t get_serialized_size_bno055_imu__msg__BNO055Data(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _BNO055Data__ros_msg_type * ros_message = static_cast<const _BNO055Data__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: header
  current_alignment += get_serialized_size_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);

  // Field name: orientation
  current_alignment += get_serialized_size_geometry_msgs__msg__Quaternion(
    &(ros_message->orientation), current_alignment);

  // Field name: euler_heading
  {
    size_t item_size = sizeof(ros_message->euler_heading);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: euler_roll
  {
    size_t item_size = sizeof(ros_message->euler_roll);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: euler_pitch
  {
    size_t item_size = sizeof(ros_message->euler_pitch);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: linear_acceleration
  current_alignment += get_serialized_size_geometry_msgs__msg__Vector3(
    &(ros_message->linear_acceleration), current_alignment);

  // Field name: angular_velocity
  current_alignment += get_serialized_size_geometry_msgs__msg__Vector3(
    &(ros_message->angular_velocity), current_alignment);

  // Field name: magnetic_field
  current_alignment += get_serialized_size_geometry_msgs__msg__Vector3(
    &(ros_message->magnetic_field), current_alignment);

  // Field name: temperature
  {
    size_t item_size = sizeof(ros_message->temperature);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: calibration_system
  {
    size_t item_size = sizeof(ros_message->calibration_system);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: calibration_gyro
  {
    size_t item_size = sizeof(ros_message->calibration_gyro);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: calibration_accel
  {
    size_t item_size = sizeof(ros_message->calibration_accel);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: calibration_mag
  {
    size_t item_size = sizeof(ros_message->calibration_mag);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_bno055_imu
size_t max_serialized_size_bno055_imu__msg__BNO055Data(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // Field name: header
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_std_msgs__msg__Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: orientation
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_geometry_msgs__msg__Quaternion(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: euler_heading
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: euler_roll
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: euler_pitch
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: linear_acceleration
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_geometry_msgs__msg__Vector3(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: angular_velocity
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_geometry_msgs__msg__Vector3(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: magnetic_field
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_geometry_msgs__msg__Vector3(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: temperature
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: calibration_system
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: calibration_gyro
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: calibration_accel
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: calibration_mag
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = bno055_imu__msg__BNO055Data;
    is_plain =
      (
      offsetof(DataType, calibration_mag) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_bno055_imu
bool cdr_serialize_key_bno055_imu__msg__BNO055Data(
  const bno055_imu__msg__BNO055Data * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: header
  {
    cdr_serialize_key_std_msgs__msg__Header(
      &ros_message->header, cdr);
  }

  // Field name: orientation
  {
    cdr_serialize_key_geometry_msgs__msg__Quaternion(
      &ros_message->orientation, cdr);
  }

  // Field name: euler_heading
  {
    cdr << ros_message->euler_heading;
  }

  // Field name: euler_roll
  {
    cdr << ros_message->euler_roll;
  }

  // Field name: euler_pitch
  {
    cdr << ros_message->euler_pitch;
  }

  // Field name: linear_acceleration
  {
    cdr_serialize_key_geometry_msgs__msg__Vector3(
      &ros_message->linear_acceleration, cdr);
  }

  // Field name: angular_velocity
  {
    cdr_serialize_key_geometry_msgs__msg__Vector3(
      &ros_message->angular_velocity, cdr);
  }

  // Field name: magnetic_field
  {
    cdr_serialize_key_geometry_msgs__msg__Vector3(
      &ros_message->magnetic_field, cdr);
  }

  // Field name: temperature
  {
    cdr << ros_message->temperature;
  }

  // Field name: calibration_system
  {
    cdr << ros_message->calibration_system;
  }

  // Field name: calibration_gyro
  {
    cdr << ros_message->calibration_gyro;
  }

  // Field name: calibration_accel
  {
    cdr << ros_message->calibration_accel;
  }

  // Field name: calibration_mag
  {
    cdr << ros_message->calibration_mag;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_bno055_imu
size_t get_serialized_size_key_bno055_imu__msg__BNO055Data(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _BNO055Data__ros_msg_type * ros_message = static_cast<const _BNO055Data__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: header
  current_alignment += get_serialized_size_key_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);

  // Field name: orientation
  current_alignment += get_serialized_size_key_geometry_msgs__msg__Quaternion(
    &(ros_message->orientation), current_alignment);

  // Field name: euler_heading
  {
    size_t item_size = sizeof(ros_message->euler_heading);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: euler_roll
  {
    size_t item_size = sizeof(ros_message->euler_roll);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: euler_pitch
  {
    size_t item_size = sizeof(ros_message->euler_pitch);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: linear_acceleration
  current_alignment += get_serialized_size_key_geometry_msgs__msg__Vector3(
    &(ros_message->linear_acceleration), current_alignment);

  // Field name: angular_velocity
  current_alignment += get_serialized_size_key_geometry_msgs__msg__Vector3(
    &(ros_message->angular_velocity), current_alignment);

  // Field name: magnetic_field
  current_alignment += get_serialized_size_key_geometry_msgs__msg__Vector3(
    &(ros_message->magnetic_field), current_alignment);

  // Field name: temperature
  {
    size_t item_size = sizeof(ros_message->temperature);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: calibration_system
  {
    size_t item_size = sizeof(ros_message->calibration_system);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: calibration_gyro
  {
    size_t item_size = sizeof(ros_message->calibration_gyro);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: calibration_accel
  {
    size_t item_size = sizeof(ros_message->calibration_accel);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: calibration_mag
  {
    size_t item_size = sizeof(ros_message->calibration_mag);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_bno055_imu
size_t max_serialized_size_key_bno055_imu__msg__BNO055Data(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;
  // Field name: header
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_std_msgs__msg__Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: orientation
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_geometry_msgs__msg__Quaternion(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: euler_heading
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: euler_roll
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: euler_pitch
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: linear_acceleration
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_geometry_msgs__msg__Vector3(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: angular_velocity
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_geometry_msgs__msg__Vector3(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: magnetic_field
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_geometry_msgs__msg__Vector3(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: temperature
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: calibration_system
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: calibration_gyro
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: calibration_accel
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: calibration_mag
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = bno055_imu__msg__BNO055Data;
    is_plain =
      (
      offsetof(DataType, calibration_mag) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _BNO055Data__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const bno055_imu__msg__BNO055Data * ros_message = static_cast<const bno055_imu__msg__BNO055Data *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_bno055_imu__msg__BNO055Data(ros_message, cdr);
}

static bool _BNO055Data__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  bno055_imu__msg__BNO055Data * ros_message = static_cast<bno055_imu__msg__BNO055Data *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_bno055_imu__msg__BNO055Data(cdr, ros_message);
}

static uint32_t _BNO055Data__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_bno055_imu__msg__BNO055Data(
      untyped_ros_message, 0));
}

static size_t _BNO055Data__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_bno055_imu__msg__BNO055Data(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_BNO055Data = {
  "bno055_imu::msg",
  "BNO055Data",
  _BNO055Data__cdr_serialize,
  _BNO055Data__cdr_deserialize,
  _BNO055Data__get_serialized_size,
  _BNO055Data__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _BNO055Data__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_BNO055Data,
  get_message_typesupport_handle_function,
  &bno055_imu__msg__BNO055Data__get_type_hash,
  &bno055_imu__msg__BNO055Data__get_type_description,
  &bno055_imu__msg__BNO055Data__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, bno055_imu, msg, BNO055Data)() {
  return &_BNO055Data__type_support;
}

#if defined(__cplusplus)
}
#endif
