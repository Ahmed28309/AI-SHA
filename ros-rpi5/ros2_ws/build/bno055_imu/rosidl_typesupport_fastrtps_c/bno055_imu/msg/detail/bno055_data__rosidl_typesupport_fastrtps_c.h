// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from bno055_imu:msg/BNO055Data.idl
// generated code does not contain a copyright notice
#ifndef BNO055_IMU__MSG__DETAIL__BNO055_DATA__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define BNO055_IMU__MSG__DETAIL__BNO055_DATA__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "bno055_imu/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "bno055_imu/msg/detail/bno055_data__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_bno055_imu
bool cdr_serialize_bno055_imu__msg__BNO055Data(
  const bno055_imu__msg__BNO055Data * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_bno055_imu
bool cdr_deserialize_bno055_imu__msg__BNO055Data(
  eprosima::fastcdr::Cdr &,
  bno055_imu__msg__BNO055Data * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_bno055_imu
size_t get_serialized_size_bno055_imu__msg__BNO055Data(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_bno055_imu
size_t max_serialized_size_bno055_imu__msg__BNO055Data(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_bno055_imu
bool cdr_serialize_key_bno055_imu__msg__BNO055Data(
  const bno055_imu__msg__BNO055Data * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_bno055_imu
size_t get_serialized_size_key_bno055_imu__msg__BNO055Data(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_bno055_imu
size_t max_serialized_size_key_bno055_imu__msg__BNO055Data(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_bno055_imu
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, bno055_imu, msg, BNO055Data)();

#ifdef __cplusplus
}
#endif

#endif  // BNO055_IMU__MSG__DETAIL__BNO055_DATA__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
