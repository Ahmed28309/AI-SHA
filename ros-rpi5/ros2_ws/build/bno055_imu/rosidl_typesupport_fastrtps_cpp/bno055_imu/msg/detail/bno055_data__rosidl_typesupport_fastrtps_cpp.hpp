// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from bno055_imu:msg/BNO055Data.idl
// generated code does not contain a copyright notice

#ifndef BNO055_IMU__MSG__DETAIL__BNO055_DATA__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define BNO055_IMU__MSG__DETAIL__BNO055_DATA__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include <cstddef>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "bno055_imu/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "bno055_imu/msg/detail/bno055_data__struct.hpp"

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

#include "fastcdr/Cdr.h"

namespace bno055_imu
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_bno055_imu
cdr_serialize(
  const bno055_imu::msg::BNO055Data & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_bno055_imu
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  bno055_imu::msg::BNO055Data & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_bno055_imu
get_serialized_size(
  const bno055_imu::msg::BNO055Data & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_bno055_imu
max_serialized_size_BNO055Data(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_bno055_imu
cdr_serialize_key(
  const bno055_imu::msg::BNO055Data & ros_message,
  eprosima::fastcdr::Cdr &);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_bno055_imu
get_serialized_size_key(
  const bno055_imu::msg::BNO055Data & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_bno055_imu
max_serialized_size_key_BNO055Data(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace bno055_imu

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_bno055_imu
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, bno055_imu, msg, BNO055Data)();

#ifdef __cplusplus
}
#endif

#endif  // BNO055_IMU__MSG__DETAIL__BNO055_DATA__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
