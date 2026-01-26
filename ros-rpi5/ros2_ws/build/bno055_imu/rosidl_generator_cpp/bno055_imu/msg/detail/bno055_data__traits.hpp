// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from bno055_imu:msg/BNO055Data.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "bno055_imu/msg/bno055_data.hpp"


#ifndef BNO055_IMU__MSG__DETAIL__BNO055_DATA__TRAITS_HPP_
#define BNO055_IMU__MSG__DETAIL__BNO055_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "bno055_imu/msg/detail/bno055_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'orientation'
#include "geometry_msgs/msg/detail/quaternion__traits.hpp"
// Member 'linear_acceleration'
// Member 'angular_velocity'
// Member 'magnetic_field'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"

namespace bno055_imu
{

namespace msg
{

inline void to_flow_style_yaml(
  const BNO055Data & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: orientation
  {
    out << "orientation: ";
    to_flow_style_yaml(msg.orientation, out);
    out << ", ";
  }

  // member: euler_heading
  {
    out << "euler_heading: ";
    rosidl_generator_traits::value_to_yaml(msg.euler_heading, out);
    out << ", ";
  }

  // member: euler_roll
  {
    out << "euler_roll: ";
    rosidl_generator_traits::value_to_yaml(msg.euler_roll, out);
    out << ", ";
  }

  // member: euler_pitch
  {
    out << "euler_pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.euler_pitch, out);
    out << ", ";
  }

  // member: linear_acceleration
  {
    out << "linear_acceleration: ";
    to_flow_style_yaml(msg.linear_acceleration, out);
    out << ", ";
  }

  // member: angular_velocity
  {
    out << "angular_velocity: ";
    to_flow_style_yaml(msg.angular_velocity, out);
    out << ", ";
  }

  // member: magnetic_field
  {
    out << "magnetic_field: ";
    to_flow_style_yaml(msg.magnetic_field, out);
    out << ", ";
  }

  // member: temperature
  {
    out << "temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.temperature, out);
    out << ", ";
  }

  // member: calibration_system
  {
    out << "calibration_system: ";
    rosidl_generator_traits::value_to_yaml(msg.calibration_system, out);
    out << ", ";
  }

  // member: calibration_gyro
  {
    out << "calibration_gyro: ";
    rosidl_generator_traits::value_to_yaml(msg.calibration_gyro, out);
    out << ", ";
  }

  // member: calibration_accel
  {
    out << "calibration_accel: ";
    rosidl_generator_traits::value_to_yaml(msg.calibration_accel, out);
    out << ", ";
  }

  // member: calibration_mag
  {
    out << "calibration_mag: ";
    rosidl_generator_traits::value_to_yaml(msg.calibration_mag, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BNO055Data & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: orientation
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "orientation:\n";
    to_block_style_yaml(msg.orientation, out, indentation + 2);
  }

  // member: euler_heading
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "euler_heading: ";
    rosidl_generator_traits::value_to_yaml(msg.euler_heading, out);
    out << "\n";
  }

  // member: euler_roll
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "euler_roll: ";
    rosidl_generator_traits::value_to_yaml(msg.euler_roll, out);
    out << "\n";
  }

  // member: euler_pitch
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "euler_pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.euler_pitch, out);
    out << "\n";
  }

  // member: linear_acceleration
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "linear_acceleration:\n";
    to_block_style_yaml(msg.linear_acceleration, out, indentation + 2);
  }

  // member: angular_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angular_velocity:\n";
    to_block_style_yaml(msg.angular_velocity, out, indentation + 2);
  }

  // member: magnetic_field
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "magnetic_field:\n";
    to_block_style_yaml(msg.magnetic_field, out, indentation + 2);
  }

  // member: temperature
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.temperature, out);
    out << "\n";
  }

  // member: calibration_system
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "calibration_system: ";
    rosidl_generator_traits::value_to_yaml(msg.calibration_system, out);
    out << "\n";
  }

  // member: calibration_gyro
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "calibration_gyro: ";
    rosidl_generator_traits::value_to_yaml(msg.calibration_gyro, out);
    out << "\n";
  }

  // member: calibration_accel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "calibration_accel: ";
    rosidl_generator_traits::value_to_yaml(msg.calibration_accel, out);
    out << "\n";
  }

  // member: calibration_mag
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "calibration_mag: ";
    rosidl_generator_traits::value_to_yaml(msg.calibration_mag, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BNO055Data & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace bno055_imu

namespace rosidl_generator_traits
{

[[deprecated("use bno055_imu::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const bno055_imu::msg::BNO055Data & msg,
  std::ostream & out, size_t indentation = 0)
{
  bno055_imu::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use bno055_imu::msg::to_yaml() instead")]]
inline std::string to_yaml(const bno055_imu::msg::BNO055Data & msg)
{
  return bno055_imu::msg::to_yaml(msg);
}

template<>
inline const char * data_type<bno055_imu::msg::BNO055Data>()
{
  return "bno055_imu::msg::BNO055Data";
}

template<>
inline const char * name<bno055_imu::msg::BNO055Data>()
{
  return "bno055_imu/msg/BNO055Data";
}

template<>
struct has_fixed_size<bno055_imu::msg::BNO055Data>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Quaternion>::value && has_fixed_size<geometry_msgs::msg::Vector3>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<bno055_imu::msg::BNO055Data>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Quaternion>::value && has_bounded_size<geometry_msgs::msg::Vector3>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<bno055_imu::msg::BNO055Data>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // BNO055_IMU__MSG__DETAIL__BNO055_DATA__TRAITS_HPP_
