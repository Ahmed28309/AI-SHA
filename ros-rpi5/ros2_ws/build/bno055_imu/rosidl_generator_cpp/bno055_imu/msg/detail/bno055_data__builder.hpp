// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from bno055_imu:msg/BNO055Data.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "bno055_imu/msg/bno055_data.hpp"


#ifndef BNO055_IMU__MSG__DETAIL__BNO055_DATA__BUILDER_HPP_
#define BNO055_IMU__MSG__DETAIL__BNO055_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "bno055_imu/msg/detail/bno055_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace bno055_imu
{

namespace msg
{

namespace builder
{

class Init_BNO055Data_calibration_mag
{
public:
  explicit Init_BNO055Data_calibration_mag(::bno055_imu::msg::BNO055Data & msg)
  : msg_(msg)
  {}
  ::bno055_imu::msg::BNO055Data calibration_mag(::bno055_imu::msg::BNO055Data::_calibration_mag_type arg)
  {
    msg_.calibration_mag = std::move(arg);
    return std::move(msg_);
  }

private:
  ::bno055_imu::msg::BNO055Data msg_;
};

class Init_BNO055Data_calibration_accel
{
public:
  explicit Init_BNO055Data_calibration_accel(::bno055_imu::msg::BNO055Data & msg)
  : msg_(msg)
  {}
  Init_BNO055Data_calibration_mag calibration_accel(::bno055_imu::msg::BNO055Data::_calibration_accel_type arg)
  {
    msg_.calibration_accel = std::move(arg);
    return Init_BNO055Data_calibration_mag(msg_);
  }

private:
  ::bno055_imu::msg::BNO055Data msg_;
};

class Init_BNO055Data_calibration_gyro
{
public:
  explicit Init_BNO055Data_calibration_gyro(::bno055_imu::msg::BNO055Data & msg)
  : msg_(msg)
  {}
  Init_BNO055Data_calibration_accel calibration_gyro(::bno055_imu::msg::BNO055Data::_calibration_gyro_type arg)
  {
    msg_.calibration_gyro = std::move(arg);
    return Init_BNO055Data_calibration_accel(msg_);
  }

private:
  ::bno055_imu::msg::BNO055Data msg_;
};

class Init_BNO055Data_calibration_system
{
public:
  explicit Init_BNO055Data_calibration_system(::bno055_imu::msg::BNO055Data & msg)
  : msg_(msg)
  {}
  Init_BNO055Data_calibration_gyro calibration_system(::bno055_imu::msg::BNO055Data::_calibration_system_type arg)
  {
    msg_.calibration_system = std::move(arg);
    return Init_BNO055Data_calibration_gyro(msg_);
  }

private:
  ::bno055_imu::msg::BNO055Data msg_;
};

class Init_BNO055Data_temperature
{
public:
  explicit Init_BNO055Data_temperature(::bno055_imu::msg::BNO055Data & msg)
  : msg_(msg)
  {}
  Init_BNO055Data_calibration_system temperature(::bno055_imu::msg::BNO055Data::_temperature_type arg)
  {
    msg_.temperature = std::move(arg);
    return Init_BNO055Data_calibration_system(msg_);
  }

private:
  ::bno055_imu::msg::BNO055Data msg_;
};

class Init_BNO055Data_magnetic_field
{
public:
  explicit Init_BNO055Data_magnetic_field(::bno055_imu::msg::BNO055Data & msg)
  : msg_(msg)
  {}
  Init_BNO055Data_temperature magnetic_field(::bno055_imu::msg::BNO055Data::_magnetic_field_type arg)
  {
    msg_.magnetic_field = std::move(arg);
    return Init_BNO055Data_temperature(msg_);
  }

private:
  ::bno055_imu::msg::BNO055Data msg_;
};

class Init_BNO055Data_angular_velocity
{
public:
  explicit Init_BNO055Data_angular_velocity(::bno055_imu::msg::BNO055Data & msg)
  : msg_(msg)
  {}
  Init_BNO055Data_magnetic_field angular_velocity(::bno055_imu::msg::BNO055Data::_angular_velocity_type arg)
  {
    msg_.angular_velocity = std::move(arg);
    return Init_BNO055Data_magnetic_field(msg_);
  }

private:
  ::bno055_imu::msg::BNO055Data msg_;
};

class Init_BNO055Data_linear_acceleration
{
public:
  explicit Init_BNO055Data_linear_acceleration(::bno055_imu::msg::BNO055Data & msg)
  : msg_(msg)
  {}
  Init_BNO055Data_angular_velocity linear_acceleration(::bno055_imu::msg::BNO055Data::_linear_acceleration_type arg)
  {
    msg_.linear_acceleration = std::move(arg);
    return Init_BNO055Data_angular_velocity(msg_);
  }

private:
  ::bno055_imu::msg::BNO055Data msg_;
};

class Init_BNO055Data_euler_pitch
{
public:
  explicit Init_BNO055Data_euler_pitch(::bno055_imu::msg::BNO055Data & msg)
  : msg_(msg)
  {}
  Init_BNO055Data_linear_acceleration euler_pitch(::bno055_imu::msg::BNO055Data::_euler_pitch_type arg)
  {
    msg_.euler_pitch = std::move(arg);
    return Init_BNO055Data_linear_acceleration(msg_);
  }

private:
  ::bno055_imu::msg::BNO055Data msg_;
};

class Init_BNO055Data_euler_roll
{
public:
  explicit Init_BNO055Data_euler_roll(::bno055_imu::msg::BNO055Data & msg)
  : msg_(msg)
  {}
  Init_BNO055Data_euler_pitch euler_roll(::bno055_imu::msg::BNO055Data::_euler_roll_type arg)
  {
    msg_.euler_roll = std::move(arg);
    return Init_BNO055Data_euler_pitch(msg_);
  }

private:
  ::bno055_imu::msg::BNO055Data msg_;
};

class Init_BNO055Data_euler_heading
{
public:
  explicit Init_BNO055Data_euler_heading(::bno055_imu::msg::BNO055Data & msg)
  : msg_(msg)
  {}
  Init_BNO055Data_euler_roll euler_heading(::bno055_imu::msg::BNO055Data::_euler_heading_type arg)
  {
    msg_.euler_heading = std::move(arg);
    return Init_BNO055Data_euler_roll(msg_);
  }

private:
  ::bno055_imu::msg::BNO055Data msg_;
};

class Init_BNO055Data_orientation
{
public:
  explicit Init_BNO055Data_orientation(::bno055_imu::msg::BNO055Data & msg)
  : msg_(msg)
  {}
  Init_BNO055Data_euler_heading orientation(::bno055_imu::msg::BNO055Data::_orientation_type arg)
  {
    msg_.orientation = std::move(arg);
    return Init_BNO055Data_euler_heading(msg_);
  }

private:
  ::bno055_imu::msg::BNO055Data msg_;
};

class Init_BNO055Data_header
{
public:
  Init_BNO055Data_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BNO055Data_orientation header(::bno055_imu::msg::BNO055Data::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_BNO055Data_orientation(msg_);
  }

private:
  ::bno055_imu::msg::BNO055Data msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::bno055_imu::msg::BNO055Data>()
{
  return bno055_imu::msg::builder::Init_BNO055Data_header();
}

}  // namespace bno055_imu

#endif  // BNO055_IMU__MSG__DETAIL__BNO055_DATA__BUILDER_HPP_
