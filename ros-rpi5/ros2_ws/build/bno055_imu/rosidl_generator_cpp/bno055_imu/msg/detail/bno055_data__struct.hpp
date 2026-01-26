// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from bno055_imu:msg/BNO055Data.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "bno055_imu/msg/bno055_data.hpp"


#ifndef BNO055_IMU__MSG__DETAIL__BNO055_DATA__STRUCT_HPP_
#define BNO055_IMU__MSG__DETAIL__BNO055_DATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'orientation'
#include "geometry_msgs/msg/detail/quaternion__struct.hpp"
// Member 'linear_acceleration'
// Member 'angular_velocity'
// Member 'magnetic_field'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__bno055_imu__msg__BNO055Data __attribute__((deprecated))
#else
# define DEPRECATED__bno055_imu__msg__BNO055Data __declspec(deprecated)
#endif

namespace bno055_imu
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BNO055Data_
{
  using Type = BNO055Data_<ContainerAllocator>;

  explicit BNO055Data_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    orientation(_init),
    linear_acceleration(_init),
    angular_velocity(_init),
    magnetic_field(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->euler_heading = 0.0;
      this->euler_roll = 0.0;
      this->euler_pitch = 0.0;
      this->temperature = 0.0;
      this->calibration_system = 0;
      this->calibration_gyro = 0;
      this->calibration_accel = 0;
      this->calibration_mag = 0;
    }
  }

  explicit BNO055Data_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    orientation(_alloc, _init),
    linear_acceleration(_alloc, _init),
    angular_velocity(_alloc, _init),
    magnetic_field(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->euler_heading = 0.0;
      this->euler_roll = 0.0;
      this->euler_pitch = 0.0;
      this->temperature = 0.0;
      this->calibration_system = 0;
      this->calibration_gyro = 0;
      this->calibration_accel = 0;
      this->calibration_mag = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _orientation_type =
    geometry_msgs::msg::Quaternion_<ContainerAllocator>;
  _orientation_type orientation;
  using _euler_heading_type =
    double;
  _euler_heading_type euler_heading;
  using _euler_roll_type =
    double;
  _euler_roll_type euler_roll;
  using _euler_pitch_type =
    double;
  _euler_pitch_type euler_pitch;
  using _linear_acceleration_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _linear_acceleration_type linear_acceleration;
  using _angular_velocity_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _angular_velocity_type angular_velocity;
  using _magnetic_field_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _magnetic_field_type magnetic_field;
  using _temperature_type =
    double;
  _temperature_type temperature;
  using _calibration_system_type =
    uint8_t;
  _calibration_system_type calibration_system;
  using _calibration_gyro_type =
    uint8_t;
  _calibration_gyro_type calibration_gyro;
  using _calibration_accel_type =
    uint8_t;
  _calibration_accel_type calibration_accel;
  using _calibration_mag_type =
    uint8_t;
  _calibration_mag_type calibration_mag;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__orientation(
    const geometry_msgs::msg::Quaternion_<ContainerAllocator> & _arg)
  {
    this->orientation = _arg;
    return *this;
  }
  Type & set__euler_heading(
    const double & _arg)
  {
    this->euler_heading = _arg;
    return *this;
  }
  Type & set__euler_roll(
    const double & _arg)
  {
    this->euler_roll = _arg;
    return *this;
  }
  Type & set__euler_pitch(
    const double & _arg)
  {
    this->euler_pitch = _arg;
    return *this;
  }
  Type & set__linear_acceleration(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->linear_acceleration = _arg;
    return *this;
  }
  Type & set__angular_velocity(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->angular_velocity = _arg;
    return *this;
  }
  Type & set__magnetic_field(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->magnetic_field = _arg;
    return *this;
  }
  Type & set__temperature(
    const double & _arg)
  {
    this->temperature = _arg;
    return *this;
  }
  Type & set__calibration_system(
    const uint8_t & _arg)
  {
    this->calibration_system = _arg;
    return *this;
  }
  Type & set__calibration_gyro(
    const uint8_t & _arg)
  {
    this->calibration_gyro = _arg;
    return *this;
  }
  Type & set__calibration_accel(
    const uint8_t & _arg)
  {
    this->calibration_accel = _arg;
    return *this;
  }
  Type & set__calibration_mag(
    const uint8_t & _arg)
  {
    this->calibration_mag = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    bno055_imu::msg::BNO055Data_<ContainerAllocator> *;
  using ConstRawPtr =
    const bno055_imu::msg::BNO055Data_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<bno055_imu::msg::BNO055Data_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<bno055_imu::msg::BNO055Data_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      bno055_imu::msg::BNO055Data_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<bno055_imu::msg::BNO055Data_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      bno055_imu::msg::BNO055Data_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<bno055_imu::msg::BNO055Data_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<bno055_imu::msg::BNO055Data_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<bno055_imu::msg::BNO055Data_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__bno055_imu__msg__BNO055Data
    std::shared_ptr<bno055_imu::msg::BNO055Data_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__bno055_imu__msg__BNO055Data
    std::shared_ptr<bno055_imu::msg::BNO055Data_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BNO055Data_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->orientation != other.orientation) {
      return false;
    }
    if (this->euler_heading != other.euler_heading) {
      return false;
    }
    if (this->euler_roll != other.euler_roll) {
      return false;
    }
    if (this->euler_pitch != other.euler_pitch) {
      return false;
    }
    if (this->linear_acceleration != other.linear_acceleration) {
      return false;
    }
    if (this->angular_velocity != other.angular_velocity) {
      return false;
    }
    if (this->magnetic_field != other.magnetic_field) {
      return false;
    }
    if (this->temperature != other.temperature) {
      return false;
    }
    if (this->calibration_system != other.calibration_system) {
      return false;
    }
    if (this->calibration_gyro != other.calibration_gyro) {
      return false;
    }
    if (this->calibration_accel != other.calibration_accel) {
      return false;
    }
    if (this->calibration_mag != other.calibration_mag) {
      return false;
    }
    return true;
  }
  bool operator!=(const BNO055Data_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BNO055Data_

// alias to use template instance with default allocator
using BNO055Data =
  bno055_imu::msg::BNO055Data_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace bno055_imu

#endif  // BNO055_IMU__MSG__DETAIL__BNO055_DATA__STRUCT_HPP_
