// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from bno055_imu:msg/BNO055Data.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "bno055_imu/msg/bno055_data.h"


#ifndef BNO055_IMU__MSG__DETAIL__BNO055_DATA__STRUCT_H_
#define BNO055_IMU__MSG__DETAIL__BNO055_DATA__STRUCT_H_

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
// Member 'orientation'
#include "geometry_msgs/msg/detail/quaternion__struct.h"
// Member 'linear_acceleration'
// Member 'angular_velocity'
// Member 'magnetic_field'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/BNO055Data in the package bno055_imu.
/**
  * BNO055 9-DOF IMU Complete Data Message
  * All sensor data organized in one message
 */
typedef struct bno055_imu__msg__BNO055Data
{
  /// Header with timestamp and frame_id
  std_msgs__msg__Header header;
  /// === ORIENTATION (Quaternion) ===
  /// Fused orientation from sensor fusion
  geometry_msgs__msg__Quaternion orientation;
  /// === ORIENTATION (Euler Angles in degrees) ===
  /// Heading, Roll, Pitch
  /// 0 to 360 degrees
  double euler_heading;
  /// -180 to 180 degrees
  double euler_roll;
  /// -90 to 90 degrees
  double euler_pitch;
  /// === LINEAR ACCELERATION (m/s²) ===
  /// 3-axis accelerometer data
  geometry_msgs__msg__Vector3 linear_acceleration;
  /// === ANGULAR VELOCITY (rad/s) ===
  /// 3-axis gyroscope data
  geometry_msgs__msg__Vector3 angular_velocity;
  /// === MAGNETIC FIELD (micro Tesla) ===
  /// 3-axis magnetometer data
  geometry_msgs__msg__Vector3 magnetic_field;
  /// === TEMPERATURE (°C) ===
  double temperature;
  /// === CALIBRATION STATUS (0-3, where 3 is fully calibrated) ===
  /// Overall system calibration
  uint8_t calibration_system;
  /// Gyroscope calibration
  uint8_t calibration_gyro;
  /// Accelerometer calibration
  uint8_t calibration_accel;
  /// Magnetometer calibration
  uint8_t calibration_mag;
} bno055_imu__msg__BNO055Data;

// Struct for a sequence of bno055_imu__msg__BNO055Data.
typedef struct bno055_imu__msg__BNO055Data__Sequence
{
  bno055_imu__msg__BNO055Data * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} bno055_imu__msg__BNO055Data__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BNO055_IMU__MSG__DETAIL__BNO055_DATA__STRUCT_H_
