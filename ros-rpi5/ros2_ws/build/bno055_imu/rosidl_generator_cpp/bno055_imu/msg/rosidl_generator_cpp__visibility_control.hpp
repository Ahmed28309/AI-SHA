// generated from rosidl_generator_cpp/resource/rosidl_generator_cpp__visibility_control.hpp.in
// generated code does not contain a copyright notice

#ifndef BNO055_IMU__MSG__ROSIDL_GENERATOR_CPP__VISIBILITY_CONTROL_HPP_
#define BNO055_IMU__MSG__ROSIDL_GENERATOR_CPP__VISIBILITY_CONTROL_HPP_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROSIDL_GENERATOR_CPP_EXPORT_bno055_imu __attribute__ ((dllexport))
    #define ROSIDL_GENERATOR_CPP_IMPORT_bno055_imu __attribute__ ((dllimport))
  #else
    #define ROSIDL_GENERATOR_CPP_EXPORT_bno055_imu __declspec(dllexport)
    #define ROSIDL_GENERATOR_CPP_IMPORT_bno055_imu __declspec(dllimport)
  #endif
  #ifdef ROSIDL_GENERATOR_CPP_BUILDING_DLL_bno055_imu
    #define ROSIDL_GENERATOR_CPP_PUBLIC_bno055_imu ROSIDL_GENERATOR_CPP_EXPORT_bno055_imu
  #else
    #define ROSIDL_GENERATOR_CPP_PUBLIC_bno055_imu ROSIDL_GENERATOR_CPP_IMPORT_bno055_imu
  #endif
#else
  #define ROSIDL_GENERATOR_CPP_EXPORT_bno055_imu __attribute__ ((visibility("default")))
  #define ROSIDL_GENERATOR_CPP_IMPORT_bno055_imu
  #if __GNUC__ >= 4
    #define ROSIDL_GENERATOR_CPP_PUBLIC_bno055_imu __attribute__ ((visibility("default")))
  #else
    #define ROSIDL_GENERATOR_CPP_PUBLIC_bno055_imu
  #endif
#endif

#ifdef __cplusplus
}
#endif

#endif  // BNO055_IMU__MSG__ROSIDL_GENERATOR_CPP__VISIBILITY_CONTROL_HPP_
