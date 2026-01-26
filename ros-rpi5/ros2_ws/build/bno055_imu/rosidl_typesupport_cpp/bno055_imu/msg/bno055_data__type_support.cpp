// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from bno055_imu:msg/BNO055Data.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "bno055_imu/msg/detail/bno055_data__functions.h"
#include "bno055_imu/msg/detail/bno055_data__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace bno055_imu
{

namespace msg
{

namespace rosidl_typesupport_cpp
{

typedef struct _BNO055Data_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _BNO055Data_type_support_ids_t;

static const _BNO055Data_type_support_ids_t _BNO055Data_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _BNO055Data_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _BNO055Data_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _BNO055Data_type_support_symbol_names_t _BNO055Data_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, bno055_imu, msg, BNO055Data)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, bno055_imu, msg, BNO055Data)),
  }
};

typedef struct _BNO055Data_type_support_data_t
{
  void * data[2];
} _BNO055Data_type_support_data_t;

static _BNO055Data_type_support_data_t _BNO055Data_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _BNO055Data_message_typesupport_map = {
  2,
  "bno055_imu",
  &_BNO055Data_message_typesupport_ids.typesupport_identifier[0],
  &_BNO055Data_message_typesupport_symbol_names.symbol_name[0],
  &_BNO055Data_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t BNO055Data_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_BNO055Data_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &bno055_imu__msg__BNO055Data__get_type_hash,
  &bno055_imu__msg__BNO055Data__get_type_description,
  &bno055_imu__msg__BNO055Data__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace msg

}  // namespace bno055_imu

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<bno055_imu::msg::BNO055Data>()
{
  return &::bno055_imu::msg::rosidl_typesupport_cpp::BNO055Data_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, bno055_imu, msg, BNO055Data)() {
  return get_message_type_support_handle<bno055_imu::msg::BNO055Data>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp
