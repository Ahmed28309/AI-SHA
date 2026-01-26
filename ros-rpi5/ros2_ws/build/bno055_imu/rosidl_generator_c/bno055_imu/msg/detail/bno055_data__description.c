// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from bno055_imu:msg/BNO055Data.idl
// generated code does not contain a copyright notice

#include "bno055_imu/msg/detail/bno055_data__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_bno055_imu
const rosidl_type_hash_t *
bno055_imu__msg__BNO055Data__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x51, 0x97, 0x76, 0xb5, 0x64, 0x26, 0x19, 0xa2,
      0xd5, 0xd5, 0x92, 0x4f, 0xfe, 0xde, 0x19, 0xcd,
      0xb5, 0xad, 0xce, 0xa6, 0xef, 0x9c, 0xb4, 0xe1,
      0x52, 0xa6, 0xcd, 0x2d, 0xac, 0xdb, 0x0e, 0x01,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "geometry_msgs/msg/detail/quaternion__functions.h"
#include "std_msgs/msg/detail/header__functions.h"
#include "geometry_msgs/msg/detail/vector3__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t geometry_msgs__msg__Quaternion__EXPECTED_HASH = {1, {
    0x8a, 0x76, 0x5f, 0x66, 0x77, 0x8c, 0x8f, 0xf7,
    0xc8, 0xab, 0x94, 0xaf, 0xcc, 0x59, 0x0a, 0x2e,
    0xd5, 0x32, 0x5a, 0x1d, 0x9a, 0x07, 0x6f, 0xff,
    0xf3, 0x8f, 0xbc, 0xe3, 0x6f, 0x45, 0x86, 0x84,
  }};
static const rosidl_type_hash_t geometry_msgs__msg__Vector3__EXPECTED_HASH = {1, {
    0xcc, 0x12, 0xfe, 0x83, 0xe4, 0xc0, 0x27, 0x19,
    0xf1, 0xce, 0x80, 0x70, 0xbf, 0xd1, 0x4a, 0xec,
    0xd4, 0x0f, 0x75, 0xa9, 0x66, 0x96, 0xa6, 0x7a,
    0x2a, 0x1f, 0x37, 0xf7, 0xdb, 0xb0, 0x76, 0x5d,
  }};
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
#endif

static char bno055_imu__msg__BNO055Data__TYPE_NAME[] = "bno055_imu/msg/BNO055Data";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char geometry_msgs__msg__Quaternion__TYPE_NAME[] = "geometry_msgs/msg/Quaternion";
static char geometry_msgs__msg__Vector3__TYPE_NAME[] = "geometry_msgs/msg/Vector3";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char bno055_imu__msg__BNO055Data__FIELD_NAME__header[] = "header";
static char bno055_imu__msg__BNO055Data__FIELD_NAME__orientation[] = "orientation";
static char bno055_imu__msg__BNO055Data__FIELD_NAME__euler_heading[] = "euler_heading";
static char bno055_imu__msg__BNO055Data__FIELD_NAME__euler_roll[] = "euler_roll";
static char bno055_imu__msg__BNO055Data__FIELD_NAME__euler_pitch[] = "euler_pitch";
static char bno055_imu__msg__BNO055Data__FIELD_NAME__linear_acceleration[] = "linear_acceleration";
static char bno055_imu__msg__BNO055Data__FIELD_NAME__angular_velocity[] = "angular_velocity";
static char bno055_imu__msg__BNO055Data__FIELD_NAME__magnetic_field[] = "magnetic_field";
static char bno055_imu__msg__BNO055Data__FIELD_NAME__temperature[] = "temperature";
static char bno055_imu__msg__BNO055Data__FIELD_NAME__calibration_system[] = "calibration_system";
static char bno055_imu__msg__BNO055Data__FIELD_NAME__calibration_gyro[] = "calibration_gyro";
static char bno055_imu__msg__BNO055Data__FIELD_NAME__calibration_accel[] = "calibration_accel";
static char bno055_imu__msg__BNO055Data__FIELD_NAME__calibration_mag[] = "calibration_mag";

static rosidl_runtime_c__type_description__Field bno055_imu__msg__BNO055Data__FIELDS[] = {
  {
    {bno055_imu__msg__BNO055Data__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {bno055_imu__msg__BNO055Data__FIELD_NAME__orientation, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {geometry_msgs__msg__Quaternion__TYPE_NAME, 28, 28},
    },
    {NULL, 0, 0},
  },
  {
    {bno055_imu__msg__BNO055Data__FIELD_NAME__euler_heading, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {bno055_imu__msg__BNO055Data__FIELD_NAME__euler_roll, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {bno055_imu__msg__BNO055Data__FIELD_NAME__euler_pitch, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {bno055_imu__msg__BNO055Data__FIELD_NAME__linear_acceleration, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {geometry_msgs__msg__Vector3__TYPE_NAME, 25, 25},
    },
    {NULL, 0, 0},
  },
  {
    {bno055_imu__msg__BNO055Data__FIELD_NAME__angular_velocity, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {geometry_msgs__msg__Vector3__TYPE_NAME, 25, 25},
    },
    {NULL, 0, 0},
  },
  {
    {bno055_imu__msg__BNO055Data__FIELD_NAME__magnetic_field, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {geometry_msgs__msg__Vector3__TYPE_NAME, 25, 25},
    },
    {NULL, 0, 0},
  },
  {
    {bno055_imu__msg__BNO055Data__FIELD_NAME__temperature, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {bno055_imu__msg__BNO055Data__FIELD_NAME__calibration_system, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {bno055_imu__msg__BNO055Data__FIELD_NAME__calibration_gyro, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {bno055_imu__msg__BNO055Data__FIELD_NAME__calibration_accel, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {bno055_imu__msg__BNO055Data__FIELD_NAME__calibration_mag, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription bno055_imu__msg__BNO055Data__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Quaternion__TYPE_NAME, 28, 28},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Vector3__TYPE_NAME, 25, 25},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
bno055_imu__msg__BNO055Data__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {bno055_imu__msg__BNO055Data__TYPE_NAME, 25, 25},
      {bno055_imu__msg__BNO055Data__FIELDS, 13, 13},
    },
    {bno055_imu__msg__BNO055Data__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Quaternion__EXPECTED_HASH, geometry_msgs__msg__Quaternion__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = geometry_msgs__msg__Quaternion__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Vector3__EXPECTED_HASH, geometry_msgs__msg__Vector3__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = geometry_msgs__msg__Vector3__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# BNO055 9-DOF IMU Complete Data Message\n"
  "# All sensor data organized in one message\n"
  "\n"
  "# Header with timestamp and frame_id\n"
  "std_msgs/Header header\n"
  "\n"
  "# === ORIENTATION (Quaternion) ===\n"
  "# Fused orientation from sensor fusion\n"
  "geometry_msgs/Quaternion orientation\n"
  "\n"
  "# === ORIENTATION (Euler Angles in degrees) ===\n"
  "# Heading, Roll, Pitch\n"
  "float64 euler_heading      # 0 to 360 degrees\n"
  "float64 euler_roll         # -180 to 180 degrees\n"
  "float64 euler_pitch        # -90 to 90 degrees\n"
  "\n"
  "# === LINEAR ACCELERATION (m/s\\xc2\\xb2) ===\n"
  "# 3-axis accelerometer data\n"
  "geometry_msgs/Vector3 linear_acceleration\n"
  "\n"
  "# === ANGULAR VELOCITY (rad/s) ===\n"
  "# 3-axis gyroscope data\n"
  "geometry_msgs/Vector3 angular_velocity\n"
  "\n"
  "# === MAGNETIC FIELD (micro Tesla) ===\n"
  "# 3-axis magnetometer data\n"
  "geometry_msgs/Vector3 magnetic_field\n"
  "\n"
  "# === TEMPERATURE (\\xc2\\xb0C) ===\n"
  "float64 temperature\n"
  "\n"
  "# === CALIBRATION STATUS (0-3, where 3 is fully calibrated) ===\n"
  "uint8 calibration_system        # Overall system calibration\n"
  "uint8 calibration_gyro          # Gyroscope calibration\n"
  "uint8 calibration_accel         # Accelerometer calibration\n"
  "uint8 calibration_mag           # Magnetometer calibration";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
bno055_imu__msg__BNO055Data__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {bno055_imu__msg__BNO055Data__TYPE_NAME, 25, 25},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 1131, 1131},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
bno055_imu__msg__BNO055Data__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *bno055_imu__msg__BNO055Data__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *geometry_msgs__msg__Quaternion__get_individual_type_description_source(NULL);
    sources[3] = *geometry_msgs__msg__Vector3__get_individual_type_description_source(NULL);
    sources[4] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
