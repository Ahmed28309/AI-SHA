// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from bno055_imu:msg/BNO055Data.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "bno055_imu/msg/detail/bno055_data__struct.h"
#include "bno055_imu/msg/detail/bno055_data__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__quaternion__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__quaternion__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__vector3__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__vector3__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__vector3__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__vector3__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__vector3__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__vector3__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool bno055_imu__msg__bno055_data__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[39];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("bno055_imu.msg._bno055_data.BNO055Data", full_classname_dest, 38) == 0);
  }
  bno055_imu__msg__BNO055Data * ros_message = _ros_message;
  {  // header
    PyObject * field = PyObject_GetAttrString(_pymsg, "header");
    if (!field) {
      return false;
    }
    if (!std_msgs__msg__header__convert_from_py(field, &ros_message->header)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // orientation
    PyObject * field = PyObject_GetAttrString(_pymsg, "orientation");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__quaternion__convert_from_py(field, &ros_message->orientation)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // euler_heading
    PyObject * field = PyObject_GetAttrString(_pymsg, "euler_heading");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->euler_heading = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // euler_roll
    PyObject * field = PyObject_GetAttrString(_pymsg, "euler_roll");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->euler_roll = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // euler_pitch
    PyObject * field = PyObject_GetAttrString(_pymsg, "euler_pitch");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->euler_pitch = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // linear_acceleration
    PyObject * field = PyObject_GetAttrString(_pymsg, "linear_acceleration");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__vector3__convert_from_py(field, &ros_message->linear_acceleration)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // angular_velocity
    PyObject * field = PyObject_GetAttrString(_pymsg, "angular_velocity");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__vector3__convert_from_py(field, &ros_message->angular_velocity)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // magnetic_field
    PyObject * field = PyObject_GetAttrString(_pymsg, "magnetic_field");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__vector3__convert_from_py(field, &ros_message->magnetic_field)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // temperature
    PyObject * field = PyObject_GetAttrString(_pymsg, "temperature");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->temperature = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // calibration_system
    PyObject * field = PyObject_GetAttrString(_pymsg, "calibration_system");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->calibration_system = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // calibration_gyro
    PyObject * field = PyObject_GetAttrString(_pymsg, "calibration_gyro");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->calibration_gyro = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // calibration_accel
    PyObject * field = PyObject_GetAttrString(_pymsg, "calibration_accel");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->calibration_accel = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // calibration_mag
    PyObject * field = PyObject_GetAttrString(_pymsg, "calibration_mag");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->calibration_mag = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * bno055_imu__msg__bno055_data__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of BNO055Data */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("bno055_imu.msg._bno055_data");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "BNO055Data");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  bno055_imu__msg__BNO055Data * ros_message = (bno055_imu__msg__BNO055Data *)raw_ros_message;
  {  // header
    PyObject * field = NULL;
    field = std_msgs__msg__header__convert_to_py(&ros_message->header);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "header", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // orientation
    PyObject * field = NULL;
    field = geometry_msgs__msg__quaternion__convert_to_py(&ros_message->orientation);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "orientation", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // euler_heading
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->euler_heading);
    {
      int rc = PyObject_SetAttrString(_pymessage, "euler_heading", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // euler_roll
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->euler_roll);
    {
      int rc = PyObject_SetAttrString(_pymessage, "euler_roll", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // euler_pitch
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->euler_pitch);
    {
      int rc = PyObject_SetAttrString(_pymessage, "euler_pitch", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // linear_acceleration
    PyObject * field = NULL;
    field = geometry_msgs__msg__vector3__convert_to_py(&ros_message->linear_acceleration);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "linear_acceleration", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // angular_velocity
    PyObject * field = NULL;
    field = geometry_msgs__msg__vector3__convert_to_py(&ros_message->angular_velocity);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "angular_velocity", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // magnetic_field
    PyObject * field = NULL;
    field = geometry_msgs__msg__vector3__convert_to_py(&ros_message->magnetic_field);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "magnetic_field", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // temperature
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->temperature);
    {
      int rc = PyObject_SetAttrString(_pymessage, "temperature", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // calibration_system
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->calibration_system);
    {
      int rc = PyObject_SetAttrString(_pymessage, "calibration_system", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // calibration_gyro
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->calibration_gyro);
    {
      int rc = PyObject_SetAttrString(_pymessage, "calibration_gyro", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // calibration_accel
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->calibration_accel);
    {
      int rc = PyObject_SetAttrString(_pymessage, "calibration_accel", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // calibration_mag
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->calibration_mag);
    {
      int rc = PyObject_SetAttrString(_pymessage, "calibration_mag", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
