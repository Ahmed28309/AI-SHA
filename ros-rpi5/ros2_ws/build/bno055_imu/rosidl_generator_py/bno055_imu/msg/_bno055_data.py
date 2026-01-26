# generated from rosidl_generator_py/resource/_idl.py.em
# with input from bno055_imu:msg/BNO055Data.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_BNO055Data(type):
    """Metaclass of message 'BNO055Data'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('bno055_imu')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'bno055_imu.msg.BNO055Data')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__bno055_data
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__bno055_data
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__bno055_data
            cls._TYPE_SUPPORT = module.type_support_msg__msg__bno055_data
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__bno055_data

            from geometry_msgs.msg import Quaternion
            if Quaternion.__class__._TYPE_SUPPORT is None:
                Quaternion.__class__.__import_type_support__()

            from geometry_msgs.msg import Vector3
            if Vector3.__class__._TYPE_SUPPORT is None:
                Vector3.__class__.__import_type_support__()

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class BNO055Data(metaclass=Metaclass_BNO055Data):
    """Message class 'BNO055Data'."""

    __slots__ = [
        '_header',
        '_orientation',
        '_euler_heading',
        '_euler_roll',
        '_euler_pitch',
        '_linear_acceleration',
        '_angular_velocity',
        '_magnetic_field',
        '_temperature',
        '_calibration_system',
        '_calibration_gyro',
        '_calibration_accel',
        '_calibration_mag',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'orientation': 'geometry_msgs/Quaternion',
        'euler_heading': 'double',
        'euler_roll': 'double',
        'euler_pitch': 'double',
        'linear_acceleration': 'geometry_msgs/Vector3',
        'angular_velocity': 'geometry_msgs/Vector3',
        'magnetic_field': 'geometry_msgs/Vector3',
        'temperature': 'double',
        'calibration_system': 'uint8',
        'calibration_gyro': 'uint8',
        'calibration_accel': 'uint8',
        'calibration_mag': 'uint8',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Quaternion'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        if 'check_fields' in kwargs:
            self._check_fields = kwargs['check_fields']
        else:
            self._check_fields = ros_python_check_fields == '1'
        if self._check_fields:
            assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
                'Invalid arguments passed to constructor: %s' % \
                ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        from geometry_msgs.msg import Quaternion
        self.orientation = kwargs.get('orientation', Quaternion())
        self.euler_heading = kwargs.get('euler_heading', float())
        self.euler_roll = kwargs.get('euler_roll', float())
        self.euler_pitch = kwargs.get('euler_pitch', float())
        from geometry_msgs.msg import Vector3
        self.linear_acceleration = kwargs.get('linear_acceleration', Vector3())
        from geometry_msgs.msg import Vector3
        self.angular_velocity = kwargs.get('angular_velocity', Vector3())
        from geometry_msgs.msg import Vector3
        self.magnetic_field = kwargs.get('magnetic_field', Vector3())
        self.temperature = kwargs.get('temperature', float())
        self.calibration_system = kwargs.get('calibration_system', int())
        self.calibration_gyro = kwargs.get('calibration_gyro', int())
        self.calibration_accel = kwargs.get('calibration_accel', int())
        self.calibration_mag = kwargs.get('calibration_mag', int())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.get_fields_and_field_types().keys(), self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    if self._check_fields:
                        assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.header != other.header:
            return False
        if self.orientation != other.orientation:
            return False
        if self.euler_heading != other.euler_heading:
            return False
        if self.euler_roll != other.euler_roll:
            return False
        if self.euler_pitch != other.euler_pitch:
            return False
        if self.linear_acceleration != other.linear_acceleration:
            return False
        if self.angular_velocity != other.angular_velocity:
            return False
        if self.magnetic_field != other.magnetic_field:
            return False
        if self.temperature != other.temperature:
            return False
        if self.calibration_system != other.calibration_system:
            return False
        if self.calibration_gyro != other.calibration_gyro:
            return False
        if self.calibration_accel != other.calibration_accel:
            return False
        if self.calibration_mag != other.calibration_mag:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if self._check_fields:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def orientation(self):
        """Message field 'orientation'."""
        return self._orientation

    @orientation.setter
    def orientation(self, value):
        if self._check_fields:
            from geometry_msgs.msg import Quaternion
            assert \
                isinstance(value, Quaternion), \
                "The 'orientation' field must be a sub message of type 'Quaternion'"
        self._orientation = value

    @builtins.property
    def euler_heading(self):
        """Message field 'euler_heading'."""
        return self._euler_heading

    @euler_heading.setter
    def euler_heading(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'euler_heading' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'euler_heading' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._euler_heading = value

    @builtins.property
    def euler_roll(self):
        """Message field 'euler_roll'."""
        return self._euler_roll

    @euler_roll.setter
    def euler_roll(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'euler_roll' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'euler_roll' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._euler_roll = value

    @builtins.property
    def euler_pitch(self):
        """Message field 'euler_pitch'."""
        return self._euler_pitch

    @euler_pitch.setter
    def euler_pitch(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'euler_pitch' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'euler_pitch' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._euler_pitch = value

    @builtins.property
    def linear_acceleration(self):
        """Message field 'linear_acceleration'."""
        return self._linear_acceleration

    @linear_acceleration.setter
    def linear_acceleration(self, value):
        if self._check_fields:
            from geometry_msgs.msg import Vector3
            assert \
                isinstance(value, Vector3), \
                "The 'linear_acceleration' field must be a sub message of type 'Vector3'"
        self._linear_acceleration = value

    @builtins.property
    def angular_velocity(self):
        """Message field 'angular_velocity'."""
        return self._angular_velocity

    @angular_velocity.setter
    def angular_velocity(self, value):
        if self._check_fields:
            from geometry_msgs.msg import Vector3
            assert \
                isinstance(value, Vector3), \
                "The 'angular_velocity' field must be a sub message of type 'Vector3'"
        self._angular_velocity = value

    @builtins.property
    def magnetic_field(self):
        """Message field 'magnetic_field'."""
        return self._magnetic_field

    @magnetic_field.setter
    def magnetic_field(self, value):
        if self._check_fields:
            from geometry_msgs.msg import Vector3
            assert \
                isinstance(value, Vector3), \
                "The 'magnetic_field' field must be a sub message of type 'Vector3'"
        self._magnetic_field = value

    @builtins.property
    def temperature(self):
        """Message field 'temperature'."""
        return self._temperature

    @temperature.setter
    def temperature(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'temperature' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'temperature' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._temperature = value

    @builtins.property
    def calibration_system(self):
        """Message field 'calibration_system'."""
        return self._calibration_system

    @calibration_system.setter
    def calibration_system(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'calibration_system' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'calibration_system' field must be an unsigned integer in [0, 255]"
        self._calibration_system = value

    @builtins.property
    def calibration_gyro(self):
        """Message field 'calibration_gyro'."""
        return self._calibration_gyro

    @calibration_gyro.setter
    def calibration_gyro(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'calibration_gyro' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'calibration_gyro' field must be an unsigned integer in [0, 255]"
        self._calibration_gyro = value

    @builtins.property
    def calibration_accel(self):
        """Message field 'calibration_accel'."""
        return self._calibration_accel

    @calibration_accel.setter
    def calibration_accel(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'calibration_accel' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'calibration_accel' field must be an unsigned integer in [0, 255]"
        self._calibration_accel = value

    @builtins.property
    def calibration_mag(self):
        """Message field 'calibration_mag'."""
        return self._calibration_mag

    @calibration_mag.setter
    def calibration_mag(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'calibration_mag' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'calibration_mag' field must be an unsigned integer in [0, 255]"
        self._calibration_mag = value
