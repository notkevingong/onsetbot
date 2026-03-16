# generated from rosidl_generator_py/resource/_idl.py.em
# with input from onset_interfaces:msg/LaunchCommand.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_LaunchCommand(type):
    """Metaclass of message 'LaunchCommand'."""

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
            module = import_type_support('onset_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'onset_interfaces.msg.LaunchCommand')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__launch_command
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__launch_command
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__launch_command
            cls._TYPE_SUPPORT = module.type_support_msg__msg__launch_command
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__launch_command

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class LaunchCommand(metaclass=Metaclass_LaunchCommand):
    """Message class 'LaunchCommand'."""

    __slots__ = [
        '_velocity',
        '_angle_launch',
        '_angle_turret',
    ]

    _fields_and_field_types = {
        'velocity': 'double',
        'angle_launch': 'double',
        'angle_turret': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.velocity = kwargs.get('velocity', float())
        self.angle_launch = kwargs.get('angle_launch', float())
        self.angle_turret = kwargs.get('angle_turret', float())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
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
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.velocity != other.velocity:
            return False
        if self.angle_launch != other.angle_launch:
            return False
        if self.angle_turret != other.angle_turret:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def velocity(self):
        """Message field 'velocity'."""
        return self._velocity

    @velocity.setter
    def velocity(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'velocity' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'velocity' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._velocity = value

    @builtins.property
    def angle_launch(self):
        """Message field 'angle_launch'."""
        return self._angle_launch

    @angle_launch.setter
    def angle_launch(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'angle_launch' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'angle_launch' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._angle_launch = value

    @builtins.property
    def angle_turret(self):
        """Message field 'angle_turret'."""
        return self._angle_turret

    @angle_turret.setter
    def angle_turret(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'angle_turret' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'angle_turret' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._angle_turret = value
