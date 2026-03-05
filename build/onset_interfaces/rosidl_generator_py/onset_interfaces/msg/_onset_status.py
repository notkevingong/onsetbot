# generated from rosidl_generator_py/resource/_idl.py.em
# with input from onset_interfaces:msg/OnsetStatus.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_OnsetStatus(type):
    """Metaclass of message 'OnsetStatus'."""

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
                'onset_interfaces.msg.OnsetStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__onset_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__onset_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__onset_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__onset_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__onset_status

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class OnsetStatus(metaclass=Metaclass_OnsetStatus):
    """Message class 'OnsetStatus'."""

    __slots__ = [
        '_bool_homed',
        '_bool_busy',
    ]

    _fields_and_field_types = {
        'bool_homed': 'boolean',
        'bool_busy': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.bool_homed = kwargs.get('bool_homed', bool())
        self.bool_busy = kwargs.get('bool_busy', bool())

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
        if self.bool_homed != other.bool_homed:
            return False
        if self.bool_busy != other.bool_busy:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def bool_homed(self):
        """Message field 'bool_homed'."""
        return self._bool_homed

    @bool_homed.setter
    def bool_homed(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'bool_homed' field must be of type 'bool'"
        self._bool_homed = value

    @builtins.property
    def bool_busy(self):
        """Message field 'bool_busy'."""
        return self._bool_busy

    @bool_busy.setter
    def bool_busy(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'bool_busy' field must be of type 'bool'"
        self._bool_busy = value
