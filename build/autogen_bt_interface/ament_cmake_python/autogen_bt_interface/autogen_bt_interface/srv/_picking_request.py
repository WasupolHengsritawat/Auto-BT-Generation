# generated from rosidl_generator_py/resource/_idl.py.em
# with input from autogen_bt_interface:srv/PickingRequest.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_PickingRequest_Request(type):
    """Metaclass of message 'PickingRequest_Request'."""

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
            module = import_type_support('autogen_bt_interface')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'autogen_bt_interface.srv.PickingRequest_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__picking_request__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__picking_request__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__picking_request__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__picking_request__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__picking_request__request

            from std_msgs.msg import Bool
            if Bool.__class__._TYPE_SUPPORT is None:
                Bool.__class__.__import_type_support__()

            from std_msgs.msg import String
            if String.__class__._TYPE_SUPPORT is None:
                String.__class__.__import_type_support__()

            from std_msgs.msg import UInt32
            if UInt32.__class__._TYPE_SUPPORT is None:
                UInt32.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class PickingRequest_Request(metaclass=Metaclass_PickingRequest_Request):
    """Message class 'PickingRequest_Request'."""

    __slots__ = [
        '_host_name',
        '_obj_id',
        '_status',
    ]

    _fields_and_field_types = {
        'host_name': 'std_msgs/String',
        'obj_id': 'std_msgs/UInt32',
        'status': 'std_msgs/Bool',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'String'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'UInt32'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Bool'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import String
        self.host_name = kwargs.get('host_name', String())
        from std_msgs.msg import UInt32
        self.obj_id = kwargs.get('obj_id', UInt32())
        from std_msgs.msg import Bool
        self.status = kwargs.get('status', Bool())

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
        if self.host_name != other.host_name:
            return False
        if self.obj_id != other.obj_id:
            return False
        if self.status != other.status:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def host_name(self):
        """Message field 'host_name'."""
        return self._host_name

    @host_name.setter
    def host_name(self, value):
        if __debug__:
            from std_msgs.msg import String
            assert \
                isinstance(value, String), \
                "The 'host_name' field must be a sub message of type 'String'"
        self._host_name = value

    @builtins.property
    def obj_id(self):
        """Message field 'obj_id'."""
        return self._obj_id

    @obj_id.setter
    def obj_id(self, value):
        if __debug__:
            from std_msgs.msg import UInt32
            assert \
                isinstance(value, UInt32), \
                "The 'obj_id' field must be a sub message of type 'UInt32'"
        self._obj_id = value

    @builtins.property
    def status(self):
        """Message field 'status'."""
        return self._status

    @status.setter
    def status(self, value):
        if __debug__:
            from std_msgs.msg import Bool
            assert \
                isinstance(value, Bool), \
                "The 'status' field must be a sub message of type 'Bool'"
        self._status = value


# Import statements for member types

# already imported above
# import rosidl_parser.definition


class Metaclass_PickingRequest_Response(type):
    """Metaclass of message 'PickingRequest_Response'."""

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
            module = import_type_support('autogen_bt_interface')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'autogen_bt_interface.srv.PickingRequest_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__picking_request__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__picking_request__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__picking_request__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__picking_request__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__picking_request__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class PickingRequest_Response(metaclass=Metaclass_PickingRequest_Response):
    """Message class 'PickingRequest_Response'."""

    __slots__ = [
    ]

    _fields_and_field_types = {
    }

    SLOT_TYPES = (
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))

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
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)


class Metaclass_PickingRequest(type):
    """Metaclass of service 'PickingRequest'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('autogen_bt_interface')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'autogen_bt_interface.srv.PickingRequest')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__picking_request

            from autogen_bt_interface.srv import _picking_request
            if _picking_request.Metaclass_PickingRequest_Request._TYPE_SUPPORT is None:
                _picking_request.Metaclass_PickingRequest_Request.__import_type_support__()
            if _picking_request.Metaclass_PickingRequest_Response._TYPE_SUPPORT is None:
                _picking_request.Metaclass_PickingRequest_Response.__import_type_support__()


class PickingRequest(metaclass=Metaclass_PickingRequest):
    from autogen_bt_interface.srv._picking_request import PickingRequest_Request as Request
    from autogen_bt_interface.srv._picking_request import PickingRequest_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
