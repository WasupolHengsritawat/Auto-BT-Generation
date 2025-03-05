// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from autogen_bt_interface:srv/PickingRequest.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "autogen_bt_interface/srv/detail/picking_request__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace autogen_bt_interface
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _PickingRequest_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PickingRequest_Request_type_support_ids_t;

static const _PickingRequest_Request_type_support_ids_t _PickingRequest_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _PickingRequest_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PickingRequest_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PickingRequest_Request_type_support_symbol_names_t _PickingRequest_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, autogen_bt_interface, srv, PickingRequest_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, autogen_bt_interface, srv, PickingRequest_Request)),
  }
};

typedef struct _PickingRequest_Request_type_support_data_t
{
  void * data[2];
} _PickingRequest_Request_type_support_data_t;

static _PickingRequest_Request_type_support_data_t _PickingRequest_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PickingRequest_Request_message_typesupport_map = {
  2,
  "autogen_bt_interface",
  &_PickingRequest_Request_message_typesupport_ids.typesupport_identifier[0],
  &_PickingRequest_Request_message_typesupport_symbol_names.symbol_name[0],
  &_PickingRequest_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t PickingRequest_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PickingRequest_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace autogen_bt_interface

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<autogen_bt_interface::srv::PickingRequest_Request>()
{
  return &::autogen_bt_interface::srv::rosidl_typesupport_cpp::PickingRequest_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, autogen_bt_interface, srv, PickingRequest_Request)() {
  return get_message_type_support_handle<autogen_bt_interface::srv::PickingRequest_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "autogen_bt_interface/srv/detail/picking_request__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace autogen_bt_interface
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _PickingRequest_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PickingRequest_Response_type_support_ids_t;

static const _PickingRequest_Response_type_support_ids_t _PickingRequest_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _PickingRequest_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PickingRequest_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PickingRequest_Response_type_support_symbol_names_t _PickingRequest_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, autogen_bt_interface, srv, PickingRequest_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, autogen_bt_interface, srv, PickingRequest_Response)),
  }
};

typedef struct _PickingRequest_Response_type_support_data_t
{
  void * data[2];
} _PickingRequest_Response_type_support_data_t;

static _PickingRequest_Response_type_support_data_t _PickingRequest_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PickingRequest_Response_message_typesupport_map = {
  2,
  "autogen_bt_interface",
  &_PickingRequest_Response_message_typesupport_ids.typesupport_identifier[0],
  &_PickingRequest_Response_message_typesupport_symbol_names.symbol_name[0],
  &_PickingRequest_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t PickingRequest_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PickingRequest_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace autogen_bt_interface

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<autogen_bt_interface::srv::PickingRequest_Response>()
{
  return &::autogen_bt_interface::srv::rosidl_typesupport_cpp::PickingRequest_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, autogen_bt_interface, srv, PickingRequest_Response)() {
  return get_message_type_support_handle<autogen_bt_interface::srv::PickingRequest_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "autogen_bt_interface/srv/detail/picking_request__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace autogen_bt_interface
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _PickingRequest_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PickingRequest_type_support_ids_t;

static const _PickingRequest_type_support_ids_t _PickingRequest_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _PickingRequest_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PickingRequest_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PickingRequest_type_support_symbol_names_t _PickingRequest_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, autogen_bt_interface, srv, PickingRequest)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, autogen_bt_interface, srv, PickingRequest)),
  }
};

typedef struct _PickingRequest_type_support_data_t
{
  void * data[2];
} _PickingRequest_type_support_data_t;

static _PickingRequest_type_support_data_t _PickingRequest_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PickingRequest_service_typesupport_map = {
  2,
  "autogen_bt_interface",
  &_PickingRequest_service_typesupport_ids.typesupport_identifier[0],
  &_PickingRequest_service_typesupport_symbol_names.symbol_name[0],
  &_PickingRequest_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t PickingRequest_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PickingRequest_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace autogen_bt_interface

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<autogen_bt_interface::srv::PickingRequest>()
{
  return &::autogen_bt_interface::srv::rosidl_typesupport_cpp::PickingRequest_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, autogen_bt_interface, srv, PickingRequest)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<autogen_bt_interface::srv::PickingRequest>();
}

#ifdef __cplusplus
}
#endif
