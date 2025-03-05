// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from autogen_bt_interface:srv/ChargingRequest.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "autogen_bt_interface/srv/detail/charging_request__struct.hpp"
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

typedef struct _ChargingRequest_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ChargingRequest_Request_type_support_ids_t;

static const _ChargingRequest_Request_type_support_ids_t _ChargingRequest_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ChargingRequest_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ChargingRequest_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ChargingRequest_Request_type_support_symbol_names_t _ChargingRequest_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, autogen_bt_interface, srv, ChargingRequest_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, autogen_bt_interface, srv, ChargingRequest_Request)),
  }
};

typedef struct _ChargingRequest_Request_type_support_data_t
{
  void * data[2];
} _ChargingRequest_Request_type_support_data_t;

static _ChargingRequest_Request_type_support_data_t _ChargingRequest_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ChargingRequest_Request_message_typesupport_map = {
  2,
  "autogen_bt_interface",
  &_ChargingRequest_Request_message_typesupport_ids.typesupport_identifier[0],
  &_ChargingRequest_Request_message_typesupport_symbol_names.symbol_name[0],
  &_ChargingRequest_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ChargingRequest_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ChargingRequest_Request_message_typesupport_map),
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
get_message_type_support_handle<autogen_bt_interface::srv::ChargingRequest_Request>()
{
  return &::autogen_bt_interface::srv::rosidl_typesupport_cpp::ChargingRequest_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, autogen_bt_interface, srv, ChargingRequest_Request)() {
  return get_message_type_support_handle<autogen_bt_interface::srv::ChargingRequest_Request>();
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
// #include "autogen_bt_interface/srv/detail/charging_request__struct.hpp"
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

typedef struct _ChargingRequest_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ChargingRequest_Response_type_support_ids_t;

static const _ChargingRequest_Response_type_support_ids_t _ChargingRequest_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ChargingRequest_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ChargingRequest_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ChargingRequest_Response_type_support_symbol_names_t _ChargingRequest_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, autogen_bt_interface, srv, ChargingRequest_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, autogen_bt_interface, srv, ChargingRequest_Response)),
  }
};

typedef struct _ChargingRequest_Response_type_support_data_t
{
  void * data[2];
} _ChargingRequest_Response_type_support_data_t;

static _ChargingRequest_Response_type_support_data_t _ChargingRequest_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ChargingRequest_Response_message_typesupport_map = {
  2,
  "autogen_bt_interface",
  &_ChargingRequest_Response_message_typesupport_ids.typesupport_identifier[0],
  &_ChargingRequest_Response_message_typesupport_symbol_names.symbol_name[0],
  &_ChargingRequest_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ChargingRequest_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ChargingRequest_Response_message_typesupport_map),
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
get_message_type_support_handle<autogen_bt_interface::srv::ChargingRequest_Response>()
{
  return &::autogen_bt_interface::srv::rosidl_typesupport_cpp::ChargingRequest_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, autogen_bt_interface, srv, ChargingRequest_Response)() {
  return get_message_type_support_handle<autogen_bt_interface::srv::ChargingRequest_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "autogen_bt_interface/srv/detail/charging_request__struct.hpp"
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

typedef struct _ChargingRequest_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ChargingRequest_type_support_ids_t;

static const _ChargingRequest_type_support_ids_t _ChargingRequest_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ChargingRequest_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ChargingRequest_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ChargingRequest_type_support_symbol_names_t _ChargingRequest_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, autogen_bt_interface, srv, ChargingRequest)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, autogen_bt_interface, srv, ChargingRequest)),
  }
};

typedef struct _ChargingRequest_type_support_data_t
{
  void * data[2];
} _ChargingRequest_type_support_data_t;

static _ChargingRequest_type_support_data_t _ChargingRequest_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ChargingRequest_service_typesupport_map = {
  2,
  "autogen_bt_interface",
  &_ChargingRequest_service_typesupport_ids.typesupport_identifier[0],
  &_ChargingRequest_service_typesupport_symbol_names.symbol_name[0],
  &_ChargingRequest_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t ChargingRequest_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ChargingRequest_service_typesupport_map),
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
get_service_type_support_handle<autogen_bt_interface::srv::ChargingRequest>()
{
  return &::autogen_bt_interface::srv::rosidl_typesupport_cpp::ChargingRequest_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, autogen_bt_interface, srv, ChargingRequest)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<autogen_bt_interface::srv::ChargingRequest>();
}

#ifdef __cplusplus
}
#endif
