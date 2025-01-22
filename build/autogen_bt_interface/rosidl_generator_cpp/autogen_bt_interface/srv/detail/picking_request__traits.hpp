// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from autogen_bt_interface:srv/PickingRequest.idl
// generated code does not contain a copyright notice

#ifndef AUTOGEN_BT_INTERFACE__SRV__DETAIL__PICKING_REQUEST__TRAITS_HPP_
#define AUTOGEN_BT_INTERFACE__SRV__DETAIL__PICKING_REQUEST__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "autogen_bt_interface/srv/detail/picking_request__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'host_name'
#include "std_msgs/msg/detail/string__traits.hpp"
// Member 'obj_id'
#include "std_msgs/msg/detail/u_int32__traits.hpp"
// Member 'status'
#include "std_msgs/msg/detail/bool__traits.hpp"

namespace autogen_bt_interface
{

namespace srv
{

inline void to_flow_style_yaml(
  const PickingRequest_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: host_name
  {
    out << "host_name: ";
    to_flow_style_yaml(msg.host_name, out);
    out << ", ";
  }

  // member: obj_id
  {
    out << "obj_id: ";
    to_flow_style_yaml(msg.obj_id, out);
    out << ", ";
  }

  // member: status
  {
    out << "status: ";
    to_flow_style_yaml(msg.status, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PickingRequest_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: host_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "host_name:\n";
    to_block_style_yaml(msg.host_name, out, indentation + 2);
  }

  // member: obj_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "obj_id:\n";
    to_block_style_yaml(msg.obj_id, out, indentation + 2);
  }

  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status:\n";
    to_block_style_yaml(msg.status, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PickingRequest_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace autogen_bt_interface

namespace rosidl_generator_traits
{

[[deprecated("use autogen_bt_interface::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const autogen_bt_interface::srv::PickingRequest_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  autogen_bt_interface::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autogen_bt_interface::srv::to_yaml() instead")]]
inline std::string to_yaml(const autogen_bt_interface::srv::PickingRequest_Request & msg)
{
  return autogen_bt_interface::srv::to_yaml(msg);
}

template<>
inline const char * data_type<autogen_bt_interface::srv::PickingRequest_Request>()
{
  return "autogen_bt_interface::srv::PickingRequest_Request";
}

template<>
inline const char * name<autogen_bt_interface::srv::PickingRequest_Request>()
{
  return "autogen_bt_interface/srv/PickingRequest_Request";
}

template<>
struct has_fixed_size<autogen_bt_interface::srv::PickingRequest_Request>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Bool>::value && has_fixed_size<std_msgs::msg::String>::value && has_fixed_size<std_msgs::msg::UInt32>::value> {};

template<>
struct has_bounded_size<autogen_bt_interface::srv::PickingRequest_Request>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Bool>::value && has_bounded_size<std_msgs::msg::String>::value && has_bounded_size<std_msgs::msg::UInt32>::value> {};

template<>
struct is_message<autogen_bt_interface::srv::PickingRequest_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace autogen_bt_interface
{

namespace srv
{

inline void to_flow_style_yaml(
  const PickingRequest_Response & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PickingRequest_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PickingRequest_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace autogen_bt_interface

namespace rosidl_generator_traits
{

[[deprecated("use autogen_bt_interface::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const autogen_bt_interface::srv::PickingRequest_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  autogen_bt_interface::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autogen_bt_interface::srv::to_yaml() instead")]]
inline std::string to_yaml(const autogen_bt_interface::srv::PickingRequest_Response & msg)
{
  return autogen_bt_interface::srv::to_yaml(msg);
}

template<>
inline const char * data_type<autogen_bt_interface::srv::PickingRequest_Response>()
{
  return "autogen_bt_interface::srv::PickingRequest_Response";
}

template<>
inline const char * name<autogen_bt_interface::srv::PickingRequest_Response>()
{
  return "autogen_bt_interface/srv/PickingRequest_Response";
}

template<>
struct has_fixed_size<autogen_bt_interface::srv::PickingRequest_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<autogen_bt_interface::srv::PickingRequest_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<autogen_bt_interface::srv::PickingRequest_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<autogen_bt_interface::srv::PickingRequest>()
{
  return "autogen_bt_interface::srv::PickingRequest";
}

template<>
inline const char * name<autogen_bt_interface::srv::PickingRequest>()
{
  return "autogen_bt_interface/srv/PickingRequest";
}

template<>
struct has_fixed_size<autogen_bt_interface::srv::PickingRequest>
  : std::integral_constant<
    bool,
    has_fixed_size<autogen_bt_interface::srv::PickingRequest_Request>::value &&
    has_fixed_size<autogen_bt_interface::srv::PickingRequest_Response>::value
  >
{
};

template<>
struct has_bounded_size<autogen_bt_interface::srv::PickingRequest>
  : std::integral_constant<
    bool,
    has_bounded_size<autogen_bt_interface::srv::PickingRequest_Request>::value &&
    has_bounded_size<autogen_bt_interface::srv::PickingRequest_Response>::value
  >
{
};

template<>
struct is_service<autogen_bt_interface::srv::PickingRequest>
  : std::true_type
{
};

template<>
struct is_service_request<autogen_bt_interface::srv::PickingRequest_Request>
  : std::true_type
{
};

template<>
struct is_service_response<autogen_bt_interface::srv::PickingRequest_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // AUTOGEN_BT_INTERFACE__SRV__DETAIL__PICKING_REQUEST__TRAITS_HPP_
