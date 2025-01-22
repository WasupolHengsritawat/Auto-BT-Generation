// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from autogen_bt_interface:srv/ChargingRequest.idl
// generated code does not contain a copyright notice

#ifndef AUTOGEN_BT_INTERFACE__SRV__DETAIL__CHARGING_REQUEST__TRAITS_HPP_
#define AUTOGEN_BT_INTERFACE__SRV__DETAIL__CHARGING_REQUEST__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "autogen_bt_interface/srv/detail/charging_request__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'status'
#include "std_msgs/msg/detail/bool__traits.hpp"

namespace autogen_bt_interface
{

namespace srv
{

inline void to_flow_style_yaml(
  const ChargingRequest_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    to_flow_style_yaml(msg.status, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ChargingRequest_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status:\n";
    to_block_style_yaml(msg.status, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ChargingRequest_Request & msg, bool use_flow_style = false)
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
  const autogen_bt_interface::srv::ChargingRequest_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  autogen_bt_interface::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autogen_bt_interface::srv::to_yaml() instead")]]
inline std::string to_yaml(const autogen_bt_interface::srv::ChargingRequest_Request & msg)
{
  return autogen_bt_interface::srv::to_yaml(msg);
}

template<>
inline const char * data_type<autogen_bt_interface::srv::ChargingRequest_Request>()
{
  return "autogen_bt_interface::srv::ChargingRequest_Request";
}

template<>
inline const char * name<autogen_bt_interface::srv::ChargingRequest_Request>()
{
  return "autogen_bt_interface/srv/ChargingRequest_Request";
}

template<>
struct has_fixed_size<autogen_bt_interface::srv::ChargingRequest_Request>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Bool>::value> {};

template<>
struct has_bounded_size<autogen_bt_interface::srv::ChargingRequest_Request>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Bool>::value> {};

template<>
struct is_message<autogen_bt_interface::srv::ChargingRequest_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace autogen_bt_interface
{

namespace srv
{

inline void to_flow_style_yaml(
  const ChargingRequest_Response & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ChargingRequest_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ChargingRequest_Response & msg, bool use_flow_style = false)
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
  const autogen_bt_interface::srv::ChargingRequest_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  autogen_bt_interface::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autogen_bt_interface::srv::to_yaml() instead")]]
inline std::string to_yaml(const autogen_bt_interface::srv::ChargingRequest_Response & msg)
{
  return autogen_bt_interface::srv::to_yaml(msg);
}

template<>
inline const char * data_type<autogen_bt_interface::srv::ChargingRequest_Response>()
{
  return "autogen_bt_interface::srv::ChargingRequest_Response";
}

template<>
inline const char * name<autogen_bt_interface::srv::ChargingRequest_Response>()
{
  return "autogen_bt_interface/srv/ChargingRequest_Response";
}

template<>
struct has_fixed_size<autogen_bt_interface::srv::ChargingRequest_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<autogen_bt_interface::srv::ChargingRequest_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<autogen_bt_interface::srv::ChargingRequest_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<autogen_bt_interface::srv::ChargingRequest>()
{
  return "autogen_bt_interface::srv::ChargingRequest";
}

template<>
inline const char * name<autogen_bt_interface::srv::ChargingRequest>()
{
  return "autogen_bt_interface/srv/ChargingRequest";
}

template<>
struct has_fixed_size<autogen_bt_interface::srv::ChargingRequest>
  : std::integral_constant<
    bool,
    has_fixed_size<autogen_bt_interface::srv::ChargingRequest_Request>::value &&
    has_fixed_size<autogen_bt_interface::srv::ChargingRequest_Response>::value
  >
{
};

template<>
struct has_bounded_size<autogen_bt_interface::srv::ChargingRequest>
  : std::integral_constant<
    bool,
    has_bounded_size<autogen_bt_interface::srv::ChargingRequest_Request>::value &&
    has_bounded_size<autogen_bt_interface::srv::ChargingRequest_Response>::value
  >
{
};

template<>
struct is_service<autogen_bt_interface::srv::ChargingRequest>
  : std::true_type
{
};

template<>
struct is_service_request<autogen_bt_interface::srv::ChargingRequest_Request>
  : std::true_type
{
};

template<>
struct is_service_response<autogen_bt_interface::srv::ChargingRequest_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // AUTOGEN_BT_INTERFACE__SRV__DETAIL__CHARGING_REQUEST__TRAITS_HPP_
