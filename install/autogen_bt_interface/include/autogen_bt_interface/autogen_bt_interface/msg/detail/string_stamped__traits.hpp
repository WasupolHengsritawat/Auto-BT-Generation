// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from autogen_bt_interface:msg/StringStamped.idl
// generated code does not contain a copyright notice

#ifndef AUTOGEN_BT_INTERFACE__MSG__DETAIL__STRING_STAMPED__TRAITS_HPP_
#define AUTOGEN_BT_INTERFACE__MSG__DETAIL__STRING_STAMPED__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "autogen_bt_interface/msg/detail/string_stamped__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace autogen_bt_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const StringStamped & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: data
  {
    out << "data: ";
    rosidl_generator_traits::value_to_yaml(msg.data, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const StringStamped & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "data: ";
    rosidl_generator_traits::value_to_yaml(msg.data, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const StringStamped & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace autogen_bt_interface

namespace rosidl_generator_traits
{

[[deprecated("use autogen_bt_interface::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const autogen_bt_interface::msg::StringStamped & msg,
  std::ostream & out, size_t indentation = 0)
{
  autogen_bt_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autogen_bt_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const autogen_bt_interface::msg::StringStamped & msg)
{
  return autogen_bt_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<autogen_bt_interface::msg::StringStamped>()
{
  return "autogen_bt_interface::msg::StringStamped";
}

template<>
inline const char * name<autogen_bt_interface::msg::StringStamped>()
{
  return "autogen_bt_interface/msg/StringStamped";
}

template<>
struct has_fixed_size<autogen_bt_interface::msg::StringStamped>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autogen_bt_interface::msg::StringStamped>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autogen_bt_interface::msg::StringStamped>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUTOGEN_BT_INTERFACE__MSG__DETAIL__STRING_STAMPED__TRAITS_HPP_
