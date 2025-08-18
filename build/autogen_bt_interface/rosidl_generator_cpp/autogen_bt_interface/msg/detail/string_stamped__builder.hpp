// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autogen_bt_interface:msg/StringStamped.idl
// generated code does not contain a copyright notice

#ifndef AUTOGEN_BT_INTERFACE__MSG__DETAIL__STRING_STAMPED__BUILDER_HPP_
#define AUTOGEN_BT_INTERFACE__MSG__DETAIL__STRING_STAMPED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autogen_bt_interface/msg/detail/string_stamped__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autogen_bt_interface
{

namespace msg
{

namespace builder
{

class Init_StringStamped_data
{
public:
  explicit Init_StringStamped_data(::autogen_bt_interface::msg::StringStamped & msg)
  : msg_(msg)
  {}
  ::autogen_bt_interface::msg::StringStamped data(::autogen_bt_interface::msg::StringStamped::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autogen_bt_interface::msg::StringStamped msg_;
};

class Init_StringStamped_header
{
public:
  Init_StringStamped_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_StringStamped_data header(::autogen_bt_interface::msg::StringStamped::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_StringStamped_data(msg_);
  }

private:
  ::autogen_bt_interface::msg::StringStamped msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autogen_bt_interface::msg::StringStamped>()
{
  return autogen_bt_interface::msg::builder::Init_StringStamped_header();
}

}  // namespace autogen_bt_interface

#endif  // AUTOGEN_BT_INTERFACE__MSG__DETAIL__STRING_STAMPED__BUILDER_HPP_
