// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autogen_bt_interface:srv/ChargingRequest.idl
// generated code does not contain a copyright notice

#ifndef AUTOGEN_BT_INTERFACE__SRV__DETAIL__CHARGING_REQUEST__BUILDER_HPP_
#define AUTOGEN_BT_INTERFACE__SRV__DETAIL__CHARGING_REQUEST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autogen_bt_interface/srv/detail/charging_request__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autogen_bt_interface
{

namespace srv
{

namespace builder
{

class Init_ChargingRequest_Request_status
{
public:
  Init_ChargingRequest_Request_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::autogen_bt_interface::srv::ChargingRequest_Request status(::autogen_bt_interface::srv::ChargingRequest_Request::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autogen_bt_interface::srv::ChargingRequest_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autogen_bt_interface::srv::ChargingRequest_Request>()
{
  return autogen_bt_interface::srv::builder::Init_ChargingRequest_Request_status();
}

}  // namespace autogen_bt_interface


namespace autogen_bt_interface
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autogen_bt_interface::srv::ChargingRequest_Response>()
{
  return ::autogen_bt_interface::srv::ChargingRequest_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace autogen_bt_interface

#endif  // AUTOGEN_BT_INTERFACE__SRV__DETAIL__CHARGING_REQUEST__BUILDER_HPP_
