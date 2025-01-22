// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autogen_bt_interface:srv/PickingRequest.idl
// generated code does not contain a copyright notice

#ifndef AUTOGEN_BT_INTERFACE__SRV__DETAIL__PICKING_REQUEST__BUILDER_HPP_
#define AUTOGEN_BT_INTERFACE__SRV__DETAIL__PICKING_REQUEST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autogen_bt_interface/srv/detail/picking_request__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autogen_bt_interface
{

namespace srv
{

namespace builder
{

class Init_PickingRequest_Request_status
{
public:
  explicit Init_PickingRequest_Request_status(::autogen_bt_interface::srv::PickingRequest_Request & msg)
  : msg_(msg)
  {}
  ::autogen_bt_interface::srv::PickingRequest_Request status(::autogen_bt_interface::srv::PickingRequest_Request::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autogen_bt_interface::srv::PickingRequest_Request msg_;
};

class Init_PickingRequest_Request_obj_id
{
public:
  explicit Init_PickingRequest_Request_obj_id(::autogen_bt_interface::srv::PickingRequest_Request & msg)
  : msg_(msg)
  {}
  Init_PickingRequest_Request_status obj_id(::autogen_bt_interface::srv::PickingRequest_Request::_obj_id_type arg)
  {
    msg_.obj_id = std::move(arg);
    return Init_PickingRequest_Request_status(msg_);
  }

private:
  ::autogen_bt_interface::srv::PickingRequest_Request msg_;
};

class Init_PickingRequest_Request_host_name
{
public:
  Init_PickingRequest_Request_host_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PickingRequest_Request_obj_id host_name(::autogen_bt_interface::srv::PickingRequest_Request::_host_name_type arg)
  {
    msg_.host_name = std::move(arg);
    return Init_PickingRequest_Request_obj_id(msg_);
  }

private:
  ::autogen_bt_interface::srv::PickingRequest_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autogen_bt_interface::srv::PickingRequest_Request>()
{
  return autogen_bt_interface::srv::builder::Init_PickingRequest_Request_host_name();
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
auto build<::autogen_bt_interface::srv::PickingRequest_Response>()
{
  return ::autogen_bt_interface::srv::PickingRequest_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace autogen_bt_interface

#endif  // AUTOGEN_BT_INTERFACE__SRV__DETAIL__PICKING_REQUEST__BUILDER_HPP_
