// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from autogen_bt_interface:srv/PickingRequest.idl
// generated code does not contain a copyright notice

#ifndef AUTOGEN_BT_INTERFACE__SRV__DETAIL__PICKING_REQUEST__STRUCT_HPP_
#define AUTOGEN_BT_INTERFACE__SRV__DETAIL__PICKING_REQUEST__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'host_name'
#include "std_msgs/msg/detail/string__struct.hpp"
// Member 'obj_id'
#include "std_msgs/msg/detail/u_int32__struct.hpp"
// Member 'status'
#include "std_msgs/msg/detail/bool__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__autogen_bt_interface__srv__PickingRequest_Request __attribute__((deprecated))
#else
# define DEPRECATED__autogen_bt_interface__srv__PickingRequest_Request __declspec(deprecated)
#endif

namespace autogen_bt_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct PickingRequest_Request_
{
  using Type = PickingRequest_Request_<ContainerAllocator>;

  explicit PickingRequest_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : host_name(_init),
    obj_id(_init),
    status(_init)
  {
    (void)_init;
  }

  explicit PickingRequest_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : host_name(_alloc, _init),
    obj_id(_alloc, _init),
    status(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _host_name_type =
    std_msgs::msg::String_<ContainerAllocator>;
  _host_name_type host_name;
  using _obj_id_type =
    std_msgs::msg::UInt32_<ContainerAllocator>;
  _obj_id_type obj_id;
  using _status_type =
    std_msgs::msg::Bool_<ContainerAllocator>;
  _status_type status;

  // setters for named parameter idiom
  Type & set__host_name(
    const std_msgs::msg::String_<ContainerAllocator> & _arg)
  {
    this->host_name = _arg;
    return *this;
  }
  Type & set__obj_id(
    const std_msgs::msg::UInt32_<ContainerAllocator> & _arg)
  {
    this->obj_id = _arg;
    return *this;
  }
  Type & set__status(
    const std_msgs::msg::Bool_<ContainerAllocator> & _arg)
  {
    this->status = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autogen_bt_interface::srv::PickingRequest_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const autogen_bt_interface::srv::PickingRequest_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autogen_bt_interface::srv::PickingRequest_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autogen_bt_interface::srv::PickingRequest_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autogen_bt_interface::srv::PickingRequest_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autogen_bt_interface::srv::PickingRequest_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autogen_bt_interface::srv::PickingRequest_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autogen_bt_interface::srv::PickingRequest_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autogen_bt_interface::srv::PickingRequest_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autogen_bt_interface::srv::PickingRequest_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autogen_bt_interface__srv__PickingRequest_Request
    std::shared_ptr<autogen_bt_interface::srv::PickingRequest_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autogen_bt_interface__srv__PickingRequest_Request
    std::shared_ptr<autogen_bt_interface::srv::PickingRequest_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PickingRequest_Request_ & other) const
  {
    if (this->host_name != other.host_name) {
      return false;
    }
    if (this->obj_id != other.obj_id) {
      return false;
    }
    if (this->status != other.status) {
      return false;
    }
    return true;
  }
  bool operator!=(const PickingRequest_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PickingRequest_Request_

// alias to use template instance with default allocator
using PickingRequest_Request =
  autogen_bt_interface::srv::PickingRequest_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace autogen_bt_interface


#ifndef _WIN32
# define DEPRECATED__autogen_bt_interface__srv__PickingRequest_Response __attribute__((deprecated))
#else
# define DEPRECATED__autogen_bt_interface__srv__PickingRequest_Response __declspec(deprecated)
#endif

namespace autogen_bt_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct PickingRequest_Response_
{
  using Type = PickingRequest_Response_<ContainerAllocator>;

  explicit PickingRequest_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit PickingRequest_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    autogen_bt_interface::srv::PickingRequest_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const autogen_bt_interface::srv::PickingRequest_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autogen_bt_interface::srv::PickingRequest_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autogen_bt_interface::srv::PickingRequest_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autogen_bt_interface::srv::PickingRequest_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autogen_bt_interface::srv::PickingRequest_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autogen_bt_interface::srv::PickingRequest_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autogen_bt_interface::srv::PickingRequest_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autogen_bt_interface::srv::PickingRequest_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autogen_bt_interface::srv::PickingRequest_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autogen_bt_interface__srv__PickingRequest_Response
    std::shared_ptr<autogen_bt_interface::srv::PickingRequest_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autogen_bt_interface__srv__PickingRequest_Response
    std::shared_ptr<autogen_bt_interface::srv::PickingRequest_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PickingRequest_Response_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const PickingRequest_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PickingRequest_Response_

// alias to use template instance with default allocator
using PickingRequest_Response =
  autogen_bt_interface::srv::PickingRequest_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace autogen_bt_interface

namespace autogen_bt_interface
{

namespace srv
{

struct PickingRequest
{
  using Request = autogen_bt_interface::srv::PickingRequest_Request;
  using Response = autogen_bt_interface::srv::PickingRequest_Response;
};

}  // namespace srv

}  // namespace autogen_bt_interface

#endif  // AUTOGEN_BT_INTERFACE__SRV__DETAIL__PICKING_REQUEST__STRUCT_HPP_
