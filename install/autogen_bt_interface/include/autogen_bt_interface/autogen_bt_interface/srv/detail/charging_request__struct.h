// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autogen_bt_interface:srv/ChargingRequest.idl
// generated code does not contain a copyright notice

#ifndef AUTOGEN_BT_INTERFACE__SRV__DETAIL__CHARGING_REQUEST__STRUCT_H_
#define AUTOGEN_BT_INTERFACE__SRV__DETAIL__CHARGING_REQUEST__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'status'
#include "std_msgs/msg/detail/bool__struct.h"

/// Struct defined in srv/ChargingRequest in the package autogen_bt_interface.
typedef struct autogen_bt_interface__srv__ChargingRequest_Request
{
  std_msgs__msg__Bool status;
} autogen_bt_interface__srv__ChargingRequest_Request;

// Struct for a sequence of autogen_bt_interface__srv__ChargingRequest_Request.
typedef struct autogen_bt_interface__srv__ChargingRequest_Request__Sequence
{
  autogen_bt_interface__srv__ChargingRequest_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autogen_bt_interface__srv__ChargingRequest_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/ChargingRequest in the package autogen_bt_interface.
typedef struct autogen_bt_interface__srv__ChargingRequest_Response
{
  uint8_t structure_needs_at_least_one_member;
} autogen_bt_interface__srv__ChargingRequest_Response;

// Struct for a sequence of autogen_bt_interface__srv__ChargingRequest_Response.
typedef struct autogen_bt_interface__srv__ChargingRequest_Response__Sequence
{
  autogen_bt_interface__srv__ChargingRequest_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autogen_bt_interface__srv__ChargingRequest_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTOGEN_BT_INTERFACE__SRV__DETAIL__CHARGING_REQUEST__STRUCT_H_
