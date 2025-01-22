// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from autogen_bt_interface:srv/ChargingRequest.idl
// generated code does not contain a copyright notice
#include "autogen_bt_interface/srv/detail/charging_request__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `status`
#include "std_msgs/msg/detail/bool__functions.h"

bool
autogen_bt_interface__srv__ChargingRequest_Request__init(autogen_bt_interface__srv__ChargingRequest_Request * msg)
{
  if (!msg) {
    return false;
  }
  // status
  if (!std_msgs__msg__Bool__init(&msg->status)) {
    autogen_bt_interface__srv__ChargingRequest_Request__fini(msg);
    return false;
  }
  return true;
}

void
autogen_bt_interface__srv__ChargingRequest_Request__fini(autogen_bt_interface__srv__ChargingRequest_Request * msg)
{
  if (!msg) {
    return;
  }
  // status
  std_msgs__msg__Bool__fini(&msg->status);
}

bool
autogen_bt_interface__srv__ChargingRequest_Request__are_equal(const autogen_bt_interface__srv__ChargingRequest_Request * lhs, const autogen_bt_interface__srv__ChargingRequest_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (!std_msgs__msg__Bool__are_equal(
      &(lhs->status), &(rhs->status)))
  {
    return false;
  }
  return true;
}

bool
autogen_bt_interface__srv__ChargingRequest_Request__copy(
  const autogen_bt_interface__srv__ChargingRequest_Request * input,
  autogen_bt_interface__srv__ChargingRequest_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  if (!std_msgs__msg__Bool__copy(
      &(input->status), &(output->status)))
  {
    return false;
  }
  return true;
}

autogen_bt_interface__srv__ChargingRequest_Request *
autogen_bt_interface__srv__ChargingRequest_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autogen_bt_interface__srv__ChargingRequest_Request * msg = (autogen_bt_interface__srv__ChargingRequest_Request *)allocator.allocate(sizeof(autogen_bt_interface__srv__ChargingRequest_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autogen_bt_interface__srv__ChargingRequest_Request));
  bool success = autogen_bt_interface__srv__ChargingRequest_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autogen_bt_interface__srv__ChargingRequest_Request__destroy(autogen_bt_interface__srv__ChargingRequest_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autogen_bt_interface__srv__ChargingRequest_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autogen_bt_interface__srv__ChargingRequest_Request__Sequence__init(autogen_bt_interface__srv__ChargingRequest_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autogen_bt_interface__srv__ChargingRequest_Request * data = NULL;

  if (size) {
    data = (autogen_bt_interface__srv__ChargingRequest_Request *)allocator.zero_allocate(size, sizeof(autogen_bt_interface__srv__ChargingRequest_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autogen_bt_interface__srv__ChargingRequest_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autogen_bt_interface__srv__ChargingRequest_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
autogen_bt_interface__srv__ChargingRequest_Request__Sequence__fini(autogen_bt_interface__srv__ChargingRequest_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      autogen_bt_interface__srv__ChargingRequest_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

autogen_bt_interface__srv__ChargingRequest_Request__Sequence *
autogen_bt_interface__srv__ChargingRequest_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autogen_bt_interface__srv__ChargingRequest_Request__Sequence * array = (autogen_bt_interface__srv__ChargingRequest_Request__Sequence *)allocator.allocate(sizeof(autogen_bt_interface__srv__ChargingRequest_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autogen_bt_interface__srv__ChargingRequest_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autogen_bt_interface__srv__ChargingRequest_Request__Sequence__destroy(autogen_bt_interface__srv__ChargingRequest_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autogen_bt_interface__srv__ChargingRequest_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autogen_bt_interface__srv__ChargingRequest_Request__Sequence__are_equal(const autogen_bt_interface__srv__ChargingRequest_Request__Sequence * lhs, const autogen_bt_interface__srv__ChargingRequest_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autogen_bt_interface__srv__ChargingRequest_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autogen_bt_interface__srv__ChargingRequest_Request__Sequence__copy(
  const autogen_bt_interface__srv__ChargingRequest_Request__Sequence * input,
  autogen_bt_interface__srv__ChargingRequest_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autogen_bt_interface__srv__ChargingRequest_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autogen_bt_interface__srv__ChargingRequest_Request * data =
      (autogen_bt_interface__srv__ChargingRequest_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autogen_bt_interface__srv__ChargingRequest_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autogen_bt_interface__srv__ChargingRequest_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autogen_bt_interface__srv__ChargingRequest_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
autogen_bt_interface__srv__ChargingRequest_Response__init(autogen_bt_interface__srv__ChargingRequest_Response * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
autogen_bt_interface__srv__ChargingRequest_Response__fini(autogen_bt_interface__srv__ChargingRequest_Response * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
autogen_bt_interface__srv__ChargingRequest_Response__are_equal(const autogen_bt_interface__srv__ChargingRequest_Response * lhs, const autogen_bt_interface__srv__ChargingRequest_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
autogen_bt_interface__srv__ChargingRequest_Response__copy(
  const autogen_bt_interface__srv__ChargingRequest_Response * input,
  autogen_bt_interface__srv__ChargingRequest_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

autogen_bt_interface__srv__ChargingRequest_Response *
autogen_bt_interface__srv__ChargingRequest_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autogen_bt_interface__srv__ChargingRequest_Response * msg = (autogen_bt_interface__srv__ChargingRequest_Response *)allocator.allocate(sizeof(autogen_bt_interface__srv__ChargingRequest_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autogen_bt_interface__srv__ChargingRequest_Response));
  bool success = autogen_bt_interface__srv__ChargingRequest_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autogen_bt_interface__srv__ChargingRequest_Response__destroy(autogen_bt_interface__srv__ChargingRequest_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autogen_bt_interface__srv__ChargingRequest_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autogen_bt_interface__srv__ChargingRequest_Response__Sequence__init(autogen_bt_interface__srv__ChargingRequest_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autogen_bt_interface__srv__ChargingRequest_Response * data = NULL;

  if (size) {
    data = (autogen_bt_interface__srv__ChargingRequest_Response *)allocator.zero_allocate(size, sizeof(autogen_bt_interface__srv__ChargingRequest_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autogen_bt_interface__srv__ChargingRequest_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autogen_bt_interface__srv__ChargingRequest_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
autogen_bt_interface__srv__ChargingRequest_Response__Sequence__fini(autogen_bt_interface__srv__ChargingRequest_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      autogen_bt_interface__srv__ChargingRequest_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

autogen_bt_interface__srv__ChargingRequest_Response__Sequence *
autogen_bt_interface__srv__ChargingRequest_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autogen_bt_interface__srv__ChargingRequest_Response__Sequence * array = (autogen_bt_interface__srv__ChargingRequest_Response__Sequence *)allocator.allocate(sizeof(autogen_bt_interface__srv__ChargingRequest_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autogen_bt_interface__srv__ChargingRequest_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autogen_bt_interface__srv__ChargingRequest_Response__Sequence__destroy(autogen_bt_interface__srv__ChargingRequest_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autogen_bt_interface__srv__ChargingRequest_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autogen_bt_interface__srv__ChargingRequest_Response__Sequence__are_equal(const autogen_bt_interface__srv__ChargingRequest_Response__Sequence * lhs, const autogen_bt_interface__srv__ChargingRequest_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autogen_bt_interface__srv__ChargingRequest_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autogen_bt_interface__srv__ChargingRequest_Response__Sequence__copy(
  const autogen_bt_interface__srv__ChargingRequest_Response__Sequence * input,
  autogen_bt_interface__srv__ChargingRequest_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autogen_bt_interface__srv__ChargingRequest_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autogen_bt_interface__srv__ChargingRequest_Response * data =
      (autogen_bt_interface__srv__ChargingRequest_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autogen_bt_interface__srv__ChargingRequest_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autogen_bt_interface__srv__ChargingRequest_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autogen_bt_interface__srv__ChargingRequest_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
