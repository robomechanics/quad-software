// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from quad_msgs:msg/FootPlanDiscrete.idl
// generated code does not contain a copyright notice
#include "quad_msgs/msg/detail/foot_plan_discrete__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `footholds`
#include "quad_msgs/msg/detail/foot_state__functions.h"

bool
quad_msgs__msg__FootPlanDiscrete__init(quad_msgs__msg__FootPlanDiscrete * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    quad_msgs__msg__FootPlanDiscrete__fini(msg);
    return false;
  }
  // footholds
  if (!quad_msgs__msg__FootState__Sequence__init(&msg->footholds, 0)) {
    quad_msgs__msg__FootPlanDiscrete__fini(msg);
    return false;
  }
  return true;
}

void
quad_msgs__msg__FootPlanDiscrete__fini(quad_msgs__msg__FootPlanDiscrete * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // footholds
  quad_msgs__msg__FootState__Sequence__fini(&msg->footholds);
}

bool
quad_msgs__msg__FootPlanDiscrete__are_equal(const quad_msgs__msg__FootPlanDiscrete * lhs, const quad_msgs__msg__FootPlanDiscrete * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // footholds
  if (!quad_msgs__msg__FootState__Sequence__are_equal(
      &(lhs->footholds), &(rhs->footholds)))
  {
    return false;
  }
  return true;
}

bool
quad_msgs__msg__FootPlanDiscrete__copy(
  const quad_msgs__msg__FootPlanDiscrete * input,
  quad_msgs__msg__FootPlanDiscrete * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // footholds
  if (!quad_msgs__msg__FootState__Sequence__copy(
      &(input->footholds), &(output->footholds)))
  {
    return false;
  }
  return true;
}

quad_msgs__msg__FootPlanDiscrete *
quad_msgs__msg__FootPlanDiscrete__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  quad_msgs__msg__FootPlanDiscrete * msg = (quad_msgs__msg__FootPlanDiscrete *)allocator.allocate(sizeof(quad_msgs__msg__FootPlanDiscrete), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(quad_msgs__msg__FootPlanDiscrete));
  bool success = quad_msgs__msg__FootPlanDiscrete__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
quad_msgs__msg__FootPlanDiscrete__destroy(quad_msgs__msg__FootPlanDiscrete * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    quad_msgs__msg__FootPlanDiscrete__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
quad_msgs__msg__FootPlanDiscrete__Sequence__init(quad_msgs__msg__FootPlanDiscrete__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  quad_msgs__msg__FootPlanDiscrete * data = NULL;

  if (size) {
    data = (quad_msgs__msg__FootPlanDiscrete *)allocator.zero_allocate(size, sizeof(quad_msgs__msg__FootPlanDiscrete), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = quad_msgs__msg__FootPlanDiscrete__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        quad_msgs__msg__FootPlanDiscrete__fini(&data[i - 1]);
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
quad_msgs__msg__FootPlanDiscrete__Sequence__fini(quad_msgs__msg__FootPlanDiscrete__Sequence * array)
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
      quad_msgs__msg__FootPlanDiscrete__fini(&array->data[i]);
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

quad_msgs__msg__FootPlanDiscrete__Sequence *
quad_msgs__msg__FootPlanDiscrete__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  quad_msgs__msg__FootPlanDiscrete__Sequence * array = (quad_msgs__msg__FootPlanDiscrete__Sequence *)allocator.allocate(sizeof(quad_msgs__msg__FootPlanDiscrete__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = quad_msgs__msg__FootPlanDiscrete__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
quad_msgs__msg__FootPlanDiscrete__Sequence__destroy(quad_msgs__msg__FootPlanDiscrete__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    quad_msgs__msg__FootPlanDiscrete__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
quad_msgs__msg__FootPlanDiscrete__Sequence__are_equal(const quad_msgs__msg__FootPlanDiscrete__Sequence * lhs, const quad_msgs__msg__FootPlanDiscrete__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!quad_msgs__msg__FootPlanDiscrete__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
quad_msgs__msg__FootPlanDiscrete__Sequence__copy(
  const quad_msgs__msg__FootPlanDiscrete__Sequence * input,
  quad_msgs__msg__FootPlanDiscrete__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(quad_msgs__msg__FootPlanDiscrete);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    quad_msgs__msg__FootPlanDiscrete * data =
      (quad_msgs__msg__FootPlanDiscrete *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!quad_msgs__msg__FootPlanDiscrete__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          quad_msgs__msg__FootPlanDiscrete__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!quad_msgs__msg__FootPlanDiscrete__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
