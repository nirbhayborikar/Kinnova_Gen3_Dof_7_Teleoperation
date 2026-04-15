// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from apriltag_msgs:msg/AprilTagDetection.idl
// generated code does not contain a copyright notice
#include "apriltag_msgs/msg/detail/april_tag_detection__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "apriltag_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "apriltag_msgs/msg/detail/april_tag_detection__struct.h"
#include "apriltag_msgs/msg/detail/april_tag_detection__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "apriltag_msgs/msg/detail/point__functions.h"  // centre, corners
#include "rosidl_runtime_c/string.h"  // family
#include "rosidl_runtime_c/string_functions.h"  // family

// forward declare type support functions

bool cdr_serialize_apriltag_msgs__msg__Point(
  const apriltag_msgs__msg__Point * ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool cdr_deserialize_apriltag_msgs__msg__Point(
  eprosima::fastcdr::Cdr & cdr,
  apriltag_msgs__msg__Point * ros_message);

size_t get_serialized_size_apriltag_msgs__msg__Point(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_apriltag_msgs__msg__Point(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool cdr_serialize_key_apriltag_msgs__msg__Point(
  const apriltag_msgs__msg__Point * ros_message,
  eprosima::fastcdr::Cdr & cdr);

size_t get_serialized_size_key_apriltag_msgs__msg__Point(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_key_apriltag_msgs__msg__Point(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, apriltag_msgs, msg, Point)();


using _AprilTagDetection__ros_msg_type = apriltag_msgs__msg__AprilTagDetection;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_apriltag_msgs
bool cdr_serialize_apriltag_msgs__msg__AprilTagDetection(
  const apriltag_msgs__msg__AprilTagDetection * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: family
  {
    const rosidl_runtime_c__String * str = &ros_message->family;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: id
  {
    cdr << ros_message->id;
  }

  // Field name: hamming
  {
    cdr << ros_message->hamming;
  }

  // Field name: goodness
  {
    cdr << ros_message->goodness;
  }

  // Field name: decision_margin
  {
    cdr << ros_message->decision_margin;
  }

  // Field name: centre
  {
    cdr_serialize_apriltag_msgs__msg__Point(
      &ros_message->centre, cdr);
  }

  // Field name: corners
  {
    size_t size = 4;
    auto array_ptr = ros_message->corners;
    for (size_t i = 0; i < size; ++i) {
      cdr_serialize_apriltag_msgs__msg__Point(
        &array_ptr[i], cdr);
    }
  }

  // Field name: homography
  {
    size_t size = 9;
    auto array_ptr = ros_message->homography;
    cdr.serialize_array(array_ptr, size);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_apriltag_msgs
bool cdr_deserialize_apriltag_msgs__msg__AprilTagDetection(
  eprosima::fastcdr::Cdr & cdr,
  apriltag_msgs__msg__AprilTagDetection * ros_message)
{
  // Field name: family
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->family.data) {
      rosidl_runtime_c__String__init(&ros_message->family);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->family,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'family'\n");
      return false;
    }
  }

  // Field name: id
  {
    cdr >> ros_message->id;
  }

  // Field name: hamming
  {
    cdr >> ros_message->hamming;
  }

  // Field name: goodness
  {
    cdr >> ros_message->goodness;
  }

  // Field name: decision_margin
  {
    cdr >> ros_message->decision_margin;
  }

  // Field name: centre
  {
    cdr_deserialize_apriltag_msgs__msg__Point(cdr, &ros_message->centre);
  }

  // Field name: corners
  {
    size_t size = 4;
    auto array_ptr = ros_message->corners;
    for (size_t i = 0; i < size; ++i) {
      cdr_deserialize_apriltag_msgs__msg__Point(cdr, &array_ptr[i]);
    }
  }

  // Field name: homography
  {
    size_t size = 9;
    auto array_ptr = ros_message->homography;
    cdr.deserialize_array(array_ptr, size);
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_apriltag_msgs
size_t get_serialized_size_apriltag_msgs__msg__AprilTagDetection(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _AprilTagDetection__ros_msg_type * ros_message = static_cast<const _AprilTagDetection__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: family
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->family.size + 1);

  // Field name: id
  {
    size_t item_size = sizeof(ros_message->id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: hamming
  {
    size_t item_size = sizeof(ros_message->hamming);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: goodness
  {
    size_t item_size = sizeof(ros_message->goodness);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: decision_margin
  {
    size_t item_size = sizeof(ros_message->decision_margin);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: centre
  current_alignment += get_serialized_size_apriltag_msgs__msg__Point(
    &(ros_message->centre), current_alignment);

  // Field name: corners
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->corners;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_apriltag_msgs__msg__Point(
        &array_ptr[index], current_alignment);
    }
  }

  // Field name: homography
  {
    size_t array_size = 9;
    auto array_ptr = ros_message->homography;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_apriltag_msgs
size_t max_serialized_size_apriltag_msgs__msg__AprilTagDetection(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // Field name: family
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Field name: id
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: hamming
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: goodness
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: decision_margin
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: centre
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_apriltag_msgs__msg__Point(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: corners
  {
    size_t array_size = 4;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_apriltag_msgs__msg__Point(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: homography
  {
    size_t array_size = 9;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = apriltag_msgs__msg__AprilTagDetection;
    is_plain =
      (
      offsetof(DataType, homography) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_apriltag_msgs
bool cdr_serialize_key_apriltag_msgs__msg__AprilTagDetection(
  const apriltag_msgs__msg__AprilTagDetection * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: family
  {
    const rosidl_runtime_c__String * str = &ros_message->family;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: id
  {
    cdr << ros_message->id;
  }

  // Field name: hamming
  {
    cdr << ros_message->hamming;
  }

  // Field name: goodness
  {
    cdr << ros_message->goodness;
  }

  // Field name: decision_margin
  {
    cdr << ros_message->decision_margin;
  }

  // Field name: centre
  {
    cdr_serialize_key_apriltag_msgs__msg__Point(
      &ros_message->centre, cdr);
  }

  // Field name: corners
  {
    size_t size = 4;
    auto array_ptr = ros_message->corners;
    for (size_t i = 0; i < size; ++i) {
      cdr_serialize_key_apriltag_msgs__msg__Point(
        &array_ptr[i], cdr);
    }
  }

  // Field name: homography
  {
    size_t size = 9;
    auto array_ptr = ros_message->homography;
    cdr.serialize_array(array_ptr, size);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_apriltag_msgs
size_t get_serialized_size_key_apriltag_msgs__msg__AprilTagDetection(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _AprilTagDetection__ros_msg_type * ros_message = static_cast<const _AprilTagDetection__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: family
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->family.size + 1);

  // Field name: id
  {
    size_t item_size = sizeof(ros_message->id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: hamming
  {
    size_t item_size = sizeof(ros_message->hamming);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: goodness
  {
    size_t item_size = sizeof(ros_message->goodness);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: decision_margin
  {
    size_t item_size = sizeof(ros_message->decision_margin);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: centre
  current_alignment += get_serialized_size_key_apriltag_msgs__msg__Point(
    &(ros_message->centre), current_alignment);

  // Field name: corners
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->corners;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_key_apriltag_msgs__msg__Point(
        &array_ptr[index], current_alignment);
    }
  }

  // Field name: homography
  {
    size_t array_size = 9;
    auto array_ptr = ros_message->homography;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_apriltag_msgs
size_t max_serialized_size_key_apriltag_msgs__msg__AprilTagDetection(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;
  // Field name: family
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Field name: id
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: hamming
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: goodness
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: decision_margin
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: centre
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_apriltag_msgs__msg__Point(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: corners
  {
    size_t array_size = 4;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_apriltag_msgs__msg__Point(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: homography
  {
    size_t array_size = 9;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = apriltag_msgs__msg__AprilTagDetection;
    is_plain =
      (
      offsetof(DataType, homography) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _AprilTagDetection__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const apriltag_msgs__msg__AprilTagDetection * ros_message = static_cast<const apriltag_msgs__msg__AprilTagDetection *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_apriltag_msgs__msg__AprilTagDetection(ros_message, cdr);
}

static bool _AprilTagDetection__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  apriltag_msgs__msg__AprilTagDetection * ros_message = static_cast<apriltag_msgs__msg__AprilTagDetection *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_apriltag_msgs__msg__AprilTagDetection(cdr, ros_message);
}

static uint32_t _AprilTagDetection__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_apriltag_msgs__msg__AprilTagDetection(
      untyped_ros_message, 0));
}

static size_t _AprilTagDetection__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_apriltag_msgs__msg__AprilTagDetection(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_AprilTagDetection = {
  "apriltag_msgs::msg",
  "AprilTagDetection",
  _AprilTagDetection__cdr_serialize,
  _AprilTagDetection__cdr_deserialize,
  _AprilTagDetection__get_serialized_size,
  _AprilTagDetection__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _AprilTagDetection__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_AprilTagDetection,
  get_message_typesupport_handle_function,
  &apriltag_msgs__msg__AprilTagDetection__get_type_hash,
  &apriltag_msgs__msg__AprilTagDetection__get_type_description,
  &apriltag_msgs__msg__AprilTagDetection__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, apriltag_msgs, msg, AprilTagDetection)() {
  return &_AprilTagDetection__type_support;
}

#if defined(__cplusplus)
}
#endif
