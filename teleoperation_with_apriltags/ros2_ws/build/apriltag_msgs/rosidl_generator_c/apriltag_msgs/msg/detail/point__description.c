// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from apriltag_msgs:msg/Point.idl
// generated code does not contain a copyright notice

#include "apriltag_msgs/msg/detail/point__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_apriltag_msgs
const rosidl_type_hash_t *
apriltag_msgs__msg__Point__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xc4, 0x28, 0x99, 0x76, 0x7c, 0xe6, 0x20, 0xe6,
      0xbe, 0xb2, 0x6a, 0xfe, 0xaf, 0x1e, 0xc6, 0x58,
      0xe6, 0xe3, 0xdb, 0xfd, 0x51, 0x20, 0x36, 0x83,
      0x30, 0x56, 0x63, 0x23, 0xdd, 0xdf, 0x62, 0xf4,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char apriltag_msgs__msg__Point__TYPE_NAME[] = "apriltag_msgs/msg/Point";

// Define type names, field names, and default values
static char apriltag_msgs__msg__Point__FIELD_NAME__x[] = "x";
static char apriltag_msgs__msg__Point__FIELD_NAME__y[] = "y";

static rosidl_runtime_c__type_description__Field apriltag_msgs__msg__Point__FIELDS[] = {
  {
    {apriltag_msgs__msg__Point__FIELD_NAME__x, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {apriltag_msgs__msg__Point__FIELD_NAME__y, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
apriltag_msgs__msg__Point__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {apriltag_msgs__msg__Point__TYPE_NAME, 23, 23},
      {apriltag_msgs__msg__Point__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float64 x\n"
  "float64 y";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
apriltag_msgs__msg__Point__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {apriltag_msgs__msg__Point__TYPE_NAME, 23, 23},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 20, 20},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
apriltag_msgs__msg__Point__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *apriltag_msgs__msg__Point__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
