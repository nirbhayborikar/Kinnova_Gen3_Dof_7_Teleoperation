// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from apriltag_msgs:msg/AprilTagDetectionArray.idl
// generated code does not contain a copyright notice

#include "apriltag_msgs/msg/detail/april_tag_detection_array__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_apriltag_msgs
const rosidl_type_hash_t *
apriltag_msgs__msg__AprilTagDetectionArray__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x60, 0x16, 0xbf, 0xe5, 0x13, 0xa7, 0xc7, 0x41,
      0xcc, 0xbd, 0xbc, 0x64, 0x19, 0xed, 0x6c, 0x4c,
      0xab, 0xfe, 0x5f, 0x11, 0x27, 0x1a, 0x96, 0x55,
      0x5f, 0xbf, 0x26, 0x56, 0x5c, 0x6b, 0x98, 0x26,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "apriltag_msgs/msg/detail/point__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "apriltag_msgs/msg/detail/april_tag_detection__functions.h"
#include "std_msgs/msg/detail/header__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t apriltag_msgs__msg__AprilTagDetection__EXPECTED_HASH = {1, {
    0x77, 0xc6, 0xbb, 0x37, 0xe0, 0xa9, 0xfe, 0x5f,
    0x15, 0x96, 0x66, 0xb1, 0x4f, 0x33, 0x40, 0xf9,
    0x89, 0x74, 0x9d, 0x43, 0x6a, 0x2e, 0xe0, 0x1b,
    0x5c, 0x53, 0xbd, 0xd7, 0xd7, 0x08, 0x5d, 0xbb,
  }};
static const rosidl_type_hash_t apriltag_msgs__msg__Point__EXPECTED_HASH = {1, {
    0xc4, 0x28, 0x99, 0x76, 0x7c, 0xe6, 0x20, 0xe6,
    0xbe, 0xb2, 0x6a, 0xfe, 0xaf, 0x1e, 0xc6, 0x58,
    0xe6, 0xe3, 0xdb, 0xfd, 0x51, 0x20, 0x36, 0x83,
    0x30, 0x56, 0x63, 0x23, 0xdd, 0xdf, 0x62, 0xf4,
  }};
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
#endif

static char apriltag_msgs__msg__AprilTagDetectionArray__TYPE_NAME[] = "apriltag_msgs/msg/AprilTagDetectionArray";
static char apriltag_msgs__msg__AprilTagDetection__TYPE_NAME[] = "apriltag_msgs/msg/AprilTagDetection";
static char apriltag_msgs__msg__Point__TYPE_NAME[] = "apriltag_msgs/msg/Point";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char apriltag_msgs__msg__AprilTagDetectionArray__FIELD_NAME__header[] = "header";
static char apriltag_msgs__msg__AprilTagDetectionArray__FIELD_NAME__detections[] = "detections";

static rosidl_runtime_c__type_description__Field apriltag_msgs__msg__AprilTagDetectionArray__FIELDS[] = {
  {
    {apriltag_msgs__msg__AprilTagDetectionArray__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {apriltag_msgs__msg__AprilTagDetectionArray__FIELD_NAME__detections, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {apriltag_msgs__msg__AprilTagDetection__TYPE_NAME, 35, 35},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription apriltag_msgs__msg__AprilTagDetectionArray__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {apriltag_msgs__msg__AprilTagDetection__TYPE_NAME, 35, 35},
    {NULL, 0, 0},
  },
  {
    {apriltag_msgs__msg__Point__TYPE_NAME, 23, 23},
    {NULL, 0, 0},
  },
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
apriltag_msgs__msg__AprilTagDetectionArray__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {apriltag_msgs__msg__AprilTagDetectionArray__TYPE_NAME, 40, 40},
      {apriltag_msgs__msg__AprilTagDetectionArray__FIELDS, 2, 2},
    },
    {apriltag_msgs__msg__AprilTagDetectionArray__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&apriltag_msgs__msg__AprilTagDetection__EXPECTED_HASH, apriltag_msgs__msg__AprilTagDetection__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = apriltag_msgs__msg__AprilTagDetection__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&apriltag_msgs__msg__Point__EXPECTED_HASH, apriltag_msgs__msg__Point__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = apriltag_msgs__msg__Point__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "std_msgs/Header header\n"
  "AprilTagDetection[] detections";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
apriltag_msgs__msg__AprilTagDetectionArray__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {apriltag_msgs__msg__AprilTagDetectionArray__TYPE_NAME, 40, 40},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 54, 54},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
apriltag_msgs__msg__AprilTagDetectionArray__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *apriltag_msgs__msg__AprilTagDetectionArray__get_individual_type_description_source(NULL),
    sources[1] = *apriltag_msgs__msg__AprilTagDetection__get_individual_type_description_source(NULL);
    sources[2] = *apriltag_msgs__msg__Point__get_individual_type_description_source(NULL);
    sources[3] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[4] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
