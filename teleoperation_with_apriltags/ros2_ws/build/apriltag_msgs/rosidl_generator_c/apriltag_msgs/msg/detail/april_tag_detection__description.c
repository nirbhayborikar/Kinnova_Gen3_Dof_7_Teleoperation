// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from apriltag_msgs:msg/AprilTagDetection.idl
// generated code does not contain a copyright notice

#include "apriltag_msgs/msg/detail/april_tag_detection__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_apriltag_msgs
const rosidl_type_hash_t *
apriltag_msgs__msg__AprilTagDetection__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x77, 0xc6, 0xbb, 0x37, 0xe0, 0xa9, 0xfe, 0x5f,
      0x15, 0x96, 0x66, 0xb1, 0x4f, 0x33, 0x40, 0xf9,
      0x89, 0x74, 0x9d, 0x43, 0x6a, 0x2e, 0xe0, 0x1b,
      0x5c, 0x53, 0xbd, 0xd7, 0xd7, 0x08, 0x5d, 0xbb,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "apriltag_msgs/msg/detail/point__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t apriltag_msgs__msg__Point__EXPECTED_HASH = {1, {
    0xc4, 0x28, 0x99, 0x76, 0x7c, 0xe6, 0x20, 0xe6,
    0xbe, 0xb2, 0x6a, 0xfe, 0xaf, 0x1e, 0xc6, 0x58,
    0xe6, 0xe3, 0xdb, 0xfd, 0x51, 0x20, 0x36, 0x83,
    0x30, 0x56, 0x63, 0x23, 0xdd, 0xdf, 0x62, 0xf4,
  }};
#endif

static char apriltag_msgs__msg__AprilTagDetection__TYPE_NAME[] = "apriltag_msgs/msg/AprilTagDetection";
static char apriltag_msgs__msg__Point__TYPE_NAME[] = "apriltag_msgs/msg/Point";

// Define type names, field names, and default values
static char apriltag_msgs__msg__AprilTagDetection__FIELD_NAME__family[] = "family";
static char apriltag_msgs__msg__AprilTagDetection__FIELD_NAME__id[] = "id";
static char apriltag_msgs__msg__AprilTagDetection__FIELD_NAME__hamming[] = "hamming";
static char apriltag_msgs__msg__AprilTagDetection__FIELD_NAME__goodness[] = "goodness";
static char apriltag_msgs__msg__AprilTagDetection__FIELD_NAME__decision_margin[] = "decision_margin";
static char apriltag_msgs__msg__AprilTagDetection__FIELD_NAME__centre[] = "centre";
static char apriltag_msgs__msg__AprilTagDetection__FIELD_NAME__corners[] = "corners";
static char apriltag_msgs__msg__AprilTagDetection__FIELD_NAME__homography[] = "homography";

static rosidl_runtime_c__type_description__Field apriltag_msgs__msg__AprilTagDetection__FIELDS[] = {
  {
    {apriltag_msgs__msg__AprilTagDetection__FIELD_NAME__family, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {apriltag_msgs__msg__AprilTagDetection__FIELD_NAME__id, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {apriltag_msgs__msg__AprilTagDetection__FIELD_NAME__hamming, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {apriltag_msgs__msg__AprilTagDetection__FIELD_NAME__goodness, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {apriltag_msgs__msg__AprilTagDetection__FIELD_NAME__decision_margin, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {apriltag_msgs__msg__AprilTagDetection__FIELD_NAME__centre, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {apriltag_msgs__msg__Point__TYPE_NAME, 23, 23},
    },
    {NULL, 0, 0},
  },
  {
    {apriltag_msgs__msg__AprilTagDetection__FIELD_NAME__corners, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_ARRAY,
      4,
      0,
      {apriltag_msgs__msg__Point__TYPE_NAME, 23, 23},
    },
    {NULL, 0, 0},
  },
  {
    {apriltag_msgs__msg__AprilTagDetection__FIELD_NAME__homography, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE_ARRAY,
      9,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription apriltag_msgs__msg__AprilTagDetection__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {apriltag_msgs__msg__Point__TYPE_NAME, 23, 23},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
apriltag_msgs__msg__AprilTagDetection__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {apriltag_msgs__msg__AprilTagDetection__TYPE_NAME, 35, 35},
      {apriltag_msgs__msg__AprilTagDetection__FIELDS, 8, 8},
    },
    {apriltag_msgs__msg__AprilTagDetection__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&apriltag_msgs__msg__Point__EXPECTED_HASH, apriltag_msgs__msg__Point__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = apriltag_msgs__msg__Point__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "string family\n"
  "int32 id\n"
  "int32 hamming\n"
  "float32 goodness\n"
  "float32 decision_margin\n"
  "Point centre                    # centre in (x,y) pixel coordinates\n"
  "Point[4] corners                # corners of tag ((x1,y1),(x2,y2),...)\n"
  "float64[9] homography           # 3x3 row-major homography matrix";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
apriltag_msgs__msg__AprilTagDetection__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {apriltag_msgs__msg__AprilTagDetection__TYPE_NAME, 35, 35},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 283, 283},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
apriltag_msgs__msg__AprilTagDetection__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *apriltag_msgs__msg__AprilTagDetection__get_individual_type_description_source(NULL),
    sources[1] = *apriltag_msgs__msg__Point__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
