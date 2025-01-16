// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from quad_msgs:msg/ContactMode.idl
// generated code does not contain a copyright notice

#include "quad_msgs/msg/detail/contact_mode__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_quad_msgs
const rosidl_type_hash_t *
quad_msgs__msg__ContactMode__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xc6, 0x1d, 0x49, 0xdf, 0x05, 0xb0, 0x01, 0xfd,
      0xb6, 0xe2, 0xa4, 0xd7, 0xbe, 0x6c, 0xc3, 0x9b,
      0x56, 0x13, 0x55, 0xb1, 0x88, 0xc0, 0x58, 0xcf,
      0x44, 0x39, 0x84, 0xce, 0x6b, 0x82, 0xbb, 0xbe,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "quad_msgs/msg/detail/leg_contact_mode__functions.h"
#include "std_msgs/msg/detail/header__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "geometry_msgs/msg/detail/vector3__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t geometry_msgs__msg__Vector3__EXPECTED_HASH = {1, {
    0xcc, 0x12, 0xfe, 0x83, 0xe4, 0xc0, 0x27, 0x19,
    0xf1, 0xce, 0x80, 0x70, 0xbf, 0xd1, 0x4a, 0xec,
    0xd4, 0x0f, 0x75, 0xa9, 0x66, 0x96, 0xa6, 0x7a,
    0x2a, 0x1f, 0x37, 0xf7, 0xdb, 0xb0, 0x76, 0x5d,
  }};
static const rosidl_type_hash_t quad_msgs__msg__LegContactMode__EXPECTED_HASH = {1, {
    0xf8, 0x97, 0xa2, 0x15, 0x9a, 0x2d, 0x3b, 0xae,
    0x6f, 0x27, 0x8c, 0xd2, 0x11, 0x6f, 0x65, 0xd1,
    0x1b, 0x64, 0x2f, 0x9a, 0x2d, 0x72, 0x68, 0x35,
    0xb1, 0xfc, 0x00, 0x6c, 0x9a, 0x41, 0x0a, 0xd1,
  }};
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
#endif

static char quad_msgs__msg__ContactMode__TYPE_NAME[] = "quad_msgs/msg/ContactMode";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char geometry_msgs__msg__Vector3__TYPE_NAME[] = "geometry_msgs/msg/Vector3";
static char quad_msgs__msg__LegContactMode__TYPE_NAME[] = "quad_msgs/msg/LegContactMode";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char quad_msgs__msg__ContactMode__FIELD_NAME__header[] = "header";
static char quad_msgs__msg__ContactMode__FIELD_NAME__leg_contacts[] = "leg_contacts";

static rosidl_runtime_c__type_description__Field quad_msgs__msg__ContactMode__FIELDS[] = {
  {
    {quad_msgs__msg__ContactMode__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {quad_msgs__msg__ContactMode__FIELD_NAME__leg_contacts, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {quad_msgs__msg__LegContactMode__TYPE_NAME, 28, 28},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription quad_msgs__msg__ContactMode__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Vector3__TYPE_NAME, 25, 25},
    {NULL, 0, 0},
  },
  {
    {quad_msgs__msg__LegContactMode__TYPE_NAME, 28, 28},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
quad_msgs__msg__ContactMode__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {quad_msgs__msg__ContactMode__TYPE_NAME, 25, 25},
      {quad_msgs__msg__ContactMode__FIELDS, 2, 2},
    },
    {quad_msgs__msg__ContactMode__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Vector3__EXPECTED_HASH, geometry_msgs__msg__Vector3__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = geometry_msgs__msg__Vector3__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&quad_msgs__msg__LegContactMode__EXPECTED_HASH, quad_msgs__msg__LegContactMode__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = quad_msgs__msg__LegContactMode__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# This is a message to hold contact states of the robot\n"
  "std_msgs/Header header\n"
  "\n"
  "LegContactMode[] leg_contacts # FL Bl FR BR";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
quad_msgs__msg__ContactMode__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {quad_msgs__msg__ContactMode__TYPE_NAME, 25, 25},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 124, 124},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
quad_msgs__msg__ContactMode__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *quad_msgs__msg__ContactMode__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *geometry_msgs__msg__Vector3__get_individual_type_description_source(NULL);
    sources[3] = *quad_msgs__msg__LegContactMode__get_individual_type_description_source(NULL);
    sources[4] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
