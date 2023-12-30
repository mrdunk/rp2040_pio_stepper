#ifndef UPDATE_TYPES__H
#define UPDATE_TYPES__H

#include <sys/types.h>

/* This file contains objects that are passed over Ethernet UDP between host and RP2040.
 * Multiple structs can be packed in an array.
 * The array should be null terminated; The last entry in the array should be a
 * uint32_t of zero value. */


#define MSG_NONE                     0
#define MSG_TIMING                   0x88888888  // Contains packet count and time sent.
#define MSG_SET_AXIS_ENABLED         3  // Set absolute axis position.
#define MSG_SET_AXIS_ABS_POS         4  // Set absolute axis position.
#define MSG_SET_AXIS_VELOCITY        5  // Set relative axis position. (Velocity)
#define MSG_SET_AXIS_MAX_VELOCITY    6  // Not yet implemented.
#define MSG_SET_AXIS_MAX_ACCEL       7  // Not yet implemented.
#define MSG_SET_AXIS_IO_STEP         8  // Set GPIO step pin.
#define MSG_SET_AXIS_IO_DIR          9  // Set GPIO direction pin.
#define MSG_SET_AXIS_CONFIG          10 // Set all config for a joint.

struct Message_header {
  uint32_t type;
};

struct Message_timing {
  uint32_t type;
  uint32_t update_id;
  uint32_t time;
};

struct Message_set_max_velocity {
  uint32_t type;
  uint32_t axis;
  double value;
};

struct Message_set_max_accel {
  uint32_t type;
  uint32_t axis;
  double value;
};

struct Message_set_abs_pos {
  uint32_t type;
  uint32_t axis;
  double value;
};

struct Message_set_velocity {
  uint32_t type;
  uint32_t axis;
  double value;
};

struct Message_joint_enable {
  uint32_t type;
  uint32_t axis;
  uint8_t value;
};

struct Message_joint_gpio {
  uint32_t type;
  uint32_t axis;
  int8_t value;
};

struct Message_joint_config {
  uint32_t type;
  uint32_t axis;
  uint8_t enable;
  int8_t gpio_step;
  int8_t gpio_dir;
  double max_velocity;
  double max_accel;
};

union MessageAny {
  struct Message_header header;
  struct Message_timing timing;
  struct Message_set_max_velocity set_max_velocity;
  struct Message_set_max_accel set_max_accel;
  struct Message_set_abs_pos set_abs_pos;
  struct Message_set_velocity set_velocity;
  struct Message_joint_enable joint_enable;
  struct Message_joint_gpio joint_gpio;
  struct Message_joint_config joint_config;
};


#define REPLY_NONE                   0
#define REPLY_TIMING                 1
#define REPLY_AXIS_MOVEMENT          2
#define REPLY_AXIS_CONFIG            3
#define REPLY_AXIS_METRICS           4

struct Reply_header {
  uint32_t type;
};

struct Reply_timing {
  uint32_t type;
  uint32_t update_id;
  int32_t time_diff;
  uint32_t rp_update_len;
};

struct Reply_axis_movement {
  uint32_t type;
  uint32_t axis;
  int32_t abs_pos_acheived;
  int32_t velocity_acheived;
};

struct Reply_axis_config {
  uint32_t type;
  uint32_t axis;
  uint8_t enable;
  int8_t gpio_step;
  int8_t gpio_dir;
  double max_velocity;
  double max_accel;
};

struct Reply_axis_metrics {
  uint32_t type;
  uint32_t axis;
  int32_t velocity_requested;
  int32_t step_len_ticks;
  //int32_t pos_error;
};

union ReplyAny {
  struct Reply_header header;
  struct Reply_timing timing;
  struct Reply_axis_movement joint_movement;
  struct Reply_axis_config joint_config;
  struct Reply_axis_metrics joint_metrics;
};

#endif  // UPDATE_TYPES__H
