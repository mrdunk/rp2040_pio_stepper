#ifndef UPDATE_TYPES__H
#define UPDATE_TYPES__H
#include <sys/types.h>

/* This file contains objects that are passed over Ethernet UDP between host and RP2040.
 * Multiple structs can be backed in an array.
 * The array should be null terminated; The last entry in the array should be a
 * uint32_t of zero value. */


#define MSG_NONE                     0
#define MSG_TIMING                   1  // Contains packet count and time sent.
#define MSG_SET_AXIS_ENABLED         3  // Set absolute axis position.
#define MSG_SET_AXIS_ABS_POS         4  // Set absolute axis position.
#define MSG_SET_AXIS_REL_POS         5  // Set relative axis position. (Velocity)
#define MSG_SET_AXIS_MAX_SPEED       6  // Not yet implemented.
#define MSG_SET_AXIS_MAX_ACCEL       7  // Not yet implemented.
#define MSG_SET_AXIS_PID_KP          8  // Multiplier for position updates.
#define MSG_SET_AXIS_IO_STEP         9  // Set GPIO step pin.
#define MSG_SET_AXIS_IO_DIR         10  // Set GPIO direction pin.

struct Message {
  uint32_t type;
};

struct Message_timing {
  uint32_t type;
  uint32_t update_id;
  uint32_t time;
};

struct Message_set_kp {
  uint32_t type;
  uint32_t axis;
  float value;
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

struct Message_set_rel_pos {
  uint32_t type;
  uint32_t axis;
  double value;
};

struct Message_joint_enable {
  uint32_t type;
  uint32_t axis;
  uint32_t value;
};

struct Message_joint_gpio {
  uint32_t type;
  uint32_t axis;
  uint8_t value;
};

union MessageAny {
  struct Message_timing timing;
  struct Message_set_kp set_kp;
  struct Message_set_max_velocity set_max_velocity;
  struct Message_set_max_accel set_max_accel;
  struct Message_set_abs_pos set_abs_pos;
  struct Message_set_abs_pos set_rel_pos;
  struct Message_joint_enable joint_enable;
  struct Message_joint_gpio joint_gpio;
};


#define REPLY_NONE                   0
#define REPLY_METRICS                1
#define REPLY_GLOBAL_CONFIG          2
#define REPLY_AXIS_CONFIG            3
#define REPLY_AXIS_POS               4

struct Reply_metrics {
  uint32_t type;
  uint32_t update_id;
  int32_t time_diff;
  uint32_t rp_update_len;
} static Reply_metrics_default = { REPLY_METRICS };

struct Reply_global_config {
  uint32_t type;
  uint32_t update_rate;
  uint32_t update_time_us;  // (1,000,000) / update_rate
  uint32_t update_time_ticks;
} static Reply_global_conf_default = { REPLY_GLOBAL_CONFIG };

struct Reply_axis_config {
  uint32_t type;
  uint32_t axis;
  uint32_t abs_pos_acheived;
  uint32_t max_velocity;
  uint32_t max_accel_ticks;
  int32_t velocity_requested;  // TODO: Remove me once done debugging.
  int32_t velocity_acheived;
} static Reply_axis_config_default = { REPLY_AXIS_CONFIG };

struct Reply_axis_pos {
  uint32_t type;
  uint32_t axis;
  uint32_t abs_pos_acheived;
} static Reply_axis_pos_default = { REPLY_AXIS_POS };

#endif  // UPDATE_TYPES__H
