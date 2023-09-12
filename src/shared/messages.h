#ifndef UPDATE_TYPES__H
#define UPDATE_TYPES__H
#include <sys/types.h>

/* This file contains objects that are passed over Ethernet UDP between host and RP2040.
 * Multiple structs can be backed in an array.
 * The array should be null terminated; The last entry in the array should be a
 * uint32_t of zero value. */


static const char* human_help =
"Description              | id | params\r\n"
"-------------------------+----+------------------------\r\n"
"SET_GLOAL_UPDATE_RATE    | 1  | value: uint32_t\r\n"
"SET_AXIS_ABS_POS         | 2  | axis: uint32_t, value: uint32_t\r\n"
"SET_AXIS_REL_POS         | 3  | axis: uint32_t, value: int32_t\r\n"
"SET_AXIS_MAX_SPEED       | 4  | Not yet implemented\r\n"
"SET_AXIS_MAX_ACCEL       | 5  | Not yet implemented\r\n"
"SET_AXIS_ABS_POS_AT_TIME | 6  | Not yet implemented\r\n"
"SET_PID_KP               | 7  | PID control values\r\n"
"SET_PID_KD               | 8  | PID control values\r\n"
"SET_PID_KD               | 9  | PID control values\r\n"
"GET_GLOBAL_CONFIG        | 10 | no params\r\n"
"GET_AXIS_CONFIG          | 11 | axis: uint32_t\r\n"
"GET_AXIS_POS             | 12 | axis: uint32_t\r\n";

#define MSG_NONE                     0
#define MSG_SET_GLOAL_UPDATE_RATE    1
#define MSG_SET_AXIS_ABS_POS         2
#define MSG_SET_AXIS_REL_POS         3
#define MSG_SET_AXIS_MAX_SPEED       4
#define MSG_SET_AXIS_MAX_ACCEL       5
#define MSG_SET_AXIS_ABS_POS_AT_TIME 6
#define MSG_SET_PID_KP               7  // These will likely get removed.
#define MSG_SET_PID_KI               8  // These will likely get removed.
#define MSG_SET_PID_KD               9  // These will likely get removed.

#define MSG_GET_GLOBAL_CONFIG        10
#define MSG_GET_AXIS_CONFIG          11
#define MSG_GET_AXIS_POS             12

struct Message {
  uint32_t type;
};

struct Message_uint {
  uint32_t type;
  uint32_t value;
};

struct Message_uint_uint {
  uint32_t type;
  uint32_t axis;
  uint32_t value;
};

struct Message_uint_int {
  uint32_t type;
  uint32_t axis;
  int32_t value;
};

struct Message_uint_float {
  uint32_t type;
  uint32_t axis;
  float value;
};


#define REPLY_NONE                   0
#define REPLY_GLOBAL_CONFIG          1
#define REPLY_AXIS_CONFIG            2
#define REPLY_AXIS_POS               3

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
  uint32_t min_step_len_ticks;
  uint32_t max_accel_ticks;
  int32_t velocity_acheived;
} static Reply_axis_config_default = { REPLY_AXIS_CONFIG };

struct Reply_axis_pos {
  uint32_t type;
  uint32_t axis;
  uint32_t abs_pos_acheived;
} static Reply_axis_pos_default = { REPLY_AXIS_POS };

#endif  // UPDATE_TYPES__H