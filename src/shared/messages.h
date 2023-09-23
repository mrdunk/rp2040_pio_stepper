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
"TIMING                   | 1  | count: uint32_t, time: uint32_t\r\n"
"SET_GLOAL_UPDATE_RATE    | 2  | value: uint32_t\r\n"
"SET_AXIS_ABS_POS         | 3  | axis: uint32_t, value: uint32_t\r\n"
"SET_AXIS_REL_POS         | 4  | axis: uint32_t, value: int32_t\r\n"
"SET_AXIS_MAX_SPEED       | 5  | Not yet implemented\r\n"
"SET_AXIS_MAX_ACCEL       | 6  | Not yet implemented\r\n"
"SET_AXIS_ABS_POS_AT_TIME | 7  | Not yet implemented\r\n"
"SET_PID_KP               | 8  | PID control values\r\n"
"SET_PID_KD               | 9  | PID control values\r\n"
"SET_PID_KD               | 10  | PID control values\r\n"
"GET_GLOBAL_CONFIG        | 11 | no params\r\n"
"GET_AXIS_CONFIG          | 12 | axis: uint32_t\r\n"
"GET_AXIS_POS             | 13 | axis: uint32_t\r\n";

#define MSG_NONE                     0
#define MSG_TIMING                   1
#define MSG_SET_GLOAL_UPDATE_RATE    2
#define MSG_SET_AXIS_ABS_POS         3
#define MSG_SET_AXIS_REL_POS         4
#define MSG_SET_AXIS_MAX_SPEED       5
#define MSG_SET_AXIS_MAX_ACCEL       6
#define MSG_SET_AXIS_ABS_POS_AT_TIME 7
#define MSG_SET_PID_KP               8  // These will likely get removed.
#define MSG_SET_PID_KI               9  // These will likely get removed.
#define MSG_SET_PID_KD               10  // These will likely get removed.

#define MSG_GET_GLOBAL_CONFIG        11
#define MSG_GET_AXIS_CONFIG          12
#define MSG_GET_AXIS_POS             13

struct Message {
  uint32_t type;
};

struct Message_timing {
  uint32_t type;
  uint32_t update_id;
  uint32_t time;
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
#define REPLY_METRICS                1
#define REPLY_GLOBAL_CONFIG          2
#define REPLY_AXIS_CONFIG            3
#define REPLY_AXIS_POS               4

struct Reply_metrics {
  uint32_t type;
  uint32_t update_id;
  int32_t time_diff;
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
