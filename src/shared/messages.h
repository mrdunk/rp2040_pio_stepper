#ifndef UPDATE_TYPES__H
#define UPDATE_TYPES__H

#include <sys/types.h>

/* This file contains objects that are passed over Ethernet UDP between host and RP2040.
 * Multiple structs can be packed in an array. */

#define MAX_AXIS 4
#define MAX_GPIO 64
#define MAX_I2C_MCP 4


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
#define MSG_SET_GPIO                 11 // Set values for a bank (32) of GPIO.
#define MSG_SET_GPIO_CONFIG          12 // Set config for a single GPIO.


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

struct Message_gpio {
  uint32_t type;                  // MSG_SET_GPIO
  uint8_t bank;                   // A bank of 32 IO pins.
  uint8_t confirmation_pending;   // If set the RP should always send a Reply_gpio in response.
  uint32_t values;                // Values to be sent to any of the IO pins that are outputs.
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

struct Message_gpio_config {
  uint32_t type;
  uint8_t gpio_type;
  uint8_t gpio_count;  // The HAL side index of which gpio this is. 
  uint8_t index;       // The RP side component index. IO pin number for RP native.
  uint8_t address;     // The i2c address if applicable.
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
  struct Message_gpio gpio;
  struct Message_joint_config joint_config;
  struct Message_gpio_config gpio_config;
};


#define REPLY_NONE                   0
#define REPLY_TIMING                 1
#define REPLY_AXIS_MOVEMENT          2
#define REPLY_AXIS_CONFIG            3
#define REPLY_AXIS_METRICS           4
#define REPLY_GPIO                   5

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

struct Reply_gpio_config {
  uint32_t type;
  uint8_t gpio_type;
  uint8_t gpio_count;
  uint8_t index;
  uint8_t address;
};

struct Reply_axis_metrics {
  uint32_t type;
  uint32_t axis;
  int32_t velocity_requested;
  int32_t step_len_ticks;
};

struct Reply_gpio {
  uint32_t type;
  uint8_t bank;
  uint8_t confirmation_pending;
  uint32_t values;
};

union ReplyAny {
  struct Reply_header header;
  struct Reply_timing timing;
  struct Reply_axis_movement joint_movement;
  struct Reply_axis_config joint_config;
  struct Reply_gpio_config gpio_config;
  struct Reply_axis_metrics joint_metrics;
};

#define GPIO_TYPE_NOT_SET            0
#define GPIO_TYPE_NATIVE_IN          1
#define GPIO_TYPE_NATIVE_OUT         2
#define GPIO_TYPE_NATIVE_IN_DEBUG    3
#define GPIO_TYPE_NATIVE_OUT_DEBUG   4
#define GPIO_TYPE_I2C_MCP_IN         5
#define GPIO_TYPE_I2C_MCP_OUT        6

#endif  // UPDATE_TYPES__H
