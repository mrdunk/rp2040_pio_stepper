#ifndef UPDATE_TYPES__H
#define UPDATE_TYPES__H

#include <sys/types.h>
#include <stdint.h>

/* This file contains objects that are passed over Ethernet UDP between host and RP2040.
 * Multiple structs can be packed in an array. */

#ifdef BUILD_TESTS

#define MAX_JOINT 4
#define MAX_GPIO 64
#define MAX_GPIO_BANK (MAX_GPIO / 32)
#define MAX_SPINDLE 4
#define MAX_I2C_MCP 4

#else // BUILD_TESTS

#define MAX_JOINT 4
#define MAX_GPIO 32
#define MAX_GPIO_BANK (MAX_GPIO / 32)
#define MAX_SPINDLE 4
#define MAX_I2C_MCP 4

#endif  // BUILD_TESTS


#define MSG_NONE                     0
#define MSG_TIMING                   1  // Contains packet count and time sent.
#define MSG_SET_JOINT_ENABLED        2  // Enable/disable joint.
#define MSG_SET_JOINT_ABS_POS        3  // Set absolute joint position.
#define MSG_SET_JOINT_CONFIG         4  // Set all config for a joint.
#define MSG_SET_GPIO                 5  // Set values for a bank (32) of GPIO.
#define MSG_SET_GPIO_CONFIG          6  // Set config for a single GPIO.
#define MSG_SET_SPINDLE_CONFIG       7  // Set spindle configuration
#define MSG_SET_SPINDLE_SPEED        8  // Set spindle speed

struct Message_header {
  uint8_t type;
};

struct Message_timing {
  uint8_t type;                   // MSG_TIMING
  uint32_t update_id;
  uint32_t time;
};

struct Message_set_joints_pos {
  uint8_t type;                   // MSG_SET_JOINT_ABS_POS
  double position[MAX_JOINT];
  double velocity[MAX_JOINT];
};

// TODO: Not currently used.
// Should be refactored to do all joints in one update.
struct Message_joint_enable {
  uint8_t type;                   // MSG_SET_JOINT_ENABLED
  uint8_t joint;
  uint8_t value;
};

struct Message_gpio {
  uint8_t type;                   // MSG_SET_GPIO
  uint8_t bank;                   // A bank of 32 IO pins.
  uint8_t confirmation_pending;   // If set the RP should always send a Reply_gpio in response.
  uint32_t values;                // Values to be sent to any of the IO pins that are outputs.
};

struct Message_joint_config {
  uint8_t type;                   // MSG_SET_JOINT_CONFIG
  uint8_t joint;
  uint8_t enable;
  int8_t gpio_step;               // Negative if disabled.
  int8_t gpio_dir;                // Negative if disabled.

  // TODO: These could be uint32_t.
  double max_velocity;
  double max_accel;
};

struct Message_spindle_config {
  uint8_t type;                   // MSG_SET_SPINDLE_CONFIG
  uint8_t spindle_index;
  uint8_t modbus_address;
  uint8_t vfd_type;
  uint16_t bitrate;
};

struct Message_spindle_speed {
  uint8_t type;                   // MSG_SET_SPINDLE_SPEED
  float speed[MAX_SPINDLE];
};

struct Message_gpio_config {
  uint8_t type;                   // MSG_SET_GPIO_CONFIG
  uint8_t gpio_type;
  uint8_t gpio_count;             // The HAL side index of which gpio this is.
  uint8_t index;                  // The RP side component index. IO pin number for RP native.
  uint8_t address;                // The i2c address if applicable.
};

union MessageAny {
  struct Message_header header;
  struct Message_timing timing;
  struct Message_set_joints_pos set_abs_pos;
  struct Message_joint_enable joint_enable;
  struct Message_gpio gpio;
  struct Message_joint_config joint_config;
  struct Message_spindle_config spindle_config;
  struct Message_spindle_speed spindle_speed;
  struct Message_gpio_config gpio_config;
};


#define REPLY_NONE                   0
#define REPLY_TIMING                 1
#define REPLY_JOINT_MOVEMENT         2
#define REPLY_JOINT_CONFIG           3
#define REPLY_JOINT_METRICS          4
#define REPLY_GPIO                   5
#define REPLY_GPIO_CONFIG            6
#define REPLY_SPINDLE_SPEED          7
#define REPLY_SPINDLE_CONFIG         8

struct Reply_header {
  uint8_t type;
};

struct Reply_timing {
  uint8_t type;
  uint32_t update_id;
  int32_t time_diff;
  uint32_t rp_update_len;
};

struct Reply_joint_movement {
  uint8_t type;
  int32_t abs_pos_achieved[MAX_JOINT];
  int32_t velocity_achieved[MAX_JOINT];
  int32_t position_error[MAX_JOINT];
};

struct Reply_joint_config {
  uint8_t type;
  uint8_t joint;
  uint8_t enable;
  int8_t gpio_step;
  int8_t gpio_dir;
  double max_velocity;
  double max_accel;
};

struct Reply_gpio_config {
  uint8_t type;
  uint8_t gpio_type;
  uint8_t gpio_count;
  uint8_t index;
  uint8_t address;
};

struct Reply_joint_metrics {
  uint8_t type;
  int32_t velocity_requested_tm1[MAX_JOINT];
  int32_t step_len_ticks[MAX_JOINT];
};

struct Reply_gpio {
  uint8_t type;
  uint8_t bank;
  uint8_t confirmation_pending;
  uint32_t values;
};

struct Reply_spindle_speed {
  // TODO: Needs parameters per spindle if support for multiple spindles is added.
  uint8_t type;
  uint8_t spindle_index;
  double speed;
  uint16_t crc_errors;
  uint16_t unanswered;
  uint16_t unknown;
  uint16_t got_status:1;
  uint16_t got_set_frequency:1;
  uint16_t got_act_frequency:1;
};

struct Reply_spindle_config {
  uint8_t type;
  uint8_t spindle_index;
  uint8_t modbus_address;
  uint8_t vfd_type;
  uint16_t bitrate;
};

union ReplyAny {
  struct Reply_header header;
  struct Reply_timing timing;
  struct Reply_joint_movement joint_movement;
  struct Reply_joint_config joint_config;
  struct Reply_gpio_config gpio_config;
  struct Reply_joint_metrics joint_metrics;
  struct Reply_spindle_speed spindle_speed;
};

#define GPIO_TYPE_NOT_SET            0
#define GPIO_TYPE_NATIVE_IN          1
#define GPIO_TYPE_NATIVE_OUT         2
#define GPIO_TYPE_NATIVE_IN_DEBUG    3
#define GPIO_TYPE_NATIVE_OUT_DEBUG   4
#define GPIO_TYPE_I2C_MCP_IN         5
#define GPIO_TYPE_I2C_MCP_OUT        6

#endif  // UPDATE_TYPES__H
