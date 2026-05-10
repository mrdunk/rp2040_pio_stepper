#ifndef UPDATE_TYPES__H
#define UPDATE_TYPES__H

#include <sys/types.h>
#include <stdint.h>

/* This file contains objects that are passed over Ethernet UDP between host and RP2040.
 * Multiple structs can be packed in an array.
 *
 * IMPORTANT: Any change to a struct here changes the wire format. After editing,
 * recompile and reinstall the LinuxCNC driver (hal_rp2040_eth.so) and reflash the
 * RP2040 firmware. A stale driver binary shows: "WARN: Unconsumed RX buffer
 * remainder: N bytes" where N equals the size of the new/changed field(s). */

#ifndef MAX_JOINT
  #define MAX_JOINT 4
#endif

#ifdef BUILD_TESTS
#define MAX_GPIO 64
#else
#define MAX_GPIO 32
#endif

#define MAX_GPIO_BANK (MAX_GPIO / 32)
#define MAX_SPINDLE 4
#define MAX_I2C_MCP 4


#define MSG_NONE                     0
#define MSG_TIMING                   1  // Contains packet count and time sent.
#define MSG_SET_JOINT_ENABLED        2  // Enable/disable joint.
#define MSG_SET_JOINT_ABS_POS        3  // Set absolute joint position.
#define MSG_SET_JOINT_CONFIG         4  // Set all config for a joint.
#define MSG_SET_GPIO                 5  // Set values for a bank (32) of GPIO.
#define MSG_SET_GPIO_CONFIG          6  // Set config for a single GPIO.
#define MSG_SET_SPINDLE_CONFIG       7  // Set spindle configuration
#define MSG_SET_SPINDLE_SPEED        8  // Set spindle speed

struct __attribute__((packed)) Message_header {
  uint8_t type;
};

struct __attribute__((packed)) Message_timing {
  uint8_t type;                   // MSG_TIMING
  uint32_t update_id;
  uint32_t time;
};

struct __attribute__((packed)) Message_set_joints_pos {
  uint8_t type;                   // MSG_SET_JOINT_ABS_POS
  uint8_t _pad[7];                // align doubles to 8-byte boundary
  double position[MAX_JOINT];
  double velocity[MAX_JOINT];
};

struct __attribute__((packed)) Message_joint_enable {
  uint8_t type;                   // MSG_SET_JOINT_ENABLED
  uint8_t joint;
  uint8_t value;
};

struct __attribute__((packed)) Message_gpio {
  uint8_t type;                   // MSG_SET_GPIO
  uint8_t bank;                   // A bank of 32 IO pins.
  uint8_t confirmation_pending;   // If set the RP should always send a Reply_gpio in response.
  uint32_t values;                // Values to be sent to any of the IO pins that are outputs.
};

struct __attribute__((packed)) Message_joint_config {
  uint8_t type;                   // MSG_SET_JOINT_CONFIG
  uint8_t joint;
  uint8_t enable;
  int8_t gpio_step;               // Negative if disabled.
  int8_t gpio_dir;                // Negative if disabled.
  uint8_t cmd_type;               // JOINT_CMD_POSITION or JOINT_CMD_VELOCITY
  uint8_t _pad[2];                // align floats to 4-byte boundary
  float max_velocity;
  float max_accel;
};

struct __attribute__((packed)) Message_spindle_config {
  uint8_t type;                   // MSG_SET_SPINDLE_CONFIG
  uint8_t spindle_index;
  uint8_t modbus_address;
  uint8_t vfd_type;
  uint16_t bitrate;
};

struct __attribute__((packed)) Message_spindle_speed {
  uint8_t type;                   // MSG_SET_SPINDLE_SPEED
  float speed[MAX_SPINDLE];
};

struct __attribute__((packed)) Message_gpio_config {
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

struct __attribute__((packed)) Reply_header {
  uint8_t type;
};

struct __attribute__((packed)) Reply_timing {
  uint8_t type;
  uint32_t update_id;
  int32_t time_diff;
  uint32_t rp_update_len;
};

struct __attribute__((packed)) Reply_joint_movement {
  uint8_t type;
  int32_t abs_pos_achieved[MAX_JOINT];
  int32_t velocity_achieved[MAX_JOINT];
  uint8_t enabled[MAX_JOINT];
  float velocity_cmd[MAX_JOINT];
  uint32_t update_period_us;
  uint32_t core1_tick;
};

struct __attribute__((packed)) Reply_joint_config {
  uint8_t type;
  uint8_t joint;
  uint8_t enable;
  int8_t gpio_step;
  int8_t gpio_dir;
  uint8_t cmd_type;               // JOINT_CMD_POSITION or JOINT_CMD_VELOCITY
  uint8_t _pad[2];                // align floats to 4-byte boundary
  float max_velocity;
  float max_accel;
};

struct __attribute__((packed)) Reply_gpio_config {
  uint8_t type;
  uint8_t gpio_type;
  uint8_t gpio_count;
  uint8_t index;
  uint8_t address;
};

struct __attribute__((packed)) Reply_joint_metrics {
  uint8_t  type;
  uint8_t  overrun_occurred;   /* 1 if any joint overran this tick, else 0 */
  uint8_t  underrun_occurred;  /* 1 if any joint underran this tick, else 0 */
  uint8_t  _pad;               /* align uint32_t fields to 4-byte boundary */
  uint32_t core1_work_us;      /* µs Core1 spent working last period (excl. wait_for_packet) */
  uint32_t core0_work_us;      /* µs Core0 spent working last period (packet rx → response tx) */
};

struct __attribute__((packed)) Reply_gpio {
  uint8_t type;
  uint8_t bank;
  uint8_t confirmation_pending;
  uint32_t values;
};

struct __attribute__((packed)) Reply_spindle_speed {
  // TODO: Needs parameters per spindle if support for multiple spindles is added.
  uint8_t type;
  uint8_t spindle_index;
  uint8_t _pad[2];                // align float to 4-byte boundary
  float speed;
  uint16_t crc_errors;
  uint16_t unanswered;
  uint16_t unknown;
  uint16_t got_status:1;
  uint16_t got_set_frequency:1;
  uint16_t got_act_frequency:1;
};

struct __attribute__((packed)) Reply_spindle_config {
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
  struct Reply_gpio gpio;
  struct Reply_spindle_speed spindle_speed;
};

#define JOINT_CMD_POSITION           0   // stepgen follows abs_pos_requested (default)
#define JOINT_CMD_VELOCITY           1   // stepgen follows velocity_requested

#define GPIO_TYPE_NOT_SET            0
#define GPIO_TYPE_NATIVE_OUT         1
#define GPIO_TYPE_NATIVE_IN          2
#define GPIO_TYPE_NATIVE_OUT_DEBUG   3
#define GPIO_TYPE_NATIVE_IN_DEBUG    4
#define GPIO_TYPE_I2C_MCP_OUT        5
#define GPIO_TYPE_I2C_MCP_IN         6
#define GPIO_TYPE_I2C_MCP_OUT_PULLUP 7

#endif  // UPDATE_TYPES__H
