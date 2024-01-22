#ifndef DRIVER_MOCKS__H
#define DRIVER_MOCKS__H

#include <errno.h>
#include <stdbool.h>
#include <limits.h>
#include <stdint.h>
#include "../../driver/rp2040_defines.h"
#include "../../shared/messages.h"

#define JOINTS 4

#define RTAPI_MSG_ERR 1

typedef uint32_t hal_u32_t;
typedef int32_t hal_s32_t;
typedef float hal_float_t;
typedef bool hal_bit_t;

typedef struct {
  hal_u32_t* metric_update_id;
  hal_s32_t* metric_time_diff;
  hal_u32_t* metric_rp_update_len;
  hal_u32_t* metric_missed_packets;
  hal_bit_t* metric_eth_state;
  hal_bit_t* joint_enable[JOINTS];
  hal_bit_t* joint_velocity_mode[JOINTS];
  hal_s32_t* joint_gpio_step[JOINTS];
  hal_s32_t* joint_gpio_dir[JOINTS];
  hal_float_t* joint_max_velocity[JOINTS];
  hal_float_t* joint_max_accel[JOINTS];
  hal_float_t* joint_scale[JOINTS];
  hal_float_t* joint_pos_cmd[JOINTS];
  hal_float_t* joint_vel_cmd[JOINTS];
  hal_float_t* joint_pos_feedback[JOINTS];
  hal_s32_t* joint_step_len_ticks[JOINTS];
  hal_float_t* joint_velocity_cmd[JOINTS];
  hal_float_t* joint_velocity_feedback[JOINTS];
  hal_bit_t* gpio_data_in[MAX_GPIO];
  hal_bit_t* gpio_data_out[MAX_GPIO];
  hal_u32_t* gpio_type[MAX_GPIO];
  hal_u32_t gpio_data_received[MAX_GPIO / 32];
  hal_bit_t gpio_confirmation_pending[MAX_GPIO / 32];
} skeleton_t;


void rtapi_print_msg();


#endif  // DRIVER_MOCKS__H
