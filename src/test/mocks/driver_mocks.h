#ifndef DRIVER_MOCKS__H
#define DRIVER_MOCKS__H

#include <errno.h>
#include <stdbool.h>
#include <limits.h>
#include <stdint.h>
#include "../../driver/rp2040_defines.h"
#include "../../shared/messages.h"


#define RTAPI_MSG_ERR 1

typedef uint32_t hal_u32_t;
typedef int32_t hal_s32_t;
typedef double hal_float_t;
typedef bool hal_bit_t;

typedef struct {
  hal_u32_t* metric_update_id;
  hal_s32_t* metric_time_diff;
  hal_u32_t* metric_rp_update_len;
  hal_u32_t* metric_missed_packets;
  hal_bit_t* metric_eth_state;
  hal_bit_t* joint_enable[MAX_JOINT];
  hal_bit_t* joint_velocity_mode[MAX_JOINT];
  hal_s32_t* joint_gpio_step[MAX_JOINT];
  hal_s32_t* joint_gpio_dir[MAX_JOINT];
  hal_float_t* joint_max_velocity[MAX_JOINT];
  hal_float_t* joint_max_accel[MAX_JOINT];
  hal_float_t* joint_scale[MAX_JOINT];
  hal_float_t* joint_position[MAX_JOINT];
  hal_float_t* joint_velocity[MAX_JOINT];
  hal_float_t* joint_pos_feedback[MAX_JOINT];
  hal_s32_t* joint_step_len_ticks[MAX_JOINT];
  hal_float_t* joint_velocity_cmd[MAX_JOINT];
  hal_float_t* joint_velocity_feedback[MAX_JOINT];
  hal_bit_t* gpio_data_in[MAX_GPIO];
  hal_bit_t* gpio_data_out[MAX_GPIO];
  hal_u32_t* gpio_type[MAX_GPIO];
  hal_u32_t gpio_data_received[MAX_GPIO_BANK];
  hal_bit_t gpio_confirmation_pending[MAX_GPIO_BANK];
  
  hal_bit_t* spindle_fwd[MAX_SPINDLE];
  hal_bit_t* spindle_rev[MAX_SPINDLE];
  hal_float_t* spindle_speed_out[MAX_SPINDLE];
  hal_float_t* spindle_speed_in[MAX_SPINDLE];
  hal_bit_t* spindle_at_speed[MAX_SPINDLE];

  hal_u32_t spindle_vfd_type[MAX_SPINDLE];
  hal_u32_t spindle_address[MAX_SPINDLE];
  hal_float_t spindle_poles[MAX_SPINDLE];
  hal_u32_t spindle_bitrate[MAX_SPINDLE];
} skeleton_t;


void rtapi_print_msg();


#endif  // DRIVER_MOCKS__H
