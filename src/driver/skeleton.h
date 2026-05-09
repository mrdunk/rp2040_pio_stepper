#ifndef SKELETON_H
#define SKELETON_H

/* Runtime data for a single HAL port/channel.
 * Assumes hal_u32_t, hal_s32_t, hal_float_t, hal_bit_t are defined by the includer
 * (hal.h in production; mock typedefs in test builds). */

typedef struct {
  hal_u32_t* seq_in;
  hal_u32_t* seq_out;
  hal_s32_t* packet_interval;
  hal_u32_t* rx_miss_count;
  hal_bit_t* eth_up;
  hal_bit_t* machine_on;
  hal_bit_t* joint_enable_cmd[MAX_JOINT];
  hal_s32_t  joint_gpio_step[MAX_JOINT];
  hal_s32_t  joint_gpio_dir[MAX_JOINT];
  hal_u32_t  joint_cmd_type[MAX_JOINT];
  hal_float_t* joint_vel_limit[MAX_JOINT];
  hal_float_t* joint_accel_limit[MAX_JOINT];
  hal_float_t* joint_scale[MAX_JOINT];
  hal_float_t* joint_pos_cmd[MAX_JOINT];
  hal_float_t* joint_vel_cmd[MAX_JOINT];
  hal_float_t* joint_pos_fb[MAX_JOINT];
  hal_float_t* joint_vel_fb[MAX_JOINT];
  hal_s32_t* joint_pos_error_fb[MAX_JOINT];
  hal_bit_t* joint_enable_fb[MAX_JOINT];
  hal_float_t* joint_vel_calculated[MAX_JOINT];
  hal_float_t* joint_ferror_suggest[MAX_JOINT];

  hal_u32_t* core1_period;
  hal_u32_t* core1_tick;
  hal_u32_t* core1_work_us;
  hal_u32_t* core0_work_us;
  hal_float_t* update_overrun;
  hal_float_t* update_underrun;

  double ema_overrun;
  double ema_underrun;

  hal_bit_t* gpio_data_in[MAX_GPIO];
  hal_bit_t* gpio_data_in_not[MAX_GPIO];
  hal_bit_t* gpio_data_out[MAX_GPIO];
  hal_bit_t* gpio_data_out_invert[MAX_GPIO];
  hal_u32_t  gpio_type[MAX_GPIO];
  hal_u32_t  gpio_index[MAX_GPIO];
  hal_u32_t  gpio_address[MAX_GPIO];

  hal_u32_t gpio_data_received[MAX_GPIO_BANK];
  hal_bit_t gpio_confirmation_pending[MAX_GPIO_BANK];

  hal_bit_t* spindle_fwd[MAX_SPINDLE];
  hal_bit_t* spindle_rev[MAX_SPINDLE];
  hal_float_t* spindle_speed_fb[MAX_SPINDLE];
  hal_float_t* spindle_speed_cmd[MAX_SPINDLE];
  hal_bit_t* spindle_at_speed[MAX_SPINDLE];

  hal_u32_t spindle_vfd_type[MAX_SPINDLE];
  hal_u32_t spindle_address[MAX_SPINDLE];
  hal_float_t spindle_poles[MAX_SPINDLE];
  hal_u32_t spindle_bitrate[MAX_SPINDLE];
} skeleton_t;

#endif  // SKELETON_H
