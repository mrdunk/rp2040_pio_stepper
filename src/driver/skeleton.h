#ifndef SKELETON_H
#define SKELETON_H

/* Runtime data for a single HAL port/channel.
 * Assumes hal_u32_t, hal_s32_t, hal_float_t, hal_bit_t are defined by the includer
 * (hal.h in production; mock typedefs in test builds). */

typedef struct {
  hal_u32_t* metric_update_id;
  hal_u32_t* metric_tx_update_id;
  hal_s32_t* metric_time_diff;
  hal_u32_t* metric_rp_update_len;
  hal_u32_t* metric_missed_packets;
  hal_bit_t* metric_eth_state;
  hal_bit_t* machine_enable_out;
  hal_bit_t* joint_enable[MAX_JOINT];
  hal_s32_t* joint_gpio_step[MAX_JOINT];
  hal_s32_t* joint_gpio_dir[MAX_JOINT];
  hal_float_t* joint_max_velocity[MAX_JOINT];
  hal_float_t* joint_max_accel[MAX_JOINT];
  hal_float_t* joint_scale[MAX_JOINT];
  hal_float_t* joint_position[MAX_JOINT];
  hal_float_t* joint_velocity[MAX_JOINT];
  hal_float_t* joint_pos_feedback[MAX_JOINT];
  // The actual velocity achieved on the RP.
  hal_float_t* joint_velocity_feedback[MAX_JOINT];
  // Difference between requested position and actual position on RP.
  hal_s32_t* joint_pos_error[MAX_JOINT];
  // RP2040's view of joint enabled state.
  hal_bit_t* joint_rp_enabled[MAX_JOINT];
  hal_float_t* joint_rp_velocity_cmd[MAX_JOINT];

  hal_u32_t* rp_update_period;
  hal_u32_t* rp_core1_tick;
  hal_float_t* metric_overrun_ratio;
  hal_float_t* metric_underrun_ratio;

  double ema_overrun;
  double ema_underrun;

  hal_bit_t* gpio_data_in[MAX_GPIO];
  hal_bit_t* gpio_data_in_not[MAX_GPIO];
  hal_bit_t* gpio_data_out[MAX_GPIO];
  hal_bit_t* gpio_data_out_invert[MAX_GPIO];
  hal_u32_t* gpio_type[MAX_GPIO];
  hal_u32_t* gpio_index[MAX_GPIO];
  hal_u32_t* gpio_address[MAX_GPIO];

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

#endif  // SKELETON_H
