#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// PIO related.
#include "hardware/pio.h"
#include "hardware/clocks.h"
// The compiled .pio.
#include "pico_stepper.pio.h"

#include "pico_stepper.h"
#include "messages.h"



struct ConfigGlobal config = {
  .update_rate = 1000,       // 1kHz.
  .update_time_us = 1000,    // 1000us.
  .axis = {
    {
      // Azis 0.
      .abs_pos = UINT_MAX / 2,
      .min_step_len_us = 500
    },
    {
      // Azis 1.
      .abs_pos = UINT_MAX / 2,
      .min_step_len_us = 500
    },
    {
      // Azis 2.
      .abs_pos = UINT_MAX / 2,
      .min_step_len_us = 500
    },
    {
      // Azis 3.
      .abs_pos = UINT_MAX / 2,
      .min_step_len_us = 500
    },
    {
      // Azis 4.
      .abs_pos = UINT_MAX / 2,
      .min_step_len_us = 500
    },
    {
      // Azis 5.
      .abs_pos = UINT_MAX / 2,
      .min_step_len_us = 500
    },
    {
      // Azis 6.
      .abs_pos = UINT_MAX / 2,
      .min_step_len_us = 500
    },
    {
      // Azis 7.
      .abs_pos = UINT_MAX / 2,
      .min_step_len_us = 500
    }
  }
};

/* Return speed in steps/second for a particular step length. */
inline static uint step_len_to_speed(const uint step_len_us) {
  return clock_get_hz(clk_sys) / 2 / step_len_us;
}

inline static uint speed_to_step_len(const uint speed) {
  return clock_get_hz(clk_sys) / 2 / speed;
}

void init_pio(
    const uint stepper,
    const uint pin_step,
    const uint pin_direction
    )
{
  uint offset, sm;
  switch (stepper) {
    case 0:
    case 1:
    case 2:
    case 3:
      offset = pio_add_program(pio0, &step_program);
      sm = stepper;
      step_program_init(pio0, sm, offset, pin_step, pin_direction);
      pio_sm_set_enabled(pio0, sm, true);
      break;
    case 4:
    case 5:
    case 6:
    case 7:
      offset = pio_add_program(pio1, &step_program);
      sm = stepper - 4;
      step_program_init(pio1, sm, offset, pin_step, pin_direction);
      pio_sm_set_enabled(pio1, sm, true);
      break;
    default:
      printf("WARN: Invalid stepper index: %ld\n", stepper);
  }
}

uint send_pio_steps(
    const uint stepper,
    uint step_count,
    uint step_len_us,
    const uint direction) {
  if(step_count == 0) {
    // No steps to add.
    return config.axis[stepper].abs_pos;
  }
  if(step_len_us <= 9) {  // TODO: Should this be "=" rather than "<=" ?
    // The PIO program has a 9 instruction overhead so steps shorter than this are
    // not possible.
    printf("WARN: Step too short: %ld\n", step_len_us);
    return config.axis[stepper].abs_pos;
  }

  PIO pio;
  uint sm;

  switch (stepper) {
    case 0:
    case 1:
    case 2:
    case 3:
      pio = pio0;
      sm = stepper;
      break;
    case 4:
    case 5:
    case 6:
    case 7:
      pio = pio1;
      sm = stepper - 4;
      break;
    default:
      printf("Invalid stepper index: %ld\n", stepper);
  }

  if(!pio_sm_is_tx_fifo_empty(pio, sm)) {
    // Previous write has not yet been processed.
    return config.axis[stepper].abs_pos;
  }

  if(step_len_us < config.axis[stepper].min_step_len_us) {
    // Limit maximum speed.
    step_len_us = config.axis[stepper].min_step_len_us;
  }

  // Track absolute position.
  // TODO: Implement acceleration values.
  if (direction > 0) {
    config.axis[stepper].abs_pos += step_count;
  } else {
    config.axis[stepper].abs_pos -= step_count;
  }

  // PIO program generates 1 more step than it's told to.
  step_count -= 1;

  // PIO program makes steps 9 instructions longer than it's told to.
  step_len_us -= 9;

  pio_sm_put(pio, sm, direction);
  pio_sm_put(pio, sm, step_len_us);
  pio_sm_put(pio, sm, step_count);

  return config.axis[stepper].abs_pos;
}

uint set_relative_position_at_time(
    const uint stepper,
    const int position_diff,
    const uint time_slice) {
  uint step_len_us = time_slice / abs(position_diff);
  uint direction = 0;
  if(position_diff > 0) {
    direction = 1;
  }

  return send_pio_steps(stepper, abs(position_diff), step_len_us, direction);
}

uint set_relative_position(
    const uint stepper,
    const int position_diff) {
  uint step_len_us = config.update_time_us / abs(position_diff);
  uint direction = 0;
  if(position_diff > 0) {
    direction = 1;
  }

  return send_pio_steps(stepper, abs(position_diff), step_len_us, direction);
}

uint set_absolute_position_at_time(
    const uint stepper,
    const uint new_position,
    const uint time_slice_us) {
  int position_diff = new_position - config.axis[stepper].abs_pos;
  return set_relative_position_at_time(stepper, position_diff, time_slice_us);
}

uint set_absolute_position(
    const uint stepper,
    const uint new_position) {
  const uint time_slice_us = config.update_time_us;
  int position_diff = new_position - config.axis[stepper].abs_pos;
  return set_relative_position_at_time(stepper, position_diff, time_slice_us);
}

uint get_absolute_position(uint stepper) {
  return config.axis[stepper].abs_pos;
}

void set_max_speed(
    const uint stepper,
    const uint max_speed_step_sec
    ) {
  config.axis[stepper].min_step_len_us = speed_to_step_len(max_speed_step_sec);
}

uint get_max_speed(
    const uint stepper,
    const uint max_speed_step_sec
    ) {
  return step_len_to_speed(config.axis[stepper].min_step_len_us);
}

uint set_global_update_rate(uint update_rate) {
  config.update_rate = update_rate;
  config.update_time_us = 1000000 / update_rate;
  return 1000000 / config.update_time_us;
}

void get_global_config(
    uint8_t* msg_human,
    size_t msg_human_len_max,
    uint8_t* msg_machine,
    size_t* msg_machine_len,
    size_t msg_machine_len_max) {
  if(msg_human_len_max > 0) {
    size_t len = strlen(msg_human);
    snprintf(msg_human + len,
        msg_human_len_max - len,
        "ConfigGlobal summary:\r\n"
        "  ConfigGlobal.update_rate:    %lu\r\n"
        "  ConfigGlobal.update_time_us: %lu\r\n",
        config.update_rate,
        config.update_time_us);
  }
  if(*msg_machine_len + sizeof(struct Reply_global_config) <= msg_machine_len_max) {
    struct Reply_global_config reply = Reply_global_conf_default;
    reply.update_rate = config.update_rate;
    reply.update_time_us = config.update_time_us;
    memcpy(msg_machine + *msg_machine_len, &reply, sizeof(struct Reply_global_config));
    printf("** %lu %lu %lu\n", ((uint*)msg_machine)[0], ((uint*)msg_machine)[1], ((uint*)msg_machine)[2]);
    *msg_machine_len += sizeof(struct Reply_global_config);
  }
}

void get_axis_config(
    const uint axis,
    uint8_t* msg_human,
    size_t msg_human_len_max,
    uint8_t* msg_machine,
    size_t* msg_machine_len,
    size_t msg_machine_len_max) {
  if(msg_human_len_max > 0) {
    size_t len = strlen(msg_human);
    snprintf(msg_human + len,
        msg_human_len_max - len,
        "ConfigAxis[%lu] summary:\r\n"
        "  ConfigAxis.abs_pos:    %lu\r\n"
        "  ConfigAxis.min_step_len_us: %lu\r\n",
        axis,
        config.axis[axis].abs_pos,
        config.axis[axis].min_step_len_us);
  }
  if(*msg_machine_len + sizeof(struct Reply_axis_config) <= msg_machine_len_max) {
    struct Reply_axis_config reply = Reply_axis_config_default;
    reply.abs_pos = config.axis[axis].abs_pos;
    reply.min_step_len_us = config.axis[axis].min_step_len_us;
    memcpy(msg_machine + *msg_machine_len, &reply, sizeof(struct Reply_axis_config));
    *msg_machine_len += sizeof(struct Reply_axis_config);
  }
}

void get_axis_pos(
    const uint axis,
    uint8_t* msg_human,
    size_t msg_human_len_max,
    uint8_t* msg_machine,
    size_t* msg_machine_len,
    size_t msg_machine_len_max) {
  if(msg_human_len_max > 0) {
    size_t len = strlen(msg_human);
    snprintf(msg_human + len, msg_human_len_max - len,
        "ConfigAxis[%lu] summary:\r\n"
        "  ConfigAxis.abs_pos:    %lu\r\n",
        axis,
        config.axis[axis].abs_pos);
  }
  if(*msg_machine_len + sizeof(struct Reply_axis_pos) <= msg_machine_len_max) {
    struct Reply_axis_pos reply = Reply_axis_pos_default;
    reply.abs_pos = config.axis[axis].abs_pos;
    memcpy(msg_machine + *msg_machine_len, &reply, sizeof(struct Reply_axis_pos));
    *msg_machine_len += sizeof(struct Reply_axis_pos);
  }
}

