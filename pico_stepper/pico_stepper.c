#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// PIO related.
#include "hardware/pio.h"
#include "hardware/clocks.h"
// The compiled .pio.
#include "pico_stepper.pio.h"
// Core1
#include "pico/multicore.h"

#include "pico_stepper.h"
#include "messages.h"

uint32_t core1_stack[CORE1_STACK_SIZE];
repeating_timer_t hw_update_timer;
volatile struct AxisUpdate stepper_mvmnt_c0[MAX_AXIS] = {0};
volatile struct AxisUpdate stepper_mvmnt_c1[MAX_AXIS] = {0};


struct ConfigGlobal config_c0 = {
  .update_rate = 1000,       // 1kHz.
  .update_time_us = 1000,    // 1000us.
  .update_time_ticks = 133000,
  .axis = {
    {
      // Azis 0.
      .abs_pos = UINT_MAX / 2,
      .min_step_len_ticks = 50
    },
    {
      // Azis 1.
      .abs_pos = UINT_MAX / 2,
      .min_step_len_ticks = 50
    },
    {
      // Azis 2.
      .abs_pos = UINT_MAX / 2,
      .min_step_len_ticks = 50
    },
    {
      // Azis 3.
      .abs_pos = UINT_MAX / 2,
      .min_step_len_ticks = 50
    },
    {
      // Azis 4.
      .abs_pos = UINT_MAX / 2,
      .min_step_len_ticks = 50
    },
    {
      // Azis 5.
      .abs_pos = UINT_MAX / 2,
      .min_step_len_ticks = 50
    },
    {
      // Azis 6.
      .abs_pos = UINT_MAX / 2,
      .min_step_len_ticks = 50
    },
    {
      // Azis 7.
      .abs_pos = UINT_MAX / 2,
      .min_step_len_ticks = 50
    }
  }
};

struct ConfigGlobal config_c1 = {
  .update_rate = 1000,       // 1kHz.
  .update_time_us = 1000,    // 1000us.
  .update_time_ticks = 133000,
  .axis = {
    {
      // Azis 0.
      .abs_pos = UINT_MAX / 2,
      .min_step_len_ticks = 50
    },
    {
      // Azis 1.
      .abs_pos = UINT_MAX / 2,
      .min_step_len_ticks = 50
    },
    {
      // Azis 2.
      .abs_pos = UINT_MAX / 2,
      .min_step_len_ticks = 50
    },
    {
      // Azis 3.
      .abs_pos = UINT_MAX / 2,
      .min_step_len_ticks = 50
    },
    {
      // Azis 4.
      .abs_pos = UINT_MAX / 2,
      .min_step_len_ticks = 50
    },
    {
      // Azis 5.
      .abs_pos = UINT_MAX / 2,
      .min_step_len_ticks = 50
    },
    {
      // Azis 6.
      .abs_pos = UINT_MAX / 2,
      .min_step_len_ticks = 50
    },
    {
      // Azis 7.
      .abs_pos = UINT_MAX / 2,
      .min_step_len_ticks = 50
    }
  }
};


#define RB_LEN 1000

struct Rb {
  size_t buf[RB_LEN];
  size_t head;
  size_t total;
} static rb_default = {.buf = {0}, .head = 0, .total = 0};

size_t rb(struct Rb* data, size_t new_val) {
  //static size_t buf[RB_LEN] = {0};
  //static size_t head = 0;
  //static size_t total = 0;

  size_t tail_val = data->buf[data->head];
  data->buf[data->head] = new_val;
  data->head++;
  if(data->head++ >= RB_LEN) {
    data->head = 0;
  }
  data->total -= tail_val;
  data->total += new_val;

  return data->total;
}

void axis_to_pio(const uint32_t stepper, PIO* pio, uint32_t* sm) {
  switch (stepper) {
    case 0:
    case 1:
    case 2:
    case 3:
      *pio = pio0;
      *sm = stepper;
      break;
    case 4:
    case 5:
    case 6:
    case 7:
      *pio = pio1;
      *sm = stepper - 4;
      break;
    default:
      printf("WARN: Invalid stepper index: %ld\n", stepper);
  }
}

void core1_entry() {
  static uint32_t last_pos[MAX_AXIS];     // axis abs_pos when we received new data.
  static uint32_t current_pos[MAX_AXIS];  // axis abs_pos we sent to pio.
  static uint8_t init_done = 0;
  uint8_t axis;
  PIO pio;
  uint32_t sm;
  struct Rb run_time = rb_default;
  size_t section_count = 0;
  uint32_t message_section_len = config_c0.update_time_us / MESSAGE_SECTIONS;
  uint32_t byte;
  size_t section_start_time, start_time, end_time, diff_time,
         ave_time = 100,
         worst_time = 0, count = 0, buff_eror = 0;

  printf("Hello core1\n");


  while(1) {
    start_time = time_us_64();

    //printf("loop core1\n");
    if(multicore_fifo_rvalid() == true) {
      //printf("core1: pop\n");
      for(axis = 0; axis < MAX_AXIS * sizeof(struct AxisUpdate) / sizeof(uint32_t); axis++) {
        if(multicore_fifo_pop_timeout_us(0, &byte) == true) {
          ((uint32_t*)stepper_mvmnt_c1)[axis] = byte;
        } else {
          printf("FAIL: core1 pop on the %lu byte.\n", axis);
          buff_eror++;
          break;
        }
      }
      section_count = 0;
      // TODO: Send position data via FIFO.
    }

    for(axis = 0; axis < MAX_AXIS; axis++) {
      if(stepper_mvmnt_c1[axis].updated > 0) {
        printf("core1: "
            "command: %u  axis: %u  value: %lu\r\n",
            stepper_mvmnt_c1[axis].command,
            stepper_mvmnt_c1[axis].axis,
            stepper_mvmnt_c1[axis].value);
        stepper_mvmnt_c1[axis].updated = 0;
        //break;  // Print the rest on the next time around the loop.
      }
    }

    for(uint8_t axis = 0; axis < MAX_AXIS; axis++) {
      switch(stepper_mvmnt_c1[axis].command) {
        case CMND_SET_POS:
          config_c1.axis[axis].abs_pos = stepper_mvmnt_c1[axis].value;
          break;
        case CMND_SET_MIN_STEP_LEN:
          config_c1.axis[axis].min_step_len_ticks = stepper_mvmnt_c1[axis].value;
          break;
        case CMND_SET_DESIRED_POS:
          axis_to_pio(axis, &pio, &sm);
          if(pio_sm_is_tx_fifo_empty(pio, sm)) {
            section_start_time = start_time;
            uint32_t desired_pos = stepper_mvmnt_c1[axis].value;
            uint32_t current_pos = config_c1.axis[axis].abs_pos;
            int32_t pos_dif = desired_pos - current_pos;
            //part_step_change = 
            //if(stepper_mvmnt_c1[stepper].step_change > 0) {
            //  direction = 1;
            //} else {
            //  direction = 0;
            //}
            section_count++;
          }
          break;
        default:
          //printf("core1: Unknown update command: %u\n", stepper_mvmnt_c1[axis].command);
          break;


      }
    }

    // Below here is gathering stats.
    end_time = time_us_64();
    diff_time = end_time - start_time;
    if(diff_time > worst_time) {
      worst_time = diff_time;
    }

    ave_time = rb(&run_time, diff_time);
    if((count % 1000000) == 0) {
      //printf("%lu\t%lu\t%lu\t%lu\n", count, worst_time, ave_time, buff_eror);
      worst_time = 0;
    }
    count++;
  }

  //pio_sm_put(pio, sm, direction);
  //pio_sm_put(pio, sm, step_len_ticks);
  //pio_sm_put(pio, sm, step_count);
}


uint32_t todo(uint8_t stepper, int32_t step_change) {
  PIO pio;
  uint32_t sm;

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
      printf("WARN: Invalid stepper index: %ld\n", stepper);
  }

  uint32_t step_count = abs(step_change);
  uint32_t step_len_ticks = config_c0.update_time_ticks / step_count;

  if(step_len_ticks <= 9) {  // TODO: Should this be "=" rather than "<=" ?
    // The PIO program has a 9 instruction overhead so steps shorter than this are
    // not possible.
    printf("WARN: Step too short: %ld\n", step_len_ticks);
    return config_c0.axis[stepper].abs_pos;
  }

  if(!pio_sm_is_tx_fifo_empty(pio, sm)) {
    // Previous write has not yet been processed.
    return config_c0.axis[stepper].abs_pos;
  }


  // PIO program generates 1 more step than it's told to.
  step_count -= 1;

  // PIO program makes steps 9 instructions longer than it's told to.
  step_len_ticks -= 9;


  return config_c0.axis[stepper].abs_pos;
}


/* Return speed in steps/second for a particular step length. */
inline static uint32_t step_ticks_to_speed(const uint32_t step_len_ticks) {
  return clock_get_hz(clk_sys) / 2 / step_len_ticks;
}

inline static uint32_t speed_to_step_ticks(const uint32_t speed) {
  return clock_get_hz(clk_sys) / 2 / speed;
}

inline static uint32_t time_to_ticks(const uint32_t time_us) {
  return time_us * (clock_get_hz(clk_sys) / 1000000);
}

/* Send data to core1 via FIFO. */
static bool timer_callback_c0(repeating_timer_t *rt) {
  static int32_t jitter = 0;
  //printf("timer_callback_c0. %i\n", get_core_num());
  for(size_t stepper = 0; stepper < MAX_AXIS; stepper++) {
    volatile struct AxisUpdate* to_serialise = &(stepper_mvmnt_c0[stepper]);
    //if(to_serialise->updated) {
    //  printf("core0: updating %lu.  step_change: %li\n",
    //      to_serialise->count, to_serialise->step_change);
    //}
    for (size_t i = 0; i < sizeof(struct AxisUpdate) / sizeof(uint32_t); i++) {
      uint32_t byte = ((uint32_t*)to_serialise)[i];
      if(multicore_fifo_push_timeout_us(byte, 100000) == false) {
        printf("FAIL: core0 push fail for axis %u\n", stepper);
      }
    }
    to_serialise->updated = 0;
  }

  return true; // keep repeating
}


void init_core1() {
  static uint8_t done = 0;
  if(done > 0) {
    return;
  }
  printf("Initializing PIO handlers.\n");

  // Launch core1.
  multicore_launch_core1_with_stack(&core1_entry, core1_stack, CORE1_STACK_SIZE);

  // Start timer to push data from core0 > core1.
  // negative timeout means exact delay (rather than delay between callbacks)
  // TODO: Make the period match config_c0.update_time_us.
  if (!add_repeating_timer_us(-1000, timer_callback_c0, NULL, &hw_update_timer)) {
    printf("ERROR: Failed to add timer\n");
  }

  // Push config to core1.
  //sleep_ms(1000);
  irq_set_enabled(TIMER_IRQ_3, 0);
  for(size_t axis = 0; axis < MAX_AXIS; axis++) {
    printf("CMND_SET_POS axis: %u  value: %lu\r\n",
        axis, config_c0.axis[axis].abs_pos);
    stepper_mvmnt_c0[axis] = (struct AxisUpdate){
      .padding = 0,
        .command = CMND_SET_POS,
        .updated = 1,
        .axis = axis,
        .value = config_c0.axis[axis].abs_pos
    };
  }
  irq_set_enabled(TIMER_IRQ_3, 1);

  sleep_ms(1000);

  irq_set_enabled(TIMER_IRQ_3, 0);
  for(size_t axis = 0; axis < MAX_AXIS; axis++) {
    printf("CMND_SET_MIN_STEP_LEN axis: %u  value: %lu\r\n",
        axis, config_c0.axis[axis].min_step_len_ticks);
    stepper_mvmnt_c0[axis] = (struct AxisUpdate){
      .padding = 0,
        .command = CMND_SET_MIN_STEP_LEN,
        .updated = 1,
        .axis = axis,
        .value = config_c0.axis[axis].min_step_len_ticks
    };
  }
  irq_set_enabled(TIMER_IRQ_3, 1);

  sleep_ms(1000);

  done = 1;
}

void init_pio(
    const uint32_t stepper,
    const uint32_t pin_step,
    const uint32_t pin_direction
    )
{
  uint32_t offset, sm;
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

uint32_t send_pio_steps(
    const uint32_t stepper,
    int32_t desired_pos) {
  if(config_c0.axis[stepper].abs_pos == desired_pos) {
    // No steps to add.
    // TODO: Maybe we want to update core1 anyway in case to needs to decelerate?
    return config_c0.axis[stepper].abs_pos;
  }

  // TODO: This probably wants to be replaced with code that updates config_c0
  // with the steps performed as reported by core0.
  config_c0.axis[stepper].abs_pos = desired_pos;

  // Queue stepper movement data;
  // Disable timer interrupt to make sure the ISR doesn't read partial data.
  // TODO: Could the timer number change with library updates?
  // TODO: can we disable only this alarm rather than the whole interrupt?
  irq_set_enabled(TIMER_IRQ_3, 0);
  if(stepper_mvmnt_c0[stepper].updated > 0) {
    // Data hasn't been pulled in time.
    printf("WARN: Data not transferred to core1 in time.");
    gpio_put(LED_PIN, 1);
  }
  stepper_mvmnt_c0[stepper] = (struct AxisUpdate){
      .padding = 0,
      .command = CMND_SET_DESIRED_POS,
      .updated = 1,
      .axis = stepper,
      .value = desired_pos
  };
  irq_set_enabled(TIMER_IRQ_3, 1);

  return config_c0.axis[stepper].abs_pos;
}

uint32_t set_relative_position(
    const uint32_t stepper,
    const int position_diff) {

  uint32_t new_position = config_c0.axis[stepper].abs_pos + position_diff;
  return send_pio_steps(stepper, new_position);
}

uint32_t set_absolute_position(
    const uint32_t stepper,
    const uint32_t new_position) {
  return send_pio_steps(stepper, new_position);
}

uint32_t get_absolute_position(uint32_t stepper) {
  return config_c0.axis[stepper].abs_pos;
}

void set_max_speed(
    const uint32_t stepper,
    const uint32_t max_speed_step_sec
    ) {
  config_c0.axis[stepper].min_step_len_ticks = speed_to_step_ticks(max_speed_step_sec);
}

uint32_t get_max_speed(
    const uint32_t stepper,
    const uint32_t max_speed_step_sec
    ) {
  return step_ticks_to_speed(config_c0.axis[stepper].min_step_len_ticks);
}

uint32_t set_global_update_rate(uint32_t update_rate) {
  config_c0.update_rate = update_rate;
  config_c0.update_time_us = 1000000 / update_rate;
  config_c0.update_time_ticks = time_to_ticks(config_c0.update_time_us);
  return config_c0.update_time_us;
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
        "  ConfigGlobal.update_time_us: %lu\r\n"
        "  ConfigGlobal.update_time_ticks: %lu\r\n"
        "  ConfigGlobal.cpu_frequency: %lu\r\n" ,
        config_c0.update_rate,
        config_c0.update_time_us,
        config_c0.update_time_ticks,
        clock_get_hz(clk_sys));
  }
  if(*msg_machine_len + sizeof(struct Reply_global_config) <= msg_machine_len_max) {
    struct Reply_global_config reply = Reply_global_conf_default;
    reply.update_rate = config_c0.update_rate;
    reply.update_time_us = config_c0.update_time_us;
    reply.update_time_ticks = config_c0.update_time_ticks;
    memcpy(msg_machine + *msg_machine_len, &reply, sizeof(struct Reply_global_config));
    *msg_machine_len += sizeof(struct Reply_global_config);
  }
}

void get_axis_config(
    const uint32_t axis,
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
        "  ConfigAxis.min_step_len_ticks: %lu\r\n"
        "  ConfigAxis.velocity: %li\r\n",
        axis,
        config_c0.axis[axis].abs_pos,
        config_c0.axis[axis].min_step_len_ticks,
        config_c0.axis[axis].velocity);
  }
  if(*msg_machine_len + sizeof(struct Reply_axis_config) <= msg_machine_len_max) {
    struct Reply_axis_config reply = Reply_axis_config_default;
    reply.abs_pos = config_c0.axis[axis].abs_pos;
    reply.min_step_len_ticks = config_c0.axis[axis].min_step_len_ticks;
    memcpy(msg_machine + *msg_machine_len, &reply, sizeof(struct Reply_axis_config));
    *msg_machine_len += sizeof(struct Reply_axis_config);
  }
}

void get_axis_pos(
    const uint32_t axis,
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
        config_c0.axis[axis].abs_pos);
  }
  if(*msg_machine_len + sizeof(struct Reply_axis_pos) <= msg_machine_len_max) {
    struct Reply_axis_pos reply = Reply_axis_pos_default;
    reply.abs_pos = config_c0.axis[axis].abs_pos;
    memcpy(msg_machine + *msg_machine_len, &reply, sizeof(struct Reply_axis_pos));
    *msg_machine_len += sizeof(struct Reply_axis_pos);
  }
}
