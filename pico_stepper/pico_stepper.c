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
// IRQ safe queue
#include "pico/util/queue.h"

#include "pico_stepper.h"
#include "messages.h"

uint32_t core1_stack[CORE1_STACK_SIZE];
repeating_timer_t hw_update_timer;
volatile struct AxisUpdate axis_mvmnt_c0[MAX_AXIS] = {0};
queue_t axis_mvmnt_c1;

struct ConfigGlobal config_c0 = {
  .update_rate = 1000,       // 1kHz.
  .update_time_us = 1000,    // 1000us.
  .update_time_ticks = 133000,
  .axis = {
    {
      // Azis 0.
      .abs_pos = UINT_MAX / 2,
      .min_step_len_ticks = 500,
      .max_accel_ticks = 200
    },
    {
      // Azis 1.
      .abs_pos = UINT_MAX / 2,
      .min_step_len_ticks = 500,
      .max_accel_ticks = 200
    },
    {
      // Azis 2.
      .abs_pos = UINT_MAX / 2,
      .min_step_len_ticks = 500,
      .max_accel_ticks = 200
    },
    {
      // Azis 3.
      .abs_pos = UINT_MAX / 2,
      .min_step_len_ticks = 500,
      .max_accel_ticks = 200
    },
    {
      // Azis 4.
      .abs_pos = UINT_MAX / 2,
      .min_step_len_ticks = 500,
      .max_accel_ticks = 200
    },
    {
      // Azis 5.
      .abs_pos = UINT_MAX / 2,
      .min_step_len_ticks = 500,
      .max_accel_ticks = 200
    },
    {
      // Azis 6.
      .abs_pos = UINT_MAX / 2,
      .min_step_len_ticks = 500,
      .max_accel_ticks = 200
    },
    {
      // Azis 7.
      .abs_pos = UINT_MAX / 2,
      .min_step_len_ticks = 500,
      .max_accel_ticks = 200
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
      .min_step_len_ticks = 500,
      .max_accel_ticks = 200,
    },
    {
      // Azis 1.
      .abs_pos = UINT_MAX / 2,
      .min_step_len_ticks = 500,
      .max_accel_ticks = 200
    },
    {
      // Azis 2.
      .abs_pos = UINT_MAX / 2,
      .min_step_len_ticks = 500,
      .max_accel_ticks = 200
    },
    {
      // Azis 3.
      .abs_pos = UINT_MAX / 2,
      .min_step_len_ticks = 500,
      .max_accel_ticks = 200
    },
    {
      // Azis 4.
      .abs_pos = UINT_MAX / 2,
      .min_step_len_ticks = 500,
      .max_accel_ticks = 200
    },
    {
      // Azis 5.
      .abs_pos = UINT_MAX / 2,
      .min_step_len_ticks = 500,
      .max_accel_ticks = 200
    },
    {
      // Azis 6.
      .abs_pos = UINT_MAX / 2,
      .min_step_len_ticks = 500,
      .max_accel_ticks = 200
    },
    {
      // Azis 7.
      .abs_pos = UINT_MAX / 2,
      .min_step_len_ticks = 500,
      .max_accel_ticks = 200
    }
  }
};

//#define RB_LEN 1000
//
//struct Rb {
//  size_t buf[RB_LEN];
//  size_t head;
//  size_t total;
//} static rb_default = {.buf = {0}, .head = 0, .total = 0};
//
//size_t rb(struct Rb* data, size_t new_val) {
//  //static size_t buf[RB_LEN] = {0};
//  //static size_t head = 0;
//  //static size_t total = 0;
//
//  size_t tail_val = data->buf[data->head];
//  data->buf[data->head] = new_val;
//  data->head++;
//  if(data->head++ >= RB_LEN) {
//    data->head = 0;
//  }
//  data->total -= tail_val;
//  data->total += new_val;
//
//  return data->total;
//}

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

void distribute_steps(struct AxisUpdate* update, uint32_t* step_lens)
{
  uint8_t axis = update->axis;
  const uint32_t update_time_ticks = config_c1.update_time_ticks;
  uint32_t step_count = abs(config_c1.axis[axis].abs_pos - update->value);
  uint8_t direction = (config_c1.axis[axis].abs_pos > update->value) ? 0 : 1;
  const int32_t prev_step_count = config_c1.axis[axis].velocity;
  const uint32_t min_step_len_ticks = config_c1.axis[axis].min_step_len_ticks;
  const uint32_t max_accel = 0;  // TODO.
  uint32_t step_len;

  //printf("axis: %u\tdirection: %i\tstep_count: %li\tupdate_time_ticks: %lu"
  //    "\tmin_step_len_ticks: %lu \n",
  //    axis, direction, step_count, update_time_ticks, min_step_len_ticks);

  if(step_count > MAX_STEPS_PER_UPDATE) {
    // Oops. Trying to send more steps than we have space for.
    step_count = MAX_STEPS_PER_UPDATE;
  }

  if(step_count == 0) {
    // Special case: No steps requested.
    // Just create a single entry spanning the whole duration with no IO pin changes.
    step_lens[0] = update_time_ticks + (direction << 29) + (direction << 31);
    return;
  }

  // Limit steps according to maximum allowed acceleration/deceleration.
  if((max_accel > 0) && (step_count > prev_step_count + max_accel)) {
    step_count = prev_step_count + max_accel;
  } else if((max_accel > 0) && (step_count < prev_step_count - max_accel)) {
    step_count = prev_step_count - max_accel;
  }

  uint32_t time_total_ticks = 0;
  uint32_t accumilator = 0;
  uint32_t step_len_ticks = update_time_ticks / step_count;
  if(step_len_ticks < min_step_len_ticks) {
    // Limit the minimum allowed step length.
    // This is another way of saying "limit the max speed."
    step_count = update_time_ticks / min_step_len_ticks;
    step_len_ticks = update_time_ticks / step_count;
  }
  uint32_t mod = update_time_ticks % step_count;

  // Work out how long each step needs to be to fit step_count in update_time_ticks.
  for(uint32_t step = 0; step < step_count; step++) {
    accumilator += mod;
    if(accumilator > step_count) {
      accumilator -= step_count;
      time_total_ticks += step_len_ticks + 1;
      // TODO: The PIO has some overhead; It inserts a few extra instructions.
      // Subtract those from the step_lens value here.
      step_len = update_time_ticks + 1 +(direction << 28) + (direction << 30) + (1 << 31);
    } else {
      time_total_ticks += step_len_ticks;
      // TODO: The PIO has some overhead; It inserts a few extra instructions.
      // Subtract those from the step_lens value here.
      step_len = update_time_ticks + (direction << 28) + (direction << 30) + (1 << 31);
    }
    if(step_count > MAX_STEPS_PER_UPDATE) {
      printf("ERROR: core1: step_lens buffer is full. %ul\n", step);
      break;
    }
    step_lens[step] = step_len;
    //printf("%lu\t%lu\t%lu\n", step, step_lens[step], time_total_ticks);
  }
  printf("axis: %u\tdirection: %i\tstep_count: %li\tstep_len_ticks: %lu\n",
      axis, direction, step_count, step_len_ticks);

  config_c1.axis[axis].velocity = step_count;
}

void core1_do_update(uint8_t axis, uint32_t* step_lens) {
  struct AxisUpdate update;
  queue_remove_blocking(&axis_mvmnt_c1, &update);
  switch(update.command) {
    case CMND_SET_POS:
      config_c1.axis[axis].abs_pos = update.value;
      break;
    case CMND_SET_MIN_STEP_LEN:
      config_c1.axis[axis].min_step_len_ticks = update.value;
      break;
    case CMND_SET_MAX_ACCEL:
      config_c1.axis[axis].max_accel_ticks = update.value;
      break;
    case CMND_SET_DESIRED_POS:
      distribute_steps(&update, step_lens);
      break;
    default:
      //printf("core1: Unknown update command: %u\n", update.command);
      break;
  }
}

/* This interrupt handler is triggered when data from core1 appears on the FIFO. */
void core1_interrupt_handler() {
  static size_t last_time = 0;

  size_t start_time = time_us_64();

  if (multicore_fifo_rvalid()){
    uint32_t raw_0;
    uint32_t raw_1;
    size_t count = 0;
    // Pull the data from core0 off the FIFO.
    while(multicore_fifo_pop_timeout_us(0, &raw_0) &&
        multicore_fifo_pop_timeout_us(0, &raw_1))
    {
      struct AxisUpdate update;
      ((uint32_t*)(&update))[0] = raw_0;
      ((uint32_t*)(&update))[1] = raw_1;

      if (!queue_try_add(&axis_mvmnt_c1, &update)) {
        printf("ERROR: core1.interrupt: FIFO was full\n");
        break;
      }

      count++;
    }

    if(count != MAX_AXIS) {
      printf("core1: WARN: corrupt data received.\n");
      // Drain the incomplete queue.
      while(queue_try_remove(&axis_mvmnt_c1, NULL));
    }

  } else {
    printf("WARN: core1_interrupt_handler fired with no new data.\n");
  }

  multicore_fifo_clear_irq(); // Clear interrupt

  // Time profiling output.
  size_t now = time_us_64();
  printf("core1.irq:\t\t%lu\t%lu\n", start_time - last_time, now - start_time);
  last_time = start_time;
}

void core1_entry() {
    multicore_fifo_clear_irq();
    irq_set_exclusive_handler(SIO_IRQ_PROC1, core1_interrupt_handler);

    queue_init(&axis_mvmnt_c1, sizeof(struct AxisUpdate), MAX_AXIS);
    uint32_t step_lens[MAX_AXIS][MAX_STEPS_PER_UPDATE] = {0};

    irq_set_enabled(SIO_IRQ_PROC1, true);

    // Infinite While Loop to wait for interrupt
    while (1){
      if(!queue_is_empty(&axis_mvmnt_c1)) {
        size_t t1 = time_us_64();
        for(uint8_t axis = 0; axis < MAX_AXIS; axis++) {
          core1_do_update(axis, step_lens[axis]);
        }
        size_t t2 = time_us_64();
        printf("core1 update took:\t\t%lu\n", t2 - t1);
      }
    }
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
  static size_t last_time = 0;

  size_t start_time = time_us_64();

  //printf("timer_callback_c0. %i\n", get_core_num());
  for(size_t stepper = 0; stepper < MAX_AXIS; stepper++) {
    volatile struct AxisUpdate* to_serialise = &(axis_mvmnt_c0[stepper]);
    for (size_t i = 0; i < sizeof(struct AxisUpdate) / sizeof(uint32_t); i++) {
      uint32_t byte = ((uint32_t*)to_serialise)[i];

      // Shouldn't need much of a timeout here as core1 reacts to new data via 
      // interrupt and drains the buffer immediately.
      // As a result it should never block.
      if(multicore_fifo_push_timeout_us(byte, 100) == false) {
        printf("FAIL: core0 push fail for axis %u\n", stepper);
      }
    }
    to_serialise->updated = 0;
  }
  memset((void*)axis_mvmnt_c0, 0, sizeof(struct AxisUpdate) * MAX_AXIS); 

  size_t now = time_us_64(); 
  printf("core0.timer_irq:\t%lu\t%lu\n", start_time - last_time, now - start_time);
  last_time = start_time;

  return true; // keep repeating
}

/* Push config from core0 to core1. */
void sync_config_c0_to_c1() {
  for(size_t prop = 0; prop < sizeof(struct ConfigAxis) / sizeof(uint32_t); prop++) {
    irq_set_enabled(TIMER_IRQ_3, 0);
    for(size_t axis = 0; axis < MAX_AXIS; axis++) {
      switch(prop) {
        case(0):
          printf("CMND_SET_POS axis: %u  value: %lu\r\n",
              axis, config_c0.axis[axis].abs_pos);
          axis_mvmnt_c0[axis] = (struct AxisUpdate){
            .padding = 0,
              .command = CMND_SET_POS,
              .updated = 1,
              .axis = axis,
              .value = config_c0.axis[axis].abs_pos
          };
          break;
        case(1):
          printf("CMND_SET_MIN_STEP_LEN axis: %u  value: %lu\r\n",
              axis, config_c0.axis[axis].abs_pos);
          axis_mvmnt_c0[axis] = (struct AxisUpdate){
            .padding = 0,
              .command = CMND_SET_MIN_STEP_LEN,
              .updated = 1,
              .axis = axis,
              .value = config_c0.axis[axis].min_step_len_ticks
          };
          break;
        case(2):
          printf("CMND_SET_MAX_ACCEL axis: %u  value: %lu\r\n",
              axis, config_c0.axis[axis].abs_pos);
          axis_mvmnt_c0[axis] = (struct AxisUpdate){
            .padding = 0,
              .command = CMND_SET_MAX_ACCEL,
              .updated = 1,
              .axis = axis,
              .value = config_c0.axis[axis].max_accel_ticks
          };
          break;
        default:
          break;
      }
    }
    irq_set_enabled(TIMER_IRQ_3, 1);


    size_t all_updated = 1;
    while(all_updated) {
      for(size_t axis = 0; axis < MAX_AXIS; axis++) {
        all_updated &= axis_mvmnt_c0[axis].updated;
      }
    }
  }
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
  if (!add_repeating_timer_us(-1000000, timer_callback_c0, NULL, &hw_update_timer)) {
    printf("ERROR: Failed to add timer\n");
  }

  sync_config_c0_to_c1();

  // Initialise PIOs.
  // TODO: Set up pins through config.
  uint32_t stepper_count = MAX_AXIS;
  uint32_t pins_step[8] =      {0, 2, 4, 6, 8, 10, 12, 14};
  uint32_t pins_direction[8] = {1, 3, 5, 7, 9, 11, 13, 15};

  for (uint32_t stepper = 0; stepper < stepper_count; stepper++) {
    init_pio(stepper, pins_step[stepper], pins_direction[stepper]);
    //while(1);
  }

  //sleep_ms(1000);

  done = 1;
}

void init_pio(
    const uint32_t stepper,
    const uint32_t pin_step,
    const uint32_t pin_direction)
{
  static uint32_t offset_pio0 = 0;
  static uint32_t offset_pio1 = 0;

  uint32_t sm;
  switch (stepper) {
    case 0:
    case 1:
    case 2:
    case 3:
      if (offset_pio0 == 0) {
        offset_pio0 = pio_add_program(pio0, &step_program);
      }
      sm = pio_claim_unused_sm(pio0, true);
      step_program_init(pio0, sm, offset_pio0, pin_step, pin_direction);
      pio_sm_set_enabled(pio0, sm, true);
      break;
    case 4:
    case 5:
    case 6:
    case 7:
      if (offset_pio1 == 0) {
        offset_pio1 = pio_add_program(pio1, &step_program);
      }
      sm = pio_claim_unused_sm(pio1, true);
      step_program_init(pio1, sm, offset_pio1, pin_step, pin_direction);
      pio_sm_set_enabled(pio1, sm, true);
      break;
    default:
      printf("WARN: Invalid stepper index: %ld\n", stepper);
  }
}

uint32_t send_pio_steps(
    const uint32_t axis,
    int32_t desired_pos) {
  if(config_c0.axis[axis].abs_pos == desired_pos) {
    // No steps to add.
    // TODO: Maybe we want to update core1 anyway in case to needs to decelerate?
    return config_c0.axis[axis].abs_pos;
  }

  // TODO: This probably wants to be replaced with code that updates config_c0
  // with the steps performed as reported by core0.
  config_c0.axis[axis].abs_pos = desired_pos;

  // Queue axis movement data;
  // Disable timer interrupt to make sure the ISR doesn't read partial data.
  // TODO: Could the timer number change with library updates?
  // TODO: can we disable only this alarm rather than the whole interrupt?
  irq_set_enabled(TIMER_IRQ_3, 0);
  axis_mvmnt_c0[axis] = (struct AxisUpdate){
      .padding = 0,
      .command = CMND_SET_DESIRED_POS,
      .updated = 1,
      .axis = axis,
      .value = desired_pos
  };
  irq_set_enabled(TIMER_IRQ_3, 1);

  return config_c0.axis[axis].abs_pos;
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
