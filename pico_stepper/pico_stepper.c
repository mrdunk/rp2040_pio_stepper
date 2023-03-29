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

#define SZOF_AXIS_UPDATE (sizeof(struct AxisUpdate) / sizeof(uint32_t))

uint32_t core1_stack[CORE1_STACK_SIZE];
repeating_timer_t hw_update_timer;
volatile struct AxisUpdate axis_mvmnt_c0[MAX_AXIS] = {0};

volatile struct ConfigGlobal config_c0 = {
  .update_rate = 1000,       // 1kHz.
  .update_time_us = 1000,    // 1000us.
  .update_time_ticks = 133000,
  .axis = {
    {
      // Azis 0.
      .updated = 0,
      .abs_pos = UINT_MAX / 2,
      .min_step_len_ticks = 500,
      .max_accel_ticks = 200
    },
    {
      // Azis 1.
      .updated = 0,
      .abs_pos = UINT_MAX / 2,
      .min_step_len_ticks = 500,
      .max_accel_ticks = 200
    },
    {
      // Azis 2.
      .updated = 0,
      .abs_pos = UINT_MAX / 2,
      .min_step_len_ticks = 500,
      .max_accel_ticks = 200
    },
    {
      // Azis 3.
      .updated = 0,
      .abs_pos = UINT_MAX / 2,
      .min_step_len_ticks = 500,
      .max_accel_ticks = 200
    },
    {
      // Azis 4.
      .updated = 0,
      .abs_pos = UINT_MAX / 2,
      .min_step_len_ticks = 500,
      .max_accel_ticks = 200
    },
    {
      // Azis 5.
      .updated = 0,
      .abs_pos = UINT_MAX / 2,
      .min_step_len_ticks = 500,
      .max_accel_ticks = 200
    },
    /*{
      // Azis 6.
      .updated = 0,
      .abs_pos = UINT_MAX / 2,
      .min_step_len_ticks = 500,
      .max_accel_ticks = 200
    },
    {
      // Azis 7.
      .updated = 0,
      .abs_pos = UINT_MAX / 2,
      .min_step_len_ticks = 500,
      .max_accel_ticks = 200
    }*/
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
    /*{
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
    }*/
  }
};

/* A ring buffer that returns the average value of it's contents. */
#define RING_BUF_AVE_LEN 1000
struct Ring_buf_ave {
  uint32_t buf[RING_BUF_AVE_LEN];
  size_t head;
  uint32_t total;
  size_t count;
} ring_buf_ave_default = {.buf = {0}, .head = 0, .total = 0, .count=0};

size_t ring_buf_ave(struct Ring_buf_ave* data, uint32_t new_val) {
  size_t tail_val = data->buf[data->head];
  data->buf[data->head] = new_val;
  data->head++;
  if(data->head >= RING_BUF_AVE_LEN) {
    data->head = 0;
  }
  data->total += new_val;
  if(data->count < RING_BUF_AVE_LEN) {
    data->count++;
  } else {
    data->total -= tail_val;
  }

  return data->total / data->count;
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

void distribute_steps(struct AxisUpdate* update, uint32_t* step_lens)
{
  uint8_t axis = update->axis;
  const uint32_t update_time_ticks = config_c1.update_time_ticks;
  int32_t velocity = update->value - config_c1.axis[axis].abs_pos;
  //uint8_t direction = (velocity > 0) ? 1 : 0;
  uint8_t direction = (velocity > 0);
  const int32_t prev_velocity = abs(config_c1.axis[axis].velocity);
  const uint32_t min_step_len_ticks = config_c1.axis[axis].min_step_len_ticks;
  const uint32_t max_accel = config_c1.axis[axis].max_accel_ticks;
  uint32_t step_len;
  uint32_t step_count;

  //printf("axis: %u\tdirection: %i\tstep_count: %li\tupdate_time_ticks: %lu"
  //    "\tmin_step_len_ticks: %lu \n",
  //    axis, direction, step_count, update_time_ticks, min_step_len_ticks);

  if(abs(velocity) > MAX_STEPS_PER_UPDATE) {
    // Oops. Trying to send more steps than we have space for.
    velocity = direction ? MAX_STEPS_PER_UPDATE : -MAX_STEPS_PER_UPDATE;
  }

  // Limit steps according to maximum allowed acceleration/deceleration.
  if(max_accel > 0) {
    if(direction && (velocity > (prev_velocity + (int32_t)max_accel))) {
      velocity = prev_velocity + max_accel;
      direction = (velocity > 0);
    } else if(!direction && (velocity < (prev_velocity - (int32_t)max_accel))) {
      velocity = prev_velocity - max_accel;
      direction = (velocity > 0);
    }
  }
  step_count = abs(velocity);

  if(step_count == 0) {
    // Special case: No steps requested.
    // Just create a single entry spanning the whole duration with no IO pin changes.
    step_lens[0] = update_time_ticks + (direction << 29) + (direction << 31);
    return;
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
      // Also it actually lasts 2x as long as requested.
      step_len = update_time_ticks + 1 +(direction << 28) + (direction << 30) + (1 << 31);
    } else {
      time_total_ticks += step_len_ticks;
      // TODO: The PIO has some overhead; It inserts a few extra instructions.
      // Subtract those from the step_lens value here.
      // Also it actually lasts 2x as long as requested.
      step_len = update_time_ticks + (direction << 28) + (direction << 30) + (1 << 31);
    }
    if(step_count > MAX_STEPS_PER_UPDATE) {
      printf("ERROR: core1: step_lens buffer is full. %lu\n", step);
      break;
    }
    step_lens[step] = step_len;
  }
  //printf("axis: %u\tdirection: %i\tstep_count: %li\tstep_len_ticks: %lu\n",
  //    axis, direction, step_count, step_len_ticks);

  config_c1.axis[axis].abs_pos += direction ? +step_count : -step_count;
  config_c1.axis[axis].velocity = direction ? +step_count : -step_count;
  return;
}

uint8_t core1_process_updates(queue_t* axis_mvmnt_c1, uint8_t axis, uint32_t* step_lens) {
  struct AxisUpdate update;
  queue_remove_blocking(axis_mvmnt_c1, &update);
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
  return update.updated;
}

void core1_send_to_core0() {
  struct AxisUpdate to_serialise;
  //printf("c1 > c0\n");
  for(uint8_t axis = 0; axis < MAX_AXIS; axis++) {
    to_serialise.padding = 0;
    to_serialise.command = CMND_REPORT_ABS_POS;
    to_serialise.updated = 1;
    to_serialise.axis = axis;
    to_serialise.value = config_c1.axis[axis].abs_pos;
    if(multicore_fifo_push_timeout_us(((uint32_t*)(&to_serialise))[0], 200) == false) {
      printf("FAIL: core1 push fail for axis %u\n", axis);
      break;
    }
    if(multicore_fifo_push_timeout_us(((uint32_t*)(&to_serialise))[1], 200) == false) {
      printf("FAIL: core1 push fail for axis %u\n", axis);
      break;
    }

    to_serialise.padding = 0;
    to_serialise.command = CMND_REPORT_VELOCITY;
    to_serialise.updated = 1;
    to_serialise.axis = axis;
    to_serialise.value = config_c1.axis[axis].velocity;

    if(multicore_fifo_push_timeout_us(((uint32_t*)(&to_serialise))[0], 200) == false) {
      printf("FAIL: core0 push fail for axis %u\n", axis);
      break;
    }
    if(multicore_fifo_push_timeout_us(((uint32_t*)(&to_serialise))[1], 200) == false) {
      printf("FAIL: core0 push fail for axis %u\n", axis);
      break;
    }
  }
}

/* This interrupt handler is triggered when data from core1 appears on the FIFO. */
void core1_FIFO_interrupt_handler() {
  static size_t last_time = 0;
  static const size_t buffer_in_len = sizeof(struct AxisUpdate) / sizeof(uint32_t) * MAX_AXIS;
  static struct AxisUpdate buffer_in[MAX_AXIS];
  static size_t buffer_in_point = 0;
  static size_t update_point = 0;
  static size_t fail_count = 0;
  static struct Ring_buf_ave period_average_data;
  static uint32_t ave_period = 0;
  static size_t start_time = 0;
  static bool first_run = true;
  static queue_t axis_mvmnt_c1;

  if(first_run) {
    first_run = false;
    queue_init(&axis_mvmnt_c1, sizeof(struct AxisUpdate), MAX_AXIS);
  }

  uint32_t raw;
  uint32_t fail = 0;

  if(!multicore_fifo_rvalid()) {
    #if LOG_CORE1 == 2
    printf("WARN: core1_FIFO_interrupt_handler fired with no new data.\n");
    #endif // LOG_CORE1
    return;
  }

  if(update_point == 0) {
    // Starting processing a new data set.
    start_time = time_us_64();
    if(last_time == 0) {
      period_average_data = ring_buf_ave_default;
    } else {
      ave_period = ring_buf_ave(&period_average_data, start_time - last_time);
    }
  }

  #if LOG_CORE1 == 2
  printf("core1.FIFO_irq start.\n");
  #endif // LOG_CORE1

  struct AxisUpdate update;
  while(multicore_fifo_pop_timeout_us(1, &raw) && buffer_in_point < buffer_in_len) {
    ((uint32_t*)&update)[buffer_in_point % SZOF_AXIS_UPDATE] = raw;
    buffer_in_point++;
    if(buffer_in_point % (SZOF_AXIS_UPDATE) == 0) {
      // Whole AxisUpdate object populated.
      if(update_point != update.axis) {
        printf("ERROR: core1.interrupt: Invalid data\n");
        fail = 1;
        return;
      }
      if(!queue_try_add(&axis_mvmnt_c1, &update)) {
        printf("ERROR: core1.interrupt: Queue was full\n");
        fail = 1;
        break;
    }
      update_point++;
    }
  }
  multicore_fifo_clear_irq(); // Clear interrupt

  if(buffer_in_point < buffer_in_len) {
    return;
  }

  if(fail) {
    // Drain remaining incoming entries and reset everything before restart.
    while(multicore_fifo_pop_timeout_us(10, &raw));
    buffer_in_point = 0;
    update_point = 0;
    fail_count++;
  }

  buffer_in_point = 0;
  update_point = 0;

  // Calculate and cache the step lengths.
  uint32_t step_lens[MAX_AXIS][MAX_STEPS_PER_UPDATE] = {0};
  if(queue_get_level(&axis_mvmnt_c1) == MAX_AXIS) {
    uint8_t count_updates = 0;
    for(uint8_t axis = 0; axis < MAX_AXIS; axis++) {
      count_updates += core1_process_updates(&axis_mvmnt_c1, axis, step_lens[axis]);
    }
  }

  core1_send_to_core0();

  // Profiling output.
  #ifdef LOG_CORE1
  size_t now = time_us_64();
  #endif // LOG_CORE1
  #if LOG_CORE1 == 1
  printf("\t\t\t\t\t\tc1\t%4lu%6lu%6lu%6lu\n",
      start_time - last_time,
      now - start_time,
      fail_count,
      ave_period);
  #elif LOG_CORE1 == 2
  printf("core1.FIFO_irq:\t\t%lu\t%lu\n", start_time - last_time, now - start_time);
  #endif // LOG_CORE1
  #ifdef LOG_CORE1
  last_time = start_time;
  #endif // LOG_CORE1

}

void core1_entry() {
  uint32_t step_lens[MAX_AXIS][MAX_STEPS_PER_UPDATE] = {0};

  // Set up FIFO interrupt for processing data sent from core0 > core1.
  multicore_fifo_clear_irq();
  irq_set_exclusive_handler(SIO_IRQ_PROC1, core1_FIFO_interrupt_handler);
  irq_set_enabled(SIO_IRQ_PROC1, true);

  // Infinite While Loop to wait for interrupt
  while (1){
    tight_loop_contents();
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

uint8_t number_axis_updated() {
    uint8_t updated = 0;
    for(uint8_t axis = 0; axis < MAX_AXIS; axis++) {
      updated += axis_mvmnt_c0[axis].updated;
    }

    return updated;
}

/* Send data to core1 via FIFO. */
void core0_send_to_core1() {
  static size_t last_time = 0;
  static size_t fail_count = 0;
  static struct Ring_buf_ave period_average_data;

  size_t start_time = time_us_64();
  uint32_t ave_period = 0;
  if(last_time == 0) {
    period_average_data = ring_buf_ave_default;
  } else {
    ave_period = ring_buf_ave(&period_average_data, start_time - last_time);
  }

  for(size_t axis = 0; axis < MAX_AXIS; axis++) {
    volatile struct AxisUpdate* to_serialise = &(axis_mvmnt_c0[axis]);
    to_serialise->axis = axis;
    for (size_t i = 0; i < sizeof(struct AxisUpdate) / sizeof(uint32_t); i++) {
      uint32_t byte = ((uint32_t*)to_serialise)[i];

      multicore_fifo_push_blocking(byte);
    }
    to_serialise->updated = 0;
  }
  if(number_axis_updated() != 0) {
    printf("core0: ERROR: Data not marked as sent.\n");
    fail_count++;
  }
  memset((void*)axis_mvmnt_c0, 0, sizeof(struct AxisUpdate) * MAX_AXIS); 

  #ifdef LOG_CORE0
  size_t now = time_us_64(); 
  #endif // LOG_CORE0
  #if LOG_CORE0 == 1
  printf("\t\t\t\t\t\tc0.tx\t%4lu%6lu%6lu%6lu\n",
      start_time - last_time,
      now - start_time,
      fail_count,
      ave_period);
  #elif LOG_CORE0 == 2
  printf("0tx %lu\n", ave_period);
  #endif // LOG_CORE0
  last_time = start_time;

  return;
}

void core0_FIFO_interrupt_handler() {
  static size_t last_time = 0;
  static size_t fail_count = 0;

  #ifdef LOG_CORE0
  size_t start_time = time_us_64();
  #endif // LOG_CORE0

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

      switch(update.command) {
        case CMND_REPORT_ABS_POS:
          //printf("%u\t%u\t%u\t%lu\n",
          //update.command, update.updated, update.axis, update.value);
          if(config_c0.axis[update.axis].abs_pos != update.value) {
            // printf("updating axis %u abs_pos\n", update.axis);
            config_c0.axis[update.axis].updated = 1;
          }
          config_c0.axis[update.axis].abs_pos = update.value;
          break;
        case CMND_REPORT_VELOCITY:
          //printf("%u\t%u\t%u\t%li\n",
          //update.command, update.updated, update.axis, update.value);
          if(config_c0.axis[update.axis].velocity != update.value) {
            // printf("updating axis %u velocity\n", update.axis);
            config_c0.axis[update.axis].updated = 1;
          }
          config_c0.axis[update.axis].velocity = update.value;
          break;

      }
      count++;
    }

    multicore_fifo_clear_irq(); // Clear interrupt

    if(count != MAX_AXIS * 2) {
      printf("WARN: core0: corrupt data received. %u\n", count);
      fail_count++;
    }

  } else {
    #ifdef LOG_CORE0
    printf("WARN: core0_FIFO_interrupt_handler fired with no new data.\n");
    #endif // LOG_CORE0
  }

  // Profiling output.
  #ifdef LOG_CORE0
  size_t now = time_us_64();
  #endif // LOG_CORE0
  #if LOG_CORE0 == 1
  printf("\t\t\t\t\t\tc0.rx\t%4lu%6lu%6lu\n",
      start_time - last_time,
      now - start_time,
      fail_count);
  #elif LOG_CORE0 == 2
  printf("0rx %lu\t%lu\n", start_time - last_time, now - start_time);
  #endif // LOG_CORE0
  #ifdef LOG_CORE0
  last_time = start_time;
  #endif // LOG_CORE0
}

/* Push config from core0 to core1. */
void sync_config_c0_to_c1() {
  for(size_t prop = 0; prop < 3; prop++) {
    // Wait for any active timer IRQ to finish.
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
              axis, config_c0.axis[axis].min_step_len_ticks);
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
              axis, config_c0.axis[axis].max_accel_ticks);
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
    core0_send_to_core1();
    // Wait for FIFO interrupt on core1 to process.
    sleep_ms(1);
  }
}

void init_core1() {
  static uint8_t done = 0;
  if(done > 0) {
    return;
  }
  printf("core0: Initializing PIO handlers.\n");

  // Launch core1.
  multicore_launch_core1_with_stack(&core1_entry, core1_stack, CORE1_STACK_SIZE);

  sync_config_c0_to_c1();

  // Initialise PIOs.
  // TODO: Set up pins through config.
  uint32_t stepper_count = MAX_AXIS;
  uint32_t pins_step[8] =      {0, 2, 4, 6, 8, 10, 12, 14};
  uint32_t pins_direction[8] = {1, 3, 5, 7, 9, 11, 13, 15};

  for (uint32_t stepper = 0; stepper < stepper_count; stepper++) {
    init_pio(stepper, pins_step[stepper], pins_direction[stepper]);
  }

  // Set up return FIFO path. core1 > core0.
  multicore_fifo_clear_irq();
  irq_set_exclusive_handler(SIO_IRQ_PROC0, core0_FIFO_interrupt_handler);
  irq_set_enabled(SIO_IRQ_PROC0, true);

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

void send_pio_steps(
    const uint32_t axis,
    int32_t desired_pos) {
  // Queue axis movement data;
  // Disable timer interrupt to make sure the ISR doesn't read partial data.
  // TODO: Could the timer number change with library updates?
  // TODO: can we disable only this alarm rather than the whole interrupt?
  axis_mvmnt_c0[axis] = (struct AxisUpdate){
      .padding = 0,
      .command = CMND_SET_DESIRED_POS,
      .updated = 1,
      .axis = axis,
      .value = desired_pos
  };

  return;
}

void set_relative_position(
    const uint32_t stepper,
    const int position_diff) {

  uint32_t new_position = config_c0.axis[stepper].abs_pos + position_diff;
  send_pio_steps(stepper, new_position);
  return;
}

void set_absolute_position(
    const uint32_t stepper,
    const uint32_t new_position) {
  send_pio_steps(stepper, new_position);
  return;
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

uint32_t set_global_update_rate(uint32_t update_rate) {
  config_c0.update_rate = update_rate;
  config_c0.update_time_us = 1000000 / update_rate;
  config_c0.update_time_ticks = time_to_ticks(config_c0.update_time_us);
  return config_c0.update_time_us;
}

uint32_t get_global_update_time_us() {
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
        "  ConfigAxis.max_accel_ticks: %lu\r\n"
        "  ConfigAxis.velocity: %li\r\n",
        axis,
        config_c0.axis[axis].abs_pos,
        config_c0.axis[axis].min_step_len_ticks,
        config_c0.axis[axis].max_accel_ticks,
        config_c0.axis[axis].velocity);
  }
  if(*msg_machine_len + sizeof(struct Reply_axis_config) <= msg_machine_len_max) {
    struct Reply_axis_config reply = Reply_axis_config_default;
    reply.axis = axis;
    reply.abs_pos = config_c0.axis[axis].abs_pos;
    reply.min_step_len_ticks = config_c0.axis[axis].min_step_len_ticks;
    reply.max_accel_ticks = config_c0.axis[axis].max_accel_ticks;
    reply.velocity = config_c0.axis[axis].velocity;

    memcpy(msg_machine + *msg_machine_len, &reply, sizeof(struct Reply_axis_config));
    *msg_machine_len += sizeof(struct Reply_axis_config);
  }
}

void get_axis_config_if_updated(
    const uint32_t axis,
    uint8_t* msg_human,
    size_t msg_human_len_max,
    uint8_t* msg_machine,
    size_t* msg_machine_len,
    size_t msg_machine_len_max)
{
  if(config_c0.axis[axis].updated) {
    get_axis_config(
        axis, msg_human, msg_human_len_max, msg_machine, msg_machine_len, msg_machine_len_max);
    config_c0.axis[axis].updated = 0;
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
    reply.axis = axis;
    reply.abs_pos = config_c0.axis[axis].abs_pos;

    memcpy(msg_machine + *msg_machine_len, &reply, sizeof(struct Reply_axis_pos));
    *msg_machine_len += sizeof(struct Reply_axis_pos);
  }
}
