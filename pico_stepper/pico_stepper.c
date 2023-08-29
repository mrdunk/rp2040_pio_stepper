#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/param.h>

// PIO related.
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/structs/systick.h"
// The compiled .pio.
#include "pico_stepper.pio.h"
// Core1
#include "pico/multicore.h"
// IRQ safe queue
#include "pico/util/queue.h"

#include "pico_stepper.h"
#include "messages.h"

#define SIZOF_AXIS_UPDATE (sizeof(struct AxisUpdate) / sizeof(uint32_t))

uint32_t core1_stack[CORE1_STACK_SIZE];
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
      .min_step_len_ticks = 50,
      .max_accel_ticks = 200
    },
    {
      // Azis 1.
      .updated = 0,
      .abs_pos = UINT_MAX / 2,
      .min_step_len_ticks = 50,
      .max_accel_ticks = 200
    },
    {
      // Azis 2.
      .updated = 0,
      .abs_pos = UINT_MAX / 2,
      .min_step_len_ticks = 50,
      .max_accel_ticks = 200
    },
    {
      // Azis 3.
      .updated = 0,
      .abs_pos = UINT_MAX / 2,
      .min_step_len_ticks = 50,
      .max_accel_ticks = 200
    },
    /*{
      // Azis 4.
      .updated = 0,
      .abs_pos = UINT_MAX / 2,
      .min_step_len_ticks = 50,
      .max_accel_ticks = 200
    },
    {
      // Azis 5.
      .updated = 0,
      .abs_pos = UINT_MAX / 2,
      .min_step_len_ticks = 50,
      .max_accel_ticks = 200
    },
    /*{
      // Azis 6.
      .updated = 0,
      .abs_pos = UINT_MAX / 2,
      .min_step_len_ticks = 50,
      .max_accel_ticks = 200
    },
    {
      // Azis 7.
      .updated = 0,
      .abs_pos = UINT_MAX / 2,
      .min_step_len_ticks = 50,
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
      .min_step_len_ticks = 50,
      .max_accel_ticks = 200,
    },
    {
      // Azis 1.
      .abs_pos = UINT_MAX / 2,
      .min_step_len_ticks = 50,
      .max_accel_ticks = 200
    },
    {
      // Azis 2.
      .abs_pos = UINT_MAX / 2,
      .min_step_len_ticks = 50,
      .max_accel_ticks = 200
    },
    {
      // Azis 3.
      .abs_pos = UINT_MAX / 2,
      .min_step_len_ticks = 50,
      .max_accel_ticks = 200
    },
    /*{
      // Azis 4.
      .abs_pos = UINT_MAX / 2,
      .min_step_len_ticks = 50,
      .max_accel_ticks = 200
    },
    {
      // Azis 5.
      .abs_pos = UINT_MAX / 2,
      .min_step_len_ticks = 50,
      .max_accel_ticks = 200
    },
    /*{
      // Azis 6.
      .abs_pos = UINT_MAX / 2,
      .min_step_len_ticks = 50,
      .max_accel_ticks = 200
    },
    {
      // Azis 7.
      .abs_pos = UINT_MAX / 2,
      .min_step_len_ticks = 50,
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

void axis_to_pio(const uint32_t axis, PIO* pio, uint32_t* sm) {
  switch (axis) {
    case 0:
    case 1:
    case 2:
    case 3:
      *pio = pio0;
      *sm = axis;
      break;
    case 4:
    case 5:
    case 6:
    case 7:
      *pio = pio1;
      *sm = axis - 4;
      break;
    default:
      printf("WARN: Invalid axis index: %ld\n", axis);
  }
}

/* In progress. */
void do_steps(struct AxisUpdate* update, const uint32_t update_time_us){
  const uint8_t axis = update->axis;
  const uint32_t target_pos = update->value;
  const uint32_t min_step_len_ticks = config_c1.axis[axis].min_step_len_ticks;
  const uint32_t max_accel = config_c1.axis[axis].max_accel_ticks;

  int32_t velocity = update->value - config_c1.axis[axis].abs_pos;
  uint8_t direction = (velocity > 0);
  const int32_t prev_velocity = abs(config_c1.axis[axis].velocity);
  uint32_t step_count;
  uint32_t step_len_us;

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
    step_len_us = 0;
  } else {
    step_len_us = update_time_us / step_count;

    if(step_len_us < min_step_len_ticks * 100 / 133) {
      // Limit the minimum allowed step length.
      // This is another way of saying "limit the max speed."
      step_count = update_time_us / (min_step_len_ticks * 100 / 133);
      step_len_us = update_time_us / step_count;
    }

    //if() {
    //  step_count++;
    //} else {
    //  step_len_us++;
    //}
  }

  PIO pio;
  uint32_t sm;
  axis_to_pio(update->axis, &pio, &sm);

  // Request steps.
  pio_sm_put(pio, sm, (step_len_us / 2) - 15);

  // Wait for report on starting position.

  uint32_t step_count_acheived = pio_sm_get_blocking(pio, sm);

  config_c1.axis[axis].abs_pos += direction ? +step_count_acheived : -step_count_acheived;
  config_c1.axis[axis].velocity = direction ? +step_count_acheived : -step_count_acheived;

  if(sm != 0) {
    return;
  }

  //printf("%lu\n", sm);
  printf("s: %lu\n", step_count);
  //printf("t: %lu\n", step_len_us);
  //printf("a: %lu\n", step_count_acheived);
  //printf("T: %lu\n", update_time_us);
  printf("a: %lu\n", (step_count_acheived * update_time_us * 1000000) / clock_get_hz(clk_sys));
  //printf("c: %lu\n", clock_get_hz(clk_sys));
  //printf("r: %lu\n", target_pos);

  return;
}

/* Untested attempt to use the `dma_step` PIO program. */
void distribute_steps_dynamic(struct AxisUpdate* update, uint32_t* step_lens, const uint32_t ave_period_us)
{
  // Aiming for 256kHz step rate.
  // We assemble the desired step lengths for the whole time window in a buffer
  // then use DMA to populate the PIO input as requred.
  //
  // Search term: Trapezoidal Motion Profile.
  // TODO: We accelerate or decelerate as hard as we can at the start.
  // Assuming we reach a velocity that will get us to the desired target position,
  // we then stop accelerating and continue at constant velocity.
  // Would it be better to accelerate less aggressively so smearing the acceleration
  // over the whole time segment?

  const uint8_t axis = update->axis;
  const uint32_t target_pos = update->value;
  const int32_t start_step_rate_kHz = config_c1.axis[axis].velocity;
  const uint32_t min_step_len_ticks = config_c1.axis[axis].min_step_len_ticks;
  const uint32_t max_accel = config_c1.axis[axis].max_accel_ticks;
  const uint32_t time_period = ave_period_us;
  int32_t curr_step_rate_kHz;

  uint32_t step = 0;
  uint32_t elapsed_time = 0;
  uint32_t remaining_steps;
  uint32_t remaining_time;
  uint32_t constant_step;
  uint32_t accelerating_step;
  int8_t direction;
  int8_t accel_decel;
  uint32_t step_len_ticks;

  //printf("%u%12u\t\t\t%li\n", update->axis, update->value, target_pos - config_c1.axis[axis].abs_pos);

  while(elapsed_time < time_period) {
    remaining_steps = target_pos - config_c1.axis[axis].abs_pos;
    remaining_time = time_period - elapsed_time;

    // Step duration assuming we do need to accelerate/decelerate.
    accelerating_step =
      (max_accel * elapsed_time / time_period) + start_step_rate_kHz;
    // Step duration assuming we don't need to accelerate/decelerate.
    constant_step = remaining_steps * time_period / remaining_time;

    if(remaining_steps >= 0) {
      curr_step_rate_kHz = MIN(accelerating_step, constant_step);
    } else {
      curr_step_rate_kHz = MAX(-accelerating_step, constant_step);
    }

    curr_step_rate_kHz = constant_step;

    if(curr_step_rate_kHz > -4 && curr_step_rate_kHz < 0) {
      curr_step_rate_kHz = -4;
      //printf("curr_step_rate_kHz: %li\n", curr_step_rate_kHz);
    } else if (curr_step_rate_kHz >= 0 && curr_step_rate_kHz < 4) {
      curr_step_rate_kHz = 4;
      //printf("curr_step_rate_kHz: %li\n", curr_step_rate_kHz);
    }

    step_len_ticks = time_period / abs(curr_step_rate_kHz);

    if(step_len_ticks < min_step_len_ticks) {
      // Limit the minimum allowed step length.
      // This is another way of saying "limit the max speed."
      //printf("clamp step_len_ticks: %lu\t%lu\n", step_len_ticks, min_step_len_ticks);

      int32_t tmp_step_rate = time_period / min_step_len_ticks;
      curr_step_rate_kHz = (curr_step_rate_kHz >= 0) ? tmp_step_rate : -tmp_step_rate;
      step_len_ticks = min_step_len_ticks;
    }

    if(curr_step_rate_kHz > 0) {
      config_c1.axis[axis].abs_pos++;
    } else if(curr_step_rate_kHz < 0) {
      config_c1.axis[axis].abs_pos--;
    }

    elapsed_time += step_len_ticks;

    if(step > MAX_STEPS_PER_UPDATE) {
      //printf("ERROR: step: %lu\t%lu\t%lu\t%lu\n",
      //    step, step_len_ticks, elapsed_time, time_period);
      //while(1);
      break;
    }
    step_lens[step++] = abs(step_len_ticks);
  }
  config_c1.axis[axis].velocity = curr_step_rate_kHz;
  //printf("%u%12lu\t%lu\t%lu\t%lu\n",
  //    axis, config_c1.axis[axis].abs_pos, elapsed_time, ave_period_us, step);
}

uint8_t core1_process_updates(
    queue_t* axis_mvmnt_c1, uint8_t axis, uint32_t* step_lens, uint32_t update_period_us)
{
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
      do_steps(&update, update_period_us);
      break;
    default:
      //printf("core1: Unknown update command: %u\n", update.command);
      break;
  }
  return update.updated;
}

void core1_send_to_core0() {
  struct AxisUpdate to_serialise;
  for(uint8_t axis = 0; axis < MAX_AXIS; axis++) {
    to_serialise.padding = 0;
    to_serialise.command = CMND_REPORT_ABS_POS;
    to_serialise.updated = 1;
    to_serialise.axis = axis;
    to_serialise.value = config_c1.axis[axis].abs_pos;
    if(multicore_fifo_push_timeout_us(((uint32_t*)(&to_serialise))[0], 150) == false) {
      printf("FAIL: core1 push fail for axis %u\n", axis);
      break;
    }
    if(multicore_fifo_push_timeout_us(((uint32_t*)(&to_serialise))[1], 150) == false) {
      printf("FAIL: core1 push fail for axis %u\n", axis);
      break;
    }

    to_serialise.padding = 0;
    to_serialise.command = CMND_REPORT_VELOCITY;
    to_serialise.updated = 1;
    to_serialise.axis = axis;
    to_serialise.value = config_c1.axis[axis].velocity;

    if(multicore_fifo_push_timeout_us(((uint32_t*)(&to_serialise))[0], 150) == false) {
      printf("FAIL: core0 push fail for axis %u\n", axis);
      break;
    }
    if(multicore_fifo_push_timeout_us(((uint32_t*)(&to_serialise))[1], 150) == false) {
      printf("FAIL: core0 push fail for axis %u\n", axis);
      break;
    }
  }
}

/* Handle data from core0 FIFO. */
void core1_payload() {
  static size_t last_time = 0;
  static size_t last_sync_time = 0;
  static const size_t buffer_in_len = sizeof(struct AxisUpdate) / sizeof(uint32_t) * MAX_AXIS;
  static struct AxisUpdate buffer_in[MAX_AXIS];
  static size_t buffer_in_point = 0;
  static size_t update_point = 0;
  static size_t fail_count = 0;
  static size_t sync_fail_count = 0;
  static struct Ring_buf_ave period_average_data;
  static uint32_t ave_period_us = 0;
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
    #if LOG_CORE1 == 1
    printf("-");
    #elif LOG_CORE1 == 2
    printf("WARN: core1_payload called with no new data.\n");
    #endif // LOG_CORE1
    return;
  }

  if(update_point == 0) {
    // Starting processing a new data set.
    start_time = time_us_64();
    if(last_time == 0) {
      period_average_data = ring_buf_ave_default;
    } else {
      ave_period_us = ring_buf_ave(&period_average_data, start_time - last_time);
    }
  }
  //printf("*\t%lu\t%lu\t%lu\t%lu\n", update_point, start_time, last_time, ave_period_us);

  #if LOG_CORE1 == 2
  printf("core1.FIFO_irq start.\n");
  #endif // LOG_CORE1

  struct AxisUpdate update;
  while(multicore_fifo_pop_timeout_us(1, &raw) && buffer_in_point < buffer_in_len) {
    ((uint32_t*)&update)[buffer_in_point % SIZOF_AXIS_UPDATE] = raw;
    buffer_in_point++;
    if(buffer_in_point % (SIZOF_AXIS_UPDATE) == 0) {
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

  if(buffer_in_point < buffer_in_len) {
    return;
  }

  if(fail) {
    // Drain remaining incoming entries and reset everything before restart.
    multicore_fifo_drain();
    fail_count++;
  }

  buffer_in_point = 0;
  update_point = 0;

  // Calculate and cache the step lengths.
  uint32_t step_lens[MAX_AXIS][MAX_STEPS_PER_UPDATE] = {0};
  
  //uint32_t ave_period_ticks =
  //  ave_period_us * config_c1.update_time_ticks / config_c1.update_time_us;

  if(queue_get_level(&axis_mvmnt_c1) == MAX_AXIS) {
    for(uint8_t axis = 0; axis < MAX_AXIS; axis++) {
      core1_process_updates(&axis_mvmnt_c1, axis, step_lens[axis], ave_period_us);
    }
  }

  core1_send_to_core0();

  // Profiling output.
  #ifdef LOG_CORE1
  size_t now = time_us_64();
  #endif // LOG_CORE1
  #if LOG_CORE1 == 1
  printf("\t\t\t\t\t\tc1\t%4lu%6lu%6lu%6lu%6lu\n",
      start_time - last_time,
      now - start_time,
      fail_count,
      sync_fail_count,
      ave_period_us);
  #elif LOG_CORE1 == 2
  printf("core1.FIFO_irq:\t\t%lu\t%lu\n", start_time - last_time, now - start_time);
  #endif // LOG_CORE1
  last_time = start_time;

  if(start_time - last_sync_time > ave_period_us * 2)
  {
    sync_fail_count++;
    printf("WARN: syncfail: %lu\t%lu\t%lu\t%lu\t%lu\n",
        last_sync_time, start_time, start_time - last_sync_time, ave_period_us, sync_fail_count);
  }
  last_sync_time = start_time;
}

void core1_entry() {
  // Infinite While Loop to wait for interrupt
  while (1){
    core1_payload();
    sleep_us(100);
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
  //size_t start_time = systick_hw->cvr;

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
      fail_count);
  #elif LOG_CORE0 == 2
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
    while(multicore_fifo_pop_timeout_us(1, &raw_0) &&
        multicore_fifo_pop_timeout_us(1, &raw_1))
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

/* Push config from core0 to core1.
 * Done once at startup. */
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
  const uint32_t stepper_count = MAX_AXIS;
  const uint32_t pins_step[8] =      {0, 2, 4, 6, 8, 10, 12, 14};
  const uint32_t pins_direction[8] = {1, 3, 5, 7, 9, 11, 13, 15};

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
    const uint32_t pin_direction
    )
{
  static uint32_t offset_pio0 = 0;
  static uint32_t offset_pio1 = 0;
  static uint8_t init_done = 0;

  if(init_done == 0)
  {
    offset_pio0 = pio_add_program(pio0, &step_repeated_program);
    offset_pio1 = pio_add_program(pio1, &step_repeated_program);
    init_done = 1;
  }

  uint32_t sm;
  switch (stepper) {
    case 0:
    case 1:
    case 2:
    case 3:
      sm = pio_claim_unused_sm(pio0, true);
      // From pico_stepper.pio
      step_repeated_program_init(pio0, sm, offset_pio0, pin_step);
      pio_sm_set_enabled(pio0, sm, true);
      break;
    case 4:
    case 5:
    case 6:
    case 7:
      sm = pio_claim_unused_sm(pio1, true);
      // From pico_stepper.pio
      step_repeated_program_init(pio1, sm, offset_pio0, pin_step);
      pio_sm_set_enabled(pio1, sm, true);
      break;
    default:
      printf("WARN: Invalid stepper index: %ld\n", stepper);
  }
}

void send_pio_steps(const uint32_t axis, int32_t desired_pos) {
  // Queue axis movement data;
  axis_mvmnt_c0[axis] = (struct AxisUpdate){
      .padding = 0,
      .command = CMND_SET_DESIRED_POS,
      .updated = 1,
      .axis = axis,
      .value = desired_pos
  };

  return;
}

void set_relative_position(const uint32_t stepper, const int32_t position_diff) {
  if(stepper < MAX_AXIS) {
    uint32_t new_position = config_c0.axis[stepper].abs_pos + position_diff;
    send_pio_steps(stepper, new_position);
  }
  return;
}

void set_absolute_position(const uint32_t stepper, const uint32_t new_position) {
  if(stepper < MAX_AXIS) {
    send_pio_steps(stepper, new_position);
  }
  return;
}

uint32_t get_absolute_position(uint32_t stepper) {
  if(stepper >= MAX_AXIS) {
    return 0;
  }
  return config_c0.axis[stepper].abs_pos;
}

void set_max_speed(
    const uint32_t stepper,
    const uint32_t max_speed_step_sec
    ) {
  if(stepper < MAX_AXIS) {
    config_c0.axis[stepper].min_step_len_ticks = speed_to_step_ticks(max_speed_step_sec);
  }
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
    uint8_t* msg_machine,
    size_t* msg_machine_len,
    size_t msg_machine_len_max)
{
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
    uint8_t* msg_machine,
    size_t* msg_machine_len,
    size_t msg_machine_len_max)
{
  if(axis >= MAX_AXIS) {
    return;
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
    uint8_t* msg_machine,
    size_t* msg_machine_len,
    size_t msg_machine_len_max)
{
  if(axis >= MAX_AXIS) {
    return;
  }

  if(config_c0.axis[axis].updated) {
    get_axis_config(axis, msg_machine, msg_machine_len, msg_machine_len_max);
    config_c0.axis[axis].updated = 0;
  }
}

void get_axis_pos(
    const uint32_t axis,
    uint8_t* msg_machine,
    size_t* msg_machine_len,
    size_t msg_machine_len_max)
{
  if(axis >= MAX_AXIS) {
    return;
  }

  if(*msg_machine_len + sizeof(struct Reply_axis_pos) <= msg_machine_len_max) {
    struct Reply_axis_pos reply = Reply_axis_pos_default;
    reply.axis = axis;
    reply.abs_pos = config_c0.axis[axis].abs_pos;

    memcpy(msg_machine + *msg_machine_len, &reply, sizeof(struct Reply_axis_pos));
    *msg_machine_len += sizeof(struct Reply_axis_pos);
  }
}
