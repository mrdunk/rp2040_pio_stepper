#include <stdio.h>
#include "pico/mutex.h"
#include "port_common.h"

#include "config.h"

mutex_t mtx;

volatile struct ConfigGlobal config = {
  .update_rate = 1000,       // 1kHz.
  .update_time_us = 1000,    // 1000us.
  .update_time_ticks = 133000,
  .axis = {
    {
      // Axis 0.
      .updated_c0= 0,
      .abs_pos = UINT_MAX / 2,
      .min_step_len_ticks = 50,
      .max_accel_ticks = 200
    },
    {
      // Axis 1.
      .updated_c0= 0,
      .abs_pos = UINT_MAX / 2,
      .min_step_len_ticks = 50,
      .max_accel_ticks = 200
    },
    {
      // Axis 2.
      .updated_c0= 0,
      .abs_pos = UINT_MAX / 2,
      .min_step_len_ticks = 50,
      .max_accel_ticks = 200
    },
    {
      // Axis 3.
      .updated_c0= 0,
      .abs_pos = UINT_MAX / 2,
      .min_step_len_ticks = 50,
      .max_accel_ticks = 200
    },
    {
      // Axis 4.
      .updated_c0= 0,
      .abs_pos = UINT_MAX / 2,
      .min_step_len_ticks = 50,
      .max_accel_ticks = 200
    },
    {
      // Axis 5.
      .updated_c0= 0,
      .abs_pos = UINT_MAX / 2,
      .min_step_len_ticks = 50,
      .max_accel_ticks = 200
    },
    {
      // Axis 6.
      .updated_c0= 0,
      .abs_pos = UINT_MAX / 2,
      .min_step_len_ticks = 50,
      .max_accel_ticks = 200
    },
    {
      // Axis 7.
      .updated_c0= 0,
      .abs_pos = UINT_MAX / 2,
      .min_step_len_ticks = 50,
      .max_accel_ticks = 200
    },
  }
};


void init_config()
{
  mutex_init(&mtx);
}

void update_axis(
    const uint8_t axis,
    const uint32_t* abs_pos,
    const uint32_t* min_step_len_ticks,
    const uint32_t* max_accel_ticks,
    const int32_t* velocity)
{
  if(axis >= MAX_AXIS) {
    return;
  }

  mutex_enter_blocking(&mtx);

  if(abs_pos != NULL) {
    config.axis[axis].abs_pos = *abs_pos;
    //printf("%u %u %lu\n", axis, *abs_pos == config.axis[axis].abs_pos, *abs_pos);
  }
  if(min_step_len_ticks != NULL) {
    config.axis[axis].min_step_len_ticks = *min_step_len_ticks;
  }
  if(max_accel_ticks != NULL) {
    config.axis[axis].max_accel_ticks = *max_accel_ticks;
  }
  if(velocity != NULL) {
    config.axis[axis].velocity = *velocity;
  }
  config.axis[axis].updated_c0 = 1;

  mutex_exit(&mtx);

  gpio_put(LED_PIN, (time_us_64() / 1000000) % 2);
}


uint32_t get_axis(
    const uint8_t axis,
    uint32_t* abs_pos,
    uint32_t* min_step_len_ticks,
    uint32_t* max_accel_ticks,
    int32_t* velocity)
{
  mutex_enter_blocking(&mtx);
  //if(mutex_try_enter(&mtx, NULL) == false) {
  //  return 0;
  //}

  int32_t updated_c0 = config.axis[axis].updated_c0;

  if(updated_c0 > 0) {
    *abs_pos = config.axis[axis].abs_pos;
    *min_step_len_ticks = config.axis[axis].min_step_len_ticks;
    *max_accel_ticks = config.axis[axis].max_accel_ticks;
    *velocity = config.axis[axis].velocity;
  }

  config.axis[axis].updated_c0 = 0;

  mutex_exit(&mtx);

  return updated_c0;
}

