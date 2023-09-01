#include <stdio.h>
#include <string.h>
#include "pico/mutex.h"
#include "port_common.h"

#include "config.h"
#include "messages.h"

mutex_t mtx;

volatile struct ConfigGlobal config = {
  .update_time_us = 1000,    // 1000us.
  .axis = {
    {
      // Axis 0.
      .updated_from_c0 = 0,
      .updated_from_c1 = 0,
      .abs_pos_requested = UINT_MAX / 2,
      .abs_pos_acheived = UINT_MAX / 2,
      .min_step_len_ticks = 50,
      .max_accel_ticks = 200
    },
    {
      // Axis 1.
      .updated_from_c0 = 0,
      .updated_from_c1 = 0,
      .abs_pos_requested = UINT_MAX / 2,
      .abs_pos_acheived = UINT_MAX / 2,
      .min_step_len_ticks = 50,
      .max_accel_ticks = 200
    },
    {
      // Axis 2.
      .updated_from_c0 = 0,
      .updated_from_c1 = 0,
      .abs_pos_requested = UINT_MAX / 2,
      .abs_pos_acheived = UINT_MAX / 2,
      .min_step_len_ticks = 50,
      .max_accel_ticks = 200
    },
    {
      // Axis 3.
      .updated_from_c0 = 0,
      .updated_from_c1 = 0,
      .abs_pos_requested = UINT_MAX / 2,
      .abs_pos_acheived = UINT_MAX / 2,
      .min_step_len_ticks = 50,
      .max_accel_ticks = 200
    },
    {
      // Axis 4.
      .updated_from_c0 = 0,
      .updated_from_c1 = 0,
      .abs_pos_requested = UINT_MAX / 2,
      .abs_pos_acheived = UINT_MAX / 2,
      .min_step_len_ticks = 50,
      .max_accel_ticks = 200
    },
    {
      // Axis 5.
      .updated_from_c0 = 0,
      .updated_from_c1 = 0,
      .abs_pos_requested = UINT_MAX / 2,
      .abs_pos_acheived = UINT_MAX / 2,
      .min_step_len_ticks = 50,
      .max_accel_ticks = 200
    },
    {
      // Axis 6.
      .updated_from_c0 = 0,
      .updated_from_c1 = 0,
      .abs_pos_requested = UINT_MAX / 2,
      .abs_pos_acheived = UINT_MAX / 2,
      .min_step_len_ticks = 50,
      .max_accel_ticks = 200
    },
    {
      // Axis 7.
      .updated_from_c0 = 0,
      .updated_from_c1 = 0,
      .abs_pos_requested = UINT_MAX / 2,
      .abs_pos_acheived = UINT_MAX / 2,
      .min_step_len_ticks = 50,
      .max_accel_ticks = 200
    },
  }
};


void init_config()
{
  mutex_init(&mtx);
}

void update_config(uint32_t update_time_us) {
  mutex_enter_blocking(&mtx);

  config.update_time_us = update_time_us;
  
  mutex_exit(&mtx);
}

uint32_t get_config() {
  mutex_enter_blocking(&mtx);

  uint32_t retval = config.update_time_us;
  
  mutex_exit(&mtx);

  return retval;
}

void update_axis_config(
    const uint8_t axis,
    const uint8_t core,
    const uint32_t* abs_pos_requested,
    const uint32_t* abs_pos_acheived,
    const uint32_t* min_step_len_ticks,
    const uint32_t* max_accel_ticks,
    const int32_t* velocity_acheived)
{
  if(axis >= MAX_AXIS) {
    return;
  }

  mutex_enter_blocking(&mtx);

  if(abs_pos_requested != NULL) {
    config.axis[axis].abs_pos_requested = *abs_pos_requested;
  }
  if(abs_pos_acheived != NULL) {
    config.axis[axis].abs_pos_acheived = *abs_pos_acheived;
  }
  if(min_step_len_ticks != NULL) {
    config.axis[axis].min_step_len_ticks = *min_step_len_ticks;
  }
  if(max_accel_ticks != NULL) {
    config.axis[axis].max_accel_ticks = *max_accel_ticks;
  }
  if(velocity_acheived != NULL) {
    config.axis[axis].velocity_acheived = *velocity_acheived;
  }
  switch(core) {
    case CORE0:
      config.axis[axis].updated_from_c0++;
      break;
    case CORE1:
      config.axis[axis].updated_from_c1++;
      break;
  }

  mutex_exit(&mtx);
}


uint32_t get_axis_config(
    const uint8_t axis,
    const uint8_t core,
    uint32_t* abs_pos_requested,
    uint32_t* abs_pos_acheived,
    uint32_t* min_step_len_ticks,
    uint32_t* max_accel_ticks,
    int32_t* velocity_acheived)
{
  if(axis >= MAX_AXIS) {
    return 0;
  }

  mutex_enter_blocking(&mtx);
  //if(mutex_try_enter(&mtx, NULL) == false) {
  //  return 0;
  //}

  int32_t updated_by_other_core;
  switch(core) {
    case CORE0:
      updated_by_other_core = config.axis[axis].updated_from_c1;
      config.axis[axis].updated_from_c1 = 0;
      break;
    case CORE1:
      updated_by_other_core = config.axis[axis].updated_from_c0;
      config.axis[axis].updated_from_c0 = 0;
      break;
  }

  *abs_pos_requested = config.axis[axis].abs_pos_requested;
  *abs_pos_acheived = config.axis[axis].abs_pos_acheived;
  *min_step_len_ticks = config.axis[axis].min_step_len_ticks;
  *max_accel_ticks = config.axis[axis].max_accel_ticks;
  *velocity_acheived = config.axis[axis].velocity_acheived;


  mutex_exit(&mtx);

  return updated_by_other_core;
}

void serialise_axis_config(
    const uint32_t axis,
    uint8_t* msg_machine,
    size_t* msg_machine_len,
    size_t msg_machine_len_max,
    uint8_t always)
{
  if(axis >= MAX_AXIS) {
    return;
  }

  uint32_t abs_pos_acheived;
  uint32_t abs_pos_requested;
  uint32_t min_step_len_ticks;
  uint32_t max_accel_ticks;
  uint32_t velocity_acheived;
  uint32_t updated;

  updated = get_axis_config(
      axis,
      CORE0,
      &abs_pos_requested,
      &abs_pos_acheived,
      &min_step_len_ticks,
      &max_accel_ticks,
      &velocity_acheived
      );

  if(updated > 0) {
    if(updated > 1) {
      printf("WARN: C0, multiple updates: %u \t%lu\n", axis, updated);
    }
    printf("%u \t%lu\n", axis, abs_pos_acheived);
  } else if (always == false) {
    // No new data since last call.
    printf("No new data since last call.  %u \t%lu\n", axis, updated);
    return;
  }

  if(*msg_machine_len + sizeof(struct Reply_axis_config) <= msg_machine_len_max) {
    struct Reply_axis_config reply = Reply_axis_config_default;
    reply.axis = axis;
    reply.abs_pos_acheived = abs_pos_acheived;
    reply.min_step_len_ticks = min_step_len_ticks;
    reply.max_accel_ticks = max_accel_ticks;
    reply.velocity_acheived = velocity_acheived;

    memcpy(msg_machine + *msg_machine_len, &reply, sizeof(struct Reply_axis_config));
    *msg_machine_len += sizeof(struct Reply_axis_config);
  }
}


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

