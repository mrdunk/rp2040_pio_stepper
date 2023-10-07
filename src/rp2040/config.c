#include <stdio.h>
#include <string.h>
#include "pico/mutex.h"
#include "port_common.h"

#include "config.h"
#include "messages.h"

// Mutexes for locking the main config which is shared between cores.
mutex_t mtx_top;
mutex_t mtx_axis[MAX_AXIS];

volatile uint32_t tick = 0;

volatile struct ConfigGlobal config = {
  .last_update_id = 0,
  .last_update_time = 0,
  .update_time_us = 1000,    // 1000us.
  .axis = {
    {
      // Axis 0.
      .updated_from_c0 = 0,
      .updated_from_c1 = 0,
      .enabled = 0,
      .io_pos_step = -1,
      .io_pos_dir = -1,
      .abs_pos_requested = UINT_MAX / 2,
      .abs_pos_acheived = UINT_MAX / 2,
      .min_step_len_ticks = 50,
      .max_accel_ticks = 2,
      .velocity_requested = 0,
      .velocity_acheived = 0,
      .kp = 0.5f,
    },
    {
      // Axis 1.
      .updated_from_c0 = 0,
      .updated_from_c1 = 0,
      .enabled = 0,
      .io_pos_step = -1,
      .io_pos_dir = -1,
      .abs_pos_requested = UINT_MAX / 2,
      .abs_pos_acheived = UINT_MAX / 2,
      .min_step_len_ticks = 50,
      .max_accel_ticks = 10,
      .velocity_requested = 0,
      .velocity_acheived = 0,
      .kp = 0.5f,
    },
    {
      // Axis 2.
      .updated_from_c0 = 0,
      .updated_from_c1 = 0,
      .enabled = 0,
      .io_pos_step = -1,
      .io_pos_dir = -1,
      .abs_pos_requested = UINT_MAX / 2,
      .abs_pos_acheived = UINT_MAX / 2,
      .min_step_len_ticks = 50,
      .max_accel_ticks = 200,
      .velocity_requested = 0,
      .velocity_acheived = 0,
      .kp = 0.5f,
    },
    {
      // Axis 3.
      .updated_from_c0 = 0,
      .updated_from_c1 = 0,
      .enabled = 0,
      .io_pos_step = -1,
      .io_pos_dir = -1,
      .abs_pos_requested = UINT_MAX / 2,
      .abs_pos_acheived = UINT_MAX / 2,
      .min_step_len_ticks = 50,
      .max_accel_ticks = 200,
      .velocity_requested = 0,
      .velocity_acheived = 0,
      .kp = 0.5f,
    },
  }
};


void init_config()
{
  mutex_init(&mtx_top);
  for(uint8_t axis = 0; axis < MAX_AXIS; axis++) {
    mutex_init(&mtx_axis[axis]);
  }
}

/* Update the period of the main timing loop.
 * This should closely match the rate at which we receive axis position data. */
void update_period(uint32_t update_time_us) {
  mutex_enter_blocking(&mtx_top);

  config.update_time_us = update_time_us;

  mutex_exit(&mtx_top);
}

/* Get the period of the main timing loop.
 * This should closely match the rate at which we receive axis position data. */
uint32_t get_period() {
  mutex_enter_blocking(&mtx_top);

  uint32_t retval = config.update_time_us;

  mutex_exit(&mtx_top);

  return retval;
}

/* Set metrics for tracking successful update transmission and jitter. */
uint32_t update_packet_metrics(
    uint32_t update_id, uint32_t time, int32_t* id_diff, int32_t* time_diff
) {
  mutex_enter_blocking(&mtx_top);

  *id_diff = update_id - config.last_update_id;
  *time_diff = time - config.last_update_time;


  if(*id_diff == 0) {
      printf("LinuxCNC started.\n");
  } else if(*id_diff != 1) {
    printf("WARNING: Updates out of sequence. %i %i %i\n",
        config.last_update_id, update_id, *id_diff);
    if(update_id == 0) {
      printf("Reason: LinuxCNC restarted.\n");
    } else if(config.last_update_id == 0) {
      printf("Reason: RP restarted.\n");
    }
  }

  config.last_update_id = update_id;
  config.last_update_time = time;

  mutex_exit(&mtx_top);
}

uint8_t has_new_c0_data(const uint8_t axis) {
  static uint32_t count = 0;
  mutex_enter_blocking(&mtx_axis[axis]);
  uint8_t updated = config.axis[axis].updated_from_c0;

  /*
  if(updated == 0) {
    volatile struct Ring_buf* abs_pos_requested_buf = &config.axis[axis].abs_pos_requested_buf;
    updated = abs_pos_requested_buf->len;
    if(count++ % 10000 == 0) {
      printf("%u\n", updated);
    }
  }
  */
  mutex_exit(&mtx_axis[axis]);
  return updated;
}

void update_axis_config(
    const uint8_t axis,
    const uint8_t core,
    const uint8_t* enabled,
    const int8_t* io_pos_step,
    const int8_t* io_pos_dir,
    const uint32_t* abs_pos_requested,
    const uint32_t* abs_pos_acheived,
    const uint32_t* min_step_len_ticks,
    const uint32_t* max_accel_ticks,
    const int32_t* velocity_requested,
    const int32_t* velocity_acheived,
    const float* kp
)
{
  if(axis >= MAX_AXIS) {
    return;
  }

  // printf("Setting CORE%i:%i\n", core, axis);
  mutex_enter_blocking(&mtx_axis[axis]);

  if(enabled != NULL) {
    config.axis[axis].enabled = *enabled;
  }
  if(io_pos_step != NULL) {
    config.axis[axis].io_pos_step = *io_pos_step;
  }
  if(io_pos_dir != NULL) {
    config.axis[axis].io_pos_dir = *io_pos_dir;
  }
  if(abs_pos_requested != NULL) {
    config.axis[axis].abs_pos_requested = *abs_pos_requested;
    //volatile struct Ring_buf* abs_pos_requested_buf = &config.axis[axis].abs_pos_requested_buf;
    //ring_buf_push(abs_pos_requested_buf, *abs_pos_requested);
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
  if(velocity_requested != NULL) {
    config.axis[axis].velocity_requested = *velocity_requested;
  }
  if(velocity_acheived != NULL) {
    config.axis[axis].velocity_acheived = *velocity_acheived;
  }
  if(kp != NULL) {
    config.axis[axis].kp = *kp;
  }

  switch(core) {
    case CORE0:
      config.axis[axis].updated_from_c0++;
      break;
    case CORE1:
      config.axis[axis].updated_from_c1++;
      break;
  }

  mutex_exit(&mtx_axis[axis]);
}


uint32_t get_axis_config(
    const uint8_t axis,
    const uint8_t core,
    uint8_t* enabled,
    int8_t* io_pos_step,
    int8_t* io_pos_dir,
    uint32_t* abs_pos_requested,
    uint32_t* abs_pos_acheived,
    uint32_t* min_step_len_ticks,
    uint32_t* max_accel_ticks,
    int32_t* velocity_requested,
    int32_t* velocity_acheived,
    float* kp)
{
  if(axis >= MAX_AXIS) {
    return 0;
  }

  mutex_enter_blocking(&mtx_axis[axis]);
  //if(mutex_try_enter(&mtx_axis[axis], NULL) == false) {
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

  if(enabled != NULL) {
    *enabled = config.axis[axis].enabled;
  }
  if(io_pos_step != NULL) {
    *io_pos_step = config.axis[axis].io_pos_step;
  }
  if(io_pos_dir != NULL) {
    *io_pos_dir = config.axis[axis].io_pos_dir;
  }
  if(abs_pos_requested != NULL) {
    *abs_pos_requested = config.axis[axis].abs_pos_requested;
    //volatile struct Ring_buf* abs_pos_requested_buf = &config.axis[axis].abs_pos_requested_buf;
    //*abs_pos_requested = ring_buf_pop(abs_pos_requested_buf);
  }
  if(abs_pos_acheived != NULL) {
    *abs_pos_acheived = config.axis[axis].abs_pos_acheived;
  }
  if(min_step_len_ticks != NULL) {
    *min_step_len_ticks = config.axis[axis].min_step_len_ticks;
  }
  if(max_accel_ticks != NULL) {
    *max_accel_ticks = config.axis[axis].max_accel_ticks;
  }
  if(velocity_requested != NULL) {
    *velocity_requested = config.axis[axis].velocity_requested;
  }
  if(velocity_acheived != NULL) {
    *velocity_acheived = config.axis[axis].velocity_acheived;
  }
  if(kp != NULL) {
    *kp = config.axis[axis].kp;
  }

  mutex_exit(&mtx_axis[axis]);

  return updated_by_other_core;
}

/* Serialise metrics stored in global config in a format for sending over UDP. */
size_t serialise_metrics(uint8_t* tx_buf, size_t* tx_buf_len, int32_t update_id, int32_t time_diff) {
	size_t max_buf_len = DATA_BUF_SIZE - sizeof(uint32_t);

  if(*tx_buf_len + sizeof(struct Reply_metrics) <= max_buf_len) {
    struct Reply_metrics reply = Reply_metrics_default;
    reply.update_id = update_id;
    reply.time_diff = time_diff;
    reply.rp_update_len = get_period();

    memcpy(tx_buf + *tx_buf_len, &reply, sizeof(struct Reply_metrics));
    *tx_buf_len += sizeof(struct Reply_metrics);

    return 1;
  }
  return 0;
}


/* Serialise data stored in global config in a format for sending over UDP. */
size_t serialise_axis_config(
    const uint32_t axis,
    uint8_t* tx_buf,
    size_t* tx_buf_len,
    uint8_t wait_for_data)
{
  if(axis >= MAX_AXIS) {
    printf("ERROR: Invalid axis: %u\n", axis);
    return 0;
  }

  static uint32_t failcount = 0;
  //static uint32_t count = 0;

	size_t max_buf_len = DATA_BUF_SIZE - sizeof(uint32_t);

  //uint8_t enabled;
  //int8_t io_pos_step;
  //int8_t io_pos_dir;
  uint32_t abs_pos_acheived;
  //uint32_t abs_pos_requested;
  uint32_t min_step_len_ticks;
  uint32_t max_accel_ticks;
  int32_t velocity_requested;
  int32_t velocity_acheived;
  //float kp;
  uint32_t updated = 0;

  do {
    updated = get_axis_config(
        axis,
        CORE0,
        NULL, //&enabled,
        NULL, //&io_pos_step,
        NULL, //&io_pos_dir,
        NULL, //&abs_pos_requested,
        &abs_pos_acheived,
        &min_step_len_ticks,
        &max_accel_ticks,
        &velocity_requested,
        &velocity_acheived,
        NULL //&kp
        );
  } while(updated == 0 && wait_for_data);

  //count++;
  if(updated > 1) {
    failcount++;
    //printf("WC0, mult ud: %u \t%lu \t%f\n",
    //    axis, updated, (double)failcount / (double)count);
    //printf("WC0, mult ud: %u \t%lu\n", axis, updated);
  }

  if(*tx_buf_len + sizeof(struct Reply_axis_config) <= max_buf_len) {
    struct Reply_axis_config reply = Reply_axis_config_default;
    reply.axis = axis;
    reply.abs_pos_acheived = abs_pos_acheived;
    reply.min_step_len_ticks = min_step_len_ticks;
    reply.max_accel_ticks = max_accel_ticks;
    reply.velocity_requested = velocity_requested;
    reply.velocity_acheived = velocity_acheived;

    memcpy(tx_buf + *tx_buf_len, &reply, sizeof(struct Reply_axis_config));
    *tx_buf_len += sizeof(struct Reply_axis_config);

    return 1;
  }
  return 0;
}


/* A ring buffer that returns the average value of it's contents.
 * Used for calculating average period between incoming network updates. */
size_t ring_buf_ave(struct Ring_buf_ave* data, const uint32_t new_val) {
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

void ring_buf_push(volatile struct Ring_buf* data, uint32_t new_val) {
  data->buf[data->head] = new_val;

  if(data->len == RING_BUF_LEN) {
    // We are overwriting tail.
    printf("WARN: Buffer full.\n");
    data->tail++;
    if(data->tail >= RING_BUF_LEN) {
      data->tail = 0;
    }
  } else {
    data->len++;
  }

  data->head++;
  if(data->head >= RING_BUF_LEN) {
    data->head = 0;
  }
  //static uint32_t count = 0;
  //if(count++ % 1000 == 0) {
  //  printf("pu %u\t%u\t%u\t%u\n", data, data->head, data->tail, data->len);
  //}
}

uint32_t ring_buf_pop(volatile struct Ring_buf* data) {
  if(data->len == 0) {
    // No data in buffer.
    // Return the last known value.
    printf("WARN: Buffer under run.\n");
    return data->buf[data->tail];
  }
  uint32_t val = data->buf[data->tail];
  data->tail++;
  data->len--;
  if(data->tail >= RING_BUF_LEN) {
    data->tail = 0;
  }
  //printf("po %u\t%u\t%u\n", data->head, data->tail, data->len);
  return val;
}


