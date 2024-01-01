#include <stdio.h>
#include <string.h>


#ifdef BUILD_TESTS

#include "../test/mocks/rp_mocks.h"

#else  // BUILD_TESTS

#include "pico/mutex.h"
#include "port_common.h"

#endif  // BUILD_TESTS

#include "config.h"
#include "messages.h"
#include "buffer.h"

// Mutexes for locking the main config which is shared between cores.
mutex_t mtx_top;
mutex_t mtx_axis[MAX_AXIS];

// Semaphore for synchronizing cores.
volatile uint32_t tick = 0;

volatile struct ConfigGlobal config = {
  .last_update_id = 0,
  .last_update_time = 0,
  .update_time_us = 1000,    // 1000us.
  .pio_io_configured = false,
  .axis = {
    {
      // Axis 0.
      .updated_from_c0 = 0,
      .updated_from_c1 = 0,
      .enabled = 0,
      .io_pos_step = -1,
      .io_pos_dir = -1,
      .rel_pos_requested = 0,
      .abs_pos_requested = 0,
      .abs_pos_acheived = 0,
      .max_velocity = 50,
      .max_accel_ticks = 2.0,
      .velocity_requested = 0,
      .velocity_acheived = 0,
      .kp = 0.2f,
    },
    {
      // Axis 1.
      .updated_from_c0 = 0,
      .updated_from_c1 = 0,
      .enabled = 0,
      .io_pos_step = -1,
      .io_pos_dir = -1,
      .rel_pos_requested = 0,
      .abs_pos_requested = 0,
      .abs_pos_acheived = 0,
      .max_velocity = 50,
      .max_accel_ticks = 10.0,
      .velocity_requested = 0,
      .velocity_acheived = 0,
      .kp = 0.2f,
    },
    {
      // Axis 2.
      .updated_from_c0 = 0,
      .updated_from_c1 = 0,
      .enabled = 0,
      .io_pos_step = -1,
      .io_pos_dir = -1,
      .rel_pos_requested = 0,
      .abs_pos_requested = 0,
      .abs_pos_acheived = 0,
      .max_velocity = 50,
      .max_accel_ticks = 200.0,
      .velocity_requested = 0,
      .velocity_acheived = 0,
      .kp = 0.2f,
    },
    {
      // Axis 3.
      .updated_from_c0 = 0,
      .updated_from_c1 = 0,
      .enabled = 0,
      .io_pos_step = -1,
      .io_pos_dir = -1,
      .rel_pos_requested = 0,
      .abs_pos_requested = 0,
      .abs_pos_acheived = 0,
      .max_velocity = 50,
      .max_accel_ticks = 200.0,
      .velocity_requested = 0,
      .velocity_acheived = 0,
      .kp = 0.2f,
    },
  }
};


void init_gpio() {
  for(uint16_t gpio = 0; gpio < MAX_GPIO; gpio++) {
    config.gpio[gpio].type = GPIO_TYPE_NOT_SET;
    config.gpio[gpio].index = 0;
    config.gpio[gpio].address = 0;
  }
}

void init_config()
{
  init_gpio();

  mutex_init(&mtx_top);

  for(uint8_t axis = 0; axis < MAX_AXIS; axis++) {
    mutex_init(&mtx_axis[axis]);
  }
}

/* Update the period of the main timing loop.
 * This should closely match the rate at which we receive axis position data. */
void update_period(uint32_t update_time_us) {
  // TODO: The mutex probably isn't needed here.
  // Remove and test.
  mutex_enter_blocking(&mtx_top);

  config.update_time_us = update_time_us;

  mutex_exit(&mtx_top);
}

/* Get the period of the main timing loop.
 * This should closely match the rate at which we receive axis position data. */
uint32_t get_period() {
  // TODO: The mutex probably isn't needed here.
  // Remove and test.
  mutex_enter_blocking(&mtx_top);

  uint32_t retval = config.update_time_us;

  mutex_exit(&mtx_top);

  return retval;
}

/* Set metrics for tracking successful update transmission and jitter. */
void update_packet_metrics(
    struct Message_timing* message,
    int32_t* id_diff,
    int32_t* time_diff
) {
  //static bool out_of_sequence = false;

  mutex_enter_blocking(&mtx_top);

  *id_diff = message->update_id - config.last_update_id;
  *time_diff = message->time - config.last_update_time;

  /*
  if(*id_diff == 0) {
      printf("LinuxCNC started.\n");
  } else if(*id_diff != 1) {
    if(! out_of_sequence) {
      printf("WARNING: Updates out of sequence. %i %i %i\n",
          config.last_update_id, update_id, *id_diff);
      if(update_id == 0) {
        printf("Reason: LinuxCNC restarted.\n");
      } else if(config.last_update_id == 0) {
        printf("Reason: RP restarted.\n");
      }
      out_of_sequence = true;
    }
  } else if(out_of_sequence) {
    printf("       Recovered. %i\n", update_id);
    out_of_sequence = false;
  }
  */

  config.last_update_id = message->update_id;
  config.last_update_time = message->time;

  mutex_exit(&mtx_top);
}

uint8_t has_new_c0_data(const uint8_t axis) {
  mutex_enter_blocking(&mtx_axis[axis]);
  uint8_t updated = config.axis[axis].updated_from_c0;

  mutex_exit(&mtx_axis[axis]);
  return updated;
}

void update_axis_config(
    const uint8_t axis,
    const uint8_t core,
    const uint8_t* enabled,
    const int8_t* io_pos_step,
    const int8_t* io_pos_dir,
    const double* rel_pos_requested,
    const double* abs_pos_requested,
    const int32_t* abs_pos_acheived,
    const double* max_velocity,
    const double* max_accel_ticks,
    const int32_t* velocity_requested,
    const int32_t* velocity_acheived,
    const int32_t* pos_error,
    const int32_t* step_len_ticks,
    const float* kp
)
{
  if(axis >= MAX_AXIS) {
    return;
  }

  //printf("Setting CORE%i:%i\n", core, axis);
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
  if(rel_pos_requested != NULL) {
    config.axis[axis].rel_pos_requested = *rel_pos_requested;
  }
  if(abs_pos_requested != NULL) {
    config.axis[axis].abs_pos_requested = *abs_pos_requested;
  }
  if(abs_pos_acheived != NULL) {
    config.axis[axis].abs_pos_acheived = *abs_pos_acheived;
  }
  if(max_velocity != NULL) {
    config.axis[axis].max_velocity = *max_velocity;
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
  if(pos_error != NULL) {
    config.axis[axis].pos_error = *pos_error;
  }
  if(step_len_ticks != NULL) {
    config.axis[axis].step_len_ticks = *step_len_ticks;
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
    double* rel_pos_requested,
    double* abs_pos_requested,
    int32_t* abs_pos_acheived,
    double* max_velocity,
    double* max_accel_ticks,
    int32_t* velocity_requested,
    int32_t* velocity_acheived,
    int32_t* pos_error,
    int32_t* step_len_ticks,
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
  if(rel_pos_requested != NULL) {
    *rel_pos_requested = config.axis[axis].rel_pos_requested;
  }
  if(abs_pos_requested != NULL) {
    *abs_pos_requested = config.axis[axis].abs_pos_requested;
  }
  if(abs_pos_acheived != NULL) {
    *abs_pos_acheived = config.axis[axis].abs_pos_acheived;
  }
  if(max_velocity != NULL) {
    *max_velocity = config.axis[axis].max_velocity;
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
  if(pos_error != NULL) {
    *pos_error = config.axis[axis].pos_error;
  }
  if(step_len_ticks != NULL) {
    *step_len_ticks = config.axis[axis].step_len_ticks;
  }
  if(kp != NULL) {
    *kp = config.axis[axis].kp;
  }

  mutex_exit(&mtx_axis[axis]);

  return updated_by_other_core;
}

/* Serialise metrics stored in global config in a format for sending over UDP. */
bool serialise_timing(struct NWBuffer* tx_buf, int32_t update_id, int32_t time_diff) {
  struct Reply_timing reply;
  reply.type = REPLY_TIMING;
  reply.time_diff = time_diff;
  reply.rp_update_len = get_period();

  reply.update_id = update_id;

  uint16_t tx_buf_len = pack_nw_buff(tx_buf, &reply, sizeof(reply));

  if(!tx_buf_len) {
    printf("WARN: TX length greater than buffer size. %u\n", update_id);
    return false;
  }

  return true;
}

/* Serialise data stored in global config in a format for sending over UDP. */
bool serialise_axis_movement(
    const uint32_t axis,
    struct NWBuffer* tx_buf,
    uint8_t wait_for_data)
{
  if(axis >= MAX_AXIS) {
    printf("ERROR: Invalid axis: %u\n", axis);
    return false;
  }

  int32_t abs_pos_acheived;
  int32_t velocity_acheived;
  uint32_t updated = 0;

  do {
    updated = get_axis_config(
        axis,
        CORE0,
        NULL, //&enabled,
        NULL, //&io_pos_step,
        NULL, //&io_pos_dir,
        NULL, //&rel_pos_requested,
        NULL, //&abs_pos_requested,
        &abs_pos_acheived,
        NULL, //&max_velocity,
        NULL, //&max_accel_ticks,
        NULL, //&velocity_requested,
        &velocity_acheived,
        NULL, //&pos_error,
        NULL, //&step_len_ticks,
        NULL //&kp
        );
  } while(updated == 0 && wait_for_data);

  if(updated > 1) {
    printf("WC0, mult ud: %u \t%lu\n", axis, updated);
  }

  struct Reply_axis_movement reply;
  reply.type = REPLY_AXIS_MOVEMENT;
  reply.axis = axis;
  reply.abs_pos_acheived = abs_pos_acheived;
  reply.velocity_acheived = velocity_acheived;

  uint16_t tx_buf_len = pack_nw_buff(tx_buf, &reply, sizeof(reply));

  if(!tx_buf_len) {
    //printf("WARN: TX length greater than buffer size. %u\n", update_id);
    return false;
  }

  return true;
}

bool serialise_spindle_speed(struct NWBuffer* tx_buf, float speed)
{
  struct Reply_spindle_speed reply;
  reply.type = REPLY_SPINDLE_SPEED;
  reply.speed = speed;

  uint16_t tx_buf_len = pack_nw_buff(tx_buf, &reply, sizeof(reply));

  if(!tx_buf_len) {
    //printf("WARN: TX length greater than buffer size. %u\n", update_id);
    return false;
  }

  return true;
}

/* Serialise data stored in global config in a format for sending over UDP. */
bool serialise_axis_config(const uint32_t axis, struct NWBuffer* tx_buf) {
  if(axis >= MAX_AXIS) {
    printf("ERROR: Invalid axis: %u\n", axis);
    return false;
  }

  uint8_t enabled;
  int8_t io_pos_step;
  int8_t io_pos_dir;
  double max_velocity;
  double max_accel;

  get_axis_config(
        axis,
        CORE0,
        &enabled,
        &io_pos_step,
        &io_pos_dir,
        NULL, //&rel_pos_requested,
        NULL, //&abs_pos_requested,
        NULL, //&abs_pos_acheived,
        &max_velocity,
        &max_accel,
        NULL, //&velocity_requested,
        NULL, //&velocity_acheived,
        NULL, //&pos_error,
        NULL, //&step_len_ticks,
        NULL //&kp
        );

  struct Reply_axis_config reply;
  reply.type = REPLY_AXIS_CONFIG;
  reply.axis = axis;
  reply.enable = enabled;
  reply.gpio_step = io_pos_step;
  reply.gpio_dir = io_pos_dir;
  reply.max_velocity = max_velocity;
  reply.max_accel = max_accel;

  uint16_t tx_buf_len = pack_nw_buff(tx_buf, &reply, sizeof(reply));

  if(!tx_buf_len) {
    printf("WARN: TX length greater than buffer size.\n");
    return false;
  }

  return true;
}

/* Serialise data stored in global config in a format for sending over UDP. */
bool serialise_axis_metrics(const uint32_t axis, struct NWBuffer* tx_buf) {
  if(axis >= MAX_AXIS) {
    printf("ERROR: Invalid axis: %u\n", axis);
    return false;
  }

  int32_t velocity_requested;
  //int32_t pos_error;
  int32_t step_len_ticks;

  get_axis_config(
        axis,
        CORE0,
        NULL, //&enabled,
        NULL, //&io_pos_step,
        NULL, //&io_pos_dir,
        NULL, //&rel_pos_requested,
        NULL, //&abs_pos_requested,
        NULL, //&abs_pos_acheived,
        NULL, //&max_velocity,
        NULL, //&max_accel,
        &velocity_requested,
        NULL, //&velocity_acheived,
        NULL, //&pos_error,
        &step_len_ticks,
        NULL //&kp
        );

  struct Reply_axis_metrics reply;
  reply.type = REPLY_AXIS_METRICS;
  reply.axis = axis;
  reply.velocity_requested = velocity_requested;
  //reply.pos_error = pos_error;
  reply.step_len_ticks = step_len_ticks;

  uint16_t tx_buf_len = pack_nw_buff(tx_buf, &reply, sizeof(reply));

  if(!tx_buf_len) {
    printf("WARN: TX length greater than buffer size.\n");
    return false;
  }

  return true;
}

