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
#include "gpio.h"

// Mutexes for locking the main config which is shared between cores.
mutex_t mtx_top;
mutex_t mtx_joint[MAX_JOINT];

// Semaphore for synchronizing cores.
volatile uint32_t tick = 0;

volatile struct ConfigGlobal config = {
  .last_update_id = 0,
  .last_update_time = 0,
  .update_time_us = 1000,    // 1000us.
  .pio_io_configured = false,
  .joint = {
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
      .max_accel = 2.0,
      .velocity_requested = 0,
      .velocity_acheived = 0
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
      .max_accel = 10.0,
      .velocity_requested = 0,
      .velocity_acheived = 0
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
      .max_accel = 200.0,
      .velocity_requested = 0,
      .velocity_acheived = 0
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
      .max_accel = 200.0,
      .velocity_requested = 0,
      .velocity_acheived = 0
    },
  }
};


void init_gpio() {
  for(uint16_t gpio = 0; gpio < MAX_GPIO; gpio++) {
    config.gpio[gpio].type = GPIO_TYPE_NOT_SET;
    config.gpio[gpio].index = 0;
    config.gpio[gpio].address = 0;
    config.gpio[gpio].value = false;
  }

  for(uint16_t bank = 0; bank < MAX_GPIO_BANK; bank++) {
    // Set confirmation to force an initial Reply_gpio to be sent.
    config.gpio_confirmation_pending[bank] = false;
  }
}

void init_config()
{
  init_gpio();

  mutex_init(&mtx_top);

  for(uint8_t joint = 0; joint < MAX_JOINT; joint++) {
    mutex_init(&mtx_joint[joint]);
  }
}

/* Update the period of the main timing loop.
 * This should closely match the rate at which we receive joint position data. */
void update_period(uint32_t update_time_us) {
  // TODO: The mutex probably isn't needed here.
  // Remove and test.
  mutex_enter_blocking(&mtx_top);

  config.update_time_us = update_time_us;

  mutex_exit(&mtx_top);
}

/* Get the period of the main timing loop.
 * This should closely match the rate at which we receive joint position data. */
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

uint8_t has_new_c0_data(const uint8_t joint) {
  mutex_enter_blocking(&mtx_joint[joint]);
  uint8_t updated = config.joint[joint].updated_from_c0;

  mutex_exit(&mtx_joint[joint]);
  return updated;
}

void update_joint_config(
    const uint8_t joint,
    const uint8_t core,
    const uint8_t* enabled,
    const int8_t* io_pos_step,
    const int8_t* io_pos_dir,
    const double* rel_pos_requested,
    const double* abs_pos_requested,
    const int32_t* abs_pos_acheived,
    const double* max_velocity,
    const double* max_accel,
    const int32_t* velocity_requested,
    const int32_t* velocity_acheived,
    const int32_t* step_len_ticks
)
{
  if(joint >= MAX_JOINT) {
    return;
  }

  //printf("Setting CORE%i:%i\n", core, joint);
  mutex_enter_blocking(&mtx_joint[joint]);

  if(enabled != NULL) {
    config.joint[joint].enabled = *enabled;
  }
  if(io_pos_step != NULL) {
    config.joint[joint].io_pos_step = *io_pos_step;
  }
  if(io_pos_dir != NULL) {
    config.joint[joint].io_pos_dir = *io_pos_dir;
  }
  if(rel_pos_requested != NULL) {
    config.joint[joint].rel_pos_requested = *rel_pos_requested;
  }
  if(abs_pos_requested != NULL) {
    config.joint[joint].abs_pos_requested = *abs_pos_requested;
  }
  if(abs_pos_acheived != NULL) {
    config.joint[joint].abs_pos_acheived = *abs_pos_acheived;
  }
  if(max_velocity != NULL) {
    config.joint[joint].max_velocity = *max_velocity;
  }
  if(max_accel != NULL) {
    config.joint[joint].max_accel = *max_accel;
  }
  if(velocity_requested != NULL) {
    config.joint[joint].velocity_requested = *velocity_requested;
  }
  if(velocity_acheived != NULL) {
    config.joint[joint].velocity_acheived = *velocity_acheived;
  }
  if(step_len_ticks != NULL) {
    config.joint[joint].step_len_ticks = *step_len_ticks;
  }

  switch(core) {
    case CORE0:
      config.joint[joint].updated_from_c0++;
      break;
    case CORE1:
      config.joint[joint].updated_from_c1++;
      break;
  }

  mutex_exit(&mtx_joint[joint]);
}


uint32_t get_joint_config(
    const uint8_t joint,
    const uint8_t core,
    uint8_t* enabled,
    int8_t* io_pos_step,
    int8_t* io_pos_dir,
    double* rel_pos_requested,
    double* abs_pos_requested,
    int32_t* abs_pos_acheived,
    double* max_velocity,
    double* max_accel,
    int32_t* velocity_requested,
    int32_t* velocity_acheived,
    int32_t* step_len_ticks)
{
  if(joint >= MAX_JOINT) {
    return 0;
  }

  mutex_enter_blocking(&mtx_joint[joint]);
  //if(mutex_try_enter(&mtx_joint[joint], NULL) == false) {
  //  return 0;
  //}

  int32_t updated_by_other_core;
  switch(core) {
    case CORE0:
      updated_by_other_core = config.joint[joint].updated_from_c1;
      config.joint[joint].updated_from_c1 = 0;
      break;
    case CORE1:
      updated_by_other_core = config.joint[joint].updated_from_c0;
      config.joint[joint].updated_from_c0 = 0;
      break;
  }

  if(enabled != NULL) {
    *enabled = config.joint[joint].enabled;
  }
  if(io_pos_step != NULL) {
    *io_pos_step = config.joint[joint].io_pos_step;
  }
  if(io_pos_dir != NULL) {
    *io_pos_dir = config.joint[joint].io_pos_dir;
  }
  if(rel_pos_requested != NULL) {
    *rel_pos_requested = config.joint[joint].rel_pos_requested;
  }
  if(abs_pos_requested != NULL) {
    *abs_pos_requested = config.joint[joint].abs_pos_requested;
  }
  if(abs_pos_acheived != NULL) {
    *abs_pos_acheived = config.joint[joint].abs_pos_acheived;
  }
  if(max_velocity != NULL) {
    *max_velocity = config.joint[joint].max_velocity;
  }
  if(max_accel != NULL) {
    *max_accel = config.joint[joint].max_accel;
  }
  if(velocity_requested != NULL) {
    *velocity_requested = config.joint[joint].velocity_requested;
  }
  if(velocity_acheived != NULL) {
    *velocity_acheived = config.joint[joint].velocity_acheived;
  }
  if(step_len_ticks != NULL) {
    *step_len_ticks = config.joint[joint].step_len_ticks;
  }

  mutex_exit(&mtx_joint[joint]);

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
bool serialise_joint_movement(
    struct NWBuffer* tx_buf,
    uint8_t wait_for_data)
{
  int32_t abs_pos_acheived;
  int32_t velocity_acheived;
  uint32_t updated = 0;


  struct Reply_joint_movement reply;
  reply.type = REPLY_JOINT_MOVEMENT;

  for(size_t joint = 0; joint < MAX_JOINT; joint++) {
    do {
      updated = get_joint_config(
          joint,
          CORE0,
          NULL, //&enabled,
          NULL, //&io_pos_step,
          NULL, //&io_pos_dir,
          NULL, //&rel_pos_requested,
          NULL, //&abs_pos_requested,
          &abs_pos_acheived,
          NULL, //&max_velocity,
          NULL, //&max_accel,
          NULL, //&velocity_requested,
          &velocity_acheived,
          NULL //&step_len_ticks,
          );
    } while(updated == 0 && wait_for_data);

    if(updated > 1) {
      printf("WC0, mult ud: %u \t%lu\n", joint, updated);
    }

    reply.abs_pos_acheived[joint] = abs_pos_acheived;
    reply.velocity_acheived[joint] = velocity_acheived;
  }

  uint16_t tx_buf_len = pack_nw_buff(tx_buf, &reply, sizeof(reply));

  if(!tx_buf_len) {
    //printf("WARN: TX length greater than buffer size. %u\n", update_id);
    return false;
  }

  return true;
}

bool serialise_spindle_speed_out(
    struct NWBuffer* tx_buf, float speed, struct vfd_stats *vfd_stats)
{
  struct Reply_spindle_speed reply;
  reply.type = REPLY_SPINDLE_SPEED;

  reply.spindle_index = 0;
  reply.speed = speed;
  reply.crc_errors = vfd_stats->crc_errors;
  reply.unanswered = vfd_stats->unanswered;
  reply.unknown = vfd_stats->unknown;
  reply.got_status = vfd_stats->got_status;
  reply.got_set_frequency = vfd_stats->got_set_frequency;
  reply.got_act_frequency = vfd_stats->got_act_frequency;

  uint16_t tx_buf_len = pack_nw_buff(tx_buf, &reply, sizeof(reply));

  if(!tx_buf_len) {
    //printf("WARN: TX length greater than buffer size. %u\n", update_id);
    return false;
  }

  return true;
}

/* Serialise data stored in vfd_config in a format for sending over UDP. */
bool serialise_spindle_config(size_t spindle, struct NWBuffer* tx_buf) {
  if(spindle > MAX_SPINDLE) {
    printf("ERROR: Invalid spindle: %u\n", spindle);
    return false;
  }

  struct Reply_spindle_config reply;
  reply.type = REPLY_SPINDLE_CONFIG;
  reply.spindle_index = spindle;
  reply.modbus_address = vfd_config.address;
  reply.bitrate = vfd_config.bitrate;
  reply.vfd_type = vfd_config.type;

  uint16_t tx_buf_len = pack_nw_buff(tx_buf, &reply, sizeof(reply));

  if(!tx_buf_len) {
    printf("WARN: TX length greater than buffer size.\n");
    return false;
  }

  return true;
}

/* Serialise data stored in global config in a format for sending over UDP. */
bool serialise_joint_config(const uint32_t joint, struct NWBuffer* tx_buf) {
  if(joint >= MAX_JOINT) {
    printf("ERROR: Invalid joint: %u\n", joint);
    return false;
  }

  uint8_t enabled;
  int8_t io_pos_step;
  int8_t io_pos_dir;
  double max_velocity;
  double max_accel;

  get_joint_config(
        joint,
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
        NULL  //&step_len_ticks,
        );

  struct Reply_joint_config reply;
  reply.type = REPLY_JOINT_CONFIG;
  reply.joint = joint;
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
bool serialise_joint_metrics(struct NWBuffer* tx_buf) {
  struct Reply_joint_metrics reply;
  reply.type = REPLY_JOINT_METRICS;

  int32_t velocity_requested;
  int32_t step_len_ticks;

  for(size_t joint = 0; joint < MAX_JOINT; joint++) {
    get_joint_config(
        joint,
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
        &step_len_ticks
        );

    reply.velocity_requested[joint] = velocity_requested;
    reply.step_len_ticks[joint] = step_len_ticks;
  }

  uint16_t tx_buf_len = pack_nw_buff(tx_buf, &reply, sizeof(reply));

  if(!tx_buf_len) {
    printf("WARN: TX length greater than buffer size.\n");
    return false;
  }

  return true;
}

