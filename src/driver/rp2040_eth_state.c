/* rp2040_eth_state.c — HAL-free Ethernet state machine extracted from write_port().
 *
 * Include order (both production and test):
 *   1. rp2040_defines.h, messages.h, skeleton.h (and HAL type stubs in test builds)
 *   2. rp2040_network.c  — or test stubs providing the same symbols
 *   3. this file
 *
 * In production: hal_rp2040_eth.c includes rtapi.h, hal.h, skeleton.h,
 *   rp2040_network.c, then this file.
 * In tests: driver_eth_state_test.c includes driver_mocks.h, stub function
 *   definitions, then this file.
 */

#include <errno.h>
#include <stdio.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "../rp2040/modbus.h"    /* MODBUS_TYPE_NOT_SET */

#define MAX_SKIPPED_PACKETS 10

/* reset_rp_config is defined below on_eth_down but called from it. */
static void reset_rp_config(skeleton_t *data);

/* ---- module-level state (moved from write_port() static locals) ---- */

/* configure() diffs against these before sending. Initialised to {0} so the
 * first cycle always resends full config. On LinuxCNC restart the driver
 * process also restarts, reinitialising these to {0} — no RP2040-side action
 * required. On Ethernet-down, reset_rp_config() clears them to force resend
 * (including enable=false) when the link recovers. */
static struct Message_joint_config   last_joint_config[MAX_JOINT]   = {0};
static struct Message_gpio_config    last_gpio_config[MAX_GPIO]      = {0};
static struct Message_spindle_config last_spindle_config[MAX_SPINDLE]= {0};

static uint32_t last_update_id   = 0;
static int      last_errno       = 0;
static int      cooloff          = 0;
static int      send_fail_count  = 0;
static bool     waiting_logged   = false;
static size_t   last_confirmed   = (size_t)-1;


/* Reset all module state. Called from eth_state_reset() and indirectly by
 * on_eth_down() via reset_rp_config().  Must be kept in sync with the static
 * initialisers above. */
void eth_state_reset(void) {
    memset(last_joint_config,   0, sizeof(last_joint_config));
    memset(last_gpio_config,    0, sizeof(last_gpio_config));
    memset(last_spindle_config, 0, sizeof(last_spindle_config));
    last_update_id  = 0;
    last_errno      = 0;
    cooloff         = 0;
    send_fail_count = 0;
    waiting_logged  = false;
    last_confirmed  = (size_t)-1;
    reset_version_check();
}


/* ---- configure helpers (HAL-free; call serialize_* from rp2040_network.c) ---- */

static bool configure_joint(
    struct NWBuffer* tx_buffer,
    uint8_t joint,
    skeleton_t *data
) {
    bool pack_success = true;
    float max_velocity_ticks =
      (float)((*data->joint_vel_limit[joint]) * (*data->joint_scale[joint]));
    float max_accel_ticks =
      (float)((*data->joint_accel_limit[joint]) * (*data->joint_scale[joint]));
    /* Send if anything changed, or if no reply has arrived yet (last_joint_config
     * only updates in unpack_joint_config on receipt of REPLY_JOINT_CONFIG, so a
     * lost packet leaves the diff intact and causes an automatic retry). */
    if(
        last_joint_config[joint].enable != *data->joint_enable_cmd[joint]
        ||
        last_joint_config[joint].gpio_step != data->joint_gpio_step[joint]
        ||
        last_joint_config[joint].gpio_dir != data->joint_gpio_dir[joint]
        ||
        last_joint_config[joint].max_velocity != max_velocity_ticks
        ||
        last_joint_config[joint].max_accel != max_accel_ticks
        ||
        last_joint_config[joint].cmd_type != data->joint_cmd_type[joint]
      ) {
      pack_success = pack_success && serialize_joint_config(
          tx_buffer,
          joint,
          *data->joint_enable_cmd[joint],
          data->joint_gpio_step[joint],
          data->joint_gpio_dir[joint],
          max_velocity_ticks,
          max_accel_ticks,
          data->joint_cmd_type[joint]
          );
    }
    return pack_success;
}

static bool configure_gpio(
    struct NWBuffer* tx_buffer,
    uint8_t gpio,
    skeleton_t *data
) {
    bool pack_success = true;
    /* Send if anything changed, or if no reply has arrived yet (last_gpio_config
     * only updates in unpack_gpio_config on receipt of REPLY_GPIO_CONFIG, so a
     * lost packet leaves the diff intact and causes an automatic retry). */
    if(
        last_gpio_config[gpio].gpio_type != data->gpio_type[gpio]
        ||
        last_gpio_config[gpio].index != data->gpio_index[gpio]
        ||
        last_gpio_config[gpio].address != data->gpio_address[gpio]
    ) {
      pack_success = pack_success && serialize_gpio_config(
          tx_buffer,
          gpio,
          data->gpio_type[gpio],
          data->gpio_index[gpio],
          data->gpio_address[gpio]
        );
    }
    return pack_success;
}

static bool configure_spindle(
    struct NWBuffer* tx_buffer,
    uint8_t spindle,
    skeleton_t *data
) {
    if(data->spindle_vfd_type[spindle] == MODBUS_TYPE_NOT_SET) {
      return true;
    }
    if(spindle > 0) {
      printf("ERROR: More than one spindle not yet implemented.\n");
      return false;
    }

    bool pack_success = true;
    /* Send if anything changed, or if no reply has arrived yet (last_spindle_config
     * only updates in unpack_spindle_config on receipt of REPLY_SPINDLE_CONFIG, so a
     * lost packet leaves the diff intact and causes an automatic retry). */
    if(
        last_spindle_config[spindle].vfd_type != data->spindle_vfd_type[spindle]
        ||
        last_spindle_config[spindle].modbus_address != data->spindle_address[spindle]
        ||
        last_spindle_config[spindle].bitrate != data->spindle_bitrate[spindle]
    ) {
      pack_success = pack_success && serialise_spindle_config(
          tx_buffer,
          spindle,
          data->spindle_vfd_type[spindle],
          data->spindle_address[spindle],
          data->spindle_bitrate[spindle]
      );
    }
    return pack_success;
}

static size_t count_confirmed_configs(skeleton_t *data, int num_joints) {
  size_t confirmed = 0;
  for(int j = 0; j < num_joints; j++) {
    float vel = (float)((*data->joint_vel_limit[j]) * (*data->joint_scale[j]));
    float acc = (float)((*data->joint_accel_limit[j]) * (*data->joint_scale[j]));
    if(last_joint_config[j].gpio_step == data->joint_gpio_step[j]
    && last_joint_config[j].gpio_dir  == data->joint_gpio_dir[j]
    && last_joint_config[j].max_velocity == vel
    && last_joint_config[j].max_accel    == acc
    && last_joint_config[j].cmd_type  == data->joint_cmd_type[j]) {
      confirmed++;
    }
  }
  for(int g = 0; g < MAX_GPIO; g++) {
    if(last_gpio_config[g].gpio_type == data->gpio_type[g]
    && last_gpio_config[g].index     == data->gpio_index[g]
    && last_gpio_config[g].address   == data->gpio_address[g]) {
      confirmed++;
    }
  }
  for(int s = 0; s < MAX_SPINDLE; s++) {
    if(data->spindle_vfd_type[s] == MODBUS_TYPE_NOT_SET
    || (last_spindle_config[s].vfd_type        == data->spindle_vfd_type[s]
     && last_spindle_config[s].modbus_address  == data->spindle_address[s]
     && last_spindle_config[s].bitrate         == data->spindle_bitrate[s])) {
      confirmed++;
    }
  }
  return confirmed;
}

/* Only try to configure one parameter per 1ms cycle since they change infrequently
 * and don't need low latency when they do. */
static bool configure(
    struct NWBuffer* tx_buffer,
    size_t count,
    skeleton_t *data,
    int num_joints
) {
  uint8_t fw_joints = get_detected_joint_count();
  if(fw_joints > 0 && fw_joints < (uint8_t)num_joints) {
    static bool warned = false;
    if(!warned) {
      rtapi_print_msg(RTAPI_MSG_ERR,
          "RP2040: ERROR: firmware has %u joints but config expects %d; "
          "reflash firmware with MAX_JOINT>=%d\n",
          fw_joints, num_joints, num_joints);
      warned = true;
    }
  }
  size_t active_joints = (fw_joints > 0) ? fw_joints : (size_t)num_joints;
  size_t total_things = active_joints + MAX_GPIO + MAX_SPINDLE;
  size_t joint_or_gpio_or_spindle = count % total_things;

  if(joint_or_gpio_or_spindle < active_joints) {
    uint8_t joint = joint_or_gpio_or_spindle;
    return configure_joint(tx_buffer, joint, data);
  } else if(joint_or_gpio_or_spindle < active_joints + MAX_GPIO) {
    uint8_t gpio = joint_or_gpio_or_spindle - active_joints;
    return configure_gpio(tx_buffer, gpio, data);
  } else if(joint_or_gpio_or_spindle < active_joints + MAX_GPIO + MAX_SPINDLE) {
    uint8_t spindle = joint_or_gpio_or_spindle - active_joints - MAX_GPIO;
    return configure_spindle(tx_buffer, spindle, data);
  }
  return true;
}


/* ---- eth state functions ---- */

void log_network_error(const char *operation, int device, int error) {
  char addr[INET_ADDRSTRLEN];
  char port[10];
  getnameinfo((const struct sockaddr *)&remote_addr[device], sizeof(remote_addr[device]),
              addr, sizeof(addr), port, sizeof(port), NI_NUMERICHOST | NI_NUMERICSERV);
  char errormsg[256];
  rtapi_print_msg(RTAPI_MSG_ERR,
      "ERROR: failed to %s on network address %s:%s (%s)\n",
      operation, addr, port, strerror_r(error, errormsg, sizeof(errormsg)));
}

/* Put things in a sensible condition when communication between LinuxCNC and
 * the RP has been lost then re-established. */
void on_eth_up(skeleton_t *data, uint count) {
  printf("Ethernet up. Packet count: %u\n", count);
  *data->eth_up = true;
  *data->machine_on = true;
}

void on_eth_down(
    skeleton_t *data,
    uint count) {
  if(*data->rx_miss_count < MAX_SKIPPED_PACKETS) {
    return;
  }

  printf("WARN: Ethernet down. Packet count: %u\n", count);
  *data->eth_up = false;
  *data->machine_on = false;

  /* Force reconfiguration when network comes back up. */
  reset_rp_config(data);
}

/* Reset HAL's opinion of the RP config to force a full resend on recovery. */
static void reset_rp_config(skeleton_t *data) {
  for(size_t joint = 0; joint < MAX_JOINT; joint++) {
    *data->joint_enable_cmd[joint] = false;
    last_joint_config[joint].gpio_step = -1;
    last_joint_config[joint].gpio_dir = -1;
  }

  for(size_t gpio = 0; gpio < MAX_GPIO; gpio++) {
    last_gpio_config[gpio].gpio_type = GPIO_TYPE_NOT_SET;
  }

  for(size_t spindle = 0; spindle < MAX_SPINDLE; spindle++) {
    last_spindle_config[spindle].vfd_type = MODBUS_TYPE_NOT_SET;
  }

  reset_version_check();
}

void eth_state_update(skeleton_t *data, int device_num, size_t count, uint32_t now, int num_joints) {
  struct NWBuffer buffer;

  /* While eth is down, hold joint_enable_cmd=false so the RP2040 keeps
   * decelerating.  LinuxCNC may write enable=true to this HAL pin every
   * servo period; we intercept it here before the packet is built. */
  if (!*data->eth_up) {
    for (int j = 0; j < num_joints; j++) {
      *data->joint_enable_cmd[j] = false;
    }
  }

  /* Send — skipped during cooloff, but receive/eth-tracking always runs so
   * that rx_miss_count and eth_up reflect reality even when we cannot send
   * (e.g. interface administratively down). */
  if (cooloff > 0) {
    cooloff--;
  } else {
    reset_nw_buf(&buffer);
    bool pack_success = true;

    *data->seq_out = (uint32_t)count;
    pack_success = pack_success && serialize_timing(&buffer, count, now);

    if (!get_version_checked())
      serialize_version_request(&buffer);

    /* serialize_gpio() return value not checked: a failed pack still allows
     * the rest of the buffer to be sent with whatever was packed. */
    serialize_gpio(&buffer, data);

    pack_success = pack_success && configure(&buffer, count, data, num_joints);

    pack_success = pack_success && serialize_joint_pos(&buffer, data);

    if(count % 100 == 0) {
      pack_success = pack_success && serialise_spindle_speed_in(&buffer, data);
    }

    if(!pack_success) {
      printf("WARN: TX packet dropped — buffer overflow packing servo cycle %u\n",
             (unsigned)count);
    } else if (send_data(device_num, &buffer) != 0) {
      cooloff = 2000;
      if (errno != last_errno) {
        last_errno = errno;
        log_network_error("send", device_num, errno);
      }
      send_fail_count++;
      if (!(send_fail_count % 10)) {
        last_errno = 0;
      }
    } else {
      send_fail_count = 0;
    }
  }

  /* Receive data and check packets all completed round trip. */
  reset_nw_buf(&buffer);
  size_t data_length = get_reply_non_block(device_num, &buffer);
  if(data_length > 0) {
    size_t mess_received_count = 0;
    process_data(
        &buffer,
        data,
        &mess_received_count,
        data_length,
        last_joint_config,
        last_gpio_config,
        last_spindle_config
    );

    if(! *data->eth_up) {
      /* Don't signal recovery to LinuxCNC until all joints have stopped moving
       * AND the RP2040 has confirmed (via REPLY_JOINT_CONFIG) that it has
       * applied the disable.  Both conditions together mean the RP2040 is
       * stationary and will not resume on its own.
       *
       * vel_fb is Q16.16 steps/period (exact internal velocity_q); == 0.0
       * only when the firmware's velocity_q is exactly zero, so one reading
       * is sufficient — no hysteresis needed. */

      bool all_stopped = true;
      for (uint32_t joint = 0; joint < (uint32_t)num_joints; joint++) {
        if (*data->joint_vel_fb[joint] != 0.0 || last_joint_config[joint].enable) {
          all_stopped = false;
        }
      }
      if (all_stopped) {
        waiting_logged = false;
        on_eth_up(data, count);
      } else {
        if (!waiting_logged) {
          printf("INFO: waiting for joints to stop before recovery"
                 " (vel_fb[0]=%g)\n", (double)*data->joint_vel_fb[0]);
          waiting_logged = true;
        }
      }
    }

    size_t total_configs = (size_t)num_joints + MAX_GPIO + MAX_SPINDLE;
    size_t confirmed = count_confirmed_configs(data, num_joints);
    *data->config_complete = (confirmed == total_configs);

    if(confirmed != last_confirmed) {
      if(*data->config_complete) {
        printf("INFO: all %zu config updates have completed\n", total_configs);
      } else {
        printf("INFO: %zu of %zu config updates have completed\n", confirmed, total_configs);
      }
      last_confirmed = confirmed;
    }

    if(*data->config_complete && last_update_id + 1 != *data->seq_in && last_update_id != 0) {
      printf("WARN: %i missing updates (seq %u -> %u)\n",
          *data->seq_in - last_update_id - 1, last_update_id, *data->seq_in);
    }
    last_update_id = *data->seq_in;
    *data->rx_miss_count = 0;
  } else {
    if(errno != EAGAIN && last_errno != errno) {
      last_errno = errno;
      log_network_error("receive", device_num, errno);
    }
    if(*data->eth_up) {
      on_eth_down(data, count);
    }
    (*data->rx_miss_count)++;
    if (*data->rx_miss_count == 5000 || !(*data->rx_miss_count % 10000)) {
      printf("WARN: Still no connection over Ethernet link.\n");
    }
  }
}
