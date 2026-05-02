

#include "rtapi.h"      /* RTAPI realtime OS API */
#include "rtapi_app.h"    /* RTAPI realtime module decls */

#include "hal.h"        /* HAL public API decls */

#include <limits.h>
#include <stddef.h>

#include "rp2040_defines.h"
#include "../shared/messages.h"

/* module information */
MODULE_AUTHOR("Duncan Law");
MODULE_DESCRIPTION("RP2040 based IO for LinuxCNC HAL");
MODULE_LICENSE("GPL");


/***********************************************************************
 *                STRUCTURES AND GLOBAL VARIABLES                       *
 ************************************************************************/

#include "skeleton.h"


#include "rp2040_network.c"    /* This project's network code. */

/* pointer to array of skeleton_t structs in shared memory, 1 per port */
static skeleton_t *port_data_array;

/* other globals */
static int component_id;    /* component ID */

#define MAX_SKIPPED_PACKETS 10


/***********************************************************************
 *                  LOCAL FUNCTION DECLARATIONS                         *
 ************************************************************************/
/* These is the functions that actually do the I/O
   everything else is just init code
   */
static void write_port(void *arg, long period);

void on_eth_up(skeleton_t *data, uint count);
void on_eth_down(
    skeleton_t *data,
    struct Message_joint_config* last_joint_config,
    struct Message_gpio_config* last_gpio_config,
    struct Message_spindle_config* last_spindle_config,
    uint count);
void reset_rp_config(
    skeleton_t *data,
    struct Message_joint_config* last_joint_config,
    struct Message_gpio_config* last_gpio_config,
    struct Message_spindle_config* last_spindle_config
    );

/***********************************************************************
 *                       INIT AND EXIT CODE                             *
 ************************************************************************/

enum t_types {
  PIN = 0,
  U32 = 2,
  S32 = 3,
  FLOAT = 4
};

bool init_hal_pin(
    enum t_types types,
    const hal_pin_dir_t hal_pin_dir,
    void* data_p,
    const int component_id,
    const int device_num,
    const char* io_type,
    const int chan_num,
    const int chan_num_len,
    const char* specific_name
) {
    int retval;
    if(chan_num < 0) {
      switch(types) {
        case PIN:
          retval = hal_pin_bit_newf(hal_pin_dir, data_p, component_id, "rp2040_eth.%d.%s", device_num, io_type);
          break;
        case U32:
          retval = hal_pin_u32_newf(hal_pin_dir, data_p, component_id, "rp2040_eth.%d.%s", device_num, io_type);
          break;
        case S32:
          retval = hal_pin_s32_newf(hal_pin_dir, data_p, component_id, "rp2040_eth.%d.%s", device_num, io_type);
          break;
        case FLOAT:
          retval = hal_pin_float_newf(hal_pin_dir, data_p, component_id, "rp2040_eth.%d.%s", device_num, io_type);
          break;
      }
    } else {
      char format[64];
      int check = snprintf(format, 64, "rp2040_eth.%%d.%%s.%%0%dd.%%s", chan_num_len);
      if (check < 0 || check >= 64) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "ERROR: Invalid string length=%i\n",
            check);
        hal_exit(component_id);
        return false;
      }

      switch(types) {
        case PIN:
          retval = hal_pin_bit_newf(hal_pin_dir, data_p, component_id, format, device_num, io_type, chan_num, specific_name);
          break;
        case U32:
          retval = hal_pin_u32_newf(hal_pin_dir, data_p, component_id, format, device_num, io_type, chan_num, specific_name);
          break;
        case S32:
          retval = hal_pin_s32_newf(hal_pin_dir, data_p, component_id, format, device_num, io_type, chan_num, specific_name);
          break;
        case FLOAT:
          retval = hal_pin_float_newf(hal_pin_dir, data_p, component_id, format, device_num, io_type, chan_num, specific_name);
          break;
      }
    }
    if (retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,
                      "RP2040: ERROR: var export failed with err=%i\n",
                      retval);
      hal_exit(component_id);
      return false;
    }
    return true;
}


#define ARRAY_SIZE(a) ((int)(sizeof(a) / sizeof((a)[0])))

/* Per-channel pointer fields in skeleton_t are plain arrays of pointers with no
 * padding between elements. stride = sizeof(ptr) steps correctly through them. */
typedef struct {
    enum t_types   type;
    hal_pin_dir_t  dir;
    size_t         offset;
    size_t         stride;
    const char*    io_type;
    int            chan_num;       /* -1 = scalar (no channel suffix); ignored for
                                     per-channel tables — loop variable used instead */
    int            chan_num_len;
    const char*    specific_name;
} PinDef;

static const PinDef gpio_pins[] = {
    { PIN, HAL_OUT, offsetof(skeleton_t, gpio_data_in),         sizeof(hal_bit_t*),   "gpio", 0, 2, "in"         },
    { PIN, HAL_OUT, offsetof(skeleton_t, gpio_data_in_not),     sizeof(hal_bit_t*),   "gpio", 0, 2, "in-not"     },
    { PIN, HAL_IN,  offsetof(skeleton_t, gpio_data_out),        sizeof(hal_bit_t*),   "gpio", 0, 2, "out"        },
    { PIN, HAL_IN,  offsetof(skeleton_t, gpio_data_out_invert), sizeof(hal_bit_t*),   "gpio", 0, 2, "out-invert" },
    { U32, HAL_IN,  offsetof(skeleton_t, gpio_type),            sizeof(hal_u32_t*),   "gpio", 0, 2, "type"       },
    { U32, HAL_IN,  offsetof(skeleton_t, gpio_index),           sizeof(hal_u32_t*),   "gpio", 0, 2, "index"      },
    { U32, HAL_IN,  offsetof(skeleton_t, gpio_address),         sizeof(hal_u32_t*),   "gpio", 0, 2, "address"    },
};

static const PinDef joint_pins[] = {
    { PIN,   HAL_IN,  offsetof(skeleton_t, joint_enable_cmd),     sizeof(hal_bit_t*),   "joint", 0, 1, "enable-cmd"       },
    { S32,   HAL_IN,  offsetof(skeleton_t, joint_gpio_step),      sizeof(hal_s32_t*),   "joint", 0, 1, "gpio-step"        },
    { S32,   HAL_IN,  offsetof(skeleton_t, joint_gpio_dir),       sizeof(hal_s32_t*),   "joint", 0, 1, "gpio-dir"         },
    { FLOAT, HAL_IN,  offsetof(skeleton_t, joint_vel_limit),      sizeof(hal_float_t*), "joint", 0, 1, "vel-limit"        },
    { FLOAT, HAL_IN,  offsetof(skeleton_t, joint_accel_limit),    sizeof(hal_float_t*), "joint", 0, 1, "accel-limit"      },
    { FLOAT, HAL_IN,  offsetof(skeleton_t, joint_scale),          sizeof(hal_float_t*), "joint", 0, 1, "scale"            },
    { FLOAT, HAL_IN,  offsetof(skeleton_t, joint_pos_cmd),        sizeof(hal_float_t*), "joint", 0, 1, "pos-cmd"          },
    { FLOAT, HAL_IN,  offsetof(skeleton_t, joint_vel_cmd),        sizeof(hal_float_t*), "joint", 0, 1, "vel-cmd"          },
    { FLOAT, HAL_OUT, offsetof(skeleton_t, joint_pos_fb),         sizeof(hal_float_t*), "joint", 0, 1, "pos-fb"           },
    { FLOAT, HAL_OUT, offsetof(skeleton_t, joint_vel_fb),         sizeof(hal_float_t*), "joint", 0, 1, "vel-fb"           },
    { S32,   HAL_OUT, offsetof(skeleton_t, joint_pos_error_fb),   sizeof(hal_s32_t*),   "joint", 0, 1, "pos-error-fb"     },
    { PIN,   HAL_OUT, offsetof(skeleton_t, joint_enable_fb),      sizeof(hal_bit_t*),   "joint", 0, 1, "enable-fb"        },
    { FLOAT, HAL_OUT, offsetof(skeleton_t, joint_vel_calculated), sizeof(hal_float_t*), "joint", 0, 1, "vel-calculated"   },
};

static const PinDef spindle_pins[] = {
    { PIN,   HAL_IN,  offsetof(skeleton_t, spindle_fwd),       sizeof(hal_bit_t*),   "spindle", 0, 1, "fwd"       },
    { PIN,   HAL_IN,  offsetof(skeleton_t, spindle_rev),       sizeof(hal_bit_t*),   "spindle", 0, 1, "rev"       },
    { FLOAT, HAL_IN,  offsetof(skeleton_t, spindle_speed_cmd), sizeof(hal_float_t*), "spindle", 0, 1, "speed-cmd" },
    { FLOAT, HAL_OUT, offsetof(skeleton_t, spindle_speed_fb),  sizeof(hal_float_t*), "spindle", 0, 1, "speed-fb"  },
    { PIN,   HAL_OUT, offsetof(skeleton_t, spindle_at_speed),  sizeof(hal_bit_t*),   "spindle", 0, 1, "at-speed"  },
};

static const PinDef scalar_pins[] = {
    { PIN,   HAL_OUT, offsetof(skeleton_t, machine_on),      0, "machine-on",      -1, 1, NULL },
    { U32,   HAL_OUT, offsetof(skeleton_t, seq_out),         0, "seq-out",         -1, 0, NULL },
    { U32,   HAL_OUT, offsetof(skeleton_t, core1_period),    0, "core1-period",    -1, 0, NULL },
    { U32,   HAL_OUT, offsetof(skeleton_t, core1_tick),      0, "core1-tick",      -1, 0, NULL },
    { S32,   HAL_IN,  offsetof(skeleton_t, packet_interval), 0, "packet-interval", -1, 0, NULL },
    { U32,   HAL_IN,  offsetof(skeleton_t, seq_in),          0, "seq-in",          -1, 0, NULL },
    { U32,   HAL_IN,  offsetof(skeleton_t, rx_miss_count),   0, "rx-miss-count",   -1, 0, NULL },
    { PIN,   HAL_IN,  offsetof(skeleton_t, eth_up),          0, "eth-up",          -1, 0, NULL },
    { FLOAT, HAL_OUT, offsetof(skeleton_t, update_overrun),  0, "update-overrun",  -1, 0, NULL },
    { FLOAT, HAL_OUT, offsetof(skeleton_t, update_underrun), 0, "update-underrun", -1, 0, NULL },
};

int rtapi_app_main(void)
{
  char name[HAL_NAME_LEN + 1];
  int retval;

  /* only one device at the moment */
  int device_num = 0;

  /* STEP 1: initialise the driver */
  component_id = hal_init("hal_rp2040_eth");
  if (component_id < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR,
        "RP2040: ERROR: hal_init() failed\n");
    return -1;
  }

  /* STEP 2: allocate shared memory for skeleton data */
  port_data_array = hal_malloc(sizeof(skeleton_t));
  if (port_data_array == 0) {
    rtapi_print_msg(RTAPI_MSG_ERR,
        "RP2040: ERROR: hal_malloc() failed\n");
    goto port_error;
  }

  /* Set some default values. */
  for(int gpio_bank = 0; gpio_bank < MAX_GPIO_BANK; gpio_bank++) {
    port_data_array->gpio_data_received[gpio_bank] = 0;
    port_data_array->gpio_confirmation_pending[gpio_bank] = true;
  }


  /* Set up the HAL pins via descriptor tables. */
  for (int i = 0; i < MAX_GPIO; i++) {
    for (int j = 0; j < ARRAY_SIZE(gpio_pins); j++) {
      const PinDef* def = &gpio_pins[j];
      void* fp = (void*)((char*)port_data_array + def->offset + i * def->stride);
      if (!init_hal_pin(def->type, def->dir, fp, component_id, device_num,
                        def->io_type, i, def->chan_num_len, def->specific_name)) {
        goto port_error;
      }
    }
  }
  /* Default values written after all pins are registered (equivalent to the
   * original interleaved approach since goto port_error fires on any failure). */
  for (int i = 0; i < MAX_GPIO; i++) {
    *port_data_array->gpio_data_in[i]         = true;
    *port_data_array->gpio_data_in_not[i]     = false;
    *port_data_array->gpio_data_out[i]        = false;
    *port_data_array->gpio_data_out_invert[i] = false;
    *port_data_array->gpio_type[i]            = GPIO_TYPE_NOT_SET;
  }

  for (int j = 0; j < ARRAY_SIZE(scalar_pins); j++) {
    const PinDef* def = &scalar_pins[j];
    void* fp = (void*)((char*)port_data_array + def->offset);
    if (!init_hal_pin(def->type, def->dir, fp, component_id, device_num,
                      def->io_type, def->chan_num, def->chan_num_len, def->specific_name)) {
      goto port_error;
    }
  }
  *port_data_array->machine_on = false;

  for (int i = 0; i < MAX_JOINT; i++) {
    for (int j = 0; j < ARRAY_SIZE(joint_pins); j++) {
      const PinDef* def = &joint_pins[j];
      void* fp = (void*)((char*)port_data_array + def->offset + i * def->stride);
      if (!init_hal_pin(def->type, def->dir, fp, component_id, device_num,
                        def->io_type, i, def->chan_num_len, def->specific_name)) {
        goto port_error;
      }
    }
  }
  for (int i = 0; i < MAX_JOINT; i++) {
    *port_data_array->joint_enable_cmd[i]    = false;
    *port_data_array->joint_gpio_step[i] = -1;
    *port_data_array->joint_gpio_dir[i]  = -1;
  }

  /* Export spindle pins. */
  for (int i = 0; i < MAX_SPINDLE; i++) {
    for (int j = 0; j < ARRAY_SIZE(spindle_pins); j++) {
      const PinDef* def = &spindle_pins[j];
      void* fp = (void*)((char*)port_data_array + def->offset + i * def->stride);
      if (!init_hal_pin(def->type, def->dir, fp, component_id, device_num,
                        def->io_type, i, def->chan_num_len, def->specific_name)) {
        goto port_error;
      }
    }

    retval = hal_param_u32_newf(HAL_RW, &(port_data_array->spindle_vfd_type[i]),
        component_id, "rp2040_eth.%d.spindle.%d.vfd-type", device_num, i);
    if (retval < 0) {
      goto port_error;
    }
    port_data_array->spindle_vfd_type[i] = MODBUS_TYPE_NOT_SET;

    retval = hal_param_u32_newf(HAL_RW, &(port_data_array->spindle_address[i]),
        component_id, "rp2040_eth.%d.spindle.%d.address", device_num, i);
    if (retval < 0) {
      goto port_error;
    }
    port_data_array->spindle_address[i] = 1;

    retval = hal_param_float_newf(HAL_RW, &(port_data_array->spindle_poles[i]),
        component_id, "rp2040_eth.%d.spindle.%d.poles", device_num, i);
    if (retval < 0) {
      goto port_error;
    }
    port_data_array->spindle_poles[i] = 2;

    retval = hal_param_u32_newf(HAL_RW, &(port_data_array->spindle_bitrate[i]),
        component_id, "rp2040_eth.%d.spindle.%d.bitrate", device_num, i);
    if (retval < 0) {
      goto port_error;
    }
    port_data_array->spindle_bitrate[i] = 9600;
  }

  /* STEP 4: export write function */
  rtapi_snprintf(name, sizeof(name), "rp2040_eth.%d.write", device_num);
  retval = hal_export_funct(name, write_port, &(port_data_array[device_num]), 1, 0,
      component_id);
  if (retval < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR,
        "RP2040: ERROR: port %d write funct export failed\n",
        device_num);
    goto port_error;
  }

  retval = init_eth(device_num);

  if (retval < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR,
        "RP2040: ERROR: Failed to find device %d on the network.\n",
        device_num);
    goto port_error;
  }

  rtapi_print_msg(RTAPI_MSG_INFO,
      "RP2040: installed driver.\n");
  hal_ready(component_id);
  return 0;

port_error:
    rtapi_print_msg(RTAPI_MSG_ERR,
        "RP2040: ERROR: port %d var export failed with err=%i\n",
        device_num, retval);
    hal_exit(component_id);
    return -1;
}

void rtapi_app_exit(void)
{
  hal_exit(component_id);
}

/**************************************************************
 * REALTIME PORT WRITE FUNCTION                                *
 **************************************************************/

void log_network_error(const char *operation, int device, int error)
{
  char addr[INET_ADDRSTRLEN];
  char port[10];
  getnameinfo((const struct sockaddr *)&remote_addr[device], sizeof(remote_addr[device]), addr, sizeof(addr), port, sizeof(port), NI_NUMERICHOST | NI_NUMERICSERV);
  char errormsg[256];
  rtapi_print_msg(RTAPI_MSG_ERR, "ERROR: failed to %s on network address %s:%s (%s)\n", operation, addr, port, strerror_r(error, errormsg, sizeof(errormsg)));
}

bool configure_joint(
    struct NWBuffer* tx_buffer,
    uint8_t joint,
    struct Message_joint_config* last_joint_config,
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
        last_joint_config[joint].gpio_step != *data->joint_gpio_step[joint]
        ||
        last_joint_config[joint].gpio_dir != *data->joint_gpio_dir[joint]
        ||
        last_joint_config[joint].max_velocity != max_velocity_ticks
        ||
        last_joint_config[joint].max_accel != max_accel_ticks
      ) {
      pack_success = pack_success && serialize_joint_config(
          tx_buffer,
          joint,
          *data->joint_enable_cmd[joint],
          *data->joint_gpio_step[joint],
          *data->joint_gpio_dir[joint],
          max_velocity_ticks,
          max_accel_ticks
          );
    }
    return pack_success;
}

bool configure_gpio(
    struct NWBuffer* tx_buffer,
    uint8_t gpio,
    struct Message_gpio_config* last_gpio_config,
    skeleton_t *data
) {
    bool pack_success = true;
    /* Send if anything changed, or if no reply has arrived yet (last_gpio_config
     * only updates in unpack_gpio_config on receipt of REPLY_GPIO_CONFIG, so a
     * lost packet leaves the diff intact and causes an automatic retry). */
    if(
        last_gpio_config[gpio].gpio_type != *data->gpio_type[gpio]
        ||
        last_gpio_config[gpio].index != *data->gpio_index[gpio]
        ||
        last_gpio_config[gpio].address != *data->gpio_address[gpio]
    ) {
      pack_success = pack_success && serialize_gpio_config(
          tx_buffer,
          gpio,
          *data->gpio_type[gpio],
          *data->gpio_index[gpio],
          *data->gpio_address[gpio]
        );
    }
    return pack_success;
}

bool configure_spindle(
    struct NWBuffer* tx_buffer,
    uint8_t spindle,
    struct Message_spindle_config* last_spindle_config,
    skeleton_t *data
) {
    if(data->spindle_vfd_type[spindle] == MODBUS_TYPE_NOT_SET) {
      // Nothing to do.
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

/* Only try to configure one parameter per 1ms cycle since they change infrequently
 * and don't need low latency when they do. */
bool configure(
    struct NWBuffer* tx_buffer,
    size_t count,
    struct Message_joint_config* last_joint_config,
    struct Message_gpio_config* last_gpio_config,
    struct Message_spindle_config* last_spindle_config,
    skeleton_t *data
) {
  size_t total_things = MAX_JOINT + MAX_GPIO + MAX_SPINDLE;
  size_t joint_or_gpio_or_spindle = count % total_things;

  if(joint_or_gpio_or_spindle < MAX_JOINT) {
    uint8_t joint = joint_or_gpio_or_spindle;
    return configure_joint(tx_buffer, joint, last_joint_config, data);
  } else if(joint_or_gpio_or_spindle < MAX_JOINT + MAX_GPIO) {
    uint8_t gpio = joint_or_gpio_or_spindle - MAX_JOINT;
    return configure_gpio(tx_buffer, gpio, last_gpio_config, data);
  } else if(joint_or_gpio_or_spindle < MAX_JOINT + MAX_GPIO + MAX_SPINDLE) {
    uint8_t spindle = joint_or_gpio_or_spindle - MAX_JOINT - MAX_GPIO;
    return configure_spindle(tx_buffer, spindle, last_spindle_config, data);
  }
  return true;
}

static void write_port(void *arg, long period)
{
  int device_num = 0;

  static size_t count = 0;
  /* configure() diffs against these before sending. Initialised to {0} so the
   * first cycle always resends full config. On LinuxCNC restart the driver
   * process also restarts, reinitialising these to {0} — no RP2040-side action
   * required. On Ethernet-down, reset_rp_config() clears them to force resend
   * (including enable=false) when the link recovers. */
  static struct Message_joint_config last_joint_config[MAX_JOINT] = {0};
  static struct Message_gpio_config last_gpio_config[MAX_GPIO] = {0};
  static struct Message_spindle_config last_spindle_config[MAX_SPINDLE] = {0};
  static uint32_t last_update_id = 0;
  static int last_errno = 0;
  static int cooloff = 0;
  static int send_fail_count = 0;

  if (cooloff > 0) {
    cooloff--;
    return;
  }

  skeleton_t *data = arg;
  struct NWBuffer buffer;
  reset_nw_buf(&buffer);
  bool pack_success = true;

  *data->seq_out = (uint32_t)count;
  pack_success = pack_success && serialize_timing(&buffer, count, rtapi_get_time());

  // Put GPIO values in network buffer.
  // serialize_gpio() return value is not checked: a failed pack still allows
  // the rest of the buffer to be sent with whatever was packed.
  serialize_gpio(&buffer, data);

  // Send configuration data to RP.
  pack_success = pack_success && configure(
      &buffer, count, last_joint_config, last_gpio_config, last_spindle_config, data);

  pack_success = pack_success && serialize_joint_pos(&buffer, data);

  // No need to update each spindle every cycle.
  if(count % 100 == 0) {
    pack_success = pack_success && serialise_spindle_speed_in(&buffer, data);
  }

  if(!pack_success) {
    printf("WARN: TX packet dropped — buffer overflow packing servo cycle %u\n", count);
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
    return;
  }
  send_fail_count = 0;

  // Receive data and check packets all completed round trip.
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
      // Network connection just came up after being down.
      on_eth_up(data, count);
    }

    if(last_update_id +1 != *data->seq_in && last_update_id != 0) {
      printf("WARN: %i missing updates. %u %u\n",
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
      // Network connection just went down after being up.
      on_eth_down(data, last_joint_config, last_gpio_config, last_spindle_config, count);
    }
    (*data->rx_miss_count)++;
    if (*data->rx_miss_count == 5000 || !(*data->rx_miss_count % 10000)) {
      printf("WARN: Still no connection over Ethernet link.\n");
    }
  }

  // If joint not enabled, assume LinuxCNC has just started and set it's position
  // to that of the RP.
  for(uint32_t joint = 0; joint < MAX_JOINT; joint++) {
    if(! *data->joint_enable_cmd[joint]) {
      *data->joint_pos_cmd[joint] = *data->joint_pos_fb[joint];
    }
  }

  count++;
}

/* Put things in a sensible condition when if communication between LinuxCNC and
 * the RP has been lost then re-established. */
void on_eth_up(skeleton_t *data, uint count) {
  printf("Ethernet up. Packet count: %u\n", count);
  *data->eth_up = true;
  *data->machine_on = true;
}

void on_eth_down(
    skeleton_t *data,
    struct Message_joint_config* last_joint_config,
    struct Message_gpio_config* last_gpio_config,
    struct Message_spindle_config* last_spindle_config,
    uint count) {
  if(*data->rx_miss_count < MAX_SKIPPED_PACKETS) {
    return;
  }

  printf("WARN: Ethernet down. Packet count: %u\n", count);
  *data->eth_up = false;
  *data->machine_on = false;

  // Force reconfiguration when network comes back up.
  reset_rp_config(data, last_joint_config, last_gpio_config, last_spindle_config);
}

/* Reset HAL's opinion of the RP config. This will force an update. */
void reset_rp_config(
    skeleton_t *data,
    struct Message_joint_config* last_joint_config,
    struct Message_gpio_config* last_gpio_config,
    struct Message_spindle_config* last_spindle_config
) {
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
}

