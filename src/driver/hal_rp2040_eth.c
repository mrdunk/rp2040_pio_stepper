

#include "rtapi.h"      /* RTAPI realtime OS API */
#include "rtapi_app.h"    /* RTAPI realtime module decls */

#include "hal.h"        /* HAL public API decls */

#include <limits.h>

#include "rp2040_defines.h"
#include "../shared/messages.h"

/* module information */
MODULE_AUTHOR("Duncan Law");
MODULE_DESCRIPTION("RP2040 based IO for LinuxCNC HAL");
MODULE_LICENSE("GPL");


/***********************************************************************
 *                STRUCTURES AND GLOBAL VARIABLES                       *
 ************************************************************************/

/* this structure contains the runtime data needed by the
   driver for a single port/channel
   */

typedef struct {
  hal_u32_t* metric_update_id;
  hal_s32_t* metric_time_diff;
  hal_u32_t* metric_rp_update_len;
  hal_u32_t* metric_missed_packets;
  hal_bit_t* metric_eth_state;
  hal_bit_t* joint_enable[JOINTS];
  hal_s32_t* joint_gpio_step[JOINTS];
  hal_s32_t* joint_gpio_dir[JOINTS];
  hal_float_t* joint_max_velocity[JOINTS];
  hal_float_t* joint_max_accel[JOINTS];
  hal_float_t* joint_scale[JOINTS];
  hal_float_t* joint_pos_cmd[JOINTS];
  hal_float_t* joint_vel_cmd[JOINTS];
  hal_float_t* joint_pos_feedback[JOINTS];
  hal_s32_t* joint_step_len_ticks[JOINTS];
  hal_float_t* joint_velocity_cmd[JOINTS];   // TODO: why a joint_vel_cmd and a joint_velocity_cmd?
  hal_float_t* joint_velocity_feedback[JOINTS];

  // For IN pins, HAL sets this to the value we want the IO pin set to on the RP.
  // For OUT pins, this is the value the RP pin is reported via the network update.
  hal_bit_t* gpio_data_in[MAX_GPIO];
  hal_bit_t* gpio_data_out[MAX_GPIO];
  hal_u32_t* gpio_type[MAX_GPIO];
  hal_u32_t* gpio_index[MAX_GPIO];
  hal_u32_t* gpio_address[MAX_GPIO];

  uint32_t gpio_data_received[MAX_GPIO / 32];
  bool gpio_confirmation_pending[MAX_GPIO / 32];

  hal_bit_t* spindle_fwd[MAX_SPINDLE];
  hal_bit_t* spindle_rev[MAX_SPINDLE];
  hal_float_t* spindle_speed_out[MAX_SPINDLE];
  hal_float_t* spindle_speed_in[MAX_SPINDLE];
  hal_bit_t* spindle_at_speed[MAX_SPINDLE];

  hal_u32_t spindle_vfd_type[MAX_SPINDLE];
  hal_u32_t spindle_address[MAX_SPINDLE];
  hal_float_t spindle_poles[MAX_SPINDLE];
  hal_u32_t spindle_bitrate[MAX_SPINDLE];
} skeleton_t;


#include "rp2040_network.c"    /* This project's network code. */

/* pointer to array of skeleton_t structs in shared memory, 1 per port */
static skeleton_t *port_data_array;

/* other globals */
static int component_id;    /* component ID */

#define MAX_SKIPPED_PACKETS 100


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
    struct Message_spindle_config* last_spindle_config
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

/*
bool init_hal_pin_float(
    hal_pin_dir_t hal_pin_dir, void* data_p, int component_id, const char* identifier, int index
) {
    int retval = hal_pin_bit_newf(hal_pin_dir, data_p, component_id, identifier, index);
    if (retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,
                      "SKELETON: ERROR: var export failed with err=%i\n",
                      retval);
      hal_exit(component_id);
      return false;
    }
    return true;
}
*/

int rtapi_app_main(void)
{
  char name[HAL_NAME_LEN + 1];
  int retval;

  /* only one device at the moment */
  int num_device = 0;

  /* STEP 1: initialise the driver */
  component_id = hal_init("hal_rp2040_eth");
  if (component_id < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR,
        "SKELETON: ERROR: hal_init() failed\n");
    return -1;
  }

  /* STEP 2: allocate shared memory for skeleton data */
  port_data_array = hal_malloc(sizeof(skeleton_t));
  if (port_data_array == 0) {
    rtapi_print_msg(RTAPI_MSG_ERR,
        "SKELETON: ERROR: hal_malloc() failed\n");
    hal_exit(component_id);
    return -1;
  }

  /* Set some default values. */
  for(int gpio_bank = 0; gpio_bank < MAX_GPIO / 32; gpio_bank++) {
    port_data_array->gpio_data_received[gpio_bank] = 0;
    port_data_array->gpio_confirmation_pending[gpio_bank] = true;
  }


  /* Set up the HAL pins. */

  for(int gpio = 0; gpio < MAX_GPIO; gpio++) {
    /* Export the GPIO */
    // From PC to RP.
    retval = hal_pin_bit_newf(HAL_IN, &(port_data_array->gpio_data_in[gpio]),
                              component_id, "rp2040_eth.%d.gpio-%d-in", num_device, gpio);
    if (retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,
                      "SKELETON: ERROR: port %d var export failed with err=%i\n",
                      num_device, retval);
      hal_exit(component_id);
      return -1;
    }

    // From RP to PC.
    retval = hal_pin_bit_newf(HAL_OUT, &(port_data_array->gpio_data_out[gpio]), component_id,
                              "rp2040_eth.%d.gpio-%d-out", num_device, gpio);
    if (retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,
                      "SKELETON: ERROR: port %d var export failed with err=%i\n",
                      num_device, retval);
      hal_exit(component_id);
      return -1;
    }

    retval = hal_pin_u32_newf(HAL_IN, &(port_data_array->gpio_type[gpio]),
                              component_id, "rp2040_eth.%d.gpio-%d-type", num_device, gpio);
    if (retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,
                      "SKELETON: ERROR: port %d var export failed with err=%i\n",
                      num_device, retval);
      hal_exit(component_id);
      return -1;
    }

    retval = hal_pin_u32_newf(HAL_IN, &(port_data_array->gpio_index[gpio]),
                              component_id, "rp2040_eth.%d.gpio-%d-index", num_device, gpio);
    if (retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,
                      "SKELETON: ERROR: port %d var export failed with err=%i\n",
                      num_device, retval);
      hal_exit(component_id);
      return -1;
    }

    retval = hal_pin_u32_newf(HAL_IN, &(port_data_array->gpio_address[gpio]),
                              component_id, "rp2040_eth.%d.gpio-%d-address", num_device, gpio);
    if (retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,
                      "SKELETON: ERROR: port %d var export failed with err=%i\n",
                      num_device, retval);
      hal_exit(component_id);
      return -1;
    }
  }

  for(int num_joint = 0; num_joint < JOINTS; num_joint++) {
    /* Export the joint position pin(s) */
    retval = hal_pin_bit_newf(HAL_IN, &(port_data_array->joint_enable[num_joint]),
                              component_id, "rp2040_eth.%d.joint-enable-%d", num_device, num_joint);
    if (retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,
                      "SKELETON: ERROR: port %d var export failed with err=%i\n",
                      num_device, retval);
      hal_exit(component_id);
      return -1;
    }

    retval = hal_pin_s32_newf(HAL_IN, &(port_data_array->joint_gpio_step[num_joint]), component_id,
                              "rp2040_eth.%d.joint-io-pos-step-%d", num_device, num_joint);
    if (retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,
                      "SKELETON: ERROR: port %d var export failed with err=%i\n", num_device, retval);
      hal_exit(component_id);
      return -1;
    }

    retval = hal_pin_s32_newf(HAL_IN, &(port_data_array->joint_gpio_dir[num_joint]),
                              component_id, "rp2040_eth.%d.joint-io-pos-dir-%d", num_device, num_joint);
    if (retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,
                      "SKELETON: ERROR: port %d var export failed with err=%i\n", num_device, retval);
      hal_exit(component_id);
      return -1;
    }

    retval = hal_pin_float_newf(HAL_IN, &(port_data_array->joint_max_velocity[num_joint]), component_id,
                                "rp2040_eth.%d.joint-max-velocity-%d", num_device, num_joint);
    if (retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,
                      "SKELETON: ERROR: port %d var export failed with err=%i\n", num_device, retval);
      hal_exit(component_id);
      return -1;
    }

    retval = hal_pin_float_newf(HAL_IN, &(port_data_array->joint_max_accel[num_joint]), component_id,
                                "rp2040_eth.%d.joint-max-accel-%d", num_device, num_joint);
    if (retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,
                      "SKELETON: ERROR: port %d var export failed with err=%i\n", num_device, retval);
      hal_exit(component_id);
      return -1;
    }

    retval = hal_pin_float_newf(HAL_IN, &(port_data_array->joint_scale[num_joint]),
                                component_id, "rp2040_eth.%d.joint-scale-%d", num_device, num_joint);
    if (retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,
                      "SKELETON: ERROR: port %d var export failed with err=%i\n", num_device, retval);
      hal_exit(component_id);
      return -1;
    }

    retval = hal_pin_float_newf(HAL_IN, &(port_data_array->joint_pos_cmd[num_joint]),
                                component_id, "rp2040_eth.%d.pos-cmd-%d", num_device, num_joint);
    if (retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,
                      "SKELETON: ERROR: port %d var export failed with err=%i\n", num_device, retval);
      hal_exit(component_id);
      return -1;
    }

    retval = hal_pin_float_newf(HAL_IN, &(port_data_array->joint_vel_cmd[num_joint]),
                                component_id, "rp2040_eth.%d.vel-cmd-%d", num_device, num_joint);
    if (retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,
                      "SKELETON: ERROR: port %d var export failed with err=%i\n", num_device, retval);
      hal_exit(component_id);
      return -1;
    }

    retval = hal_pin_float_newf(HAL_OUT, &(port_data_array->joint_pos_feedback[num_joint]),
                                component_id, "rp2040_eth.%d.pos-fb-%d", num_device, num_joint);
    if (retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,
                      "SKELETON: ERROR: port %d var export failed with err=%i\n", num_device, retval);
      hal_exit(component_id);
      return -1;
    }

    retval = hal_pin_s32_newf(HAL_OUT, &(port_data_array->joint_step_len_ticks[num_joint]),
                                component_id, "rp2040_eth.%d.step-len-ticks-%d", num_device, num_joint);
    if (retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,
                      "SKELETON: ERROR: port %d var export failed with err=%i\n", num_device, retval);
      hal_exit(component_id);
      return -1;
    }

    retval = hal_pin_float_newf(HAL_OUT, &(port_data_array->joint_velocity_cmd[num_joint]),
                                component_id, "rp2040_eth.%d.velocity-calc-%d", num_device, num_joint);
    if (retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,
                      "SKELETON: ERROR: port %d var export failed with err=%i\n", num_device, retval);
      hal_exit(component_id);
      return -1;
    }

    retval = hal_pin_float_newf(HAL_OUT, &(port_data_array->joint_velocity_feedback[num_joint]),
                                component_id, "rp2040_eth.%d.velocity-fb-%d", num_device, num_joint);
    if (retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,
                      "SKELETON: ERROR: port %d var export failed with err=%i\n", num_device, retval);
      hal_exit(component_id);
      return -1;
    }
  }

  /* Export spindle pins, */
  for(uint8_t num_spindle = 0; num_spindle < MAX_SPINDLE; num_spindle++) {
    retval = hal_pin_bit_newf(HAL_IN, &(port_data_array->spindle_fwd[num_spindle]),
        component_id, "rp2040_eth.%d.spindle-fwd-%d", num_device, num_spindle);
    if (retval < 0) {
      goto port_error;
    }

    retval = hal_pin_bit_newf(HAL_IN, &(port_data_array->spindle_rev[num_spindle]),
        component_id, "rp2040_eth.%d.spindle-rev-%d", num_device, num_spindle);
    if (retval < 0) {
      goto port_error;
    }

    retval = hal_pin_float_newf(HAL_IN, &(port_data_array->spindle_speed_in[num_spindle]),
        component_id, "rp2040_eth.%d.spindle-speed-in-%d", num_device, num_spindle);
    if (retval < 0) {
      goto port_error;
    }

    retval = hal_pin_float_newf(HAL_OUT, &(port_data_array->spindle_speed_out[num_spindle]),
        component_id, "rp2040_eth.%d.spindle-speed-out-%d", num_device, num_spindle);
    if (retval < 0) {
      goto port_error;
    }

    retval = hal_pin_bit_newf(HAL_OUT, &(port_data_array->spindle_at_speed[num_spindle]),
        component_id, "rp2040_eth.%d.spindle-at-speed-%d", num_device, num_spindle);
    if (retval < 0) {
      goto port_error;
    }

    retval = hal_param_u32_newf(HAL_RW, &(port_data_array->spindle_vfd_type[num_spindle]),
        component_id, "rp2040_eth.%d.spindle-vfd-type-%d", num_device, num_spindle);
    if (retval < 0) {
      goto port_error;
    }
    port_data_array->spindle_vfd_type = 2;

    retval = hal_param_u32_newf(HAL_RW, &(port_data_array->spindle_address[num_spindle]),
        component_id, "rp2040_eth.%d.spindle-address-%d", num_device, num_spindle);
    if (retval < 0) {
      goto port_error;
    }
    port_data_array->spindle_address = 1;

    retval = hal_param_float_newf(HAL_RW, &(port_data_array->spindle_poles[num_spindle]),
        component_id, "rp2040_eth.%d.spindle-poles-%d", num_device, num_spindle);
    if (retval < 0) {
      goto port_error;
    }
    port_data_array->spindle_poles = 2;

    retval = hal_param_u32_newf(HAL_RW, &(port_data_array->spindle_bitrate[num_spindle]),
        component_id, "rp2040_eth.%d.spindle-bitrate-%d", num_device, num_spindle);
    if (retval < 0) {
      goto port_error;
    }
    port_data_array->spindle_bitrate = 9600;
  }

  /* Export metrics pins, */
  retval = hal_pin_u32_newf(HAL_IN, &(port_data_array->metric_update_id),
      component_id, "rp2040_eth.%d.metrics-update-id", num_device);
  if (retval < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR,
        "SKELETON: ERROR: port %d var export failed with err=%i\n",
        num_device, retval);
    hal_exit(component_id);
    return -1;
  }

  retval = hal_pin_s32_newf(HAL_IN, &(port_data_array->metric_time_diff),
      component_id, "rp2040_eth.%d.metrics-time-diff", num_device);
  if (retval < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR,
        "SKELETON: ERROR: port %d var export failed with err=%i\n",
        num_device, retval);
    hal_exit(component_id);
    return -1;
  }

  retval = hal_pin_u32_newf(HAL_IN, &(port_data_array->metric_rp_update_len),
      component_id, "rp2040_eth.%d.metrics-rp-update-len", num_device);
  if (retval < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR,
        "SKELETON: ERROR: port %d var export failed with err=%i\n",
        num_device, retval);
    hal_exit(component_id);
    return -1;
  }

  retval = hal_pin_u32_newf(HAL_IN, &(port_data_array->metric_missed_packets),
      component_id, "rp2040_eth.%d.metrics-missed-packets", num_device);
  if (retval < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR,
        "SKELETON: ERROR: port %d var export failed with err=%i\n",
        num_device, retval);
    hal_exit(component_id);
    return -1;
  }

  retval = hal_pin_bit_newf(HAL_IN, &(port_data_array->metric_eth_state),
      component_id, "rp2040_eth.%d.metrics-eth-state", num_device);
  port_data_array->metric_eth_state = false;
  if (retval < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR,
        "SKELETON: ERROR: port %d var export failed with err=%i\n",
        num_device, retval);
    hal_exit(component_id);
    return -1;
  }

  /* STEP 4: export write function */
  rtapi_snprintf(name, sizeof(name), "rp2040_eth.%d.write", num_device);
  retval = hal_export_funct(name, write_port, &(port_data_array[num_device]), 1, 0,
      component_id);
  if (retval < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR,
        "SKELETON: ERROR: port %d write funct export failed\n",
        num_device);
    hal_exit(component_id);
    return -1;
  }

  retval = init_eth(num_device);

  if (retval < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR,
        "SKELETON: ERROR: Failed to find device %d on the network.\n",
        num_device);
    hal_exit(component_id);
    return -1;
  }

  rtapi_print_msg(RTAPI_MSG_INFO,
      "SKELETON: installed driver.\n");
  hal_ready(component_id);
  return 0;

port_error:
    rtapi_print_msg(RTAPI_MSG_ERR,
        "SKELETON: ERROR: port %d var export failed with err=%i\n",
        num_device, retval);
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
    if(
        last_joint_config[joint].enable != *data->joint_enable[joint]
        ||
        last_joint_config[joint].gpio_step != *data->joint_gpio_step[joint]
        ||
        last_joint_config[joint].gpio_dir != *data->joint_gpio_dir[joint]
        ||
        last_joint_config[joint].max_velocity != *data->joint_max_velocity[joint]
        ||
        last_joint_config[joint].max_accel != *data->joint_max_accel[joint]
      ) {
      pack_success = pack_success && serialize_joint_config(
          tx_buffer,
          joint,
          *data->joint_enable[joint],
          *data->joint_gpio_step[joint],
          *data->joint_gpio_dir[joint],
          *data->joint_max_velocity[joint],
          *data->joint_max_accel[joint]
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
    if(spindle > 0 && *data->spindle_vfd_type[spindle] > 0) {
      printf("ERROR: More than one spindle not yet implemented.\n");
      return false;
    }

    bool pack_success = true;
    if(
        last_spindle_config[spindle].spindle_vfd_type != *data->spindle_vfd_type[spindle]
        ||
        last_spindle_config[spindle].spindle_address != *data->spindle_address[spindle]
        ||
        last_spindle_config[spindle].spindle_bitrate != *data->spindle_bitrate[spindle]
    ) {
      pack_success = pack_success && serialise_spindle_config(
          tx_buffer,
          spindle,
          data->spindle_vfd_type,
          data->spindle_address,
          data->spindle_bitrate
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
  size_t total_things = JOINTS + MAX_GPIO + MAX_SPINDLE;
  size_t joint_or_gpio_or_spindle = count % total_things;

  if(joint_or_gpio_or_spindle < JOINTS) {
    uint8_t joint = joint_or_gpio_or_spindle;
    return configure_joint(tx_buffer, joint, last_joint_config, data);
  } else if(joint_or_gpio_or_spindle < JOINTS + MAX_GPIO) {
    uint8_t gpio = joint_or_gpio_or_spindle - JOINTS;
    return configure_gpio(tx_buffer, gpio, last_gpio_config, data);
  } else if(joint_or_gpio_or_spindle < JOINTS + MAX_GPIO + MAX_SPINDLE) {
    uint8_t spindle = joint_or_gpio_or_spindle - JOINTS - MAX_GPIO;
    return configure_spindle(tx_buffer, spindle, last_gpio_config, data);
  }
  return true;
}

static void write_port(void *arg, long period)
{
  int num_device = 0;

  static size_t count = 0;
  static struct Message_joint_config last_joint_config[JOINTS] = {0};
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

  pack_success = pack_success && serialize_timing(&buffer, count, rtapi_get_time());

  // Put GPIO values in network buffer.
  //pack_success &= serialize_gpio(&buffer, data);
  // TODO: Fix feedback of serialize_gpio.
  serialize_gpio(&buffer, data);

  // Send configuration data to RP.
  pack_success = pack_success && configure(
      &buffer, count, last_joint_config, last_gpio_config, last_spindle_config, data);

  // Iterate through joints.
  for(size_t joint = 0; joint < JOINTS; joint++) {
    // Put joint positions packet in buffer.
    double position = *data->joint_scale[joint] * *data->joint_pos_cmd[joint];
    double velocity = *data->joint_scale[joint] * *data->joint_vel_cmd[joint];
    pack_success = pack_success && serialize_joint_pos(&buffer, joint, position, velocity);
  }

  //if (!has_configs) {
  //  if (count % 100 == 0) {
  //    pack_success = pack_success && serialise_spindle_config(&buffer, data->spindle_vfd_type, data->spindle_address, data->spindle_bitrate);
  //  } else {
  //    float speed = *data->spindle_speed_in / (120.0 / data->spindle_poles);
  //    pack_success = pack_success && serialise_spindle_speed_in(&buffer, speed);
  //  }
  //}

  if(pack_success) {
    if (send_data(num_device, &buffer) != 0) {
      cooloff = 2000;
      if (errno != last_errno) {
        last_errno = errno;
        log_network_error("send", num_device, errno);
      }
      send_fail_count++;
      if (!(send_fail_count % 10)) {
        last_errno = 0;
      }
      return;
    }
  }
  send_fail_count = 0;

  // Receive data and check packets all completed round trip.
  reset_nw_buf(&buffer);
  size_t data_length = get_reply_non_block(num_device, &buffer);
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

    if(! *data->metric_eth_state) {
      // Network connection just came up after being down.
      on_eth_up(data, count);
    }

    if(last_update_id +1 != *data->metric_update_id && last_update_id != 0) {
      printf("WARN: %i missing updates. %u %u\n",
          *data->metric_update_id - last_update_id - 1, last_update_id, *data->metric_update_id);
    }
    last_update_id = *data->metric_update_id;
    *data->metric_missed_packets = 0;
  } else {
    if(errno != EAGAIN && last_errno != errno) {
      last_errno = errno;
      log_network_error("receive", num_device, errno);
    }
    if(*data->metric_eth_state) {
      // Network connection just went down after being up.
      on_eth_down(data, last_joint_config, last_gpio_config, last_spindle_config, count);
    }
    (*data->metric_missed_packets)++;
    if (*data->metric_missed_packets == 5000 || !(*data->metric_missed_packets % 10000)) {
      printf("WARN: Still no connection over Ethernet link.\n");
    }
  }

  count++;
}

/* Put things in a sensible condition when if communication between LinuxCNC and
 * the RP has been lost then re-established. */
void on_eth_up(skeleton_t *data, uint count) {
  printf("Ethernet up. Packet count: %u\n", count);
  *data->metric_eth_state = true;
}

void on_eth_down(
    skeleton_t *data,
    struct Message_joint_config* last_joint_config,
    struct Message_gpio_config* last_gpio_config,
    struct Message_spindle_config* last_spindle_config,
    uint count) {
  if(*data->metric_missed_packets < MAX_SKIPPED_PACKETS) {
    return;
  }

  printf("WARN: Ethernet down. Packet count: %u\n", count);
  *data->metric_eth_state = false;

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
  for(size_t joint = 0; joint < JOINTS; joint++) {
    *data->joint_enable[joint] = false;
    last_joint_config[joint].gpio_step = -1;
    last_joint_config[joint].gpio_dir = -1;
  }

  for(size_t gpio = 0; gpio < MAX_GPIO; gpio++) {
    last_gpio_config[gpio].gpio_type = GPIO_TYPE_NOT_SET;
  }

  for(size_t spindle = 0; spindle < MAX_SPINDLE; spindle++) {
    last_spindle_config[spindle].spindle_type = SPINDLE_TYPE_NOT_SET;
  }
}

