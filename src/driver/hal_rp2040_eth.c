

#include "rtapi.h"      /* RTAPI realtime OS API */
#include "rtapi_app.h"    /* RTAPI realtime module decls */

#include "hal.h"        /* HAL public API decls */

#include <limits.h>

#include "rp2040_defines.h"

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
  hal_bit_t* joint_velocity_mode[JOINTS];
  hal_s32_t* joint_gpio_step[JOINTS];
  hal_s32_t* joint_gpio_dir[JOINTS];
  hal_float_t* joint_kp[JOINTS];
  hal_float_t* joint_max_velocity[JOINTS];
  hal_float_t* joint_max_accel[JOINTS];
  hal_float_t* joint_scale[JOINTS];
  hal_float_t* joint_pos_cmd[JOINTS];
  hal_float_t* joint_vel_cmd[JOINTS];
  hal_float_t* joint_pos_feedback[JOINTS];
  //hal_s32_t* joint_pos_error[JOINTS];
  hal_s32_t* joint_step_len_ticks[JOINTS];
  hal_float_t* joint_velocity_cmd[JOINTS];   // TODO: why a joint_vel_cmd and a joint_velocity_cmd?
  hal_float_t* joint_velocity_feedback[JOINTS];
  hal_bit_t* pin_out[IO];
  hal_bit_t* pin_in[IO];
  //bool reset_joint[JOINTS];
  uint8_t joints_enabled_this_cycle;
  hal_bit_t* spindle_fwd;
  hal_bit_t* spindle_rev;
  hal_float_t* spindle_speed_out;
  hal_float_t* spindle_speed_in;
  hal_bit_t* spindle_at_speed;

  hal_u32_t spindle_vfd_type;
  hal_u32_t spindle_address;
  hal_float_t spindle_poles;
  hal_u32_t spindle_bitrate;
} skeleton_t;


#include "rp2040_network.c"    /* This project's network code. */

/* pointer to array of skeleton_t structs in shared memory, 1 per port */
static skeleton_t *port_data_array;

/* other globals */
static int comp_id;    /* component ID */
static int num_devices;    /* number of devices configured */

#define MAX_SKIPPED_PACKETS 100


/***********************************************************************
 *                  LOCAL FUNCTION DECLARATIONS                         *
 ************************************************************************/
/* These is the functions that actually do the I/O
   everything else is just init code
   */
static void write_port(void *arg, long period);

void on_eth_up(skeleton_t *data, uint count);
void on_eth_down(skeleton_t *data, uint count);

//void enable_joint(struct NWBuffer* buffer, bool* pack_success, int joint, skeleton_t *data);
//void enable_io(struct NWBuffer* buffer, bool* pack_success, int joint, skeleton_t *data);

/***********************************************************************
 *                       INIT AND EXIT CODE                             *
 ************************************************************************/

int rtapi_app_main(void)
{
  char name[HAL_NAME_LEN + 1];
  int num_device, retval;

  /* only one device at the moment */
  num_devices = 1;
  num_device = 0;

  /* STEP 1: initialise the driver */
  comp_id = hal_init("hal_rp2040_eth");
  if (comp_id < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR,
        "SKELETON: ERROR: hal_init() failed\n");
    return -1;
  }

  /* STEP 2: allocate shared memory for skeleton data */
  port_data_array = hal_malloc(num_devices * sizeof(skeleton_t));
  if (port_data_array == 0) {
    rtapi_print_msg(RTAPI_MSG_ERR,
        "SKELETON: ERROR: hal_malloc() failed\n");
    hal_exit(comp_id);
    return -1;
  }

  for(int num_io = 0; num_io < IO; num_io++) {
    /* Export the IO(s) */
    retval = hal_pin_bit_newf(HAL_IN, &(port_data_array->pin_in[num_io]),
                              comp_id, "rp2040_eth.%d.pin-%d-in", num_device, num_io);
    if (retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,
                      "SKELETON: ERROR: port %d var export failed with err=%i\n",
                      num_device, retval);
      hal_exit(comp_id);
      return -1;
    }

    retval = hal_pin_bit_newf(HAL_OUT, &(port_data_array->pin_out[num_io]), comp_id,
                              "rp2040_eth.%d.pin-%d-out", num_device, num_io);
    if (retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,
                      "SKELETON: ERROR: port %d var export failed with err=%i\n",
                      num_device, retval);
      hal_exit(comp_id);
      return -1;
    }
  }

  for(int num_joint = 0; num_joint < JOINTS; num_joint++) {
    /* Export the joint position pin(s) */
    retval = hal_pin_bit_newf(HAL_IN, &(port_data_array->joint_enable[num_joint]),
                              comp_id, "rp2040_eth.%d.joint-enable-%d", num_device, num_joint);
    if (retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,
                      "SKELETON: ERROR: port %d var export failed with err=%i\n",
                      num_device, retval);
      hal_exit(comp_id);
      return -1;
    }

    retval = hal_pin_bit_newf(HAL_IN, &(port_data_array->joint_velocity_mode[num_joint]),
                              comp_id, "rp2040_eth.%d.joint-velocity-mode-%d", num_device, num_joint);
    if (retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,
                      "SKELETON: ERROR: port %d var export failed with err=%i\n", num_device, retval);
      hal_exit(comp_id);
      return -1;
    }

    retval = hal_pin_s32_newf(HAL_IN, &(port_data_array->joint_gpio_step[num_joint]), comp_id,
                              "rp2040_eth.%d.joint-io-pos-step-%d", num_device, num_joint);
    if (retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,
                      "SKELETON: ERROR: port %d var export failed with err=%i\n", num_device, retval);
      hal_exit(comp_id);
      return -1;
    }

    retval = hal_pin_s32_newf(HAL_IN, &(port_data_array->joint_gpio_dir[num_joint]),
                              comp_id, "rp2040_eth.%d.joint-io-pos-dir-%d", num_device, num_joint);
    if (retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,
                      "SKELETON: ERROR: port %d var export failed with err=%i\n", num_device, retval);
      hal_exit(comp_id);
      return -1;
    }

    retval = hal_pin_float_newf(HAL_IN, &(port_data_array->joint_kp[num_joint]), comp_id,
                                "rp2040_eth.%d.joint-kp-%d", num_device, num_joint);
    if (retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,
                      "SKELETON: ERROR: port %d var export failed with err=%i\n", num_device, retval);
      hal_exit(comp_id);
      return -1;
    }

    retval = hal_pin_float_newf(HAL_IN, &(port_data_array->joint_max_velocity[num_joint]), comp_id,
                                "rp2040_eth.%d.joint-max-velocity-%d", num_device, num_joint);
    if (retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,
                      "SKELETON: ERROR: port %d var export failed with err=%i\n", num_device, retval);
      hal_exit(comp_id);
      return -1;
    }

    retval = hal_pin_float_newf(HAL_IN, &(port_data_array->joint_max_accel[num_joint]), comp_id,
                                "rp2040_eth.%d.joint-max-accel-%d", num_device, num_joint);
    if (retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,
                      "SKELETON: ERROR: port %d var export failed with err=%i\n", num_device, retval);
      hal_exit(comp_id);
      return -1;
    }

    retval = hal_pin_float_newf(HAL_IN, &(port_data_array->joint_scale[num_joint]),
                                comp_id, "rp2040_eth.%d.joint-scale-%d", num_device, num_joint);
    if (retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,
                      "SKELETON: ERROR: port %d var export failed with err=%i\n", num_device, retval);
      hal_exit(comp_id);
      return -1;
    }

    retval = hal_pin_float_newf(HAL_IN, &(port_data_array->joint_pos_cmd[num_joint]),
                                comp_id, "rp2040_eth.%d.pos-cmd-%d", num_device, num_joint);
    if (retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,
                      "SKELETON: ERROR: port %d var export failed with err=%i\n", num_device, retval);
      hal_exit(comp_id);
      return -1;
    }

    retval = hal_pin_float_newf(HAL_IN, &(port_data_array->joint_vel_cmd[num_joint]),
                                comp_id, "rp2040_eth.%d.vel-cmd-%d", num_device, num_joint);
    if (retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,
                      "SKELETON: ERROR: port %d var export failed with err=%i\n", num_device, retval);
      hal_exit(comp_id);
      return -1;
    }

    retval = hal_pin_float_newf(HAL_OUT, &(port_data_array->joint_pos_feedback[num_joint]),
                                comp_id, "rp2040_eth.%d.pos-fb-%d", num_device, num_joint);
    if (retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,
                      "SKELETON: ERROR: port %d var export failed with err=%i\n", num_device, retval);
      hal_exit(comp_id);
      return -1;
    }

    /*
    retval = hal_pin_s32_newf(HAL_OUT, &(port_data_array->joint_pos_error[num_joint]),
                                comp_id, "rp2040_eth.%d.pos-error-%d", num_device, num_joint);
    if (retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,
                      "SKELETON: ERROR: port %d var export failed with err=%i\n", num_device, retval);
      hal_exit(comp_id);
      return -1;
    }
    */

    retval = hal_pin_s32_newf(HAL_OUT, &(port_data_array->joint_step_len_ticks[num_joint]),
                                comp_id, "rp2040_eth.%d.step-len-ticks-%d", num_device, num_joint);
    if (retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,
                      "SKELETON: ERROR: port %d var export failed with err=%i\n", num_device, retval);
      hal_exit(comp_id);
      return -1;
    }

    retval = hal_pin_float_newf(HAL_OUT, &(port_data_array->joint_velocity_cmd[num_joint]),
                                comp_id, "rp2040_eth.%d.velocity-calc-%d", num_device, num_joint);
    if (retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,
                      "SKELETON: ERROR: port %d var export failed with err=%i\n", num_device, retval);
      hal_exit(comp_id);
      return -1;
    }

    retval = hal_pin_float_newf(HAL_OUT, &(port_data_array->joint_velocity_feedback[num_joint]),
                                comp_id, "rp2040_eth.%d.velocity-fb-%d", num_device, num_joint);
    if (retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,
                      "SKELETON: ERROR: port %d var export failed with err=%i\n", num_device, retval);
      hal_exit(comp_id);
      return -1;
    }
  }

  /* Export spindle pins, */
  retval = hal_pin_bit_newf(HAL_IN, &(port_data_array->spindle_fwd),
      comp_id, "rp2040_eth.%d.spindle-fwd", num_device);
  if (retval < 0) {
    goto port_error;
  }

  retval = hal_pin_bit_newf(HAL_IN, &(port_data_array->spindle_rev),
      comp_id, "rp2040_eth.%d.spindle-rev", num_device);
  if (retval < 0) {
    goto port_error;
  }

  retval = hal_pin_float_newf(HAL_IN, &(port_data_array->spindle_speed_in),
      comp_id, "rp2040_eth.%d.spindle-speed-in", num_device);
  if (retval < 0) {
    goto port_error;
  }

  retval = hal_pin_float_newf(HAL_OUT, &(port_data_array->spindle_speed_out),
      comp_id, "rp2040_eth.%d.spindle-speed-out", num_device);
  if (retval < 0) {
    goto port_error;
  }

  retval = hal_pin_bit_newf(HAL_OUT, &(port_data_array->spindle_at_speed),
      comp_id, "rp2040_eth.%d.spindle-at-speed", num_device);
  if (retval < 0) {
    goto port_error;
  }

  retval = hal_param_u32_newf(HAL_RW, &(port_data_array->spindle_address),
      comp_id, "rp2040_eth.%d.spindle-vfd-type", num_device);
  if (retval < 0) {
    goto port_error;
  }
  port_data_array->spindle_vfd_type = 2;

  retval = hal_param_u32_newf(HAL_RW, &(port_data_array->spindle_address),
      comp_id, "rp2040_eth.%d.spindle-address", num_device);
  if (retval < 0) {
    goto port_error;
  }
  port_data_array->spindle_address = 1;

  retval = hal_param_float_newf(HAL_RW, &(port_data_array->spindle_poles),
      comp_id, "rp2040_eth.%d.spindle-poles", num_device);
  if (retval < 0) {
    goto port_error;
  }
  port_data_array->spindle_poles = 2;

  retval = hal_param_u32_newf(HAL_RW, &(port_data_array->spindle_bitrate),
      comp_id, "rp2040_eth.%d.spindle-bitrate", num_device);
  if (retval < 0) {
    goto port_error;
  }
  port_data_array->spindle_bitrate = 9600;

  /* Export metrics pins, */
  retval = hal_pin_u32_newf(HAL_IN, &(port_data_array->metric_update_id),
      comp_id, "rp2040_eth.%d.metrics-update-id", num_device);
  if (retval < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR,
        "SKELETON: ERROR: port %d var export failed with err=%i\n",
        num_device, retval);
    hal_exit(comp_id);
    return -1;
  }

  retval = hal_pin_s32_newf(HAL_IN, &(port_data_array->metric_time_diff),
      comp_id, "rp2040_eth.%d.metrics-time-diff", num_device);
  if (retval < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR,
        "SKELETON: ERROR: port %d var export failed with err=%i\n",
        num_device, retval);
    hal_exit(comp_id);
    return -1;
  }

  retval = hal_pin_u32_newf(HAL_IN, &(port_data_array->metric_rp_update_len),
      comp_id, "rp2040_eth.%d.metrics-rp-update-len", num_device);
  if (retval < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR,
        "SKELETON: ERROR: port %d var export failed with err=%i\n",
        num_device, retval);
    hal_exit(comp_id);
    return -1;
  }

  retval = hal_pin_u32_newf(HAL_IN, &(port_data_array->metric_missed_packets),
      comp_id, "rp2040_eth.%d.metrics-missed-packets", num_device);
  if (retval < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR,
        "SKELETON: ERROR: port %d var export failed with err=%i\n",
        num_device, retval);
    hal_exit(comp_id);
    return -1;
  }

  retval = hal_pin_bit_newf(HAL_IN, &(port_data_array->metric_eth_state),
      comp_id, "rp2040_eth.%d.metrics-eth-state", num_device);
  port_data_array->metric_eth_state = false;
  if (retval < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR,
        "SKELETON: ERROR: port %d var export failed with err=%i\n",
        num_device, retval);
    hal_exit(comp_id);
    return -1;
  }

  /* STEP 4: export write function */
  rtapi_snprintf(name, sizeof(name), "rp2040_eth.%d.write", num_device);
  retval = hal_export_funct(name, write_port, &(port_data_array[num_device]), 1, 0,
      comp_id);
  if (retval < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR,
        "SKELETON: ERROR: port %d write funct export failed\n",
        num_device);
    hal_exit(comp_id);
    return -1;
  }

  retval = init_eth(num_device);

  if (retval < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR,
        "SKELETON: ERROR: Failed to find device %d on the network.\n",
        num_device);
    hal_exit(comp_id);
    return -1;
  }

  rtapi_print_msg(RTAPI_MSG_INFO,
      "SKELETON: installed driver for %d ports\n", num_devices);
  hal_ready(comp_id);
  return 0;

port_error:
    rtapi_print_msg(RTAPI_MSG_ERR,
        "SKELETON: ERROR: port %d var export failed with err=%i\n",
        num_device, retval);
    hal_exit(comp_id);
    return -1;
}

void rtapi_app_exit(void)
{
  hal_exit(comp_id);
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

static void write_port(void *arg, long period)
{
  int num_device = 0;

  static uint count = 0;
  //static double last_max_velocity[JOINTS] = {0, 0, 0, 0};
  //static double last_max_accel[JOINTS] = {0, 0, 0, 0};
  static struct Message_joint_config last_joint_config[JOINTS] = {0};
  static uint32_t last_update_id = 0;
  static int last_errno = 0;
  static int cooloff = 0;
  static int send_fail_count = 0;

  if (cooloff > 0) {
    cooloff--;
    return;
  }

  skeleton_t *data = arg;
  data->joints_enabled_this_cycle = 0;

  struct NWBuffer buffer;
  reset_nw_buf(&buffer);
  union MessageAny message;
  bool pack_success = true;

  pack_success = pack_success && serialize_timing(&buffer, count, rtapi_get_time());

  // Put GPIO values in buffer.
  // TODO: Not yet implemented.
  for(int num_io = 0; num_io < IO; num_io++) {
    *data->pin_in[num_io] = ((count / 1000) % 2 == 0);
  }

  int has_configs = 0;
  // Iterate through joints.
  for(int joint = 0; joint < JOINTS; joint++) {
    // Put joint positions packet in buffer.
    if(*data->joint_velocity_mode[joint] == 1) {
      // Velocity mode.
      // TODO: Not tested.
      printf("ERROR: Not implemented velocity mode. joint: %u\n", joint);
    } else {
      // Absolute position mode.
      double position = *data->joint_scale[joint] * *data->joint_pos_cmd[joint];
      double velocity = *data->joint_scale[joint] * *data->joint_vel_cmd[joint];
      pack_success = pack_success && serialize_joint_pos(&buffer, joint, position);
      pack_success = pack_success && serialize_joint_velocity(&buffer, joint, velocity);
    }

    /*
    if(last_max_velocity[joint] != *data->joint_max_velocity[joint]) {
      last_max_velocity[joint] = *data->joint_max_velocity[joint];
      pack_success = pack_success && serialize_joint_max_velocity(
          &buffer, joint, last_max_velocity[joint]);
    }

    if(last_max_accel[joint] != *data->joint_max_accel[joint]) {
      last_max_accel[joint] = *data->joint_max_accel[joint];;
      pack_success = pack_success && serialize_joint_max_accel(
          &buffer, joint, last_max_accel[joint]);
    }
    */

    if(count % JOINTS == joint) {
      // Only configure 1 joint per cycle to avoid filling NW buffer.
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
            &buffer,
            joint,
            *data->joint_enable[joint],
            *data->joint_gpio_step[joint],
            *data->joint_gpio_dir[joint],
            *data->joint_max_velocity[joint],
            *data->joint_max_accel[joint]
            );
        has_configs = 1;
      }
    }

    //enable_joint(&buffer, &pack_success, joint, data);
  }

  if (!has_configs) {
    if (count % 100 == 0) {
      pack_success = pack_success && serialise_spindle_config(&buffer, data->spindle_vfd_type, data->spindle_address, data->spindle_bitrate);
    } else {
      float speed = *data->spindle_speed_in / (120.0 / data->spindle_poles);
      pack_success = pack_success && serialise_spindle_speed_in(&buffer, speed);
    }
  }

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
    uint16_t mess_received_count = 0;
    process_data(&buffer, data, &mess_received_count, data_length, last_joint_config);

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
      on_eth_down(data, count);
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

  // Iterate through joints.
  //for(int joint = 0; joint < JOINTS; joint++) {
  //  data->reset_joint[joint] = true;
  //}
}

void on_eth_down(skeleton_t *data, uint count) {
  if(*data->metric_missed_packets < MAX_SKIPPED_PACKETS) {
    return;
  }

  printf("WARN: Ethernet down. Packet count: %u\n", count);
  *data->metric_eth_state = false;

  // Iterate through joints, disabling them in the config.
  for(int joint = 0; joint < JOINTS; joint++) {
      *data->joint_enable[joint] = false;
  }
}

/*
void enable_io(struct NWBuffer* buffer, bool* pack_success, int joint, skeleton_t *data) {
  static int last_io_pos_step[JOINTS] = {-2, -2, -2, -2};
  static int last_io_pos_dir[JOINTS] = {-2, -2, -2, -2};

  if(last_io_pos_step[joint] != *data->joint_gpio_step[joint] ||
      data->reset_joint[joint]
  ) {
    union MessageAny message;

    last_io_pos_step[joint] = *data->joint_gpio_step[joint];
    rtapi_print_msg(RTAPI_MSG_INFO, "Configure joint: %u  step io: %u\n",
        joint, *data->joint_gpio_step[joint]);
    printf("Configure joint: %u  step io: %u\n", joint, *data->joint_gpio_step[joint]);
    *pack_success = *pack_success && serialize_joint_io_step(
        buffer, joint, *data->joint_gpio_step[joint]);

    last_io_pos_dir[joint] = *data->joint_gpio_dir[joint];
    rtapi_print_msg(RTAPI_MSG_INFO, "Configure joint: %u  dir io: %u\n",
        joint, *data->joint_gpio_dir[joint]);
    printf("Configure joint: %u  dir io: %u\n", joint, *data->joint_gpio_dir[joint]);
    *pack_success = *pack_success && serialize_joint_io_dir(
        buffer, joint, *data->joint_gpio_dir[joint]);

    data->reset_joint[joint] = false;
  }
}

void enable_joint(struct NWBuffer* buffer, bool* pack_success, int joint, skeleton_t *data) {
  static int last_enabled[JOINTS] = {-1, -1, -1, -1};
  union MessageAny message = {0};

  if(last_enabled[joint] != *data->joint_enable[joint]) {
    if(data->joints_enabled_this_cycle > 0) {
      return;
    }

    data->joints_enabled_this_cycle++;

    if(*data->joint_enable[joint]) {
      printf("enable joint: %u\n", joint);
      enable_io(buffer, pack_success, joint, data);
    } else {
      printf("disable joint: %u\n", joint);
    }

    last_enabled[joint] = *data->joint_enable[joint];

    *pack_success = *pack_success && serialize_joint_enable(
        buffer, joint, *data->joint_enable[joint]);
  }
}
*/

