

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
  hal_s32_t* io_pos_step[JOINTS];
  hal_s32_t* io_pos_dir[JOINTS];
  hal_float_t* kp[JOINTS];
  hal_float_t* max_velocity[JOINTS];
  hal_float_t* max_accel[JOINTS];
  hal_float_t* scale[JOINTS];
  hal_float_t* command[JOINTS];
  hal_float_t* feedback[JOINTS];
  hal_float_t* calculated_velocity[JOINTS];
  hal_float_t* fb_velocity[JOINTS];
  hal_bit_t* pin_out[IO];
  hal_bit_t* pin_in[IO];
  bool reset_joint[JOINTS];
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

void enable_joint(
    int num_joint, size_t* buffer_space, size_t* buffer_size,
    void** buffer_iterator, skeleton_t *data);

void enable_io(
    int num_joint, size_t* buffer_space, size_t* buffer_size,
    void** buffer_iterator, skeleton_t *data);

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

    retval = hal_pin_s32_newf(HAL_IN, &(port_data_array->io_pos_step[num_joint]), comp_id,
                              "rp2040_eth.%d.joint-io-pos-step-%d", num_device, num_joint);
    if (retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,
                      "SKELETON: ERROR: port %d var export failed with err=%i\n", num_device, retval);
      hal_exit(comp_id);
      return -1;
    }

    retval = hal_pin_s32_newf(HAL_IN, &(port_data_array->io_pos_dir[num_joint]),
                              comp_id, "rp2040_eth.%d.joint-io-pos-dir-%d", num_device, num_joint);
    if (retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,
                      "SKELETON: ERROR: port %d var export failed with err=%i\n", num_device, retval);
      hal_exit(comp_id);
      return -1;
    }

    retval = hal_pin_float_newf(HAL_IN, &(port_data_array->kp[num_joint]), comp_id,
                                "rp2040_eth.%d.joint-kp-%d", num_device, num_joint);
    if (retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,
                      "SKELETON: ERROR: port %d var export failed with err=%i\n", num_device, retval);
      hal_exit(comp_id);
      return -1;
    }

    retval = hal_pin_float_newf(HAL_IN, &(port_data_array->max_velocity[num_joint]), comp_id,
                                "rp2040_eth.%d.joint-max-velocity-%d", num_device, num_joint);
    if (retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,
                      "SKELETON: ERROR: port %d var export failed with err=%i\n", num_device, retval);
      hal_exit(comp_id);
      return -1;
    }

    retval = hal_pin_float_newf(HAL_IN, &(port_data_array->max_accel[num_joint]), comp_id,
                                "rp2040_eth.%d.joint-max-accel-%d", num_device, num_joint);
    if (retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,
                      "SKELETON: ERROR: port %d var export failed with err=%i\n", num_device, retval);
      hal_exit(comp_id);
      return -1;
    }

    retval = hal_pin_float_newf(HAL_IN, &(port_data_array->scale[num_joint]),
                                comp_id, "rp2040_eth.%d.joint-scale-%d", num_device, num_joint);
    if (retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,
                      "SKELETON: ERROR: port %d var export failed with err=%i\n", num_device, retval);
      hal_exit(comp_id);
      return -1;
    }

    retval = hal_pin_float_newf(HAL_IN, &(port_data_array->command[num_joint]),
                                comp_id, "rp2040_eth.%d.pos-cmd-%d", num_device, num_joint);
    if (retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,
                      "SKELETON: ERROR: port %d var export failed with err=%i\n", num_device, retval);
      hal_exit(comp_id);
      return -1;
    }

    retval = hal_pin_float_newf(HAL_OUT, &(port_data_array->feedback[num_joint]),
                                comp_id, "rp2040_eth.%d.pos-fb-%d", num_device, num_joint);
    if (retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,
                      "SKELETON: ERROR: port %d var export failed with err=%i\n", num_device, retval);
      hal_exit(comp_id);
      return -1;
    }

    retval = hal_pin_float_newf(HAL_OUT, &(port_data_array->calculated_velocity[num_joint]),
                                comp_id, "rp2040_eth.%d.velocity-calc-%d", num_device, num_joint);
    if (retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,
                      "SKELETON: ERROR: port %d var export failed with err=%i\n", num_device, retval);
      hal_exit(comp_id);
      return -1;
    }

    retval = hal_pin_float_newf(HAL_OUT, &(port_data_array->fb_velocity[num_joint]),
                                comp_id, "rp2040_eth.%d.velocity-fb-%d", num_device, num_joint);
    if (retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,
                      "SKELETON: ERROR: port %d var export failed with err=%i\n", num_device, retval);
      hal_exit(comp_id);
      return -1;
    }
  }

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
}

void rtapi_app_exit(void)
{
  hal_exit(comp_id);
}

/**************************************************************
 * REALTIME PORT WRITE FUNCTION                                *
 **************************************************************/

static void write_port(void *arg, long period)
{
  int num_device = 0;

  static uint count = 0;
  static double last_kp[JOINTS] = {0, 0, 0, 0};
  static double last_max_velocity[JOINTS] = {0, 0, 0, 0};
  static double last_max_accel[JOINTS] = {0, 0, 0, 0};
  static double last_command[JOINTS];
  static uint32_t last_update_id = 0;
  skeleton_t *data = arg;

  char buffer[BUFSIZE];
  memset(buffer, '\0', BUFSIZE);
  void* buffer_iterator = &buffer[0];

  // Put metrics packet in buffer.
  union MessageAny message = {0};
  size_t buffer_space = BUFSIZE - sizeof(struct Message);

  message.set_abs_pos =
    (struct Message_timing){.type=MSG_TIMING, .update=count, .time=rtapi_get_time()};
  //message.set_abs_pos.type = MSG_TIMING;
  //message.set_abs_pos.update_id = count;
  //message.set_abs_pos.time = rtapi_get_time();
  size_t buffer_size += serialize_data(&message, &buffer_iterator, &buffer_space);

  // Put GPIO values in buffer.
  // TODO: Not yet implemented.
  for(int num_io = 0; num_io < IO; num_io++) {
    *data->pin_in[num_io] = ((count / 1000) % 2 == 0);
  }

  // Iterate through joints.
  for(int num_joint = 0; num_joint < JOINTS; num_joint++) {
    // Put joint positions packet in buffer.
    if(*data->joint_velocity_mode[num_joint] == 1) {
      // Velocity mode.
      // TODO: Not tested.
      double error = (last_command[num_joint] + ((*data->command)[num_joint])) / 2.0
        - (*data->feedback[num_joint]);

      message.set_rel_pos.type = MSG_SET_AXIS_REL_POS;
      message.set_rel_pos.axis = num_joint;
      message.set_rel_pos.value = *data->scale[num_joint] * error;
      buffer_size += serialize_data(&message, &buffer_iterator, &buffer_space);
      last_command[num_joint] = *data->command[num_joint];
    } else {
      // Absolute position mode.
      message.set_abs_pos.type = MSG_SET_AXIS_ABS_POS;
      message.set_abs_pos.axis = num_joint;
      message.set_abs_pos.value = *data->scale[num_joint] * *data->command[num_joint];
      buffer_size += serialize_data(&message, &buffer_iterator, &buffer_space);
    }

    // Look for parameter changes.
    if(last_kp[num_joint] != *data->kp[num_joint]) {
      last_kp[num_joint] = *data->kp[num_joint];
      message.set_kp.type = MSG_SET_AXIS_PID_KP;
      message.set_kp.axis = num_joint;
      message.set_kp.value = *data->kp[num_joint];
      buffer_size += serialize_data(&message, &buffer_iterator, &buffer_space);
    }

    if(last_max_velocity[num_joint] != *data->max_velocity[num_joint]) {
      last_max_velocity[num_joint] = *data->max_velocity[num_joint];
      message.set_max_velocity.type = MSG_SET_AXIS_MAX_SPEED;
      message.set_abs_pos.axis = num_joint;
      message.set_abs_pos.value = *data->max_velocity[num_joint];
      buffer_size += serialize_data(&message, &buffer_iterator, &buffer_space);
    }

    if(last_max_accel[num_joint] != *data->max_accel[num_joint]) {
      last_max_accel[num_joint] = *data->max_accel[num_joint];;
      message.set_max_velocity.type = MSG_SET_AXIS_MAX_ACCEL;
      message.set_abs_pos.axis = num_joint;
      message.set_abs_pos.value = *data->max_accel[num_joint];
      buffer_size += serialize_data(&message, &buffer_iterator, &buffer_space);
    }

    enable_io(num_joint, &buffer_space, &buffer_size, &buffer_iterator, data);
    enable_joint(num_joint, &buffer_space, &buffer_size, &buffer_iterator, data);
  }

  send_data(num_device, buffer, buffer_size);

  // Receive data and check packets all completed round trip.
  int receive_count;
  receive_count = get_reply_non_block(num_device, buffer);
  if(receive_count > 0) {
    process_data(buffer, data, 0);

    if(! *data->metric_eth_state) {
      // Network connection just came up after being down.
      on_eth_up(data, count);
    }

    if(last_update_id +1 != *data->metric_update_id && last_update_id != 0) {
      printf("WARN: %i missing updates.\n", *data->metric_update_id - last_update_id - 1);
    }
    last_update_id = *data->metric_update_id;
  } else {
    if(*data->metric_eth_state) {
      // Network connection just want down after being up.
      on_eth_down(data, count);
    }
    (*data->metric_missed_packets)++;
  }

  count++;
}

/* Put things in a sensible condition when if communication between LinuxCNC and
 * the RP has been lost then re-established. */
void on_eth_up(skeleton_t *data, uint count) {
  printf("Ethernet up. Packet count: %u\n", count);
  *data->metric_eth_state = true;

  if(*data->metric_missed_packets < MAX_SKIPPED_PACKETS) {
    return;
  }

  // Iterate through joints.
  for(int num_joint = 0; num_joint < JOINTS; num_joint++) {
    data->reset_joint[num_joint] = true;
  }
  *data->metric_missed_packets = 0;
}

void on_eth_down(skeleton_t *data, uint count) {
  printf("WARN: Ethernet down. Packet count: %u\n", count);
  *data->metric_eth_state = false;

  // Iterate through joints, disabling them in the config.
  for(int num_joint = 0; num_joint < JOINTS; num_joint++) {
      *data->joint_enable[num_joint] = false;
  }
}

void enable_io(
    int num_joint, size_t* buffer_space, size_t* buffer_size,
    void** buffer_iterator, skeleton_t *data
) {
  static int last_io_pos_step[JOINTS] = {-2, -2, -2, -2};
  static int last_io_pos_dir[JOINTS] = {-2, -2, -2, -2};

  if(last_io_pos_step[num_joint] != *data->io_pos_step[num_joint] ||
      data->reset_joint[num_joint]
    ) {
    struct Message_uint_uint v;
  union MessageAny message = {0};

    last_io_pos_step[num_joint] = *data->io_pos_step[num_joint];
    rtapi_print_msg(RTAPI_MSG_INFO, "Configure joint: %u  step io: %u\n",
        num_joint, *data->io_pos_step[num_joint]);
    printf("Configure joint: %u  step io: %u\n", num_joint, *data->io_pos_step[num_joint]);
    message.joint_enable.type = MSG_SET_AXIS_IO_STEP;
    message.joint_enable.axis = num_joint;
    message.joint_enable.value = *data->io_pos_step[num_joint];
    buffer_size += serialize_data(&message, &buffer_iterator, &buffer_space);

    last_io_pos_dir[num_joint] = *data->io_pos_dir[num_joint];
    rtapi_print_msg(RTAPI_MSG_INFO, "Configure joint: %u  dir io: %u\n",
        num_joint, *data->io_pos_dir[num_joint]);
    printf("Configure joint: %u  dir io: %u\n", num_joint, *data->io_pos_dir[num_joint]);
    message.joint_enable.type = MSG_SET_AXIS_IO_DIR;
    message.joint_enable.axis = num_joint;
    message.joint_enable.value = *data->io_pos_dir[num_joint];
    buffer_size += serialize_data(&message, &buffer_iterator, &buffer_space);

    data->reset_joint[num_joint] = false;
  }
}

void enable_joint(
    int num_joint, size_t* buffer_space, size_t* buffer_size,
    void** buffer_iterator, skeleton_t *data
) {
  static int last_enabled[JOINTS] = {-1, -1, -1, -1};
  union MessageAny message = {0};

  if(last_enabled[num_joint] != *data->joint_enable[num_joint]) {
    if(*data->joint_enable[num_joint]) {
      printf("enable joint: %u\n", num_joint);
    } else {
      printf("disable joint: %u\n", num_joint);
    }

    enable_io(num_joint, buffer_space, buffer_size, buffer_iterator, data);

    last_enabled[num_joint] = *data->joint_enable[num_joint];

    message.joint_enable.type = MSG_SET_AXIS_ENABLED;
    message.joint_enable.axis = num_joint;
    message.joint_enable.value = (uint32_t)(*data->joint_enable[num_joint]);
    buffer_size += serialize_data(&message, &buffer_iterator, &buffer_space);
  }
}

