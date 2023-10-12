

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
	hal_bit_t* joint_velocity[JOINTS];
  hal_s32_t* io_pos_step[JOINTS];
  hal_s32_t* io_pos_dir[JOINTS];
  hal_float_t* kp[JOINTS];
  hal_float_t* scale[JOINTS];
  hal_float_t* command[JOINTS];
  hal_float_t* remainder[JOINTS];
  hal_float_t* feedback[JOINTS];
  hal_float_t* calculated_velocity[JOINTS];
  hal_float_t* fb_velocity[JOINTS];
  hal_bit_t* pin_out[IO];
  hal_bit_t* pin_in[IO];
} skeleton_t;


#include "rp2040_network.c"    /* This project's network code. */

/* pointer to array of skeleton_t structs in shared memory, 1 per port */
static skeleton_t *port_data_array;

/* other globals */
static int comp_id;    /* component ID */
static int num_devices;    /* number of devices configured */


/***********************************************************************
 *                  LOCAL FUNCTION DECLARATIONS                         *
 ************************************************************************/
/* These is the functions that actually do the I/O
   everything else is just init code
   */
static void write_port(void *arg, long period);
void on_eth_up(skeleton_t *data);

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

		retval = hal_pin_bit_newf(HAL_OUT, &(port_data_array->pin_out[num_io]),
				comp_id, "rp2040_eth.%d.pin-%d-out", num_device, num_io);
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

		retval = hal_pin_bit_newf(HAL_IN, &(port_data_array->joint_velocity[num_joint]),
				comp_id, "rp2040_eth.%d.joint-velocity-mode-%d", num_device, num_joint);
		if (retval < 0) {
			rtapi_print_msg(RTAPI_MSG_ERR,
					"SKELETON: ERROR: port %d var export failed with err=%i\n",
          num_device, retval);
			hal_exit(comp_id);
			return -1;
		}

		retval = hal_pin_s32_newf(HAL_IN, &(port_data_array->io_pos_step[num_joint]),
				comp_id, "rp2040_eth.%d.joint-io-pos-step-%d", num_device, num_joint);
		if (retval < 0) {
			rtapi_print_msg(RTAPI_MSG_ERR,
					"SKELETON: ERROR: port %d var export failed with err=%i\n",
          num_device, retval);
			hal_exit(comp_id);
			return -1;
		}

		retval = hal_pin_s32_newf(HAL_IN, &(port_data_array->io_pos_dir[num_joint]),
				comp_id, "rp2040_eth.%d.joint-io-pos-dir-%d", num_device, num_joint);
		if (retval < 0) {
			rtapi_print_msg(RTAPI_MSG_ERR,
					"SKELETON: ERROR: port %d var export failed with err=%i\n",
          num_device, retval);
			hal_exit(comp_id);
			return -1;
		}

		retval = hal_pin_float_newf(HAL_IN, &(port_data_array->kp[num_joint]),
				comp_id, "rp2040_eth.%d.joint-kp-%d", num_device, num_joint);
		if (retval < 0) {
			rtapi_print_msg(RTAPI_MSG_ERR,
					"SKELETON: ERROR: port %d var export failed with err=%i\n",
          num_device, retval);
			hal_exit(comp_id);
			return -1;
		}

		retval = hal_pin_float_newf(HAL_IN, &(port_data_array->scale[num_joint]),
				comp_id, "rp2040_eth.%d.joint-scale-%d", num_device, num_joint);
		if (retval < 0) {
			rtapi_print_msg(RTAPI_MSG_ERR,
					"SKELETON: ERROR: port %d var export failed with err=%i\n",
          num_device, retval);
			hal_exit(comp_id);
			return -1;
		}

		retval = hal_pin_float_newf(HAL_IN, &(port_data_array->command[num_joint]),
				comp_id, "rp2040_eth.%d.pos-cmd-%d", num_device, num_joint);
		if (retval < 0) {
			rtapi_print_msg(RTAPI_MSG_ERR,
					"SKELETON: ERROR: port %d var export failed with err=%i\n",
          num_device, retval);
			hal_exit(comp_id);
			return -1;
		}

		retval = hal_pin_float_newf(HAL_IN, &(port_data_array->remainder[num_joint]),
				comp_id, "rp2040_eth.%d.pos-remainder-%d", num_device, num_joint);
		if (retval < 0) {
			rtapi_print_msg(RTAPI_MSG_ERR,
					"SKELETON: ERROR: port %d var export failed with err=%i\n",
          num_device, retval);
			hal_exit(comp_id);
			return -1;
		}

		retval = hal_pin_float_newf(HAL_OUT, &(port_data_array->feedback[num_joint]),
				comp_id, "rp2040_eth.%d.pos-fb-%d", num_device, num_joint);
		if (retval < 0) {
			rtapi_print_msg(RTAPI_MSG_ERR,
					"SKELETON: ERROR: port %d var export failed with err=%i\n",
          num_device, retval);
			hal_exit(comp_id);
			return -1;
		}

		retval = hal_pin_float_newf(HAL_OUT, &(port_data_array->calculated_velocity[num_joint]),
				comp_id, "rp2040_eth.%d.velocity-calc-%d", num_device, num_joint);
		if (retval < 0) {
			rtapi_print_msg(RTAPI_MSG_ERR,
					"SKELETON: ERROR: port %d var export failed with err=%i\n",
          num_device, retval);
			hal_exit(comp_id);
			return -1;
		}

		retval = hal_pin_float_newf(HAL_OUT, &(port_data_array->fb_velocity[num_joint]),
				comp_id, "rp2040_eth.%d.velocity-fb-%d", num_device, num_joint);
		if (retval < 0) {
			rtapi_print_msg(RTAPI_MSG_ERR,
					"SKELETON: ERROR: port %d var export failed with err=%i\n",
          num_device, retval);
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
  static uint missed_packets = 0;
  static double last_kp[JOINTS] = {0, 0, 0, 0};
  static int last_io_pos_step[JOINTS] = {-2, -2, -2, -2};
  static int last_io_pos_dir[JOINTS] = {-2, -2, -2, -2};
  static bool last_enabled[JOINTS] = {0, 0, 0, 0};
  static double last_command[JOINTS];
  static uint32_t last_update_id = 0;
  skeleton_t *data = arg;

  char buffer[BUFSIZE];
  memset(buffer, '\0', BUFSIZE);
  void* buffer_iterator = &buffer[0];

  // Put metrics packet in buffer.
  uint32_t values[4] = {0};
  values[0] = MSG_TIMING;
  values[1] = count;
  values[2] = rtapi_get_time();
  size_t buffer_space = BUFSIZE - sizeof(struct Message);
  size_t buffer_size = serialize_data(values, &buffer_iterator, &buffer_space);

  // Put GPIO values in buffer.
  // TODO: Not yet implemented.
	for(int num_io = 0; num_io < IO; num_io++) {
    *data->pin_in[num_io] = ((count / 1000) % 2 == 0);
  }

  // Iterate through joints.
	for(int num_joint = 0; num_joint < JOINTS; num_joint++) {
    // Put joint positions packet in buffer.
    if(*data->joint_velocity[num_joint] == 1) {
      // Velocity mode.
      // TODO: Not tested.
      double error = (last_command[num_joint] + ((*data->command)[num_joint])) / 2.0
        - (*data->feedback[num_joint]);
      values[0] = MSG_SET_AXIS_REL_POS;
      values[1] = num_joint;
      values[2] = *data->scale[num_joint] * error;
      buffer_space = BUFSIZE - sizeof(struct Message_uint_int);
      last_command[num_joint] = (*data->command[num_joint]);
    } else {
      // Absolute position mode.
      double d = (*data->scale[num_joint] * *data->command[num_joint]) + (UINT_MAX / 2);
      values[0] = MSG_SET_AXIS_ABS_POS;
      values[1] = num_joint;
      values[2] = (*data->scale[num_joint] * *data->command[num_joint] + 0.5) + (UINT_MAX / 2);

      *data->remainder[num_joint] = d - (double)values[2];

      buffer_space = BUFSIZE - sizeof(struct Message_uint_uint);
    }
    buffer_size += serialize_data(values, &buffer_iterator, &buffer_space);

    // Look for parameter changes.
    if(last_kp[num_joint] != *data->kp[num_joint]) {
      last_kp[num_joint] = *data->kp[num_joint];
      values[0] = MSG_SET_AXIS_PID_KP;
      values[1] = num_joint;
      values[2] = (uint32_t)(*data->kp[num_joint] * 1000.0);  // TODO: Cast this properly.
      buffer_size += serialize_data(values, &buffer_iterator, &buffer_space);
    }

    if(last_io_pos_step[num_joint] != *data->io_pos_step[num_joint]) {
      last_io_pos_step[num_joint] = *data->io_pos_step[num_joint];
      rtapi_print_msg(RTAPI_MSG_INFO, "Configure joint: %u  step io: %u\n", num_joint, *data->io_pos_step[num_joint]);
      printf("Configure joint: %u  step io: %u\n", num_joint, *data->io_pos_step[num_joint]);
      struct Message_uint_uint v;
      v.type = MSG_SET_AXIS_IO_STEP;
      v.axis = num_joint;
      v.value = *data->io_pos_step[num_joint];
      buffer_size += serialize_data(&v, &buffer_iterator, &buffer_space);
    }

    if(last_io_pos_dir[num_joint] != *data->io_pos_dir[num_joint]) {
      last_io_pos_dir[num_joint] = *data->io_pos_dir[num_joint];
      rtapi_print_msg(RTAPI_MSG_INFO, "Configure joint: %u  dir io: %u\n", num_joint, *data->io_pos_dir[num_joint]);
      printf("Configure joint: %u  dir io: %u\n", num_joint, *data->io_pos_dir[num_joint]);
      struct Message_uint_uint v;
      v.type = MSG_SET_AXIS_IO_DIR;
      v.axis = num_joint;
      v.value = *data->io_pos_dir[num_joint];
      buffer_size += serialize_data(&v, &buffer_iterator, &buffer_space);
    }

    if(last_enabled[num_joint] != *data->joint_enable[num_joint]) {
      last_enabled[num_joint] = *data->joint_enable[num_joint];
      values[0] = MSG_SET_AXIS_ENABLED;
      values[1] = num_joint;
      values[2] = (uint32_t)(*data->joint_enable[num_joint]);
      buffer_size += serialize_data(values, &buffer_iterator, &buffer_space);
    }
	}

  send_data(num_device, buffer, buffer_size);

  // Receive data and check packets all completed round trip.
  int receive_count;
  receive_count = get_reply_non_block(num_device, buffer);
  if(receive_count > 0) {
    process_data(buffer, data, 0);

    if(! *data->metric_eth_state) {
      // Network connection just came up after being down.
      printf("Ethernet up. Packet count: %u\n", count);
      *data->metric_eth_state = true;

      on_eth_up(data);
    }

    if(last_update_id +1 != *data->metric_update_id && last_update_id != 0) {
      printf("WARN: %i missing updates.\n", *data->metric_update_id - last_update_id - 1);
    }
  last_update_id = *data->metric_update_id;
  } else {
    *data->metric_missed_packets++;
    if(*data->metric_eth_state) {
      printf("WARN: Ethernet down. Packet count: %u\n", count);
      *data->metric_eth_state = false;
    }
  }

	count++;
}

/* Put things in a sensible condition when if communication between LinuxCNC and
 * the RP has been lost then re-established. */
void on_eth_up(skeleton_t *data) {
  // Iterate through joints.
	for(int num_joint = 0; num_joint < JOINTS; num_joint++) {
    // TODO: Work out a plan to reset RP position when it comes back online.

    //printf("%f\t%f\n", *data->command[num_joint], *data->feedback[num_joint]);
    //*data->command[num_joint] = *data->feedback[num_joint];
  }
}

