

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
	hal_bit_t* joint_enable[JOINTS];
  hal_float_t* requested_pos[JOINTS];
  hal_float_t* received_pos[JOINTS];
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
  comp_id = hal_init("hal_skeleton");
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
		/* Export the IO pin(s) */
		retval = hal_pin_bit_newf(HAL_IN, &(port_data_array->pin_in[num_io]),
				comp_id, "skeleton.%d.pin-%02d-in", num_device, num_io);
		if (retval < 0) {
			rtapi_print_msg(RTAPI_MSG_ERR,
					"SKELETON: ERROR: port %d var export failed with err=%i\n",
          num_device, retval);
			hal_exit(comp_id);
			return -1;
		}

		retval = hal_pin_bit_newf(HAL_OUT, &(port_data_array->pin_out[num_io]),
				comp_id, "skeleton.%d.pin-%02d-out", num_device, num_io);
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
				comp_id, "skeleton.%d.joint-enable-%02d", num_device, num_joint);
		if (retval < 0) {
			rtapi_print_msg(RTAPI_MSG_ERR,
					"SKELETON: ERROR: port %d var export failed with err=%i\n",
          num_device, retval);
			hal_exit(comp_id);
			return -1;
		}

		retval = hal_pin_float_newf(HAL_IN, &(port_data_array->requested_pos[num_joint]),
				comp_id, "skeleton.%d.pos-cmd-%02d", num_device, num_joint);
		if (retval < 0) {
			rtapi_print_msg(RTAPI_MSG_ERR,
					"SKELETON: ERROR: port %d var export failed with err=%i\n",
          num_device, retval);
			hal_exit(comp_id);
			return -1;
		}

		retval = hal_pin_float_newf(HAL_OUT, &(port_data_array->received_pos[num_joint]),
				comp_id, "skeleton.%d.pos-fb-%02d", num_device, num_joint);
		if (retval < 0) {
			rtapi_print_msg(RTAPI_MSG_ERR,
					"SKELETON: ERROR: port %d var export failed with err=%i\n",
          num_device, retval);
			hal_exit(comp_id);
			return -1;
		}
	}

  /* STEP 4: export write function */
  rtapi_snprintf(name, sizeof(name), "skeleton.%d.write", num_device);
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
  skeleton_t *data = arg;


  char buffer[BUFSIZE];
  void* buffer_iterator = &buffer[0];
  uint32_t values[4] = {0};
  values[0] = MSG_TIMING;
  values[1] = count;
  values[2] = period;
  size_t buffer_space = BUFSIZE - sizeof(struct Message);
  size_t buffer_size = serialize_data(values, &buffer_iterator, &buffer_space);

	for(int num_io = 0; num_io < IO; num_io++) {
    *data->pin_in[num_io] = ((count / 1000) % 2 == 0);
  }
	for(int num_joint = 0; num_joint < JOINTS; num_joint++) {
		//*data->received_pos[num_joint] = *data->requested_pos[num_joint];

    values[0] = MSG_SET_AXIS_ABS_POS;
    values[1] = num_joint;
    values[2] = 1000 * (*data->requested_pos[num_joint]) + (UINT_MAX / 2);
    buffer_space = BUFSIZE - sizeof(struct Message_uint_uint);
    buffer_size += serialize_data(values, &buffer_iterator, &buffer_space);
	}

  send_data(num_device, buffer, buffer_size);

  int receive_count;
  receive_count = get_reply_non_block(num_device, buffer);
  if(receive_count > 0) {
    process_data(buffer, data, (count % 10000 == 0));
  } else {
    printf("!");
    if (count % 200 == 0) {
      printf("\n");
    }
  }

	count++;

  //unsigned char outdata;
  //outdata = *(data->data_out[0]) & 0xFF;
  /* write it to the hardware */
  //rtapi_outb(outdata, 0x378);
  //printf("%i\t%c\n", outdata);
}
