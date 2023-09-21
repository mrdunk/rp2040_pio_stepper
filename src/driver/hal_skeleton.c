/********************************************************************
 * Description:  hal_skeleton.c
 *               This file, 'hal_skeleton.c', is a example that shows 
 *               how drivers for HAL components will work and serve as 
 *               a skeleton for new hardware drivers.
 *
 * Author: John Kasunich
 * License: GPL Version 2
 *    
 * Copyright (c) 2003 All rights reserved.
 *
 * Last change: 
 ********************************************************************/

/** This file, 'hal_skeleton.c', is a example that shows how
drivers for HAL components will work and serve as a skeleton
for new hardware drivers.

Most of this code is taken from the hal_parport driver from John Kasunich,
which is also a good starting point for new drivers.

This driver supports only for demonstration how to write a byte (char)
to a hardware address, here we use the parallel port (0x378).

This driver support no configuration strings so installing is easy:
  realtime: halcmd loadrt hal_skeleton

The driver creates a HAL pin and if it run in realtime a function
as follows:

Pin: 'skeleton.<portnum>.pin-<pinnum>-out'
Function: 'skeleton.<portnum>.write'

This skeleton driver also doesn't use arguments you can pass to the driver
at startup. Please look at the parport driver how to implement this if you need
this for your driver.

(added 17 Nov 2006)
The approach used for writing HAL drivers has evolved quite a bit over the
three years since this was written.  Driver writers should consult the HAL
User Manual for information about canonical device interfaces, and should
examine some of the more complex drivers, before using this as a basis for
a new driver.

*/

/** Copyright (C) 2003 John Kasunich
  <jmkasunich AT users DOT sourceforge DOT net>
  Martin Kuhnle
  <mkuhnle AT users DOT sourceforge DOT net>
  */

/** This program is free software; you can redistribute it and/or
  modify it under the terms of version 2 of the GNU General
  Public License as published by the Free Software Foundation.
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

  THE AUTHORS OF THIS LIBRARY ACCEPT ABSOLUTELY NO LIABILITY FOR
  ANY HARM OR LOSS RESULTING FROM ITS USE.  IT IS _EXTREMELY_ UNWISE
  TO RELY ON SOFTWARE ALONE FOR SAFETY.  Any machinery capable of
  harming persons must have provisions for completely removing power
  from all motors, etc, before persons enter any danger area.  All
  machinery must be designed to comply with local and national safety
  codes, and the authors of this software can not, and do not, take
  any responsibility for such compliance.

  This code was written as part of the EMC HAL project.  For more
  information, go to www.linuxcnc.org.
  */

#include <stdio.h>

#include "rtapi.h"    /* RTAPI realtime OS API */
#include "rtapi_app.h"    /* RTAPI realtime module decls */

#include "hal.h"    /* HAL public API decls */

/* module information */
MODULE_AUTHOR("Martin Kuhnle");
MODULE_DESCRIPTION("Test Driver for ISA-LED Board for EMC HAL");
MODULE_LICENSE("GPL");

#define JOINTS 4    /* number of joints on each device. */
#define IO 16       /* number of input and output pins on each device. */

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

#define MAX_PORTS 8

#define MAX_TOK ((MAX_PORTS*2)+3)

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
	static uint count = 0;
  skeleton_t *device = arg;

	for(int num_io = 0; num_io < IO; num_io++) {
    *device->pin_in[num_io] = ((count / 1000) % 2 == 0);
  }
	for(int num_joint = 0; num_joint < JOINTS; num_joint++) {
		*device->received_pos[num_joint] = *device->requested_pos[num_joint];

    //printf("%u\t%u\n", count, period);
    /*
		if(count % 10000 == 0) {
			if(num_joint == 0) {
				printf("%u\n", count);
			}
			printf("\t %i\t %u\t %f\t %f\n", num_joint, arg, *device->received_pos[num_joint], *device->requested_pos[num_joint]);
		}
    */
	}
	count++;

  //unsigned char outdata;
  //outdata = *(device->data_out[0]) & 0xFF;
  /* write it to the hardware */
  //rtapi_outb(outdata, 0x378);
  //printf("%i\t%c\n", outdata);
}
