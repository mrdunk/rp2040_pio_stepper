#ifndef SENDER__H
#define SENDER__H

#include "pico_stepper.h"

/* Clock */
#define PLL_SYS_KHZ (133 * 1000)

/* Buffer */
#define ETHERNET_BUF_MAX_SIZE (1024 * 4)

/* Socket */
#define SOCKET_NUMBER_HUMAN 0
#define SOCKET_NUMBER_MACHINE 1
#define SOCKET_NUMBER_MACHINE_V02 2

/* Port */
#define NW_PORT_HUMAN 5000
#define NW_PORT_MACHINE 5002
#define NW_PORT_MACHINE_V02 5004

#define NET_ENABLE 1
#define DATA_BUF_SIZE 1024

#define LED_PIN 25

#define MAX_MACHINE_DATA 16

#define TIME_WINDOW                0
#define SET_AXIS_ABS_POS           1
#define SET_AXIS_MAX_SPEED         2
#define SET_AXIS_MAX_ACCEL         3
#define SET_GLOBAL_UPDATE_RATE     4

struct CollectedValues {
  uint target;
  uint time_window_us;
  uint set_axis_abs_pos;
};

struct DataEntry {
    uint target;               // Identifies target resource. Eg: axis.
    uint sequence;             // Non zero if following data is related to this one..
    uint key;                  // Identifies data purpose.
    uint value;
};

struct Data {
  uint count;                  // How many DataEntry items are populated.
  struct DataEntry entry[MAX_MACHINE_DATA];
};


#endif  // SENDER__H
