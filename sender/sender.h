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

/* Port */
#define NW_PORT_HUMAN 5000
#define NW_PORT_MACHINE 5002

#define NET_ENABLE 1
#define DATA_BUF_SIZE 1024

#define LED_PIN 25

#endif  // SENDER__H
