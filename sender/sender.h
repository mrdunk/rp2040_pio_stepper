#ifndef SENDER__H
#define SENDER__H

//#define DEBUG_OUTPUT 1

//#define MAX_AXIS 2
#define MAX_AXIS 4
//#define MAX_AXIS 6

/* Clock */
#define PLL_SYS_KHZ (133 * 1000)

/* Buffer */
#define ETHERNET_BUF_MAX_SIZE (1024 * 4)

/* Socket */
#define SOCKET_NUMBER 0

/* Port */
#define NW_PORT 5002


#define DATA_BUF_SIZE 1024

#define LED_PIN 25

#endif  // SENDER__H
