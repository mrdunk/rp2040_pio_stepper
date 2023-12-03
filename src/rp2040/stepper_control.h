#ifndef SENDER__H
#define SENDER__H

//#define DEBUG_OUTPUT 1

//#define MAX_AXIS 2
#define MAX_AXIS 4

/* Clock */
#define PLL_SYS_KHZ (133 * 1000)

/* Socket */
#define SOCKET_NUMBER 0

/* Port */
#define NW_PORT 5002


#define DATA_BUF_SIZE 1024

#define LED_PIN 25

/* Get a network packet arriving via UDP. */
int32_t get_UDP(
    uint8_t socket_num,
    uint16_t port,
    uint8_t* rx_buf,
    uint8_t* data_received,
    uint8_t* destip,
    uint16_t* destport);

/* Send data over UDP. */
int32_t put_UDP(
    uint8_t socket_num,
    uint16_t port,
    uint8_t* tx_buf,
    size_t tx_buf_len,
    uint8_t* destip,
    uint16_t* destport
    );

#endif  // SENDER__H
