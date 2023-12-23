#ifndef NETWORK__H
#define NETWORK__H

#include <stdint.h>

/* Get a network packet arriving via UDP. */
int32_t get_UDP(
    uint8_t socket_num,
    uint16_t port,
    void* rx_buf,
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

#endif  // NETWORK__H

