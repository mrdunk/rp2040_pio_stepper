#ifndef SOCKET_MOCKS__H
#define SOCKET_MOCKS__H

#include <stdint.h>

#define SOCK_UDP    0x22
#define SOCK_CLOSED 0x00
#define Sn_MR_UDP   0x02

uint8_t  getSn_SR(uint8_t sn);
uint16_t getSn_RX_RSR(uint8_t sn);
int32_t  recvfrom(uint8_t sn, uint8_t *buf, uint16_t len, uint8_t *addr, uint16_t *port);
int32_t  sendto(uint8_t sn, const uint8_t *buf, uint16_t len, uint8_t *addr, uint16_t port);
int8_t   socket(uint8_t sn, uint8_t protocol, uint16_t port, uint8_t flag);

/* Test controls — set before each test via sock_mock_reset(). */
extern uint8_t  sock_mock_status;
extern uint16_t sock_mock_rx_size;
extern uint8_t  sock_mock_rx_data[300];
extern int32_t  sock_mock_socket_calls;

void sock_mock_reset(void);

#endif  // SOCKET_MOCKS__H
