#include "socket_mocks.h"
#include <string.h>

uint8_t  sock_mock_status       = SOCK_UDP;
uint16_t sock_mock_rx_size      = 0;
uint8_t  sock_mock_rx_data[300] = {0};
int32_t  sock_mock_socket_calls = 0;

void sock_mock_reset(void) {
    sock_mock_status       = SOCK_UDP;
    sock_mock_rx_size      = 0;
    sock_mock_socket_calls = 0;
    memset(sock_mock_rx_data, 0, sizeof(sock_mock_rx_data));
}

uint8_t getSn_SR(uint8_t sn) {
    (void)sn;
    return sock_mock_status;
}

uint16_t getSn_RX_RSR(uint8_t sn) {
    (void)sn;
    return sock_mock_rx_size;
}

/* Copies exactly len bytes from sock_mock_rx_data into buf, returns len. */
int32_t recvfrom(uint8_t sn, uint8_t *buf, uint16_t len, uint8_t *addr, uint16_t *port) {
    (void)sn; (void)addr; (void)port;
    memcpy(buf, sock_mock_rx_data, len);
    return (int32_t)len;
}

int32_t sendto(uint8_t sn, const uint8_t *buf, uint16_t len, uint8_t *addr, uint16_t port) {
    (void)sn; (void)buf; (void)addr; (void)port;
    return (int32_t)len;
}

int8_t socket(uint8_t sn, uint8_t protocol, uint16_t port, uint8_t flag) {
    (void)protocol; (void)port; (void)flag;
    sock_mock_socket_calls++;
    return (int8_t)sn;
}
