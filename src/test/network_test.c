#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <stdint.h>
#include <cmocka.h>

#include <stdio.h>

#include "../rp2040/config.h"
#include "../rp2040/ring_buffer.h"
#include "mocks/rp_mocks.h"

volatile uint32_t tick = 0;


int32_t put_UDP(
    uint8_t socket_num,
    uint16_t port,
    uint8_t* tx_buf,
    size_t tx_buf_len,
    uint8_t* destip,
    uint16_t* destport
) {
    return 0;
}

int32_t get_UDP(
    uint8_t socket_num,
    uint16_t port,
    uint8_t* rx_buf,
    uint8_t* data_received,
    uint8_t* destip,
    uint16_t* destport)
{
    return 0;
}

static void test_TODO__basic(void **state) {
    (void) state; /* unused */

}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_TODO__basic)
    };

    return cmocka_run_group_tests(tests, NULL, NULL);
}

