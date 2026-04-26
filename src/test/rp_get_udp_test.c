#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <stdint.h>
#include <string.h>
#include <cmocka.h>

#include "../shared/buffer.h"
#include "../rp2040/network.h"
#include "mocks/socket_mocks.h"

static int setup(void **state) {
    (void)state;
    sock_mock_reset();
    return 0;
}

static void test_get_UDP__sock_closed__opens_socket(void **state) {
    (void)state;
    struct NWBuffer rx_buf = {0};
    size_t data_received = 0;
    uint8_t destip[4] = {0};
    uint16_t destport = 0;

    sock_mock_status = SOCK_CLOSED;

    get_UDP(0, 1234, &rx_buf, &data_received, destip, &destport);

    assert_int_equal(sock_mock_socket_calls, 1);
    assert_int_equal(data_received, 0);
}

static void test_get_UDP__no_data__returns_zero(void **state) {
    (void)state;
    struct NWBuffer rx_buf = {0};
    size_t data_received = 0;
    uint8_t destip[4] = {0};
    uint16_t destport = 0;

    sock_mock_status  = SOCK_UDP;
    sock_mock_rx_size = 0;

    int32_t ret = get_UDP(0, 1234, &rx_buf, &data_received, destip, &destport);

    assert_int_equal(ret, 0);
    assert_int_equal(data_received, 0);
}

static void test_get_UDP__partial_packet__receives_correctly(void **state) {
    (void)state;
    struct NWBuffer rx_buf = {0};
    size_t data_received = 0;
    uint8_t destip[4] = {0};
    uint16_t destport = 0;

    sock_mock_rx_size = 64;
    memset(sock_mock_rx_data, 0xAA, sizeof(sock_mock_rx_data));

    int32_t ret = get_UDP(0, 1234, &rx_buf, &data_received, destip, &destport);

    assert_int_equal(ret, 64);
    assert_int_equal(data_received, 64);
    assert_int_equal(((uint8_t*)&rx_buf)[0], 0xAA);
}

/* Regression: the old cap was NW_BUF_LEN (256) instead of sizeof(NWBuffer)
 * (260), silently dropping the last 4 bytes of a full-sized packet. */
static void test_get_UDP__full_packet__receives_all_bytes(void **state) {
    (void)state;
    struct NWBuffer rx_buf = {0};
    size_t data_received = 0;
    uint8_t destip[4] = {0};
    uint16_t destport = 0;

    sock_mock_rx_size = sizeof(struct NWBuffer);
    memset(sock_mock_rx_data, 0xBB, sizeof(sock_mock_rx_data));

    int32_t ret = get_UDP(0, 1234, &rx_buf, &data_received, destip, &destport);

    assert_int_equal(ret, (int32_t)sizeof(struct NWBuffer));
    assert_int_equal(data_received, sizeof(struct NWBuffer));
    /* Last payload byte — was silently zeroed with the old NW_BUF_LEN cap. */
    assert_int_equal(rx_buf.payload[NW_BUF_LEN - 1], 0xBB);
}

static void test_get_UDP__oversized_packet__caps_at_nwbuffer(void **state) {
    (void)state;
    struct NWBuffer rx_buf = {0};
    size_t data_received = 0;
    uint8_t destip[4] = {0};
    uint16_t destport = 0;

    sock_mock_rx_size = sizeof(struct NWBuffer) + 100;
    memset(sock_mock_rx_data, 0xCC, sizeof(sock_mock_rx_data));

    int32_t ret = get_UDP(0, 1234, &rx_buf, &data_received, destip, &destport);

    assert_int_equal(ret, (int32_t)sizeof(struct NWBuffer));
    assert_int_equal(data_received, sizeof(struct NWBuffer));
    assert_int_equal(rx_buf.payload[NW_BUF_LEN - 1], 0xCC);
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test_setup(test_get_UDP__sock_closed__opens_socket, setup),
        cmocka_unit_test_setup(test_get_UDP__no_data__returns_zero, setup),
        cmocka_unit_test_setup(test_get_UDP__partial_packet__receives_correctly, setup),
        cmocka_unit_test_setup(test_get_UDP__full_packet__receives_all_bytes, setup),
        cmocka_unit_test_setup(test_get_UDP__oversized_packet__caps_at_nwbuffer, setup),
    };

    return cmocka_run_group_tests(tests, NULL, NULL);
}
