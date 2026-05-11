#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <stdint.h>
#include <string.h>
#include <sys/socket.h>
#include <cmocka.h>

#include "../shared/buffer.h"
#include "mocks/driver_mocks.h"

/* sockfd[] is defined in rp2040_network.c which is linked into this test. */
extern int sockfd[];

static uint8_t recvfrom_data[700];
static ssize_t recvfrom_return;

ssize_t __wrap_recvfrom(
    int fd, void *buf, size_t len, int flags,
    struct sockaddr *src_addr, socklen_t *addrlen)
{
    (void)fd; (void)flags; (void)src_addr; (void)addrlen;
    if(recvfrom_return < 0) {
        return recvfrom_return;
    }
    size_t copy = (size_t)recvfrom_return < len ? (size_t)recvfrom_return : len;
    memcpy(buf, recvfrom_data, copy);
    return (ssize_t)copy;
}

/* Forward declaration — defined in rp2040_network.c. */
size_t get_reply_non_block(int device, struct NWBuffer* receive_buffer);

static int setup(void **state) {
    (void)state;
    sockfd[0] = 3;
    recvfrom_return = 0;
    memset(recvfrom_data, 0, sizeof(recvfrom_data));
    return 0;
}

static void test_get_reply__no_data__returns_zero(void **state) {
    (void)state;
    struct NWBuffer rx_buf = {0};
    recvfrom_return = -1;

    size_t ret = get_reply_non_block(0, &rx_buf);
    assert_int_equal(ret, 0);
}

static void test_get_reply__partial_packet__receives_correctly(void **state) {
    (void)state;
    struct NWBuffer rx_buf = {0};
    memset(recvfrom_data, 0xAA, sizeof(recvfrom_data));
    recvfrom_return = 64;

    size_t ret = get_reply_non_block(0, &rx_buf);
    assert_int_equal(ret, 64);
    assert_int_equal(((uint8_t*)&rx_buf)[0], 0xAA);
}

/* Regression: old cap was NW_BUF_LEN (256); last 4 bytes of a full reply
 * were silently dropped. */
static void test_get_reply__full_packet__receives_all_bytes(void **state) {
    (void)state;
    struct NWBuffer rx_buf = {0};
    memset(recvfrom_data, 0xBB, sizeof(recvfrom_data));
    recvfrom_return = (ssize_t)sizeof(struct NWBuffer);

    size_t ret = get_reply_non_block(0, &rx_buf);
    assert_int_equal(ret, sizeof(struct NWBuffer));
    assert_int_equal(rx_buf.payload[NW_BUF_LEN - 1], 0xBB);
}

static void test_get_reply__oversized__caps_at_nwbuffer(void **state) {
    (void)state;
    struct NWBuffer rx_buf = {0};
    memset(recvfrom_data, 0xCC, sizeof(recvfrom_data));
    recvfrom_return = (ssize_t)(sizeof(struct NWBuffer) + 100);

    size_t ret = get_reply_non_block(0, &rx_buf);
    assert_int_equal(ret, sizeof(struct NWBuffer));
    assert_int_equal(rx_buf.payload[NW_BUF_LEN - 1], 0xCC);
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test_setup(test_get_reply__no_data__returns_zero, setup),
        cmocka_unit_test_setup(test_get_reply__partial_packet__receives_correctly, setup),
        cmocka_unit_test_setup(test_get_reply__full_packet__receives_all_bytes, setup),
        cmocka_unit_test_setup(test_get_reply__oversized__caps_at_nwbuffer, setup),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
