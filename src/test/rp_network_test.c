#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <stdint.h>
#include <cmocka.h>

#include <stdio.h>
#include <string.h>

#include "../shared/messages.h"
#include "../rp2040/core0.h"
#include "../rp2040/config.h"
#include "../rp2040/network.h"
#include "../rp2040/ring_buffer.h"
#include "mocks/rp_mocks.h"


extern volatile struct ConfigGlobal config;

void __wrap_update_packet_metrics(
    uint32_t update_id,
    uint32_t time,
    int32_t* id_diff,
    int32_t* time_diff
) {
}


/* Helper function.
 * Populates the buffer with a Message_joint_enable struct. */
size_t append_enable_message(uint8_t* rx_buf, uint32_t axis, uint8_t value) {

    struct Message_joint_enable message = {
        .type = MSG_SET_AXIS_ENABLED,
        .axis = axis,
        .value = value
    };

    memcpy(rx_buf, &message, sizeof(message));

    assert_memory_equal(rx_buf, &message, sizeof(message));

    return sizeof(message);
}

/* Helper function.
 * Populates the buffer with the provided struct. */
size_t append_message(uint8_t* rx_buf, union MessageAny message, uint8_t message_type) {
    switch(message_type) {
        case MSG_TIMING:
            memcpy(rx_buf, &message.timing, sizeof(message.timing));
            assert_memory_equal(rx_buf, &message.timing, sizeof(message.timing));
            return sizeof(message.timing);
        case MSG_SET_AXIS_ENABLED:
            return append_enable_message(
                    rx_buf, message.joint_enable.axis, message.joint_enable.value);
    }
    return 0;
}

static void test_multiple_message(void **state) {
    (void) state; /* unused */

    uint8_t rx_buf[DATA_BUF_SIZE] = {0};
    uint8_t tx_buf[DATA_BUF_SIZE] = {0};
    size_t tx_buf_len = 0;
    uint8_t received_msg_count = 0;
    memset(rx_buf, 0, DATA_BUF_SIZE);
    memset(tx_buf, 0, DATA_BUF_SIZE);

    uint8_t* rx_buf_pointer = rx_buf;

    rx_buf_pointer += append_enable_message(rx_buf_pointer, 0, 0);
    rx_buf_pointer += append_enable_message(rx_buf_pointer, 1, 1);
    rx_buf_pointer += append_enable_message(rx_buf_pointer, 2, 0);

    tx_buf_len = process_received_buffer(rx_buf, tx_buf, &received_msg_count);

    assert_int_equal(tx_buf_len, 0);
    assert_int_equal(received_msg_count, 3);

}

/* Test Message_joint_enable struct, MSG_SET_AXIS_ENABLED type works as intended. */
static void test_joint_enable_message(void **state) {
    (void) state; /* unused */

    uint8_t rx_buf[DATA_BUF_SIZE] = {0};
    uint8_t tx_buf[DATA_BUF_SIZE] = {0};
    size_t tx_buf_len = 0;
    uint8_t received_msg_count = 0;
    memset(rx_buf, 0, DATA_BUF_SIZE);
    memset(tx_buf, 0, DATA_BUF_SIZE);

    append_enable_message(rx_buf, 0, 0);

    tx_buf_len = process_received_buffer(rx_buf, tx_buf, &received_msg_count);

    assert_int_equal(tx_buf_len, 0);
    assert_int_equal(received_msg_count, 1);
}

/* Test Message_timing struct, MSG_TIMING type works as intended. */
static void test_timing_message(void **state) {
    (void) state; /* unused */

    uint8_t rx_buf[DATA_BUF_SIZE] = {0};
    uint8_t tx_buf[DATA_BUF_SIZE] = {0};
    size_t tx_buf_len = 0;
    uint8_t received_msg_count = 0;
    memset(rx_buf, 0, DATA_BUF_SIZE);
    memset(tx_buf, 0, DATA_BUF_SIZE);

    struct Message_timing timing = {
        .type = MSG_TIMING,
        .update_id = 1234,
        .time = 5678
    };

    union MessageAny message;
    message.timing = timing;

    append_message(rx_buf, message, MSG_TIMING);

    tx_buf_len = process_received_buffer(rx_buf, tx_buf, &received_msg_count);

    assert_int_equal(received_msg_count, 1);

    // The update_packet_metrics(...) method has not been mocked.
    assert_int_equal(config.last_update_id, 1234);
    assert_int_equal(config.last_update_time, 5678);

    // The serialise_metrics(...) method has not been mocked.
    assert_int_equal(tx_buf_len, sizeof(struct Reply_metrics));
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_multiple_message),
        cmocka_unit_test(test_joint_enable_message),
        cmocka_unit_test(test_timing_message)
    };

    return cmocka_run_group_tests(tests, NULL, NULL);
}

