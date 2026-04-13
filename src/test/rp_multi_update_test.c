/* rp_multi_update_test.c
 *
 * Regression tests for stale packet counting.
 *
 * When Core0 drains multiple queued W5500 packets between timer ticks,
 * updated_from_c0 accumulates to N before Core1 reads.  The N-1 extra
 * packets are stale (only the latest position/velocity data matters).
 * get_joint_config(CORE1) detects this and accumulates the count in
 * ConfigAxis.stale_packet_count, which is returned to LinuxCNC via
 * Reply_joint_metrics.stale_packet_count.
 */

#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <stdint.h>
#include <string.h>
#include <cmocka.h>

#include "../shared/messages.h"
#include "../shared/buffer.h"
#include "../rp2040/core0.h"
#include "../rp2040/config.h"
#include "mocks/rp_mocks.h"

extern volatile struct ConfigGlobal config;

/* Build a single-message packet containing one MSG_SET_JOINT_ABS_POS message.
 * Sets the same position/velocity on all joints.
 * Returns the expected_length value for process_received_buffer(). */
static size_t build_abs_pos_packet(
    struct NWBuffer *rx_buf,
    double position,
    double velocity
) {
    *rx_buf = (struct NWBuffer){0};
    size_t expected_length = sizeof(rx_buf->length) + sizeof(rx_buf->checksum);

    struct Message_set_joints_pos msg = {0};
    msg.type = MSG_SET_JOINT_ABS_POS;
    for (size_t j = 0; j < MAX_JOINT; j++) {
        msg.position[j] = position;
        msg.velocity[j] = velocity;
    }

    expected_length += pack_nw_buff(rx_buf, &msg, sizeof(msg));
    return expected_length;
}

static int test_setup(void **state) {
    (void)state;
    init_config();
    /* init_config() doesn't reset joint data — clear what this test cares about. */
    for (size_t j = 0; j < MAX_JOINT; j++) {
        config.joint[j].updated_from_c0  = 0;
        config.joint[j].updated_from_c1  = 0;
        config.joint[j].stale_packet_count = 0;
    }
    return 0;
}

/* After N packets are processed by Core0 and then Core1 reads via
 * get_joint_config(CORE1), stale_packet_count should equal N-1 (the N-1
 * extra packets that were discarded as stale). */
static void test_stale_count_accumulates_when_core1_reads(void **state) {
    (void)state;

    const int N = 9;
    struct NWBuffer rx_buf;
    struct NWBuffer tx_buf;
    uint8_t received_msg_count;
    uint16_t expected_length;

    for (int i = 0; i < N; i++) {
        tx_buf             = (struct NWBuffer){0};
        received_msg_count = 0;
        expected_length    = (uint16_t)build_abs_pos_packet(&rx_buf, (double)i * 10.0, 1.0);
        process_received_buffer(&rx_buf, &tx_buf, &received_msg_count, expected_length);
    }

    /* Simulate Core1 reading joint 0 — this is what triggers stale counting. */
    get_joint_config(0, CORE1, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
                     NULL, NULL, NULL, NULL);

    /* N packets arrived; Core1 consumed 1 read → N-1 were stale. */
    assert_int_equal(get_and_reset_stale_count(0), N - 1);

    /* Counter resets to zero on read. */
    assert_int_equal(get_and_reset_stale_count(0), 0);
}

/* A single packet followed by one Core1 read produces no stale count. */
static void test_no_stale_count_when_one_packet_per_tick(void **state) {
    (void)state;

    struct NWBuffer rx_buf;
    struct NWBuffer tx_buf = {0};
    uint8_t received_msg_count = 0;
    uint16_t expected_length = (uint16_t)build_abs_pos_packet(&rx_buf, 10.0, 1.0);

    process_received_buffer(&rx_buf, &tx_buf, &received_msg_count, expected_length);

    get_joint_config(0, CORE1, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
                     NULL, NULL, NULL, NULL);

    assert_int_equal(get_and_reset_stale_count(0), 0);
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test_setup(
            test_stale_count_accumulates_when_core1_reads, test_setup),
        cmocka_unit_test_setup(
            test_no_stale_count_when_one_packet_per_tick, test_setup),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
