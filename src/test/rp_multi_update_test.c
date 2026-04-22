/* rp_multi_update_test.c
 *
 * Regression tests for overrun/underrun counting.
 *
 * overrun:  Core0 drains N queued W5500 packets between ticks — updated_from_c0
 *           accumulates to N before Core1 reads.  N-1 packets are discarded
 *           (only the latest position/velocity matters); overrun_count += N-1.
 *
 * underrun: Core1 fires but Core0 has not yet received a new packet —
 *           updated_from_c0 == 0; underrun_count++.
 *
 * Both counts are returned to LinuxCNC via Reply_joint_metrics.
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
    return 0;
}

/* After N packets arrive and Core1 reads once, overrun_count == N-1. */
static void test_overrun_count_accumulates_when_core1_reads(void **state) {
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

    get_joint_config(0, CORE1, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
                     NULL);

    assert_int_equal(get_and_reset_overrun_count(0), N - 1);
    assert_int_equal(get_and_reset_overrun_count(0), 0);  /* resets on read */
}

/* One packet per tick: no overrun, no underrun. */
static void test_no_overrun_when_one_packet_per_tick(void **state) {
    (void)state;

    struct NWBuffer rx_buf;
    struct NWBuffer tx_buf = {0};
    uint8_t received_msg_count = 0;
    uint16_t expected_length = (uint16_t)build_abs_pos_packet(&rx_buf, 10.0, 1.0);

    process_received_buffer(&rx_buf, &tx_buf, &received_msg_count, expected_length);

    get_joint_config(0, CORE1, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
                     NULL);

    assert_int_equal(get_and_reset_overrun_count(0), 0);
    assert_int_equal(get_and_reset_underrun_count(0), 0);
}

/* Core1 reads before any packet arrives: underrun_count increments. */
static void test_underrun_count_when_core1_reads_before_packet(void **state) {
    (void)state;

    /* No packets sent — updated_from_c0 stays 0. */
    get_joint_config(0, CORE1, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
                     NULL);

    assert_int_equal(get_and_reset_underrun_count(0), 1);
    assert_int_equal(get_and_reset_underrun_count(0), 0);  /* resets on read */
    assert_int_equal(get_and_reset_overrun_count(0), 0);   /* overrun unaffected */
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test_setup(
            test_overrun_count_accumulates_when_core1_reads, test_setup),
        cmocka_unit_test_setup(
            test_no_overrun_when_one_packet_per_tick, test_setup),
        cmocka_unit_test_setup(
            test_underrun_count_when_core1_reads_before_packet, test_setup),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
