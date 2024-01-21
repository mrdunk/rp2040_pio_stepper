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


static void test_serialise_timing(void **state) {
    (void) state; /* unused */

    struct NWBuffer tx_buf = {0};
    uint16_t initial_tx_buf_len = 42;
    tx_buf.length = initial_tx_buf_len;
    uint32_t update_id = 1234;
    uint32_t time_diff = 5678;

    config.update_time_us = 9012;

    assert_int_equal(tx_buf.checksum, 0);

    bool result = serialise_timing(&tx_buf, update_id, time_diff);
    
    struct Reply_timing reply = {
        .type = REPLY_TIMING,
        .update_id = update_id,
        .time_diff = time_diff,
        .rp_update_len = config.update_time_us
    };

    assert_memory_equal(tx_buf.payload + initial_tx_buf_len, &reply, sizeof(reply));
    assert_int_equal(tx_buf.length - initial_tx_buf_len, sizeof(reply));
    assert_int_equal(result, true);
    assert_int_not_equal(tx_buf.checksum, 0);
}

/* Out of space in the buffer causes error. */
static void test_serialise_timing_overflow(void **state) {
    (void) state; /* unused */

    struct NWBuffer tx_buf = {0};
    uint16_t initial_tx_buf_len =
        NW_BUF_LEN - sizeof(struct Reply_timing) + 1;
    tx_buf.length = initial_tx_buf_len;
    uint32_t update_id = 1234;
    uint32_t time_diff = 5678;

    assert_int_equal(tx_buf.checksum, 0);
    
    bool result = serialise_timing(&tx_buf, update_id, time_diff);
    
    assert_int_equal(tx_buf.length, initial_tx_buf_len);
    assert_int_equal(tx_buf.checksum, 0);
    assert_int_equal(result, false);
}

static void test_serialise_axis_movement(void **state) {
    (void) state; /* unused */

    struct NWBuffer tx_buf = {0};
    uint16_t initial_tx_buf_len = 42;
    tx_buf.length = initial_tx_buf_len;
    size_t axis = 0;
    size_t axis_count = 0;

    config.axis[0].abs_pos_acheived = 123;
    config.axis[0].max_velocity = 456;
    config.axis[0].max_accel_ticks = 789;
    config.axis[0].velocity_requested = 135;
    config.axis[0].velocity_acheived = 791;
    config.axis[0].pos_error = 357;
    config.axis[0].step_len_ticks = 246;
    config.axis[0].updated_from_c1 = 1;

    assert_int_equal(tx_buf.length, initial_tx_buf_len);
    assert_int_equal(tx_buf.checksum, 0);

    axis_count += serialise_axis_movement(axis, &tx_buf, true);

    struct Reply_axis_movement reply = {
        .type = REPLY_AXIS_MOVEMENT,
        .axis = 0,
        .abs_pos_acheived = config.axis[0].abs_pos_acheived,
        //.max_velocity = config.axis[0].max_velocity,
        //.max_accel_ticks = config.axis[0].max_accel_ticks,
        //.velocity_requested = config.axis[0].velocity_requested,
        .velocity_acheived = config.axis[0].velocity_acheived,
        //.step_len_ticks = config.axis[0].step_len_ticks,
    };

    assert_memory_equal(tx_buf.payload + initial_tx_buf_len, &reply, sizeof(reply));
    assert_int_equal(tx_buf.length - initial_tx_buf_len, sizeof(reply));
    assert_int_equal(axis_count, 1);
    assert_int_not_equal(tx_buf.checksum, 0);
}

static void test_serialise_axis_movement_multi(void **state) {
    (void) state; /* unused */

    struct NWBuffer tx_buf = {0};
    uint16_t initial_tx_buf_len = 42;
    tx_buf.length = initial_tx_buf_len;
    size_t axis_count = 0;

    struct Reply_axis_movement replies[4];

    for(size_t axis = 0; axis < 4; axis++) {
        config.axis[axis].abs_pos_acheived = 123;
        config.axis[axis].max_velocity = 456;
        config.axis[axis].max_accel_ticks = 789;
        config.axis[axis].velocity_requested = 135;
        config.axis[axis].velocity_acheived = 791;
        config.axis[axis].pos_error = 357;
        config.axis[axis].step_len_ticks = 246;
        config.axis[axis].updated_from_c1 = 1;

        replies[axis].type = REPLY_AXIS_MOVEMENT;
        replies[axis].axis = axis;
        replies[axis].abs_pos_acheived = config.axis[axis].abs_pos_acheived;
        //replies[axis].max_velocity = config.axis[axis].max_velocity;
        //replies[axis].max_accel_ticks = config.axis[axis].max_accel_ticks;
        //replies[axis].velocity_requested = config.axis[axis].velocity_requested;
        replies[axis].velocity_acheived = config.axis[axis].velocity_acheived;
        //replies[axis].step_len_ticks = config.axis[axis].step_len_ticks;
    }

    assert_int_equal(tx_buf.length, initial_tx_buf_len);
    assert_int_equal(tx_buf.checksum, 0);

    for(size_t axis = 0; axis < 4; axis++) {
        axis_count += serialise_axis_movement(axis, &tx_buf, true);

        assert_memory_equal(
                tx_buf.payload + initial_tx_buf_len + (axis * sizeof(replies[axis])),
                &replies[axis], sizeof(replies[axis]));
        assert_int_equal(tx_buf.length, initial_tx_buf_len + ((axis + 1) * sizeof(replies[axis])));
        assert_int_equal(axis_count, axis + 1);
        assert_int_not_equal(tx_buf.checksum, 0);
    }
}

/* Out of space in the buffer causes error. */
static void test_serialise_overflow(void **state) {
    (void) state; /* unused */

    struct NWBuffer tx_buf = {0};
    uint16_t initial_tx_buf_len =
        NW_BUF_LEN - sizeof(struct Reply_axis_movement) + 1;
    tx_buf.length = initial_tx_buf_len;

    config.axis[0].updated_from_c1 = 1;

    assert_int_equal(tx_buf.checksum, 0);
    
    size_t axis_count = serialise_axis_movement(0, &tx_buf, true);

    assert_int_equal(tx_buf.length, initial_tx_buf_len);
    assert_int_equal(tx_buf.checksum, 0);
    assert_int_equal(axis_count, 0);
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_serialise_timing),
        cmocka_unit_test(test_serialise_timing_overflow),
        cmocka_unit_test(test_serialise_axis_movement),
        cmocka_unit_test(test_serialise_axis_movement_multi),
        cmocka_unit_test(test_serialise_overflow)
    };

    return cmocka_run_group_tests(tests, NULL, NULL);
}

