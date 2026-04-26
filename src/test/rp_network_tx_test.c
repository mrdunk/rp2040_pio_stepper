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

    struct Reply_timing* reply_p = (void*)tx_buf.payload + initial_tx_buf_len;
    assert_int_equal(reply_p->type, reply.type);
    assert_int_equal(reply_p->update_id, reply.update_id);
    assert_int_equal(reply_p->time_diff, reply.time_diff);
    assert_int_equal(reply_p->rp_update_len, reply.rp_update_len);

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

static void test_serialise_joint_movement(void **state) {
    (void) state; /* unused */

    struct NWBuffer tx_buf = {0};
    uint16_t initial_tx_buf_len = 42;
    tx_buf.length = initial_tx_buf_len;

    struct Reply_joint_movement reply;
    reply.type = REPLY_JOINT_MOVEMENT;

    for(size_t joint = 0; joint < MAX_JOINT; joint++) {
        config.joint[joint].abs_pos_achieved = 123;
        config.joint[joint].max_velocity = 456;
        config.joint[joint].max_accel = 789;
        config.joint[joint].velocity_achieved = 791;
        config.joint[joint].updated_from_c1 = 1;

        reply.abs_pos_achieved[joint] = config.joint[joint].abs_pos_achieved;
        reply.velocity_achieved[joint] = config.joint[joint].velocity_achieved;
    }

    assert_int_equal(tx_buf.length, initial_tx_buf_len);
    assert_int_equal(tx_buf.checksum, 0);

    size_t joint_count = serialise_joint_movement(&tx_buf, true);

    assert_int_equal(tx_buf.length, initial_tx_buf_len + sizeof(struct Reply_joint_movement));
    assert_int_equal(joint_count, 1);
    assert_int_not_equal(tx_buf.checksum, 0);

    struct Reply_joint_movement* reply_p = (void*)tx_buf.payload + initial_tx_buf_len;
    assert_int_equal(reply_p->type, reply.type);

    for(size_t joint = 0; joint < 4; joint++) {
        assert_int_equal(reply_p->abs_pos_achieved[joint], reply.abs_pos_achieved[joint]);
        assert_int_equal(reply_p->velocity_achieved[joint], reply.velocity_achieved[joint]);
    }
}

/* Any overrun/underrun across joints collapses to a single occurred flag. */
static void test_serialise_joint_metrics(void **state) {
    (void) state;

    struct NWBuffer tx_buf = {0};

    for (size_t j = 0; j < MAX_JOINT; j++) {
        config.joint[j].overrun_count  = j + 1;  /* 1, 2, 3, 4 — all non-zero */
        config.joint[j].underrun_count = j + 5;  /* 5, 6, 7, 8 — all non-zero */
    }

    bool result = serialise_joint_metrics(&tx_buf);
    assert_true(result);
    assert_int_equal(tx_buf.length, 4);  /* alligned32(sizeof(Reply_joint_metrics) = 3) */

    struct Reply_joint_metrics* reply = (void*)tx_buf.payload;
    assert_int_equal(reply->type, REPLY_JOINT_METRICS);
    assert_int_equal(reply->overrun_occurred,  1);
    assert_int_equal(reply->underrun_occurred, 1);
    for (size_t j = 0; j < MAX_JOINT; j++) {
        assert_int_equal(config.joint[j].overrun_count,  0);
        assert_int_equal(config.joint[j].underrun_count, 0);
    }
}

/* Only joint 2 has an overrun — occurred flag is still 1. */
static void test_serialise_joint_metrics_partial_events(void **state) {
    (void) state;

    struct NWBuffer tx_buf = {0};
    config.joint[2].overrun_count  = 3;
    config.joint[1].underrun_count = 7;

    bool result = serialise_joint_metrics(&tx_buf);
    assert_true(result);

    struct Reply_joint_metrics* reply = (void*)tx_buf.payload;
    assert_int_equal(reply->overrun_occurred,  1);
    assert_int_equal(reply->underrun_occurred, 1);
    for (size_t j = 0; j < MAX_JOINT; j++) {
        assert_int_equal(config.joint[j].overrun_count,  0);
        assert_int_equal(config.joint[j].underrun_count, 0);
    }
}

/* No events: all joint counts zero — occurred flags stay 0. */
static void test_serialise_joint_metrics_no_events(void **state) {
    (void) state;

    struct NWBuffer tx_buf = {0};

    bool result = serialise_joint_metrics(&tx_buf);
    assert_true(result);

    struct Reply_joint_metrics* reply = (void*)tx_buf.payload;
    assert_int_equal(reply->overrun_occurred,  0);
    assert_int_equal(reply->underrun_occurred, 0);
}

/* Out of space in the buffer causes error. */
static void test_serialise_overflow(void **state) {
    (void) state; /* unused */

    struct NWBuffer tx_buf = {0};
    uint16_t initial_tx_buf_len =
        NW_BUF_LEN - sizeof(struct Reply_joint_movement) + 1;
    tx_buf.length = initial_tx_buf_len;

    config.joint[0].updated_from_c1 = 1;
    config.joint[1].updated_from_c1 = 1;
    config.joint[2].updated_from_c1 = 1;
    config.joint[3].updated_from_c1 = 1;

    assert_int_equal(tx_buf.checksum, 0);
    
    bool joint_count = serialise_joint_movement(&tx_buf, true);

    assert_int_equal(tx_buf.length, initial_tx_buf_len);
    assert_int_equal(tx_buf.checksum, 0);
    assert_int_equal(joint_count, 0);
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_serialise_timing),
        cmocka_unit_test(test_serialise_timing_overflow),
        cmocka_unit_test(test_serialise_joint_movement),
        cmocka_unit_test(test_serialise_joint_metrics),
        cmocka_unit_test(test_serialise_joint_metrics_partial_events),
        cmocka_unit_test(test_serialise_joint_metrics_no_events),
        cmocka_unit_test(test_serialise_overflow)
    };

    return cmocka_run_group_tests(tests, NULL, NULL);
}

