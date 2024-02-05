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
        config.joint[joint].abs_pos_acheived = 123;
        config.joint[joint].max_velocity = 456;
        config.joint[joint].max_accel = 789;
        config.joint[joint].velocity_requested = 135;
        config.joint[joint].velocity_acheived = 791;
        config.joint[joint].step_len_ticks = 246;
        config.joint[joint].updated_from_c1 = 1;

        reply.abs_pos_acheived[joint] = config.joint[joint].abs_pos_acheived;
        reply.velocity_acheived[joint] = config.joint[joint].velocity_acheived;
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
        assert_int_equal(reply_p->abs_pos_acheived[joint], reply.abs_pos_acheived[joint]);
        assert_int_equal(reply_p->velocity_acheived[joint], reply.velocity_acheived[joint]);
    }
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
        cmocka_unit_test(test_serialise_overflow)
    };

    return cmocka_run_group_tests(tests, NULL, NULL);
}

