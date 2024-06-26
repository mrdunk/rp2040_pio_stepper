#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <stdint.h>
#include <cmocka.h>

#include <stdio.h>
#include <string.h>

#include "../shared/messages.h"
#include "../shared/buffer.h"
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
size_t append_enable_message(struct NWBuffer* rx_buf, uint32_t joint, uint8_t value) {

    struct Message_joint_enable message = {
        .type = MSG_SET_JOINT_ENABLED,
        .joint = joint,
        .value = value
    };

    return pack_nw_buff(rx_buf, &message, sizeof(message));
}

/* Helper function.
 * Populates the buffer with the provided struct. */
size_t append_message(struct NWBuffer* rx_buf, union MessageAny message) {
    switch(message.header.type) {
        case MSG_TIMING:
            return pack_nw_buff(rx_buf, &message.timing, sizeof(message.timing));
        case MSG_SET_JOINT_ENABLED:
            return pack_nw_buff(rx_buf, &message.joint_enable, sizeof(message.joint_enable));
        case MSG_SET_JOINT_ABS_POS:
            return pack_nw_buff(rx_buf, &message.set_abs_pos, sizeof(message.set_abs_pos));
        case MSG_SET_JOINT_CONFIG:
            return pack_nw_buff(rx_buf, &message.joint_config, sizeof(message.joint_config));
        default:
            printf("TEST HELPER ERROR: Invalid message type: %u\n", message.header.type);
            break;
    }
    return 0;
}

static void test_unpack_multiple_message(void **state) {
    (void) state; /* unused */

    struct NWBuffer rx_buf = {0};
    struct NWBuffer tx_buf = {0};
    uint8_t received_msg_count = 0;
    uint16_t expected_length = sizeof(rx_buf.length) + sizeof(rx_buf.checksum);

    expected_length += append_enable_message(&rx_buf, 0, 0);
    expected_length += append_enable_message(&rx_buf, 1, 1);
    expected_length += append_enable_message(&rx_buf, 2, 0);

    process_received_buffer(&rx_buf, &tx_buf, &received_msg_count, expected_length);

    assert_int_equal(tx_buf.length, 0);
    // 3 messages processed.
    assert_int_equal(received_msg_count, 3);
}

static void test_unpack_multiple_message_stop_at_length(void **state) {
    (void) state; /* unused */

    struct NWBuffer rx_buf = {0};
    struct NWBuffer tx_buf = {0};
    uint8_t received_msg_count = 0;
    uint16_t expected_length = sizeof(rx_buf.length) + sizeof(rx_buf.checksum);

    expected_length += append_enable_message(&rx_buf, 0, 0);
    expected_length += append_enable_message(&rx_buf, 1, 1);
    expected_length += append_enable_message(&rx_buf, 2, 0);

    // Artificially reduce the expected data length in the buffer.
    rx_buf.length -= 1;
    expected_length -= 1;
    // Fix checksum after changing length.
    uint32_t cs = 0;
    rx_buf.checksum = checksum(
            cs, 0, alligned32(sizeof(struct Message_joint_enable)) * 3 -1, &rx_buf.payload);

    process_received_buffer(&rx_buf, &tx_buf, &received_msg_count, expected_length);

    assert_int_equal(tx_buf.length, 0);
    // 2 messages processed.
    assert_int_equal(received_msg_count, 2);
}

/* Test Message_joint_enable struct, MSG_SET_JOINT_ENABLED type works as intended. */
static void test_unpack_joint_enable_message(void **state) {
    (void) state; /* unused */

    struct NWBuffer rx_buf = {0};
    struct NWBuffer tx_buf = {0};
    uint8_t received_msg_count = 0;
    uint16_t expected_length = sizeof(rx_buf.length) + sizeof(rx_buf.checksum);

    expected_length += append_enable_message(&rx_buf, 2, 1);

    process_received_buffer(&rx_buf, &tx_buf, &received_msg_count, expected_length);

    assert_int_equal(tx_buf.length, 0);
    // 1 message processed.
    assert_int_equal(received_msg_count, 1);

    // The update_joint_config(...) method has not been mocked
    // so this will result in the config actually changing.
    assert_int_equal(config.joint[2].enabled, 1);
}

/* Test unpacking the struct Message_timing type works as intended. */
static void test_unpack_timing_message(void **state) {
    (void) state; /* unused */

    struct NWBuffer rx_buf = {0};
    struct NWBuffer tx_buf = {0};
    uint8_t received_msg_count = 0;
    uint16_t expected_length = sizeof(rx_buf.length) + sizeof(rx_buf.checksum);

    struct Message_timing timing = {0};
    timing.type = MSG_TIMING;
    timing.update_id = 1234;
    timing.time = 5678;

    union MessageAny message;
    message.timing = timing;

    expected_length += append_message(&rx_buf, message);
    assert_memory_equal(&rx_buf.payload, &message, sizeof(struct Message_timing));

    process_received_buffer(&rx_buf, &tx_buf, &received_msg_count, expected_length);

    // 1 message processed.
    assert_int_equal(received_msg_count, 1);

    // The update_packet_metrics(...) method has not been mocked
    // so this will result in the config actually changing.
    assert_int_equal(config.last_update_id, 1234);
    assert_int_equal(config.last_update_time, 5678);

    // The serialise_timing(...) method has not been mocked
    // so there will be data on the tx_buf.
    assert_int_equal(tx_buf.length, sizeof(struct Reply_timing));
}

/* Test unpacking the struct Message_set_joints_pos works as intended. */
static void test_unpack_set_abs_pos_message(void **state) {
    (void) state; /* unused */

    struct NWBuffer rx_buf = {0};
    struct NWBuffer tx_buf = {0};
    uint8_t received_msg_count = 0;
    uint16_t expected_length = sizeof(rx_buf.length) + sizeof(rx_buf.checksum);

    struct Message_set_joints_pos message_set_abs_pos = {0};
    message_set_abs_pos.type = MSG_SET_JOINT_ABS_POS;

    double p = 12.34;
    double v = 56.78;
    for(size_t joint = 0; joint < MAX_JOINT; joint++) {
        message_set_abs_pos.position[joint] = p * (joint + 1);
        message_set_abs_pos.velocity[joint] = v * (joint + 1);
    }

    union MessageAny message = {0};
    message.set_abs_pos = message_set_abs_pos;

    expected_length += append_message(&rx_buf, message);
    assert_memory_equal(&rx_buf.payload, &message, sizeof(struct Message_set_joints_pos));

    process_received_buffer(&rx_buf, &tx_buf, &received_msg_count, expected_length);

    assert_int_equal(tx_buf.length, 0);
    // 1 message processed.
    assert_int_equal(received_msg_count, 1);

    // The update_joint_config(...) method has not been mocked
    // so this will result in the config actually changing.
    for(size_t joint = 0; joint < MAX_JOINT; joint++) {
        assert_double_equal(
                config.joint[joint].abs_pos_requested,
                message_set_abs_pos.position[joint],
                0.01);
        assert_double_equal(
                config.joint[joint].abs_pos_requested,
                message_set_abs_pos.position[joint],
                0.01);
    }
}

/* Test unpacking the struct Message_joint_config works as intended. */
static void test_unpack_joint_config_message(void **state) {
    (void) state; /* unused */

    struct NWBuffer rx_buf = {0};
    struct NWBuffer tx_buf = {0};
    uint8_t received_msg_count = 0;
    uint16_t expected_length = sizeof(rx_buf.length) + sizeof(rx_buf.checksum);

    struct Message_joint_config joint_config = {0};
    joint_config.type = MSG_SET_JOINT_CONFIG;
    joint_config.joint = 2;
    joint_config.enable = 1;
    joint_config.gpio_step = 1;
    joint_config.gpio_dir = 2;
    joint_config.max_velocity = 12.34;
    joint_config.max_accel = 56.78;

    union MessageAny message = {0};
    message.joint_config = joint_config;

    expected_length += append_message(&rx_buf, message);
    assert_memory_equal(&rx_buf.payload, &message, sizeof(struct Message_joint_config));

    process_received_buffer(&rx_buf, &tx_buf, &received_msg_count, expected_length);

    // The MSG_SET_JOINT_CONFIG populates tx_buf.
    assert_int_equal(tx_buf.length, sizeof(struct Reply_joint_config));
    // 1 message processed.
    assert_int_equal(received_msg_count, 1);

    // The update_joint_config(...) method has not been mocked
    // so this will result in the config actually changing.
    assert_int_equal(config.joint[2].enabled, 1);
    assert_int_equal(config.joint[2].io_pos_step, 1);
    assert_int_equal(config.joint[2].io_pos_dir, 2);
    assert_double_equal(config.joint[2].max_velocity, 12.34, 0.01);
    assert_double_equal(config.joint[2].max_accel, 56.78, 0.01);
}

static void test_unpack_unknown_message_type(void **state) {
    (void) state; /* unused */

    struct NWBuffer rx_buf = {0};
    struct NWBuffer tx_buf = {0};
    uint8_t received_msg_count = 0;
    uint16_t expected_length = sizeof(rx_buf.length) + sizeof(rx_buf.checksum);

    struct Message_joint_config joint_config = {
        .type = MSG_SET_JOINT_CONFIG,
        .joint = 3,
        .enable = 1,
        .gpio_step = 2,
        .gpio_dir = 3,
        .max_velocity = 13.34,
        .max_accel = 57.78
    };

    union MessageAny message;
    message.joint_config = joint_config;

    // Append 2 messages.
    expected_length += append_message(&rx_buf, message);
    expected_length += append_message(&rx_buf, message);

    struct Message_joint_config* message_p = (void*)rx_buf.payload;
    assert_int_equal(message_p->type, message.joint_config.type);
    assert_int_equal(message_p->joint, message.joint_config.joint);
    assert_int_equal(message_p->enable, message.joint_config.enable);
    assert_int_equal(message_p->gpio_step, message.joint_config.gpio_step);
    assert_int_equal(message_p->gpio_dir, message.joint_config.gpio_dir);
    assert_int_equal(message_p->max_velocity, message.joint_config.max_velocity);
    assert_int_equal(message_p->max_accel, message.joint_config.max_accel);

    struct Message_joint_config* message2_p = (void*)rx_buf.payload;
    assert_int_equal(message2_p->type, message.joint_config.type);
    assert_int_equal(message2_p->joint, message.joint_config.joint);
    assert_int_equal(message2_p->enable, message.joint_config.enable);
    assert_int_equal(message2_p->gpio_step, message.joint_config.gpio_step);
    assert_int_equal(message2_p->gpio_dir, message.joint_config.gpio_dir);
    assert_int_equal(message2_p->max_velocity, message.joint_config.max_velocity);
    assert_int_equal(message2_p->max_accel, message.joint_config.max_accel);

    // Everything normal so far.
    // Now let's break the first message type.
    ((struct Message_joint_config*)(rx_buf.payload))->type = 123;

    process_received_buffer(&rx_buf, &tx_buf, &received_msg_count, expected_length);

    // No messages processed.
    assert_int_equal(received_msg_count, 0);

    // The update_joint_config(...) method has not been mocked
    // so this would result in the config actually changing if we had processed the
    // 2nd message.
    assert_int_not_equal(config.joint[3].enabled, 1);
    assert_int_not_equal(config.joint[3].io_pos_step, 2);
    assert_int_not_equal(config.joint[3].io_pos_dir, 3);
    assert_double_not_equal(config.joint[3].max_velocity, 13.34, 0.01);
    assert_double_not_equal(config.joint[3].max_accel, 57.78, 0.01);

    // Should have reset the rx_buf.
    assert_int_equal(rx_buf.length, 0);
    assert_int_equal(rx_buf.checksum, 0);
}

static void test_unpack_one_of_each(void **state) {
    (void) state; /* unused */

    struct NWBuffer rx_buf = {0};
    struct NWBuffer tx_buf = {0};
    uint8_t received_msg_count = 0;
    uint16_t expected_length = sizeof(rx_buf.length) + sizeof(rx_buf.checksum);

    struct Message_timing timing = {
        .type = MSG_TIMING,
        .update_id = 1234,
        .time = 5678
    };
    expected_length += append_message(&rx_buf, (union MessageAny)timing);

    struct Message_joint_enable enable = {
        .type = MSG_SET_JOINT_ENABLED,
        .joint = 1,
        .value = 1
    };
    expected_length += append_message(&rx_buf, (union MessageAny)enable);

    struct Message_set_joints_pos message_set_abs_pos = {
        .type = MSG_SET_JOINT_ABS_POS,
        .position = {34.56, 78.90, 12.34, 56.78},
        .velocity = {78.91, 23.45, 67.89, 1.23}
    };
    expected_length += append_message(&rx_buf, (union MessageAny)message_set_abs_pos);

    struct Message_joint_config joint_config = {
        .type = MSG_SET_JOINT_CONFIG,
        .joint = 2,
        .enable = 1,
        .gpio_step = 1,
        .gpio_dir = 2,
        .max_velocity = 12.34,
        .max_accel = 56.78
    };
    expected_length += append_message(&rx_buf, (union MessageAny)joint_config);


    process_received_buffer(&rx_buf, &tx_buf, &received_msg_count, expected_length);

    // 4 messages processed.
    assert_int_equal(received_msg_count, 4);

    // The MSG_TIMING and MSG_SET_JOINT_CONFIG populates tx_buf.
    assert_int_equal(tx_buf.length, sizeof(struct Reply_timing) + sizeof(struct Reply_joint_config));

    // Should have reset the rx_buf.
    assert_int_equal(rx_buf.length, 0);
    assert_int_equal(rx_buf.checksum, 0);
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_unpack_multiple_message),
        cmocka_unit_test(test_unpack_multiple_message_stop_at_length),
        cmocka_unit_test(test_unpack_joint_enable_message),
        cmocka_unit_test(test_unpack_timing_message),
        cmocka_unit_test(test_unpack_set_abs_pos_message),
        cmocka_unit_test(test_unpack_joint_config_message),
        cmocka_unit_test(test_unpack_unknown_message_type),
        cmocka_unit_test(test_unpack_one_of_each)
    };

    return cmocka_run_group_tests(tests, NULL, NULL);
}

