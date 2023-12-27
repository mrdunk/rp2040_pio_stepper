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
size_t append_enable_message(struct NWBuffer* rx_buf, uint32_t axis, uint8_t value) {

    struct Message_joint_enable message = {
        .type = MSG_SET_AXIS_ENABLED,
        .axis = axis,
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
        case MSG_SET_AXIS_ENABLED:
            return pack_nw_buff(rx_buf, &message.joint_enable, sizeof(message.joint_enable));
        case MSG_SET_AXIS_MAX_VELOCITY:
            return pack_nw_buff(rx_buf, &message.set_max_velocity, sizeof(message.set_max_velocity));
        case MSG_SET_AXIS_MAX_ACCEL:
            return pack_nw_buff(rx_buf, &message.set_max_accel, sizeof(message.set_max_accel));
        case MSG_SET_AXIS_ABS_POS:
            return pack_nw_buff(rx_buf, &message.set_abs_pos, sizeof(message.set_abs_pos));
        case MSG_SET_AXIS_VELOCITY:
            return pack_nw_buff(rx_buf, &message.set_velocity, sizeof(message.set_velocity));
        case MSG_SET_AXIS_IO_STEP:
        case MSG_SET_AXIS_IO_DIR:
            return pack_nw_buff(rx_buf, &message.joint_gpio, sizeof(message.joint_gpio));
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
            cs, &rx_buf.payload, sizeof(struct Message_joint_enable) * 3 -1);

    process_received_buffer(&rx_buf, &tx_buf, &received_msg_count, expected_length);

    assert_int_equal(tx_buf.length, 0);
    // 2 messages processed.
    assert_int_equal(received_msg_count, 2);
}

/* Test Message_joint_enable struct, MSG_SET_AXIS_ENABLED type works as intended. */
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

    // The update_axis_config(...) method has not been mocked
    // so this will result in the config actually changing.
    assert_int_equal(config.axis[2].enabled, 1);
}

/* Test unpacking the struct Message_timing type works as intended. */
static void test_unpack_timing_message(void **state) {
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

    // The serialise_metrics(...) method has not been mocked
    // so there will be data on the tx_buf.
    assert_int_equal(tx_buf.length, sizeof(struct Reply_metrics));
}

/* Test unpacking the struct Message_set_abs_pos works as intended. */
static void test_unpack_set_abs_pos_message(void **state) {
    (void) state; /* unused */

    struct NWBuffer rx_buf = {0};
    struct NWBuffer tx_buf = {0};
    uint8_t received_msg_count = 0;
    uint16_t expected_length = sizeof(rx_buf.length) + sizeof(rx_buf.checksum);

    struct Message_set_abs_pos message_set_abs_pos = {
        .type = MSG_SET_AXIS_ABS_POS,
        .axis = 2,
        .value = 34.56
    };

    union MessageAny message;
    message.set_abs_pos = message_set_abs_pos;

    expected_length += append_message(&rx_buf, message);
    assert_memory_equal(&rx_buf.payload, &message, sizeof(struct Message_set_abs_pos));

    process_received_buffer(&rx_buf, &tx_buf, &received_msg_count, expected_length);

    assert_int_equal(tx_buf.length, 0);
    // 1 message processed.
    assert_int_equal(received_msg_count, 1);

    // The update_axis_config(...) method has not been mocked
    // so this will result in the config actually changing.
    assert_double_equal(config.axis[2].abs_pos_requested, 34.56, 0.01);
}

/* Test unpacking the struct Message_set_velocity works as intended. */
static void test_unpack_set_velocity_message(void **state) {
    (void) state; /* unused */

    struct NWBuffer rx_buf = {0};
    struct NWBuffer tx_buf = {0};
    uint8_t received_msg_count = 0;
    uint16_t expected_length = sizeof(rx_buf.length) + sizeof(rx_buf.checksum);

    struct Message_set_velocity message_set_velocity = {
        .type = MSG_SET_AXIS_VELOCITY,
        .axis = 3,
        .value = 45.67
    };

    union MessageAny message;
    message.set_velocity = message_set_velocity;

    expected_length += append_message(&rx_buf, message);
    assert_memory_equal(&rx_buf.payload, &message, sizeof(struct Message_set_velocity));

    process_received_buffer(&rx_buf, &tx_buf, &received_msg_count, expected_length);

    assert_int_equal(tx_buf.length, 0);
    // 1 message processed.
    assert_int_equal(received_msg_count, 1);

    // The update_axis_config(...) method has not been mocked
    // so this will result in the config actually changing.
    assert_double_equal(config.axis[3].rel_pos_requested, 45.67, 0.01);
}

/* Test unpacking the struct Message_set_max_velocity works as intended. */
static void test_unpack_set_max_velocity_message(void **state) {
    (void) state; /* unused */

    struct NWBuffer rx_buf = {0};
    struct NWBuffer tx_buf = {0};
    uint8_t received_msg_count = 0;
    uint16_t expected_length = sizeof(rx_buf.length) + sizeof(rx_buf.checksum);

    struct Message_set_max_velocity message_set_max_velocity = {
        .type = MSG_SET_AXIS_MAX_VELOCITY,
        .axis = 0,
        .value = 56.78
    };

    union MessageAny message;
    message.set_max_velocity = message_set_max_velocity;

    expected_length += append_message(&rx_buf, message);
    assert_memory_equal(&rx_buf.payload, &message, sizeof(struct Message_set_max_velocity));

    process_received_buffer(&rx_buf, &tx_buf, &received_msg_count, expected_length);

    assert_int_equal(tx_buf.length, 0);
    // 1 message processed.
    assert_int_equal(received_msg_count, 1);

    // The update_axis_config(...) method has not been mocked
    // so this will result in the config actually changing.
    assert_double_equal(config.axis[0].max_velocity, 56.78, 0.01);
}

/* Test unpacking the struct Message_set_max_accel works as intended. */
static void test_unpack_set_max_accel_message(void **state) {
    (void) state; /* unused */

    struct NWBuffer rx_buf = {0};
    struct NWBuffer tx_buf = {0};
    uint8_t received_msg_count = 0;
    uint16_t expected_length = sizeof(rx_buf.length) + sizeof(rx_buf.checksum);

    struct Message_set_max_accel message_set_max_accel = {
        .type = MSG_SET_AXIS_MAX_ACCEL,
        .axis = 1,
        .value = 23.45
    };

    union MessageAny message;
    message.set_max_accel = message_set_max_accel;

    expected_length += append_message(&rx_buf, message);
    assert_memory_equal(&rx_buf.payload, &message, sizeof(struct Message_set_max_accel));

    process_received_buffer(&rx_buf, &tx_buf, &received_msg_count, expected_length);

    assert_int_equal(tx_buf.length, 0);
    // 1 message processed.
    assert_int_equal(received_msg_count, 1);

    // The update_axis_config(...) method has not been mocked
    // so this will result in the config actually changing.
    assert_double_equal(config.axis[1].max_accel_ticks, 23.45, 0.01);
}

/* Test unpacking the struct Message_joint_gpio works as intended. */
static void test_unpack_joint_gpio_step_message(void **state) {
    (void) state; /* unused */

    struct NWBuffer rx_buf = {0};
    struct NWBuffer tx_buf = {0};
    //uint8_t tx_buf[DATA_BUF_SIZE] = {0};
    uint8_t received_msg_count = 0;
    uint16_t expected_length = sizeof(rx_buf.length) + sizeof(rx_buf.checksum);

    struct Message_joint_gpio joint_gpio = {
        .type = MSG_SET_AXIS_IO_STEP,
        .axis = 1,
        .value = 2
    };

    union MessageAny message;
    message.joint_gpio = joint_gpio;

    expected_length += append_message(&rx_buf, message);
    assert_memory_equal(&rx_buf.payload, &message, sizeof(struct Message_joint_gpio));

    process_received_buffer(&rx_buf, &tx_buf, &received_msg_count, expected_length);

    assert_int_equal(tx_buf.length, 0);
    // 1 message processed.
    assert_int_equal(received_msg_count, 1);

    // The update_axis_config(...) method has not been mocked
    // so this will result in the config actually changing.
    assert_int_equal(config.axis[1].io_pos_step, 2);
}

/* Test unpacking the struct Message_joint_gpio works as intended. */
static void test_unpack_joint_gpio_dir_message(void **state) {
    (void) state; /* unused */

    struct NWBuffer rx_buf = {0};
    struct NWBuffer tx_buf = {0};
    //uint8_t tx_buf[DATA_BUF_SIZE] = {0};
    uint8_t received_msg_count = 0;
    uint16_t expected_length = sizeof(rx_buf.length) + sizeof(rx_buf.checksum);

    struct Message_joint_gpio joint_gpio = {
        .type = MSG_SET_AXIS_IO_DIR,
        .axis = 2,
        .value = 3
    };

    union MessageAny message;
    message.joint_gpio = joint_gpio;

    expected_length += append_message(&rx_buf, message);
    assert_memory_equal(&rx_buf.payload, &message, sizeof(struct Message_joint_gpio));

    process_received_buffer(&rx_buf, &tx_buf, &received_msg_count, expected_length);

    assert_int_equal(tx_buf.length, 0);
    // 1 message processed.
    assert_int_equal(received_msg_count, 1);

    // The update_axis_config(...) method has not been mocked
    // so this will result in the config actually changing.
    assert_int_equal(config.axis[2].io_pos_dir, 3);
}

static void test_unpack_one_of_each(void **state) {
    (void) state; /* unused */

    struct NWBuffer rx_buf = {0};
    struct NWBuffer tx_buf = {0};
    //uint8_t tx_buf[DATA_BUF_SIZE] = {0};
    uint8_t received_msg_count = 0;
    uint16_t expected_length = sizeof(rx_buf.length) + sizeof(rx_buf.checksum);

    struct Message_timing timing = {
        .type = MSG_TIMING,
        .update_id = 1234,
        .time = 5678
    };
    expected_length += append_message(&rx_buf, (union MessageAny)timing);

    struct Message_joint_enable enable = {
        .type = MSG_SET_AXIS_ENABLED,
        .axis = 1,
        .value = 1
    };
    expected_length += append_message(&rx_buf, (union MessageAny)enable);

    struct Message_set_abs_pos message_set_abs_pos = {
        .type = MSG_SET_AXIS_ABS_POS,
        .axis = 2,
        .value = 34.56
    };
    expected_length += append_message(&rx_buf, (union MessageAny)message_set_abs_pos);

    struct Message_set_velocity message_set_velocity = {
        .type = MSG_SET_AXIS_VELOCITY,
        .axis = 3,
        .value = 45.67
    };
    expected_length += append_message(&rx_buf, (union MessageAny)message_set_velocity);

    struct Message_set_max_velocity message_set_max_velocity = {
        .type = MSG_SET_AXIS_MAX_VELOCITY,
        .axis = 0,
        .value = 56.78
    };
    expected_length += append_message(&rx_buf, (union MessageAny)message_set_max_velocity);

    struct Message_set_max_accel message_set_max_accel = {
        .type = MSG_SET_AXIS_MAX_ACCEL,
        .axis = 1,
        .value = 23.45
    };
    expected_length += append_message(&rx_buf, (union MessageAny)message_set_max_accel);

    struct Message_joint_gpio joint_gpio_step = {
        .type = MSG_SET_AXIS_IO_STEP,
        .axis = 1,
        .value = 2
    };
    expected_length += append_message(&rx_buf, (union MessageAny)joint_gpio_step);

    struct Message_joint_gpio joint_gpio_dir = {
        .type = MSG_SET_AXIS_IO_DIR,
        .axis = 1,
        .value = 2
    };
    expected_length += append_message(&rx_buf, (union MessageAny)joint_gpio_dir);


    process_received_buffer(&rx_buf, &tx_buf, &received_msg_count, expected_length);

    // 8 messages processed.
    assert_int_equal(received_msg_count, 8);

    // The MSG_TIMING populates tx_buf.
    assert_int_equal(tx_buf.length, sizeof(struct Reply_metrics));

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
        cmocka_unit_test(test_unpack_set_velocity_message),
        cmocka_unit_test(test_unpack_set_max_velocity_message),
        cmocka_unit_test(test_unpack_set_max_accel_message),
        cmocka_unit_test(test_unpack_joint_gpio_step_message),
        cmocka_unit_test(test_unpack_joint_gpio_dir_message),
        cmocka_unit_test(test_unpack_one_of_each)
    };

    return cmocka_run_group_tests(tests, NULL, NULL);
}

