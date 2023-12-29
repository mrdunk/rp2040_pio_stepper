#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <stdint.h>
#include <cmocka.h>

#include "../driver/rp2040_network.c"
#include "mocks/driver_mocks.h"
#include "../shared/messages.h"



static void test_serialize_timing(void **state) {
    (void) state; /* unused */

    struct NWBuffer buffer = {0};
    assert_int_equal(buffer.checksum, 0);
    assert_int_equal(buffer.length, 0);

    struct Message_timing message = {
        .type = MSG_TIMING,
        .update_id = 1234,
        .time = 5678
    };

    size_t data_size = serialize_timing(&buffer, message.update_id, message.time);

    assert_int_equal(data_size, sizeof(struct Message_timing));
    assert_int_equal(buffer.length, data_size);
    assert_int_not_equal(buffer.checksum, 0);

    struct Message_timing* message_p = (void*)buffer.payload;
    assert_int_equal(message.type, message_p->type);
    assert_int_equal(message.update_id, message_p->update_id);
    assert_int_equal(message.time, message_p->time);
}

static void test_serialize_jont_pos(void **state) {
    (void) state; /* unused */
    
    struct NWBuffer buffer = {0};
    assert_int_equal(buffer.checksum, 0);
    assert_int_equal(buffer.length, 0);

    struct Message_set_abs_pos message = {
        .type = MSG_SET_AXIS_ABS_POS,
        .axis = 1234,
        .value = 56.78
    };

    size_t data_size = serialize_joint_pos(&buffer, message.axis, message.value);

    assert_int_equal(data_size, sizeof(struct Message_set_abs_pos));
    assert_int_equal(buffer.length, data_size);
    assert_int_not_equal(buffer.checksum, 0);

    struct Message_set_abs_pos* message_p = (void*)buffer.payload;
    assert_int_equal(message.type, message_p->type);
    assert_int_equal(message.axis, message_p->axis);
    assert_int_equal(message.value, message_p->value);
}

static void test_serialize_joint_velocity(void **state) {
    (void) state; /* unused */

    struct NWBuffer buffer = {0};
    assert_int_equal(buffer.checksum, 0);
    assert_int_equal(buffer.length, 0);

    struct Message_set_velocity message = {
        .type = MSG_SET_AXIS_VELOCITY,
        .axis = 1234,
        .value = 56.78
    };


    size_t data_size = serialize_joint_velocity(&buffer, message.axis, message.value);

    assert_int_equal(data_size, sizeof(struct Message_set_velocity));
    assert_int_equal(buffer.length, data_size);
    assert_int_not_equal(buffer.checksum, 0);

    struct Message_set_velocity* message_p = (void*)buffer.payload;
    assert_int_equal(message.type, message_p->type);
    assert_int_equal(message.axis, message_p->axis);
    assert_int_equal(message.value, message_p->value);
}

/*
static void test_serialize_joint_max_velocity(void **state) {
    (void) state;

    struct NWBuffer buffer = {0};
    assert_int_equal(buffer.checksum, 0);
    assert_int_equal(buffer.length, 0);

    struct Message_set_max_velocity message = {
        .type = MSG_SET_AXIS_MAX_VELOCITY,
        .axis = 1234,
        .value = 56.78
    };

    size_t data_size = serialize_joint_max_velocity(&buffer, message.axis, message.value);

    assert_int_equal(data_size, sizeof(struct Message_set_max_velocity));
    assert_int_equal(buffer.length, data_size);
    assert_int_not_equal(buffer.checksum, 0);

    struct Message_set_velocity* message_p = (void*)buffer.payload;
    assert_int_equal(message.type, message_p->type);
    assert_int_equal(message.axis, message_p->axis);
    assert_int_equal(message.value, message_p->value);
}

static void test_serialize_joint_max_accel(void **state) {
    (void) state;

    struct NWBuffer buffer = {0};
    assert_int_equal(buffer.checksum, 0);
    assert_int_equal(buffer.length, 0);

    struct Message_set_max_accel message = {
        .type = MSG_SET_AXIS_MAX_ACCEL,
        .axis = 1234,
        .value = 56.78
    };

    size_t data_size = serialize_joint_max_accel(&buffer, message.axis, message.value);

    assert_int_equal(data_size, sizeof(struct Message_set_max_accel));
    assert_int_equal(buffer.length, data_size);
    assert_int_not_equal(buffer.checksum, 0);

    struct Message_set_max_accel* message_p = (void*)buffer.payload;
    assert_int_equal(message.type, message_p->type);
    assert_int_equal(message.axis, message_p->axis);
    assert_int_equal(message.value, message_p->value);
}
*/

static void test_serialize_joint_io_step(void **state) {
    (void) state; /* unused */

    struct NWBuffer buffer = {0};
    assert_int_equal(buffer.checksum, 0);
    assert_int_equal(buffer.length, 0);

    struct Message_joint_gpio message = {
        .type = MSG_SET_AXIS_IO_STEP,
        .axis = 1234,
        .value = 1
    };

    size_t data_size = serialize_joint_io_step(&buffer, message.axis, message.value);

    assert_int_equal(data_size, sizeof(struct Message_joint_gpio));
    assert_int_equal(buffer.length, data_size);
    assert_int_not_equal(buffer.checksum, 0);

    struct Message_joint_gpio* message_p = (void*)buffer.payload;
    assert_int_equal(message.type, message_p->type);
    assert_int_equal(message.axis, message_p->axis);
    assert_int_equal(message.value, message_p->value);
}

static void test_serialize_joint_io_dir(void **state) {
    (void) state; /* unused */

    struct NWBuffer buffer = {0};
    assert_int_equal(buffer.checksum, 0);
    assert_int_equal(buffer.length, 0);

    struct Message_joint_gpio message = {
        .type = MSG_SET_AXIS_IO_DIR,
        .axis = 1234,
        .value = 1
    };

    size_t data_size = serialize_joint_io_dir(&buffer, message.axis, message.value);

    assert_int_equal(data_size, sizeof(struct Message_joint_gpio));
    assert_int_equal(buffer.length, data_size);
    assert_int_not_equal(buffer.checksum, 0);

    struct Message_joint_gpio* message_p = (void*)buffer.payload;
    assert_int_equal(message.type, message_p->type);
    assert_int_equal(message.axis, message_p->axis);
    assert_int_equal(message.value, message_p->value);
}

static void test_serialize_joint_enable(void **state) {
    (void) state; /* unused */

    struct NWBuffer buffer = {0};
    assert_int_equal(buffer.checksum, 0);
    assert_int_equal(buffer.length, 0);

    struct Message_joint_enable message = {
        .type = MSG_SET_AXIS_ENABLED,
        .axis = 1234,
        .value = 1
    };


    size_t data_size = serialize_joint_enable(&buffer, message.axis, message.value);

    assert_int_equal(data_size, sizeof(struct Message_joint_enable));
    assert_int_equal(buffer.length, data_size);
    assert_int_not_equal(buffer.checksum, 0);

    struct Message_joint_enable* message_p = (void*)buffer.payload;
    assert_int_equal(message.type, message_p->type);
    assert_int_equal(message.axis, message_p->axis);
    assert_int_equal(message.value, message_p->value);
}

static void test_serialize_joint_config(void **state) {
    (void) state; /* unused */

    struct NWBuffer buffer = {0};
    assert_int_equal(buffer.checksum, 0);
    assert_int_equal(buffer.length, 0);

    struct Message_joint_config message = {
        .type = MSG_SET_AXIS_CONFIG,
        .axis = 1234,
        .enable = 1,
        .gpio_step = 1,
        .gpio_dir = 2,
        .max_velocity = 12.34,
        .max_accel = 56.78
    };

    size_t data_size = serialize_joint_config(
            &buffer,
            message.axis,
            message.enable,
            message.gpio_step,
            message.gpio_dir,
            message.max_velocity,
            message.max_accel
            );

    assert_int_equal(data_size, sizeof(struct Message_joint_config));
    assert_int_equal(buffer.length, data_size);
    assert_int_not_equal(buffer.checksum, 0);

    struct Message_joint_config* message_p = (void*)buffer.payload;
    assert_int_equal(message.type, message_p->type);
    assert_int_equal(message.axis, message_p->axis);
    assert_int_equal(message.enable, message_p->enable);
    assert_int_equal(message.gpio_step, message_p->gpio_step);
    assert_int_equal(message.gpio_dir, message_p->gpio_dir);
    assert_int_equal(message.max_velocity, message_p->max_velocity);
    assert_int_equal(message.max_accel, message_p->max_accel);
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_serialize_timing),
        cmocka_unit_test(test_serialize_jont_pos),
        cmocka_unit_test(test_serialize_joint_velocity),
        //cmocka_unit_test(test_serialize_joint_max_velocity),
        //cmocka_unit_test(test_serialize_joint_max_accel),
        cmocka_unit_test(test_serialize_joint_io_step),
        cmocka_unit_test(test_serialize_joint_io_dir),
        cmocka_unit_test(test_serialize_joint_enable),
        cmocka_unit_test(test_serialize_joint_config)
    };

    return cmocka_run_group_tests(tests, NULL, NULL);
}

