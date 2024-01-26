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

    struct NWBuffer buffer = {
        .payload={0},
        .checksum=0,
        .length=0
    };

    struct Message_timing message = {
        .type = MSG_TIMING,
        .update_id = 1234,
        .time = 5678
    };

    size_t data_size = serialize_timing(&buffer, message.update_id, message.time);

    assert_int_equal(data_size, alligned32(sizeof(struct Message_timing)));
    assert_int_equal(buffer.length, data_size);
    assert_int_not_equal(buffer.checksum, 0);

    struct Message_timing* message_p = (void*)buffer.payload;
    assert_int_equal(message.type, message_p->type);
    assert_int_equal(message.update_id, message_p->update_id);
    assert_int_equal(message.time, message_p->time);
}

static void test_serialize_jont_pos(void **state) {
    (void) state; /* unused */
    
    struct NWBuffer buffer = {
        .payload={0},
        .checksum=0,
        .length=0
    };

    struct Message_set_abs_pos message = {
        .type = MSG_SET_AXIS_ABS_POS,
        .axis = 123,
        .position = 56.78,
        .velocity = 90.12
    };

    size_t data_size = serialize_joint_pos(
            &buffer, message.axis, message.position, message.velocity);

    assert_int_equal(data_size, alligned32(sizeof(struct Message_set_abs_pos)));
    assert_int_equal(buffer.length, data_size);
    assert_int_not_equal(buffer.checksum, 0);

    struct Message_set_abs_pos* message_p = (void*)buffer.payload;
    assert_int_equal(message.type, message_p->type);
    assert_int_equal(message.axis, message_p->axis);
    assert_int_equal(message.position, message_p->position);
    assert_int_equal(message.velocity, message_p->velocity);
}

static void test_serialize_joint_io_step(void **state) {
    (void) state; /* unused */

    struct NWBuffer buffer = {
        .payload={0},
        .checksum=0,
        .length=0
    };
    memset(&buffer, 0, sizeof(buffer));

    struct Message_joint_gpio message = {
        .type = MSG_SET_AXIS_IO_STEP,
        .axis = 123,
        .value = 1
    };

    size_t data_size = serialize_joint_io_step(&buffer, message.axis, message.value);

    assert_int_equal(data_size, alligned32(sizeof(struct Message_joint_gpio)));
    assert_int_equal(buffer.length, data_size);
    assert_int_not_equal(buffer.checksum, 0);

    struct Message_joint_gpio* message_p = (void*)buffer.payload;
    assert_int_equal(message.type, message_p->type);
    assert_int_equal(message.axis, message_p->axis);
    assert_int_equal(message.value, message_p->value);
}

static void test_serialize_joint_io_dir(void **state) {
    (void) state; /* unused */

    struct NWBuffer buffer = {
        .payload={0},
        .checksum=0,
        .length=0
    };

    struct Message_joint_gpio message = {
        .type = MSG_SET_AXIS_IO_DIR,
        .axis = 123,
        .value = 1
    };

    size_t data_size = serialize_joint_io_dir(&buffer, message.axis, message.value);

    assert_int_equal(data_size, alligned32(sizeof(struct Message_joint_gpio)));
    assert_int_equal(buffer.length, data_size);
    assert_int_not_equal(buffer.checksum, 0);

    struct Message_joint_gpio* message_p = (void*)buffer.payload;
    assert_int_equal(message.type, message_p->type);
    assert_int_equal(message.axis, message_p->axis);
    assert_int_equal(message.value, message_p->value);
}

static void test_serialize_joint_enable(void **state) {
    (void) state; /* unused */

    struct NWBuffer buffer = {
        .payload={0},
        .checksum=0,
        .length=0
    };

    struct Message_joint_enable message = {
        .type = MSG_SET_AXIS_ENABLED,
        .axis = 123,
        .value = 1
    };


    size_t data_size = serialize_joint_enable(&buffer, message.axis, message.value);

    assert_int_equal(data_size, alligned32(sizeof(struct Message_joint_enable)));
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
    memset(buffer.payload, 0, sizeof(buffer.payload));
    buffer.checksum=0;
    buffer.length=0;

    struct Message_joint_config message = {0};
    message.type = MSG_SET_AXIS_CONFIG;
    message.axis = 123;
    message.enable = 1;
    message.gpio_step = 1;
    message.gpio_dir = 2;
    message.max_velocity = 12.34;
    message.max_accel = 56.78;

    size_t data_size = serialize_joint_config(
            &buffer,
            message.axis,
            message.enable,
            message.gpio_step,
            message.gpio_dir,
            message.max_velocity,
            message.max_accel
            );

    assert_int_equal(data_size, alligned32(sizeof(struct Message_joint_config)));
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
        cmocka_unit_test(test_serialize_joint_io_step),
        cmocka_unit_test(test_serialize_joint_io_dir),
        cmocka_unit_test(test_serialize_joint_enable),
        cmocka_unit_test(test_serialize_joint_config)
    };

    return cmocka_run_group_tests(tests, NULL, NULL);
}

