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
    union MessageAny message;

    size_t data_size = serialize_timing(&buffer, &message, 1234, 5678);

    assert_int_equal(data_size, sizeof(struct Message_timing));
    assert_int_equal(buffer.length, data_size);
    assert_int_not_equal(buffer.checksum, 0);
    assert_memory_equal(buffer.payload, &message, sizeof(struct Message_timing));
}

static void test_serialize_jont_pos(void **state) {
    (void) state; /* unused */
    
    struct NWBuffer buffer = {0};
    union MessageAny message;

    size_t data_size = serialize_joint_pos(&buffer, &message, 1234, 56.78);

    assert_int_equal(data_size, sizeof(struct Message_set_abs_pos));
    assert_int_equal(buffer.length, data_size);
    assert_int_not_equal(buffer.checksum, 0);
    assert_memory_equal(buffer.payload, &message, sizeof(struct Message_set_abs_pos));
}

static void test_serialize_joint_velocity(void **state) {
    (void) state; /* unused */

    struct NWBuffer buffer = {0};
    union MessageAny message;

    size_t data_size = serialize_joint_velocity(
            &buffer, &message, 1234, 56.78);

    assert_int_equal(data_size, sizeof(struct Message_set_velocity));
    assert_int_equal(buffer.length, data_size);
    assert_int_not_equal(buffer.checksum, 0);
    assert_memory_equal(buffer.payload, &message, sizeof(struct Message_set_velocity));
}

static void test_serialize_joint_max_velocity(void **state) {
    (void) state; /* unused */

    //char buffer[BUFSIZE];
    //void* buffer_iterator = &buffer[0];
    //size_t buffer_space = BUFSIZE;
    //size_t buffer_size;
    struct NWBuffer buffer = {0};
    union MessageAny message;

    //buffer_size = serialize_joint_max_velocity(
    //        &message, 1234, 56.78, &buffer_iterator, &buffer_space);
    size_t data_size = serialize_joint_max_velocity(
            &buffer, &message, 1234, 56.78);

    assert_int_equal(data_size, sizeof(struct Message_set_max_velocity));
    assert_int_equal(buffer.length, data_size);
    assert_int_not_equal(buffer.checksum, 0);
    assert_memory_equal(buffer.payload, &message, sizeof(struct Message_set_max_velocity));
}

static void test_serialize_joint_max_accel(void **state) {
    (void) state; /* unused */

    //char buffer[BUFSIZE];
    //void* buffer_iterator = &buffer[0];
    //size_t buffer_space = BUFSIZE;
    //size_t buffer_size;
    struct NWBuffer buffer = {0};
    union MessageAny message;

    //buffer_size = serialize_joint_max_accel(
    //        &message, 1234, 56.78, &buffer_iterator, &buffer_space);
    size_t data_size = serialize_joint_max_accel(
            &buffer, &message, 1234, 56.78);

    assert_int_equal(data_size, sizeof(struct Message_set_max_accel));
    assert_int_equal(buffer.length, data_size);
    assert_int_not_equal(buffer.checksum, 0);
    assert_memory_equal(buffer.payload, &message, sizeof(struct Message_set_max_accel));
}

static void test_serialize_joint_io_step(void **state) {
    (void) state; /* unused */

    //char buffer[BUFSIZE];
    //void* buffer_iterator = &buffer[0];
    //size_t buffer_space = BUFSIZE;
    //size_t buffer_size;
    struct NWBuffer buffer = {0};
    union MessageAny message;

    //buffer_size = serialize_joint_io_step(
    //        &message, 1234, 1, &buffer_iterator, &buffer_space);
    size_t data_size = serialize_joint_io_step(
        &buffer, &message, 1234, 56);

    assert_int_equal(data_size, sizeof(struct Message_joint_gpio));
    assert_int_equal(buffer.length, data_size);
    assert_int_not_equal(buffer.checksum, 0);
    assert_memory_equal(buffer.payload, &message, sizeof(struct Message_joint_gpio));
}

static void test_serialize_joint_io_dir(void **state) {
    (void) state; /* unused */

    //char buffer[BUFSIZE];
    //void* buffer_iterator = &buffer[0];
    //size_t buffer_space = BUFSIZE;
    //size_t buffer_size;
    struct NWBuffer buffer = {0};
    union MessageAny message;

    //buffer_size = serialize_joint_io_dir(
    //        &message, 1234, 2, &buffer_iterator, &buffer_space);
    size_t data_size = serialize_joint_io_dir(
        &buffer, &message, 1234, 56);

    assert_int_equal(data_size, sizeof(struct Message_joint_gpio));
    assert_int_equal(buffer.length, data_size);
    assert_int_not_equal(buffer.checksum, 0);
    assert_memory_equal(buffer.payload, &message, sizeof(struct Message_joint_gpio));
}

static void test_serialize_joint_enable(void **state) {
    (void) state; /* unused */

    //char buffer[BUFSIZE];
    //void* buffer_iterator = &buffer[0];
    //size_t buffer_space = BUFSIZE;
    //size_t buffer_size;
    struct NWBuffer buffer = {0};
    union MessageAny message;

    //buffer_size = serialize_joint_enable(
    //        &message, 1234, 0, &buffer_iterator, &buffer_space);
    size_t data_size = serialize_joint_enable(
            &buffer, &message, 1234, 1);

    assert_int_equal(data_size, sizeof(struct Message_joint_enable));
    assert_int_equal(buffer.length, data_size);
    assert_int_not_equal(buffer.checksum, 0);
    assert_memory_equal(buffer.payload, &message, sizeof(struct Message_joint_enable));
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_serialize_timing),
        cmocka_unit_test(test_serialize_jont_pos),
        cmocka_unit_test(test_serialize_joint_velocity),
        cmocka_unit_test(test_serialize_joint_max_velocity),
        cmocka_unit_test(test_serialize_joint_max_accel),
        cmocka_unit_test(test_serialize_joint_io_step),
        cmocka_unit_test(test_serialize_joint_io_dir),
        cmocka_unit_test(test_serialize_joint_enable)
    };

    return cmocka_run_group_tests(tests, NULL, NULL);
}

