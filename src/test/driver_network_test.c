#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <stdint.h>
#include <cmocka.h>

#include "../driver/rp2040_network.c"
#include "mocks/driver_mocks.h"
#include "../shared/messages.h"



static void test_serialize_data__one_message(void **state) {
    (void) state; /* unused */

    char buffer[BUFSIZE];
    memset(buffer, '\0', BUFSIZE);
    void* buffer_iterator = &buffer[0];
    union MessageAny message;
    size_t buffer_space_init = BUFSIZE;
    size_t buffer_space = buffer_space_init;
    size_t buffer_size = 0;

    message.timing.type = MSG_TIMING;
    message.timing.update_id = 1234;
    message.timing.time = 4567;
    buffer_size += serialize_data(&message, &buffer_iterator, &buffer_space);
    
    assert_int_equal(buffer_size, sizeof(struct Message_timing));
    assert_int_equal(buffer_space, buffer_space_init - sizeof(struct Message_timing));
    assert_memory_equal(buffer, &message, sizeof(struct Message_timing));
    assert_ptr_equal(buffer + sizeof(struct Message_timing), buffer_iterator);
}

static void test_serialize_data__multi_message(void **state) {
    (void) state; /* unused */

    char buffer[BUFSIZE];
    memset(buffer, '\0', BUFSIZE);
    void* buffer_iterator = &buffer[0];
    union MessageAny message;
    size_t buffer_space_init = BUFSIZE;
    size_t buffer_space = buffer_space_init;
    size_t buffer_size = 0;
    size_t total_mess_size = 0;

    
    message.timing.type = MSG_TIMING;
    message.timing.update_id = 1234;
    message.timing.time = 4567;
    buffer_size += serialize_data(&message, &buffer_iterator, &buffer_space);

    assert_memory_equal(buffer + total_mess_size, &message, sizeof(struct Message_timing));
    
    total_mess_size += sizeof(struct Message_timing);
    
    assert_int_equal(buffer_size, total_mess_size);
    assert_int_equal(buffer_space, buffer_space_init - total_mess_size);
    assert_ptr_equal(buffer + total_mess_size, buffer_iterator);

    
    message.set_rel_pos.type = MSG_SET_AXIS_REL_POS;
    message.set_rel_pos.axis = 4;
    message.set_rel_pos.value = 12345678;
    buffer_size += serialize_data(&message, &buffer_iterator, &buffer_space);

    assert_memory_equal(buffer + total_mess_size, &message, sizeof(struct Message_set_rel_pos));

    total_mess_size += sizeof(struct Message_set_rel_pos);

    assert_int_equal(buffer_size, total_mess_size);
    assert_int_equal(buffer_space, buffer_space_init - total_mess_size);
    assert_ptr_equal(buffer + total_mess_size, buffer_iterator);

    
    message.joint_enable.type = MSG_SET_AXIS_ENABLED;
    message.joint_enable.axis = 3;
    message.joint_enable.value = 0;
    buffer_size += serialize_data(&message, &buffer_iterator, &buffer_space);
    
    assert_memory_equal(buffer + total_mess_size, &message, sizeof(struct Message_joint_enable));

    total_mess_size += sizeof(struct Message_joint_enable);

    assert_int_equal(buffer_size, total_mess_size);
    assert_int_equal(buffer_space, buffer_space_init - total_mess_size);
    assert_ptr_equal(buffer + total_mess_size, buffer_iterator);
}

static void test_serialize_data__overflow(void **state) {
    (void) state; /* unused */

    char buffer[BUFSIZE];
    void* buffer_iterator = &buffer[0];
    size_t buffer_space = 2 * sizeof(struct Message_timing);
    size_t buffer_size = 0;

    union MessageAny message;
    message.timing.type = MSG_TIMING;

    buffer_size += serialize_data(&message, &buffer_iterator, &buffer_space);

    assert_int_equal(buffer_size, sizeof(struct Message_timing));

    buffer_size += serialize_data(&message, &buffer_iterator, &buffer_space);

    assert_int_equal(buffer_size, 2 * sizeof(struct Message_timing));

    // Out of space. buffer_size does not increase.
    buffer_size += serialize_data(&message, &buffer_iterator, &buffer_space);

    assert_int_equal(buffer_size, 2 * sizeof(struct Message_timing));
}

static void test_serialize_data__overflow_off_by_1(void **state) {
    (void) state; /* unused */

    char buffer[BUFSIZE];
    void* buffer_iterator = &buffer[0];
    size_t buffer_space = 3 * sizeof(struct Message_timing) - 1;
    size_t buffer_size = 0;

    union MessageAny message;
    message.timing.type = MSG_TIMING;

    buffer_size += serialize_data(&message, &buffer_iterator, &buffer_space);

    assert_int_equal(buffer_size, sizeof(struct Message_timing));

    buffer_size += serialize_data(&message, &buffer_iterator, &buffer_space);

    assert_int_equal(buffer_size, 2 * sizeof(struct Message_timing));

    // Out of space. buffer_size does not increase.
    buffer_size += serialize_data(&message, &buffer_iterator, &buffer_space);

    assert_int_equal(buffer_size, 2 * sizeof(struct Message_timing));
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_serialize_data__one_message),
        cmocka_unit_test(test_serialize_data__multi_message),
        cmocka_unit_test(test_serialize_data__overflow),
        cmocka_unit_test(test_serialize_data__overflow_off_by_1)
    };

    return cmocka_run_group_tests(tests, NULL, NULL);
}

