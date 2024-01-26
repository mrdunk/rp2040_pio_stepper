#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <stdint.h>
#include <cmocka.h>

#include <stdio.h>

#include "buffer.h"
#include "messages.h"


uint8_t mock_checksum = 0;
uint16_t __real_checksum(uint16_t val_in, void* new_data, uint16_t new_data_len);

uint16_t __wrap_checksum(uint16_t val_in, void* new_data, uint16_t new_data_len) {
    if(mock_checksum) {
        return __real_checksum(val_in, new_data, new_data_len);
    }
    return mock_type(int);
}


/* NW_BUF_LEN must be even number as NWBuffer.length is a uint16_t which is 2 bytes.*/
static void test_NW_BUF_LEN_is_even(void **state) {
    assert_int_equal(NW_BUF_LEN % 2, 0);
}

static void test_pack_one(void **state) {
    (void) state; /* unused */

    struct NWBuffer buffer = {
        .length = 0,
        .checksum = 0,
        .payload = {0}
    };

    struct TestMessage {
        uint8_t a;
        uint16_t b;
        int16_t c;
        double e;
    } test_message = {
        .a = 123,
        .b = 4567,
        .c = -8901,
        .e = 12.345
    };

    uint16_t return_val;

    will_return(__wrap_checksum, 1234);

    return_val = pack_nw_buff(&buffer, (void*)&test_message, sizeof(test_message));

    assert_int_equal(return_val, alligned32(sizeof(test_message)));
    assert_int_equal(buffer.length, alligned32(sizeof(test_message)));
    assert_int_equal(buffer.checksum, 1234);
    assert_memory_equal(buffer.payload, &test_message, alligned32(sizeof(test_message)));
    assert_int_equal(*(buffer.payload + alligned32(sizeof(test_message))), 0);
}

static void test_pack_one_in_dirty_buffer(void **state) {
    (void) state; /* unused */

    struct TestMessage {
        uint8_t a;
        uint16_t b;
        int16_t c;
        double e;
    } test_message = {
        .a = 123,
        .b = 4567,
        .c = -8901,
        .e = 12.345
    };

    struct NWBuffer buffer = {
        .length = 0,
        .checksum = 0,
        .payload = {0}
    };
    // Dirty the buffer before storing.
    for(uint16_t i = 0; i < sizeof(struct TestMessage) * 2; i++) {
        buffer.payload[1] = i;
    }

    uint16_t return_val;

    will_return(__wrap_checksum, 1234);

    return_val = pack_nw_buff(&buffer, (void*)&test_message, sizeof(test_message));

    assert_int_equal(return_val, alligned32(sizeof(test_message)));
    assert_int_equal(buffer.length, alligned32(sizeof(test_message)));
    assert_int_equal(buffer.checksum, 1234);
    assert_memory_equal(buffer.payload, &test_message, alligned32(sizeof(test_message)));
    assert_int_equal(*(buffer.payload + alligned32(sizeof(test_message))), 0);
}

static void test_pack_multiple(void **state) {
    (void) state; /* unused */

    struct NWBuffer buffer = {
        .length = 0,
        .checksum = 0,
        .payload = {0}
    };

    struct TestMessage {
        uint8_t a;
        uint16_t b;
        int16_t c;
        double e;
    } test_message = {
        .a = 123,
        .b = 4567,
        .c = -8901,
        .e = 12.345
    };

    uint16_t return_val;

    will_return(__wrap_checksum, 1234);
    return_val = pack_nw_buff(&buffer, (void*)&test_message, sizeof(test_message));
    assert_int_equal(return_val, alligned32(sizeof(test_message)));

    will_return(__wrap_checksum, 1234);
    return_val = pack_nw_buff(&buffer, (void*)&test_message, sizeof(test_message));
    assert_int_equal(return_val, alligned32(sizeof(test_message)));
    
    will_return(__wrap_checksum, 5678);
    return_val = pack_nw_buff(&buffer, (void*)&test_message, sizeof(test_message));
    assert_int_equal(return_val, alligned32(sizeof(test_message)));

    assert_int_equal(buffer.length, 3 * alligned32(sizeof(test_message)));
    assert_int_equal(buffer.checksum, 5678);
    assert_memory_equal(buffer.payload, &test_message, alligned32(sizeof(test_message)));
    assert_memory_equal(
            (buffer.payload + alligned32(sizeof(test_message))),
            &test_message,
            alligned32(sizeof(test_message)));
    assert_memory_equal(
            (buffer.payload + alligned32(sizeof(test_message)) * 2),
            &test_message,
            alligned32(sizeof(test_message)));
    assert_int_equal(*(buffer.payload + alligned32(sizeof(test_message)) * 3), 0);
}

static void test_pack_overflow(void **state) {
    (void) state; /* unused */

    struct NWBuffer buffer = {
        .length = 0,
        .checksum = 0,
        .payload = {0}
    };

    struct TestMessageSmall {
        uint8_t a;
    } test_message_small;

    struct TestMessageBig {
        uint8_t a[NW_BUF_LEN - alligned32(sizeof(test_message_small))];
    } test_message_big;

    uint16_t return_val;

    will_return(__wrap_checksum, 1234);
    return_val = pack_nw_buff(&buffer, (void*)&test_message_big, sizeof(test_message_big));
    assert_int_equal(return_val, alligned32(sizeof(test_message_big)));

    will_return(__wrap_checksum, 1234);
    return_val = pack_nw_buff(&buffer, (void*)&test_message_small, sizeof(test_message_small));
    assert_int_equal(return_val, alligned32(sizeof(test_message_small)));

    // This one will fail to populate as the buffer is full.
    return_val = pack_nw_buff(&buffer, (void*)&test_message_small, sizeof(test_message_small));
    assert_int_equal(return_val, 0);
}

static void test_unpack_one(void **state) {
    (void) state; /* unused */

    struct TestMessage {
        uint8_t a;
        uint16_t b;
        int16_t c;
        double e;
    };

    struct TestMessage test_message_sent = {
        .a = 123,
        .b = 4567,
        .c = -8901,
        .e = 12.345
    };

    struct TestMessage test_message_received;

    struct NWBuffer buffer = {
        .length = sizeof(test_message_sent),
        .checksum = 42,
        .payload = {0}
    };

    size_t accumilator = 0;

    memcpy(&(buffer.payload), &test_message_sent, sizeof(test_message_sent));

    void* data_p = unpack_nw_buff(
            &buffer, 0, &accumilator, &test_message_received, sizeof(test_message_received));

    assert_int_not_equal(data_p, NULL);
    assert_memory_equal(&test_message_sent, &test_message_received, sizeof(test_message_sent));
    assert_memory_equal(data_p, &test_message_received, sizeof(test_message_sent));
    assert_int_equal(accumilator, alligned32(sizeof(test_message_sent)));
}

/* Populating the end of the payload value is optional. */
static void test_unpack_null_accumilator(void **state) {
    (void) state; /* unused */

    struct TestMessage {
        uint8_t a;
        uint16_t b;
        int16_t c;
        double e;
    };

    struct TestMessage test_message_sent = {
        .a = 123,
        .b = 4567,
        .c = -8901,
        .e = 12.345
    };

    struct TestMessage test_message_received;

    struct NWBuffer buffer = {
        .length = alligned32(sizeof(test_message_sent)),
        .checksum = 42,
        .payload = {0}
    };

    memcpy(&(buffer.payload), &test_message_sent, sizeof(test_message_sent));

    void* data_p = unpack_nw_buff(
            &buffer, 0, NULL, &test_message_received, sizeof(test_message_received));

    assert_int_not_equal(data_p, NULL);
    assert_memory_equal(&test_message_sent, &test_message_received, sizeof(test_message_sent));
    assert_memory_equal(data_p, &test_message_received, sizeof(test_message_sent));
}

static void test_unpack_multi(void **state) {
    (void) state; /* unused */

    struct TestMessage {
        uint8_t a;
        uint16_t b;
        int16_t c;
        double e;
    };

    struct TestMessage test_message_sent_1 = {
        .a = 123,
        .b = 4567,
        .c = -8901,
        .e = 12.345
    };

    struct TestMessage test_message_sent_2 = {
        .a = 234,
        .b = 5678,
        .c = -9012,
        .e = 23.456
    };

    struct TestMessage test_message_sent_3 = {
        .a = 34,
        .b = 6789,
        .c = -1234,
        .e = 34.567
    };

    struct TestMessage test_message_received;

    struct NWBuffer buffer = {
        .length = alligned32(sizeof(struct TestMessage)) * 3,
        .checksum = 42,
        .payload = {0}
    };

    memcpy(
            &(buffer.payload),
            &test_message_sent_1,
            sizeof(struct TestMessage)
          );

    memcpy(
            ((uint8_t*)&(buffer.payload) + alligned32(sizeof(struct TestMessage))),
            &test_message_sent_2,
            sizeof(struct TestMessage)
          );
    
    memcpy(
            ((uint8_t*)&(buffer.payload) + alligned32(sizeof(struct TestMessage)) * 2),
            &test_message_sent_3,
            sizeof(struct TestMessage)
          );
    

    size_t accumilator = 0;
    void* data_p;

    // Unpack first message.
    data_p = unpack_nw_buff(
            &buffer,
            0,
            &accumilator,
            &test_message_received,
            sizeof(test_message_received));

    assert_int_not_equal(data_p, NULL);
    assert_memory_equal(&test_message_sent_1, &test_message_received, sizeof(struct TestMessage));
    assert_memory_equal(&test_message_sent_1, data_p, sizeof(struct TestMessage));
    assert_int_equal(accumilator, alligned32(sizeof(struct TestMessage)));

    
    // Unpack 2nd message.
    data_p = unpack_nw_buff(
            &buffer,
            alligned32(sizeof(struct TestMessage)),
            &accumilator,
            &test_message_received,
            sizeof(test_message_received));

    assert_int_not_equal(data_p, NULL);
    assert_memory_equal(&test_message_sent_2, &test_message_received, sizeof(struct TestMessage));
    assert_memory_equal(&test_message_sent_2, data_p, sizeof(struct TestMessage));
    assert_int_equal(accumilator, alligned32(sizeof(struct TestMessage)) * 2);


    // Unpack 3rd message.
    data_p = unpack_nw_buff(
            &buffer,
            alligned32(sizeof(struct TestMessage)) * 2,
            &accumilator,
            &test_message_received,
            sizeof(test_message_received));

    assert_int_not_equal(data_p, NULL);
    assert_memory_equal(&test_message_sent_3, &test_message_received, sizeof(struct TestMessage));
    assert_memory_equal(&test_message_sent_3, data_p, sizeof(struct TestMessage));
    assert_int_equal(accumilator, alligned32(sizeof(struct TestMessage)) * 3);
}

/* The same variable can be used to pass the input offset and receive the end of payload value. */
static void test_unpack_multi_accumilating(void **state) {
    (void) state; /* unused */

    struct TestMessage {
        uint8_t a;
        uint16_t b;
        int16_t c;
        double e;
    };

    struct TestMessage test_message_sent_1 = {
        .a = 123,
        .b = 4567,
        .c = -8901,
        .e = 12.345
    };

    struct TestMessage test_message_sent_2 = {
        .a = 234,
        .b = 5678,
        .c = -9012,
        .e = 23.456
    };

    struct TestMessage test_message_sent_3 = {
        .a = 34,
        .b = 6789,
        .c = -1234,
        .e = 34.567
    };

    struct TestMessage test_message_received;

    struct NWBuffer buffer = {
        .length = alligned32(sizeof(struct TestMessage)) * 3,
        .checksum = 42,
        .payload = {0}
    };

    memcpy(
            &(buffer.payload),
            &test_message_sent_1,
            sizeof(struct TestMessage)
          );

    memcpy(
            ((uint8_t*)&(buffer.payload) + alligned32(sizeof(struct TestMessage))),
            &test_message_sent_2,
            sizeof(struct TestMessage)
          );
    
    memcpy(
            ((uint8_t*)&(buffer.payload) + alligned32(sizeof(struct TestMessage) * 2)),
            &test_message_sent_3,
            sizeof(struct TestMessage)
          );
    

    size_t accumilator = 0;
    void* data_p;

    // Unpack first message.
    data_p = unpack_nw_buff(
            &buffer,
            accumilator,
            &accumilator,
            &test_message_received,
            sizeof(test_message_received));

    assert_int_not_equal(data_p, NULL);
    assert_memory_equal(&test_message_sent_1, &test_message_received, sizeof(struct TestMessage));
    assert_memory_equal(&test_message_sent_1, data_p, sizeof(struct TestMessage));
    assert_int_equal(accumilator, alligned32(sizeof(struct TestMessage)));

    
    // Unpack 2nd message.
    data_p = unpack_nw_buff(
            &buffer,
            accumilator,
            &accumilator,
            &test_message_received,
            sizeof(test_message_received));

    assert_int_not_equal(data_p, NULL);
    assert_memory_equal(&test_message_sent_2, &test_message_received, sizeof(struct TestMessage));
    assert_memory_equal(&test_message_sent_2, data_p, sizeof(struct TestMessage));
    assert_int_equal(accumilator, alligned32(sizeof(struct TestMessage)) * 2);


    // Unpack 3rd message.
    data_p = unpack_nw_buff(
            &buffer,
            accumilator,
            &accumilator,
            &test_message_received,
            sizeof(test_message_received));

    assert_int_not_equal(data_p, NULL);
    assert_memory_equal(&test_message_sent_3, &test_message_received, sizeof(struct TestMessage));
    assert_memory_equal(&test_message_sent_3, data_p, sizeof(struct TestMessage));
    assert_int_equal(accumilator, alligned32(sizeof(struct TestMessage)) * 3);
}

/* Try to unpack a message beyond the specified data length. */
static void test_unpack_invalid_length(void **state) {
    (void) state; /* unused */

    struct TestMessage {
        uint8_t a;
        uint16_t b;
        int16_t c;
        double e;
    };

    struct TestMessage test_message_sent_1 = {
        .a = 123,
        .b = 4567,
        .c = -8901,
        .e = 12.345
    };

    struct TestMessage test_message_received;

    struct NWBuffer buffer = {
        .length = alligned32(sizeof(struct TestMessage)) + 123,
        .checksum = 42,
        .payload = {0}
    };

    size_t start_offset = buffer.length - alligned32(sizeof(struct TestMessage));

    memcpy(
            (uint8_t*)&(buffer.payload) + start_offset,
            &test_message_sent_1,
            sizeof(struct TestMessage)
          );

    void* data_p;


    // Unpack first message.
    data_p = unpack_nw_buff(
            &buffer,
            start_offset,
            NULL,
            &test_message_received,
            sizeof(test_message_received));

    assert_int_not_equal(data_p, NULL);
    assert_memory_equal(&test_message_sent_1, &test_message_received, sizeof(struct TestMessage));
    assert_memory_equal(&test_message_sent_1, data_p, sizeof(struct TestMessage));

    
    // Request 2nd message but it exceeds the reported data length.
    data_p = unpack_nw_buff(
            &buffer,
            start_offset + alligned32(sizeof(struct TestMessage)),
            NULL,
            &test_message_received,
            sizeof(test_message_received));

    assert_int_equal(data_p, NULL);
}

/* Try to unpack a message beyond the specified data length. */
static void test_unpack_invalid_length_off_by_one(void **state) {
    (void) state; /* unused */

    struct TestMessage {
        uint8_t a;
        uint16_t b;
        int16_t c;
        double e;
    };

    struct TestMessage test_message_received;

    struct NWBuffer buffer = {
        .length = alligned32(sizeof(struct TestMessage)) + 123,
        .checksum = 42,
        .payload = {0}
    };

    size_t start_offset = buffer.length - alligned32(sizeof(struct TestMessage));


    // Unpack message fails as it finishes beyond buffer.length.
    void* data_p = unpack_nw_buff(
            &buffer,
            start_offset + 1,
            NULL,
            &test_message_received,
            sizeof(test_message_received));

    assert_int_equal(data_p, NULL);
}

/* Try to unpack a message beyond the buffer end. */
static void test_unpack_overflow(void **state) {
    (void) state; /* unused */

    struct TestMessage {
        uint8_t a;
        uint16_t b;
        int16_t c;
        double e;
    };

    struct TestMessage test_message_received;

    struct NWBuffer buffer = {
        .length = 123,
        .checksum = 42,
        .payload = {0}
    };


    // Unpack message from beyond end of buffer.
    void* data_p = unpack_nw_buff(
            &buffer,
            NW_BUF_LEN,
            NULL,
            &test_message_received,
            sizeof(test_message_received));

    assert_int_equal(data_p, NULL);
}

/* Use both wrap and unrap methods. Do not mock the checksum. */
static void test_end_to_end(void **state) {
    (void) state; /* unused */

    struct TestMessage {
        uint8_t a;
        uint16_t b;
        int16_t c;
        double e;
    };
    
    struct TestMessage test_message_send;
    memset(&test_message_send, 0, alligned32(sizeof(test_message_send)));
    test_message_send.a = 123;
    test_message_send.b = 4567;
    test_message_send.c = -8901;
    test_message_send.e = 12.345;

    struct TestMessage test_message_receive;

    struct NWBuffer buffer = {
        .length = 0,
        .checksum = 0,
        .payload = {0}
    };
    // Dirty the buffer before storing.
    for(uint16_t i = 0; i < sizeof(struct TestMessage) * 2; i++) {
        buffer.payload[i] = i;
    }

    mock_checksum = 1;
    uint16_t pack_val = pack_nw_buff(&buffer, (void*)&test_message_send, sizeof(struct TestMessage));
    assert_int_equal(pack_val, sizeof(struct TestMessage));

    void* data_p = unpack_nw_buff(
            &buffer,
            0,
            NULL,
            &test_message_receive,
            sizeof(test_message_receive));
    assert_int_not_equal(data_p, NULL);

    assert_memory_equal(&test_message_send, &test_message_receive, sizeof(struct TestMessage));
    assert_memory_equal(&test_message_send, data_p, sizeof(struct TestMessage));

    assert_int_equal(buffer.checksum, checksum(0, 0, alligned32(sizeof(test_message_send)), buffer.payload));
    mock_checksum = 0;
}

static void test_end_to_end_multi(void **state) {
    (void) state; /* unused */

    mock_checksum = 1;

    struct Message_timing m0 = {
      .type = MSG_TIMING,
      .update_id = 1234,
      .time = 5678
    };

    struct Message_gpio_config m1 = {
      .type = MSG_SET_GPIO_CONFIG,
      .gpio_type = GPIO_TYPE_NATIVE_IN,
      .gpio_count = 0,
      .index = 6,
      .address = 0
    };

    struct Message_set_abs_pos m2 = {
      .type = MSG_SET_AXIS_ABS_POS,
      .axis = 0,
      .position = 12.345,
      .velocity = 678.901
    };

    struct NWBuffer buffer = {
        .length = 0,
        .checksum = 0,
        .payload = {0}
    };
    // Dirty the buffer before storing.
    for(size_t i = 0; i < sizeof(buffer.payload); i++) {
        buffer.payload[i] = i;
    }

    struct Message_timing* m0_p;
    struct Message_gpio_config* m1_p;
    struct Message_set_abs_pos* m2_p;

    
    // Pack some things.
    size_t expected_size = 0;
    size_t buffered_size = 0;
    size_t total_size = 0;

    // Pack first thing, ( Message_timing)
    buffered_size = pack_nw_buff(&buffer, &m0, sizeof(m0));
    expected_size = alligned32(sizeof(m0));
    total_size += expected_size;
    assert_int_equal(buffered_size, expected_size);
    assert_int_equal(total_size, buffer.length);
    // Check buffer has correct data.
    m0_p = (void*)buffer.payload;
    assert_int_equal(m0_p->type, m0.type);
    assert_int_equal(m0_p->update_id, m0.update_id);
    assert_int_equal(m0_p->time, m0.time);

    // Pack next thing. (Message_gpio_config)
    buffered_size = pack_nw_buff(&buffer, &m1, sizeof(m1));
    expected_size = alligned32(sizeof(m1));
    total_size += expected_size;
    assert_int_equal(buffered_size, expected_size);
    assert_int_equal(total_size, buffer.length);
    // Check buffer has correct data.
    m1_p = (void*)buffer.payload + alligned32(sizeof(m0));
    assert_int_equal(m1_p->type, m1.type);
    assert_int_equal(m1_p->gpio_type, m1.gpio_type);
    assert_int_equal(m1_p->gpio_count, m1.gpio_count);
    assert_int_equal(m1_p->index, m1.index);
    assert_int_equal(m1_p->address, m1.address);

    // Pack next thing. (Message_set_abs_pos)
    buffered_size = pack_nw_buff(&buffer, &m2, sizeof(m2));
    expected_size = alligned32(sizeof(m2));
    total_size += expected_size;
    assert_int_equal(buffered_size, expected_size);
    assert_int_equal(total_size, buffer.length);
    // Check buffer has correct data.
    m2_p = (void*)buffer.payload + alligned32(sizeof(m0)) + alligned32(sizeof(m1));
    assert_int_equal(m2_p->type, m2.type);
    assert_int_equal(m2_p->axis, m2.axis);
    assert_double_equal(m2_p->position, m2.position, 0.01);
    assert_double_equal(m2_p->velocity, m2.velocity, 0.01);

    // Pack next thing. (Message_set_abs_pos)
    // Add this one twice.
    buffered_size = pack_nw_buff(&buffer, &m2, sizeof(m2));
    expected_size = alligned32(sizeof(m2));
    total_size += expected_size;
    assert_int_equal(buffered_size, expected_size);
    assert_int_equal(total_size, buffer.length);
    // Check buffer has correct data.
    m2_p = (void*)buffer.payload + alligned32(sizeof(m0)) + alligned32(sizeof(m1)) + alligned32(sizeof(m2));
    assert_int_equal(m2_p->type, m2.type);
    assert_int_equal(m2_p->axis, m2.axis);
    assert_double_equal(m2_p->position, m2.position, 0.01);
    assert_double_equal(m2_p->velocity, m2.velocity, 0.01);

    
    // Checksum on buffer matches.
    assert_int_equal(checkNWBuff(&buffer), 1);


    // Unpack the things.
    size_t offset = 0;
    m0_p = unpack_nw_buff(&buffer, offset, &offset, NULL, sizeof(struct Message_timing));
    assert_int_not_equal(m0_p, NULL);
    // Address must be 32 bit aligned. On ARM all memory reads must e 32 bit aligned.
    assert_int_equal((uintptr_t)m0_p % 4, 0);
    assert_int_equal(m0.type, m0_p->type);
    assert_int_equal(m0.update_id, m0_p->update_id);
    assert_int_equal(m0.time, m0_p->time);
    assert_int_equal(offset, alligned32(sizeof(m0)));

    m1_p = unpack_nw_buff(&buffer, offset, &offset, NULL, sizeof(struct Message_gpio_config));
    assert_int_not_equal(m1_p, NULL);
    // Address must be 32 bit aligned. On ARM all memory reads must e 32 bit aligned.
    assert_int_equal((uintptr_t)m1_p % 4, 0);
    assert_int_equal(m1.type, m1_p->type);
    assert_int_equal(m1.gpio_type, m1_p->gpio_type);
    assert_int_equal(m1.gpio_count, m1_p->gpio_count);
    assert_int_equal(m1.index, m1_p->index);
    assert_int_equal(m1.address, m1_p->address);
    assert_int_equal(offset, alligned32(sizeof(m0)) + alligned32(sizeof(m1)));

    m2_p = unpack_nw_buff(&buffer, offset, &offset, NULL, sizeof(struct Message_set_abs_pos));
    assert_int_not_equal(m2_p, NULL);
    // Address must be 32 bit aligned. On ARM all memory reads must e 32 bit aligned.
    assert_int_equal((uintptr_t)m2_p % 4, 0);
    assert_int_equal(m2.type, m2_p->type);
    assert_int_equal(m2.axis, m2_p->axis);
    assert_double_equal(m2.position, m2_p->position, 0.01);
    assert_double_equal(m2.velocity, m2_p->velocity, 0.01);
    assert_int_equal(offset, alligned32(sizeof(m0)) + alligned32(sizeof(m1)) + alligned32(sizeof(m2)));

    m2_p = unpack_nw_buff(&buffer, offset, &offset, NULL, sizeof(struct Message_set_abs_pos));
    assert_int_not_equal(m2_p, NULL);
    // Address must be 32 bit aligned. On ARM all memory reads must e 32 bit aligned.
    assert_int_equal((uintptr_t)m2_p % 4, 0);
    assert_int_equal(m2.type, m2_p->type);
    assert_int_equal(m2.axis, m2_p->axis);
    assert_double_equal(m2.position, m2_p->position, 0.01);
    assert_double_equal(m2.velocity, m2_p->velocity, 0.01);
    assert_int_equal(offset, alligned32(sizeof(m0)) + alligned32(sizeof(m1)) + alligned32(sizeof(m2)) + alligned32(sizeof(m2)));
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_NW_BUF_LEN_is_even),
        cmocka_unit_test(test_pack_one),
        cmocka_unit_test(test_pack_one_in_dirty_buffer),
        cmocka_unit_test(test_pack_multiple),
        cmocka_unit_test(test_pack_overflow),
        cmocka_unit_test(test_unpack_one),
        cmocka_unit_test(test_unpack_null_accumilator),
        cmocka_unit_test(test_unpack_multi),
        cmocka_unit_test(test_unpack_multi_accumilating),
        cmocka_unit_test(test_unpack_invalid_length),
        cmocka_unit_test(test_unpack_invalid_length_off_by_one),
        cmocka_unit_test(test_unpack_overflow),
        cmocka_unit_test(test_end_to_end),
        cmocka_unit_test(test_end_to_end_multi)
    };

    return cmocka_run_group_tests(tests, NULL, NULL);
}


