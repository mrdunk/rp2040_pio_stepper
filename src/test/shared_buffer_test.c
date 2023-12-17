#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <stdint.h>
#include <cmocka.h>

#include <stdio.h>

#include "buffer.h"

uint16_t __wrap_checksum(uint16_t checksum, void* new_data, uint16_t new_data_len) {
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

    return_val = packNWBuff(&buffer, (void*)&test_message, sizeof(test_message));

    assert_int_equal(return_val, sizeof(test_message));
    assert_int_equal(buffer.length, sizeof(test_message));
    assert_int_equal(buffer.checksum, 1234);
    assert_memory_equal(buffer.payload, &test_message, sizeof(test_message));
    assert_int_equal(*(buffer.payload + sizeof(test_message)), 0);
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
    return_val = packNWBuff(&buffer, (void*)&test_message, sizeof(test_message));
    assert_int_equal(return_val, sizeof(test_message));

    will_return(__wrap_checksum, 1234);
    return_val = packNWBuff(&buffer, (void*)&test_message, sizeof(test_message));
    assert_int_equal(return_val, sizeof(test_message));
    
    will_return(__wrap_checksum, 5678);
    return_val = packNWBuff(&buffer, (void*)&test_message, sizeof(test_message));
    assert_int_equal(return_val, sizeof(test_message));

    assert_int_equal(buffer.length, 3 * sizeof(test_message));
    assert_int_equal(buffer.checksum, 5678);
    assert_memory_equal(buffer.payload, &test_message, sizeof(test_message));
    assert_memory_equal(
            (buffer.payload + sizeof(test_message)),
            &test_message,
            sizeof(test_message));
    assert_memory_equal(
            (buffer.payload + sizeof(test_message) * 2),
            &test_message,
            sizeof(test_message));
    assert_int_equal(*(buffer.payload + sizeof(test_message) * 3), 0);
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
        uint8_t a[NW_BUF_LEN - sizeof(test_message_small)];
    } test_message_big;

    uint16_t return_val;

    will_return(__wrap_checksum, 1234);
    return_val = packNWBuff(&buffer, (void*)&test_message_big, sizeof(test_message_big));
    assert_int_equal(return_val, sizeof(test_message_big));

    will_return(__wrap_checksum, 1234);
    return_val = packNWBuff(&buffer, (void*)&test_message_small, sizeof(test_message_small));
    assert_int_equal(return_val, sizeof(test_message_small));

    // This one will fail to populate as the buffer is full.
    return_val = packNWBuff(&buffer, (void*)&test_message_small, sizeof(test_message_small));
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

    memcpy(&(buffer.payload), &test_message_sent, sizeof(test_message_sent));

    uint16_t return_val = unPackNWBuff(
            &buffer, 0, &test_message_received, sizeof(test_message_received));

    assert_int_equal(return_val, 1);
    assert_memory_equal(&test_message_sent, &test_message_received, sizeof(test_message_sent));
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
        .length = sizeof(struct TestMessage) * 3,
        .checksum = 42,
        .payload = {0}
    };

    memcpy(
            &(buffer.payload),
            &test_message_sent_1,
            sizeof(struct TestMessage)
          );

    memcpy(
            ((uint8_t*)&(buffer.payload) + sizeof(struct TestMessage)),
            &test_message_sent_2,
            sizeof(struct TestMessage)
          );
    
    memcpy(
            ((uint8_t*)&(buffer.payload) + sizeof(struct TestMessage) * 2),
            &test_message_sent_3,
            sizeof(struct TestMessage)
          );
    

    uint16_t return_val;

    // Unpack first message.
    return_val = unPackNWBuff(
            &buffer,
            0,
            &test_message_received,
            sizeof(test_message_received));

    assert_int_equal(return_val, 1);
    assert_memory_equal(&test_message_sent_1, &test_message_received, sizeof(struct TestMessage));

    
    // Unpack 2nd message.
    return_val = unPackNWBuff(
            &buffer,
            sizeof(struct TestMessage),
            &test_message_received,
            sizeof(test_message_received));

    assert_int_equal(return_val, 1);
    assert_memory_equal(&test_message_sent_2, &test_message_received, sizeof(struct TestMessage));


    // Unpack 3rd message.
    return_val = unPackNWBuff(
            &buffer,
            sizeof(struct TestMessage) * 2,
            &test_message_received,
            sizeof(test_message_received));

    assert_int_equal(return_val, 1);
    assert_memory_equal(&test_message_sent_3, &test_message_received, sizeof(struct TestMessage));
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
        .length = sizeof(struct TestMessage) + 123,
        .checksum = 42,
        .payload = {0}
    };

    size_t start_offset = buffer.length - sizeof(struct TestMessage);

    memcpy(
            (uint8_t*)&(buffer.payload) + start_offset,
            &test_message_sent_1,
            sizeof(struct TestMessage)
          );

    uint16_t return_val;


    // Unpack first message.
    return_val = unPackNWBuff(
            &buffer,
            start_offset,
            &test_message_received,
            sizeof(test_message_received));

    assert_int_equal(return_val, 1);
    assert_memory_equal(&test_message_sent_1, &test_message_received, sizeof(struct TestMessage));

    
    // Request 2nd message but it exceeds the reported data length.
    return_val = unPackNWBuff(
            &buffer,
            start_offset + sizeof(struct TestMessage),
            &test_message_received,
            sizeof(test_message_received));

    assert_int_equal(return_val, 0);
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
        .length = sizeof(struct TestMessage) + 123,
        .checksum = 42,
        .payload = {0}
    };

    size_t start_offset = buffer.length - sizeof(struct TestMessage);


    // Unpack message fails as it finishes beyond buffer.length.
    uint16_t return_val = unPackNWBuff(
            &buffer,
            start_offset + 1,
            &test_message_received,
            sizeof(test_message_received));

    assert_int_equal(return_val, 0);
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
    uint16_t return_val = unPackNWBuff(
            &buffer,
            NW_BUF_LEN,
            &test_message_received,
            sizeof(test_message_received));

    assert_int_equal(return_val, 0);
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_NW_BUF_LEN_is_even),
        cmocka_unit_test(test_pack_one),
        cmocka_unit_test(test_pack_multiple),
        cmocka_unit_test(test_pack_overflow),
        cmocka_unit_test(test_unpack_one),
        cmocka_unit_test(test_unpack_multi),
        cmocka_unit_test(test_unpack_invalid_length),
        cmocka_unit_test(test_unpack_invalid_length_off_by_one),
        cmocka_unit_test(test_unpack_overflow)
    };

    return cmocka_run_group_tests(tests, NULL, NULL);
}


