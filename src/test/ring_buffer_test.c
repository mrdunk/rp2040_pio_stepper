#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <stdint.h>
#include <stdio.h>

#include "cmocka.h"

#include "../rp2040/config.h"
#include "../rp2040/ring_buffer.h"

volatile uint32_t tick = 0;

size_t __wrap_time_us_64() {
    return 1;
}

//uint32_t __wrap_ring_buf_uint_ave(struct Ring_buf_uint_ave* data, const uint32_t new_val) {
//    return 1;
//}

void __wrap_tight_loop_contents() {
}

void __wrap_update_period(uint32_t update_time_us) {
}


static void test_ring_buffer_uint_ave__basic(void **state) {
    (void) state; /* unused */

    struct Ring_buf_uint_ave data = {0};
    uint32_t return_val;

    assert_int_equal(data.count, 0);
    assert_int_equal(data.total, 0);

    return_val = ring_buf_uint_ave(&data, 123);
    assert_int_equal(data.count, 1);
    assert_int_equal(data.total, 123);
    assert_int_equal(return_val, 123);

    return_val = ring_buf_uint_ave(&data, 456);
    assert_int_equal(data.count, 2);
    assert_int_equal(data.total, 123 + 456);
    assert_int_equal(return_val, (123 + 456) / 2);
}

static void test_ring_buffer_uint_ave__wrap_1(void **state) {
    (void) state; /* unused */

    struct Ring_buf_uint_ave data = {0};
    uint32_t return_val;

    assert_int_equal(data.count, 0);
    assert_int_equal(data.total, 0);

    size_t i;
    for(i = 0; i < RING_BUF_AVE_LEN; i++) {
        return_val = ring_buf_uint_ave(&data, 1);
        assert_int_equal(data.count, i + 1);
        assert_int_equal(data.total, i + 1);
        assert_int_equal(return_val, 1);
    }

    i++;
    return_val = ring_buf_uint_ave(&data, 1);
    assert_int_equal(data.count, RING_BUF_AVE_LEN);
    assert_int_equal(data.total, RING_BUF_AVE_LEN);
    assert_int_equal(return_val, 1);

    i++;
    return_val = ring_buf_uint_ave(&data, 1);
    assert_int_equal(data.count, RING_BUF_AVE_LEN);
    assert_int_equal(data.total, RING_BUF_AVE_LEN);
    assert_int_equal(return_val, 1);
}

static void test_ring_buffer_uint_ave__wrap_1000(void **state) {
    (void) state; /* unused */

    uint32_t val = RING_BUF_AVE_LEN;
    struct Ring_buf_uint_ave data = {0};
    uint32_t return_val;

    assert_int_equal(data.count, 0);
    assert_int_equal(data.total, 0);

    size_t i;
    for(i = 0; i < RING_BUF_AVE_LEN; i++) {
        return_val = ring_buf_uint_ave(&data, val);
        assert_int_equal(data.count, i + 1);
        assert_int_equal(data.total, (i + 1) * val);
        assert_int_equal(return_val, val);
    }

    i++;
    return_val = ring_buf_uint_ave(&data, val);
    assert_int_equal(data.count, RING_BUF_AVE_LEN);
    assert_int_equal(data.total, RING_BUF_AVE_LEN * val);
    assert_int_equal(return_val, val);

    i++;
    return_val = ring_buf_uint_ave(&data, val);
    assert_int_equal(data.count, RING_BUF_AVE_LEN);
    assert_int_equal(data.total, RING_BUF_AVE_LEN * val);
    assert_int_equal(return_val, val);
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_ring_buffer_uint_ave__basic),
        cmocka_unit_test(test_ring_buffer_uint_ave__wrap_1),
        cmocka_unit_test(test_ring_buffer_uint_ave__wrap_1000)
    };

    return cmocka_run_group_tests(tests, NULL, NULL);
}

