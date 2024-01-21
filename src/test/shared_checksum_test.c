#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <stdint.h>
#include <cmocka.h>

#include <stdio.h>

#include "buffer.h"

static void test_simple(void **state) {
    (void) state; /* unused */

    uint16_t data_len = 4;
    uint8_t data[4] = {1, 2, 3, 4};
    uint16_t checksum_in = 0;

    uint16_t return_val_good = checksum(checksum_in, data, data_len);

    uint16_t return_val_truncated = checksum(checksum_in, data, data_len - 1);
    assert_int_not_equal(return_val_good, return_val_truncated);

    data[2] ^= 0x1;
    uint16_t return_val_bad = checksum(checksum_in, data, data_len);
    assert_int_not_equal(return_val_good, return_val_bad);
}

static void test_in_parts(void **state) {
    (void) state; /* unused */

    uint8_t data[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    uint16_t checksum_val = 0;

    checksum_val = checksum(checksum_val, data, 2);
    checksum_val = checksum(checksum_val, data + 2, 2);
    checksum_val = checksum(checksum_val, data + 4, 2);

    uint16_t checksum_total = checksum(0, data, 6);

    assert_int_equal(checksum_val, checksum_total);
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_simple),
        cmocka_unit_test(test_in_parts)
    };

    return cmocka_run_group_tests(tests, NULL, NULL);
}



