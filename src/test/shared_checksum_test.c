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
    assert_int_equal(return_val_good, return_val_bad);
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_simple)
    };

    return cmocka_run_group_tests(tests, NULL, NULL);
}



