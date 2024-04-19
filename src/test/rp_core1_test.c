#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <stdint.h>
#include <cmocka.h>

#include "../rp2040/core1.h"
#include "../rp2040/pio.h"


extern volatile struct ConfigGlobal config;

static void test_start(void **state) {
    (void) state; /* unused */

    assert_int_equal(0, 0);
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_start)
    };

    return cmocka_run_group_tests(tests, NULL, NULL);
}

