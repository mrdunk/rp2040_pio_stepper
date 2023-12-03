#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <stdint.h>
#include <cmocka.h>

#include <stdio.h>

#include "../rp2040/config.h"
#include "../rp2040/ring_buffer.h"
#include "../rp2040/timing.h"

volatile uint32_t tick = 0;

size_t __wrap_time_us_64() {
    return 1;
}

uint32_t __wrap_ring_buf_uint_ave(struct Ring_buf_uint_ave* data, const uint32_t new_val) {
    return 1;
}

void __wrap_tight_loop_contents() {
}

void __wrap_update_period(uint32_t update_time_us) {
}


static void test_TODO__basic(void **state) {
    (void) state; /* unused */

}


int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_TODO__basic)
    };

    return cmocka_run_group_tests(tests, NULL, NULL);
}

