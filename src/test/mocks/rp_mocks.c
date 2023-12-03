#include <stdint.h>
#include "rp_mocks.h"

void tight_loop_contents() {
}

uint64_t time_us_64() {
    return 0;
}

void gpio_put(uint32_t gpio, int value) {
}

void mutex_enter_blocking(mutex_t *mtx) {
}

void mutex_exit(mutex_t *mtx) {
}

void mutex_init(mutex_t *mtx) {
}
