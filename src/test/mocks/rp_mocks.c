#include <stdint.h>
#include "rp_mocks.h"

void tight_loop_contents() {
}

uint64_t time_us_64() {
    return 0;
}

void gpio_put(uint32_t gpio, int value) {
}

int gpio_get(uint32_t gpio) {
    return 0;
}

void mutex_enter_blocking(mutex_t *mtx) {
}

void mutex_exit(mutex_t *mtx) {
}

void mutex_init(mutex_t *mtx) {
}

void gpio_init (uint8_t gpio) {
}

void gpio_set_dir (uint8_t gpio, uint8_t out) {
}

