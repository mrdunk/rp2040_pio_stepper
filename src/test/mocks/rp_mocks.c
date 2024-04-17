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

void gpio_set_function (size_t gpio, enum gpio_function fn) {
}

void mutex_enter_blocking(mutex_t *mtx) {
}

void mutex_exit(mutex_t *mtx) {
}

void mutex_init(mutex_t *mtx) {
}

void gpio_pull_up(uint8_t gpio) {
}

void gpio_init (uint8_t gpio) {
}

void gpio_set_dir (uint8_t gpio, uint8_t out) {
}

void uart_tx_wait_blocking (size_t *uart) {
}

void uart_write_blocking (size_t *uart, const uint8_t *src, size_t len) {
}

size_t uart_init (size_t *uart, size_t baudrate) {
    return 0;
}

void uart_deinit (size_t *uart) {
}

uint8_t uart_is_readable (size_t *uart) {
    return 0;
}

char uart_getc (size_t *uart) {
    return '\0';
}

struct uart_hw_t *uart_get_hw(size_t *uart) {
    return NULL;
}

