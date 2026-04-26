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

bool add_repeating_timer_us(int32_t delay_us,
                             repeating_timer_callback_t callback,
                             void *user_data,
                             repeating_timer_t *out) {
    return true;
}

bool cancel_repeating_timer(repeating_timer_t *timer) {
    return true;
}

void multicore_launch_core1(void(*entry)(void)) {
    (void)entry;
}

alarm_id_t add_alarm_at(absolute_time_t time, alarm_callback_t callback,
                         void *user_data, bool fire_if_past) {
    (void) time; (void) callback; (void) user_data; (void) fire_if_past;
    return 0;
}

bool cancel_alarm(alarm_id_t alarm_id) {
    (void) alarm_id;
    return true;
}

