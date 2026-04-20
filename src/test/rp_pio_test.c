#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <stdint.h>
#include <string.h>
#include <cmocka.h>

#include "../rp2040/pio.h"
#include "../rp2040/config.h"

extern volatile struct ConfigGlobal config;

/* ── PIO mock state ── */
static size_t   mock_rx_fifo_level  = 0;
static int32_t  mock_rx_values[8]   = {0};
static size_t   mock_rx_index       = 0;
static uint32_t last_pio_put_value  = 0;
static int      mock_tx_fifo_empty  = 0;

size_t __wrap_pio_sm_get_rx_fifo_level(size_t pio, size_t sm) {
    (void)pio; (void)sm;
    return mock_rx_fifo_level;   /* drain_rx_fifo reads level once, loops itself */
}

size_t __wrap_pio_sm_get_blocking(size_t pio, size_t sm) {
    (void)pio; (void)sm;
    if (mock_rx_index < 8) {
        return (size_t)mock_rx_values[mock_rx_index++];
    }
    return 0;
}

void __wrap_pio_sm_put(size_t pio, size_t sm, size_t data) {
    (void)pio; (void)sm;
    last_pio_put_value = (uint32_t)data;
}

int __wrap_pio_sm_is_tx_fifo_empty(size_t pio, size_t sm) {
    (void)pio; (void)sm;
    return mock_tx_fifo_empty;
}

/* ── Setup / Teardown ── */
static int test_setup(void **state) {
    (void)state;
    pio_reset_for_test();
    init_config();
    config.update_time_us = 1000;  /* init_config() does not reset this */
    for (size_t j = 0; j < MAX_JOINT; j++) {
        config.joint[j].updated_from_c0  = 0;
        config.joint[j].updated_from_c1  = 0;
        config.joint[j].overrun_count    = 0;
        config.joint[j].underrun_count   = 0;
        config.joint[j].enabled          = 0;
        config.joint[j].io_pos_step      = 1;  /* valid pin (0-31) */
        config.joint[j].io_pos_dir       = 2;  /* valid pin (0-31) */
        config.joint[j].abs_pos_requested  = 0;
        config.joint[j].abs_pos_achieved   = 0;
        config.joint[j].velocity_requested = 0;
        config.joint[j].max_velocity       = 50.0;
    }
    mock_rx_fifo_level = 0;
    mock_rx_index      = 0;
    last_pio_put_value = 0;
    mock_tx_fifo_empty = 0;
    memset(mock_rx_values, 0, sizeof(mock_rx_values));
    return 0;
}

/* placeholder — tests added in subsequent tasks */
static void test_placeholder(void **state) {
    (void)state;
    assert_true(1);
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test_setup(test_placeholder, test_setup),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
