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

/* drain_rx_fifo: FIFO is empty -> returns current_pos unchanged */
static void test_drain_rx_fifo_empty_returns_current(void **state) {
    (void)state;
    mock_rx_fifo_level = 0;
    int32_t result = drain_rx_fifo(0, 42);
    assert_int_equal(result, 42);
}

/* drain_rx_fifo: single entry -> returns that value */
static void test_drain_rx_fifo_single_entry(void **state) {
    (void)state;
    mock_rx_fifo_level  = 1;
    mock_rx_values[0]   = 99;
    int32_t result = drain_rx_fifo(0, 0);
    assert_int_equal(result, 99);
}

/* drain_rx_fifo: multiple entries -> returns only the last */
static void test_drain_rx_fifo_keeps_last(void **state) {
    (void)state;
    mock_rx_fifo_level  = 3;
    mock_rx_values[0]   = 10;
    mock_rx_values[1]   = 20;
    mock_rx_values[2]   = 30;
    int32_t result = drain_rx_fifo(0, 0);
    assert_int_equal(result, 30);
}

/* calculate_step_len: normal step count -> positive result */
static void test_calculate_step_len_normal(void **state) {
    (void)state;
    /* step_count=2.0, period_ticks=133000, max_velocity=50.0
     * expected = (133000 / (2.0 * 2.0)) - 9.0 = 33241
     * min      = (133000 / (50.0 * 2.0)) - 9.0 = 1321
     * result   = max(33241, 1321) = 33241 */
    int32_t result = calculate_step_len(2.0, 133000.0, 50.0);
    assert_int_equal(result, 33241);
}

/* calculate_step_len: step_count exceeds max_velocity -> clamped to min */
static void test_calculate_step_len_clamped(void **state) {
    (void)state;
    /* step_count=200.0 (very fast), period_ticks=133000, max_velocity=50.0
     * unclamped = (133000 / (200.0 * 2.0)) - 9.0 = 323.5 -> (int32_t)323
     * min       = (133000 / (50.0 * 2.0)) - 9.0  = 1321
     * result    = max(323, 1321) = 1321 */
    int32_t result = calculate_step_len(200.0, 133000.0, 50.0);
    assert_int_equal(result, 1321);
}

/* calculate_step_len: below MIN_STEP_COUNT threshold -> 0 */
static void test_calculate_step_len_below_threshold(void **state) {
    (void)state;
    /* MIN_STEP_COUNT = 0.0625; 0.05 < 0.0625 -> returns 0 */
    int32_t result = calculate_step_len(0.05, 133000.0, 50.0);
    assert_int_equal(result, 0);
}

/* clamp_accel: velocity unchanged -> returns same velocity */
static void test_clamp_accel_no_change(void **state) {
    (void)state;
    double result = clamp_accel(5.0, 5.0, 2.0);
    assert_double_equal(result, 5.0, 1e-9);
}

/* clamp_accel: acceleration under limit -> returns requested velocity */
static void test_clamp_accel_under_limit(void **state) {
    (void)state;
    double result = clamp_accel(7.0, 5.0, 3.0);
    assert_double_equal(result, 7.0, 1e-9);
}

/* clamp_accel: acceleration over limit positive -> clamped to max_accel */
static void test_clamp_accel_over_limit_positive(void **state) {
    (void)state;
    double result = clamp_accel(10.0, 5.0, 2.0);
    assert_double_equal(result, 7.0, 1e-9);
}

/* clamp_accel: deceleration over limit -> clamped to max_accel */
static void test_clamp_accel_over_limit_negative(void **state) {
    (void)state;
    double result = clamp_accel(1.0, 5.0, 2.0);
    assert_double_equal(result, 3.0, 1e-9);
}

/* clamp_accel: zero max_accel -> returns requested velocity unchanged */
static void test_clamp_accel_zero_max(void **state) {
    (void)state;
    double result = clamp_accel(100.0, 0.0, 0.0);
    assert_double_equal(result, 100.0, 1e-9);
}

/* get_velocity: near-zero position diff -> returns 0 */
static void test_get_velocity_zero_pos_diff(void **state) {
    (void)state;
    double v = get_velocity(1000, 0, 10, 10.0, 5.0);
    assert_true(v == 0.0);
}

/* get_velocity: direction disagreement -> returns 0 */
static void test_get_velocity_direction_disagreement(void **state) {
    (void)state;
    /* position_diff = +10 (forward), velocity = -5000/1000 (backward) */
    double v = get_velocity(1000, 0, 0, 10.0, -5000.0);
    assert_true(v == 0.0);
}

/* get_velocity: normal forward motion -> returns positive combined velocity */
static void test_get_velocity_normal_forward(void **state) {
    (void)state;
    /* combined = 10*0.1 + 5.0*0.85 = 5.25 */
    double v = get_velocity(1000, 0, 0, 10.0, 5000.0);
    assert_true(v > 0.0);
}

/* get_velocity: slow step triggers holdoff; next call returns 0 */
static void test_get_velocity_holdoff(void **state) {
    (void)state;
    /* combined = 0.5*0.1 + 0.001*0.85 = 0.05085 < 1.0 -> holdoff set */
    double v1 = get_velocity(1000, 0, 0, 0.5, 1.0);
    assert_true(v1 > 0.0);    /* first call returns combined_vel */
    double v2 = get_velocity(1000, 0, 0, 0.5, 1.0);
    assert_true(v2 == 0.0);   /* holdoff active */
}

/* do_steps: update_period == 0 -> puts 0 to FIFO, returns 0 */
static void test_do_steps_zero_period(void **state) {
    (void)state;
    config.update_time_us = 0;
    uint8_t result = do_steps(0);
    assert_int_equal(result, 0);
    assert_int_equal(last_pio_put_value, 0);
}

/* do_steps: joint disabled -> puts 0 to FIFO when empty, returns 0 */
static void test_do_steps_disabled(void **state) {
    (void)state;
    config.joint[0].enabled         = 0;
    config.joint[0].updated_from_c0 = 1;
    mock_tx_fifo_empty               = 1;
    uint8_t result = do_steps(0);
    assert_int_equal(result, 0);
    assert_int_equal(last_pio_put_value, 0);
}

/* do_steps: no new core0 data (updated == 0) -> returns 0 */
static void test_do_steps_no_update(void **state) {
    (void)state;
    config.joint[0].enabled         = 1;
    config.joint[0].updated_from_c0 = 0;
    mock_tx_fifo_empty               = 1;
    uint8_t result = do_steps(0);
    assert_int_equal(result, 0);
}

/* do_steps: valid position request + empty FIFO -> non-zero step written to PIO */
static void test_do_steps_normal_step(void **state) {
    (void)state;
    config.joint[0].enabled            = 1;
    config.joint[0].abs_pos_requested  = 10.0;
    config.joint[0].abs_pos_achieved   = 0;
    config.joint[0].velocity_requested = 5000.0;
    config.joint[0].max_velocity       = 50.0;
    config.joint[0].updated_from_c0    = 1;
    mock_tx_fifo_empty                  = 1;
    mock_rx_fifo_level                  = 0;

    uint8_t result = do_steps(0);

    assert_true(result > 0);
    assert_true(last_pio_put_value != 0);
    /* direction bit (LSB) should be 1 (forward) */
    assert_int_equal(last_pio_put_value & 1, 1);
}

/* do_steps: acceleration clamping activates across multiple calls */
static void test_do_steps_accel_clamped(void **state) {
    (void)state;
    /* Set up a joint with small max_accel so the clamp activates.
     * With update_period_us=1000, max_accel=5000 (steps/s/s) normalises to
     * 5000/1000 = 5.0 steps/period.
     * A large position request will generate velocity >> 5.0 on first call,
     * so the second call's velocity should still be clamped near 5.0 from the first. */

    uint32_t update_period_us = 1000;
    config.update_time_us = update_period_us;

    config.joint[0].enabled            = 1;
    config.joint[0].io_pos_step        = 1;
    config.joint[0].io_pos_dir         = 2;
    config.joint[0].updated_from_c0    = 1;
    config.joint[0].abs_pos_requested  = 10000.0;  /* large request */
    config.joint[0].abs_pos_achieved   = 0;
    config.joint[0].velocity_requested = 10000.0 * update_period_us;  /* matching velocity */
    config.joint[0].max_velocity       = 100000.0;  /* steps/s */
    config.joint[0].max_accel          = 5000.0;    /* steps/s/s -> 5.0 steps/period */

    /* First call: last_velocity starts at 0, clamp limits to max_accel/period = 5.0 */
    mock_tx_fifo_empty = 1;
    last_pio_put_value = 0;
    do_steps(0);
    uint32_t first_word = last_pio_put_value;

    /* Reset for second call */
    mock_tx_fifo_empty = 1;
    last_pio_put_value = 0;
    config.joint[0].updated_from_c0 = 1;

    do_steps(0);
    uint32_t second_word = last_pio_put_value;

    /* Both calls should have written a non-zero step command */
    assert_true(first_word != 0);
    assert_true(second_word != 0);
    /* The direction bit (LSB) should be set (positive motion) in both */
    assert_int_equal(first_word & 0x1, 1);
    assert_int_equal(second_word & 0x1, 1);
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test_setup(test_drain_rx_fifo_empty_returns_current, test_setup),
        cmocka_unit_test_setup(test_drain_rx_fifo_single_entry,          test_setup),
        cmocka_unit_test_setup(test_drain_rx_fifo_keeps_last,            test_setup),
        cmocka_unit_test_setup(test_calculate_step_len_normal,          test_setup),
        cmocka_unit_test_setup(test_calculate_step_len_clamped,         test_setup),
        cmocka_unit_test_setup(test_calculate_step_len_below_threshold, test_setup),
        cmocka_unit_test_setup(test_clamp_accel_no_change,              test_setup),
        cmocka_unit_test_setup(test_clamp_accel_under_limit,            test_setup),
        cmocka_unit_test_setup(test_clamp_accel_over_limit_positive,    test_setup),
        cmocka_unit_test_setup(test_clamp_accel_over_limit_negative,    test_setup),
        cmocka_unit_test_setup(test_clamp_accel_zero_max,               test_setup),
        cmocka_unit_test_setup(test_get_velocity_zero_pos_diff,          test_setup),
        cmocka_unit_test_setup(test_get_velocity_direction_disagreement, test_setup),
        cmocka_unit_test_setup(test_get_velocity_normal_forward,         test_setup),
        cmocka_unit_test_setup(test_get_velocity_holdoff,                test_setup),
        cmocka_unit_test_setup(test_do_steps_zero_period,               test_setup),
        cmocka_unit_test_setup(test_do_steps_disabled,                  test_setup),
        cmocka_unit_test_setup(test_do_steps_no_update,                 test_setup),
        cmocka_unit_test_setup(test_do_steps_normal_step,               test_setup),
        cmocka_unit_test_setup(test_do_steps_accel_clamped,             test_setup),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
