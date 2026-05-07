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
static int      pio_put_call_count  = 0;
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
    pio_put_call_count++;
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
        config.joint[j].io_pos_step = 1;  /* valid pin (0-31) */
        config.joint[j].io_pos_dir  = 2;  /* valid pin (0-31) */
        config.joint[j].max_velocity = 50.0;
    }
    mock_rx_fifo_level = 0;
    mock_rx_index      = 0;
    last_pio_put_value = 0;
    pio_put_call_count  = 0;   /* reset call counter */
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
    /* step_count=2.0 -> Q16.16=131072, period=133000, max_vel=50.0 -> Q16.16=3276800
     * 133000*65536=8716288000; 8716288000/(131072*2)=8716288000/262144=33250 exactly
     * 33250-9=33241 */
    int32_t result = calculate_step_len(131072, 133000, 3276800);
    assert_int_equal(result, 33241);
}

/* calculate_step_len: step_count exceeds max_velocity -> clamped to min */
static void test_calculate_step_len_clamped(void **state) {
    (void)state;
    /* step_count=200.0, max_vel=50.0, both in Q16.16 */
    int32_t result = calculate_step_len(13107200, 133000, 3276800);
    /* len=323, min=1321, result=1321 */
    assert_int_equal(result, 1321);
}

/* calculate_step_len: step_count_q=0 -> 0 (division-by-zero guard) */
static void test_calculate_step_len_below_threshold(void **state) {
    (void)state;
    assert_int_equal(calculate_step_len(0, 133000, 3276800), 0);
}

/* calculate_step_len: slow velocity -> capped at max_len (fits in one period) */
static void test_calculate_step_len_too_slow_skip(void **state) {
    (void)state;
    /* 0.05 steps/period (sq=3276): raw len >> period_ticks, capped to max_len=66491 */
    assert_int_equal(calculate_step_len(3276, 133000, 3276800), 66491);
    /* 0.1 steps/period (sq=6553): same cap */
    assert_int_equal(calculate_step_len(6553, 133000, 3276800), 66491);
    /* sq=1 (extreme): int64 intermediate would overflow int32, capped to max_len */
    assert_int_equal(calculate_step_len(1, 133000, 3276800), 66491);
}

/* clamp_accel: velocity unchanged -> returns same velocity */
static void test_clamp_accel_no_change(void **state) {
    (void)state;
    int32_t result = clamp_accel(327680, 327680, 131072);
    assert_int_equal(result, 327680);  /* 5.0 unchanged */
}

/* clamp_accel: acceleration under limit -> returns requested velocity */
static void test_clamp_accel_under_limit(void **state) {
    (void)state;
    int32_t result = clamp_accel(458752, 327680, 196608);
    assert_int_equal(result, 458752);  /* delta=131072 < max_accel=196608, not clamped */
}

/* clamp_accel: acceleration over limit positive -> clamped to max_accel */
static void test_clamp_accel_over_limit_positive(void **state) {
    (void)state;
    int32_t result = clamp_accel(655360, 327680, 131072);
    assert_int_equal(result, 458752);  /* 5.0 + 2.0 = 7.0 */
}

/* clamp_accel: deceleration over limit -> clamped to max_accel */
static void test_clamp_accel_over_limit_negative(void **state) {
    (void)state;
    int32_t result = clamp_accel(65536, 327680, 131072);
    assert_int_equal(result, 196608);  /* 5.0 - 2.0 = 3.0 */
}

/* clamp_accel: zero max_accel -> returns requested velocity unchanged */
static void test_clamp_accel_zero_max(void **state) {
    (void)state;
    int32_t result = clamp_accel(6553600, 0, 0);
    assert_int_equal(result, 6553600);  /* max_accel=0 -> no limit */
}

/* plan_steps: fractional accumulation -> step fires on 4th call */
static void test_plan_steps_fractional_accumulation(void **state) {
    (void)state;
    /* velocity_q = (int32_t)(0.3 * 65536) = 19660 (C truncates toward zero)
     * acc: 19660, 39320, 58980, 78640 -> steps: 0,0,0,1 */
    int32_t vq = (int32_t)(0.3 * 65536);
    assert_int_equal(plan_steps(vq, 0, 133000, 1), 0);
    assert_int_equal(plan_steps(vq, 0, 133000, 1), 0);
    assert_int_equal(plan_steps(vq, 0, 133000, 1), 0);
    assert_int_equal(plan_steps(vq, 0, 133000, 1), 1);
}

/* plan_steps: correct total over 10 periods for fractional velocity */
static void test_plan_steps_total_over_ten_periods(void **state) {
    (void)state;
    /* Use 2.75 (= 180224 in Q16.16, exactly representable); 10 periods -> 27 steps */
    int32_t total = 0;
    for (int i = 0; i < 10; i++) {
        total += plan_steps(180224, 0, 133000, 1);
    }
    assert_int_equal(total, 27);
}

/* plan_steps: excess steps returned to accumulator when max_steps limits output */
static void test_plan_steps_excess_returned_to_accumulator(void **state) {
    (void)state;
    /* velocity_q=196608 (3.0), step_period=50000, max_steps=133000/50000=2
     * desired=3, capped to 2, excess=1 returned; next call vel=0, acc=1.0 -> 1 step */
    assert_int_equal(plan_steps(196608, 0, 133000, 24991), 2);
    assert_int_equal(plan_steps(0,      0, 133000, 24991), 1);
}

/* plan_steps: zero velocity -> no steps, accumulator stays 0 */
static void test_plan_steps_zero_velocity(void **state) {
    (void)state;
    assert_int_equal(plan_steps(0, 0, 133000, 13291), 0);
    assert_int_equal(plan_steps(0, 0, 133000, 13291), 0);
}

/* plan_steps: step_len=0 (skip signal) -> max_steps=0, accumulator preserved */
static void test_plan_steps_skip_preserves_accumulator(void **state) {
    (void)state;
    /* velocity_q=65536 (1.0 step/period). Two calls with step_len=0 skip both
     * steps but accumulator grows to 2.0. Third call with normal step_len
     * issues 2 steps immediately from the banked accumulator. */
    assert_int_equal(plan_steps(65536, 0, 133000, 0), 0);
    assert_int_equal(plan_steps(65536, 0, 133000, 0), 0);
    assert_int_equal(plan_steps(0,     0, 133000, 13291), 2);
}

/* do_steps: update_period == 0, enabled joint -> returns 0 without dividing */
static void test_do_steps_zero_period(void **state) {
    (void)state;
    config.update_time_us             = 0;
    config.joint[0].enabled           = 1;
    config.joint[0].updated_from_c0   = 1;
    config.joint[0].abs_pos_requested = 10.0;
    mock_tx_fifo_empty                = 1;
    uint8_t result = do_steps(0);
    assert_int_equal(result, 0);
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

/* do_steps: no new core0 data (updated == 0), slow last_velocity -> writes 0 to PIO, returns 0 */
static void test_do_steps_no_update(void **state) {
    (void)state;
    config.joint[0].enabled         = 1;
    config.joint[0].updated_from_c0 = 0;
    mock_tx_fifo_empty               = 1;
    uint8_t result = do_steps(0);
    assert_int_equal(result, 0);
}

/* do_steps: valid position request + empty FIFO -> non-zero step written to PIO.
 * In default position mode the error (1000 steps) drives velocity well above
 * MIN_STEP_COUNT_Q so a step is issued; velocity_requested is ignored. */
static void test_do_steps_normal_step(void **state) {
    (void)state;
    config.joint[0].enabled            = 1;
    config.joint[0].abs_pos_requested  = 1000.0;  /* 1000-step error */
    config.joint[0].abs_pos_achieved   = 0;
    config.joint[0].velocity_requested = 5000.0;   /* ignored in position mode */
    config.joint[0].max_velocity       = 50000.0;
    config.joint[0].max_accel          = 0.0;  /* no accel limit so first call steps */
    config.joint[0].updated_from_c0    = 1;
    mock_tx_fifo_empty                  = 1;
    mock_rx_fifo_level                  = 0;

    uint8_t result = do_steps(0);

    assert_true(result > 0);
    assert_true(last_pio_put_value != 0);
    /* direction bit (LSB) should be 1 (forward) */
    assert_int_equal(last_pio_put_value & 1, 1);
}

/* do_steps: acceleration clamping activates across multiple calls (velocity mode) */
static void test_do_steps_accel_clamped(void **state) {
    (void)state;
    /* Set up a joint with small max_accel so the clamp activates after enable.
     * With update_period_us=1000, max_accel=5000 (steps/s/s) normalises to
     * 5000/1000 = 5.0 steps/period.
     *
     * On enable the RP2040 snaps last_velocity_q to the commanded velocity, so
     * the first call must use a low velocity (5.0 steps/period) to prime state.
     * The second call then jumps to a large velocity; the clamp limits the
     * increase to max_accel/period = 5.0, landing at 10.0 steps/period. */

    uint32_t update_period_us = 1000;
    config.update_time_us = update_period_us;

    config.joint[0].enabled            = 1;
    config.joint[0].io_pos_step        = 1;
    config.joint[0].io_pos_dir         = 2;
    config.joint[0].cmd_type           = JOINT_CMD_VELOCITY;
    config.joint[0].updated_from_c0    = 1;
    config.joint[0].abs_pos_requested  = 10000.0;
    config.joint[0].abs_pos_achieved   = 0;
    config.joint[0].velocity_requested = 5.0 * update_period_us;  /* low: snap primes to 5.0 */
    config.joint[0].max_velocity       = 100000.0;  /* steps/s */
    config.joint[0].max_accel          = 5000.0;    /* steps/s/s -> 5.0 steps/period */

    /* First call: enable transition snaps last_velocity_q to 5.0 steps/period.
     * velocity=5.0 -> step_len=(133000/(5*2))-9=13291 */
    mock_tx_fifo_empty = 1;
    last_pio_put_value = 0;
    do_steps(0);
    uint32_t first_word = last_pio_put_value;

    /* Second call: jump velocity to 10000 steps/s; clamp limits increase to 5.0,
     * so velocity reaches 10.0 steps/period -> step_len=(133000/(10*2))-9=6641 */
    mock_tx_fifo_empty = 1;
    last_pio_put_value = 0;
    config.joint[0].updated_from_c0    = 1;
    config.joint[0].velocity_requested = 10000.0 * update_period_us;

    do_steps(0);
    uint32_t second_word = last_pio_put_value;

    assert_int_equal(first_word >> 1, 13291);
    assert_int_equal(first_word & 0x1, 1);
    assert_int_equal(second_word >> 1, 6641);
    assert_int_equal(second_word & 0x1, 1);
}

/* do_steps: position mode drives toward abs_pos_requested.
 * vel_ff=0 (velocity_requested=0 in test), error=1000, Kp=0.5:
 * velocity = 0 + 1000*(1e6/1000)*0.5 = 500 000 steps/s >> MIN_STEP_COUNT_Q.
 * Direction bit (LSB) must be 1 (positive error). */
static void test_do_steps_position_mode_drives_forward(void **state) {
    (void)state;
    config.joint[0].enabled            = 1;
    config.joint[0].cmd_type           = JOINT_CMD_POSITION;
    config.joint[0].abs_pos_requested  = 1000.0;
    config.joint[0].abs_pos_achieved   = 0;
    config.joint[0].velocity_requested = 0.0;  /* ignored in position mode */
    config.joint[0].max_velocity       = 50000.0;
    config.joint[0].max_accel          = 0.0;
    config.joint[0].updated_from_c0    = 1;
    mock_tx_fifo_empty                  = 1;
    mock_rx_fifo_level                  = 0;

    uint8_t result = do_steps(0);

    assert_true(result > 0);
    assert_true(last_pio_put_value != 0);
    assert_int_equal(last_pio_put_value & 1, 1);  /* direction = forward */
}

/* do_steps: position mode reverses when past target.
 * abs_pos_achieved=1000 > abs_pos_requested=0 => error=-1000 => reverse. */
static void test_do_steps_position_mode_reverses(void **state) {
    (void)state;
    config.joint[0].enabled            = 1;
    config.joint[0].cmd_type           = JOINT_CMD_POSITION;
    config.joint[0].abs_pos_requested  = 0.0;
    config.joint[0].abs_pos_achieved   = 1000;
    config.joint[0].velocity_requested = 0.0;
    config.joint[0].max_velocity       = 50000.0;
    config.joint[0].max_accel          = 0.0;
    config.joint[0].updated_from_c0    = 1;
    mock_tx_fifo_empty                  = 1;
    mock_rx_fifo_level                  = 0;

    do_steps(0);

    assert_int_equal(last_pio_put_value & 1, 0);  /* direction = reverse */
}

/* do_steps: position mode at target -> no steps.
 * vel_ff=0 (LinuxCNC vel_cmd=0 at rest), error=0 -> velocity=0. */
static void test_do_steps_position_mode_at_target(void **state) {
    (void)state;
    config.joint[0].enabled            = 1;
    config.joint[0].cmd_type           = JOINT_CMD_POSITION;
    config.joint[0].abs_pos_requested  = 0.0;
    config.joint[0].abs_pos_achieved   = 0;
    config.joint[0].velocity_requested = 0.0;  /* vel_cmd=0: machine at rest */
    config.joint[0].max_velocity       = 50000.0;
    config.joint[0].max_accel          = 0.0;
    config.joint[0].updated_from_c0    = 1;
    mock_tx_fifo_empty                  = 1;
    mock_rx_fifo_level                  = 0;

    uint8_t result = do_steps(0);

    assert_true(result > 0);
    assert_int_equal(last_pio_put_value, 0);  /* no steps when at target */
}

/* do_steps: enabled, no position error -> n_steps=0 -> puts 0 to FIFO */
static void test_do_steps_no_motion(void **state) {
    (void)state;
    /* All positions at zero and velocity_requested=0 -> get_velocity returns 0.0
     * -> step_len=0 -> plan_steps returns 0 -> PIO should receive 0. */
    config.joint[0].enabled            = 1;
    config.joint[0].abs_pos_requested  = 0.0;
    config.joint[0].abs_pos_achieved   = 0;
    config.joint[0].velocity_requested = 0.0;
    config.joint[0].max_velocity       = 50.0;
    config.joint[0].updated_from_c0    = 1;
    mock_tx_fifo_empty                  = 1;
    mock_rx_fifo_level                  = 0;

    uint8_t result = do_steps(0);

    assert_true(result > 0);
    assert_int_equal(last_pio_put_value, 0);
}

/* do_steps: underrun (no new data from Core0) always writes 0 to PIO to prevent
 * the state machine re-executing a stale step_len and generating spurious steps. */
static void test_do_steps_underrun_stops_pio(void **state) {
    (void)state;
    config.joint[0].enabled            = 1;
    config.joint[0].abs_pos_requested  = 10.0;
    config.joint[0].abs_pos_achieved   = 0;
    config.joint[0].velocity_requested = 5000.0;
    config.joint[0].max_velocity       = 50.0;
    config.joint[0].max_accel          = 0.0;
    config.joint[0].updated_from_c0    = 1;
    mock_tx_fifo_empty                  = 1;
    do_steps(0);   /* prime last_velocity_q to a non-zero value */

    config.joint[0].updated_from_c0 = 0;
    pio_put_call_count               = 0;
    last_pio_put_value               = 0xDEADBEEF;
    mock_tx_fifo_empty               = 1;

    uint8_t result = do_steps(0);

    assert_int_equal(result, 0);
    assert_int_equal(pio_put_call_count, 1);
    assert_int_equal(last_pio_put_value, 0);
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test_setup(test_drain_rx_fifo_empty_returns_current, test_setup),
        cmocka_unit_test_setup(test_drain_rx_fifo_single_entry,          test_setup),
        cmocka_unit_test_setup(test_drain_rx_fifo_keeps_last,            test_setup),
        cmocka_unit_test_setup(test_calculate_step_len_normal,          test_setup),
        cmocka_unit_test_setup(test_calculate_step_len_clamped,         test_setup),
        cmocka_unit_test_setup(test_calculate_step_len_below_threshold, test_setup),
        cmocka_unit_test_setup(test_calculate_step_len_too_slow_skip,  test_setup),
        cmocka_unit_test_setup(test_clamp_accel_no_change,              test_setup),
        cmocka_unit_test_setup(test_clamp_accel_under_limit,            test_setup),
        cmocka_unit_test_setup(test_clamp_accel_over_limit_positive,    test_setup),
        cmocka_unit_test_setup(test_clamp_accel_over_limit_negative,    test_setup),
        cmocka_unit_test_setup(test_clamp_accel_zero_max,               test_setup),
        cmocka_unit_test_setup(test_plan_steps_fractional_accumulation,        test_setup),
        cmocka_unit_test_setup(test_plan_steps_total_over_ten_periods,         test_setup),
        cmocka_unit_test_setup(test_plan_steps_excess_returned_to_accumulator, test_setup),
        cmocka_unit_test_setup(test_plan_steps_zero_velocity,                  test_setup),
        cmocka_unit_test_setup(test_plan_steps_skip_preserves_accumulator,     test_setup),
        cmocka_unit_test_setup(test_do_steps_zero_period,               test_setup),
        cmocka_unit_test_setup(test_do_steps_disabled,                  test_setup),
        cmocka_unit_test_setup(test_do_steps_no_update,                 test_setup),
        cmocka_unit_test_setup(test_do_steps_normal_step,               test_setup),
        cmocka_unit_test_setup(test_do_steps_accel_clamped,                      test_setup),
        cmocka_unit_test_setup(test_do_steps_no_motion,                          test_setup),
        cmocka_unit_test_setup(test_do_steps_underrun_stops_pio,                 test_setup),
        cmocka_unit_test_setup(test_do_steps_position_mode_drives_forward,       test_setup),
        cmocka_unit_test_setup(test_do_steps_position_mode_reverses,             test_setup),
        cmocka_unit_test_setup(test_do_steps_position_mode_at_target,            test_setup),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
