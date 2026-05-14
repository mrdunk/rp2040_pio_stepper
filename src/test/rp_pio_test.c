#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
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

/* do_steps: joint disabled with steps in RX FIFO -> abs_pos_achieved updated.
 * Regression for estop-during-motion jitter: in-flight steps must be drained
 * while disabled so pos_fb (= abs_pos_achieved / scale) stays current.  The
 * driver clamps pos_cmd = pos_fb while disabled; without the drain, pos_cmd
 * falls behind by the in-flight step count, causing a correction move on
 * re-enable that the user sees as persistent jitter. */
static void test_do_steps_disabled_drains_rx_fifo(void **state) {
    (void)state;
    config.joint[0].enabled          = 0;
    config.joint[0].updated_from_c0  = 1;
    config.joint[0].abs_pos_achieved = 100;
    mock_tx_fifo_empty                = 1;
    mock_rx_fifo_level                = 1;
    mock_rx_values[0]                 = 103;  /* 3 extra steps took while stopping */

    uint8_t result = do_steps(0);

    assert_int_equal(result, 0);
    assert_int_equal(last_pio_put_value, 0);
    assert_int_equal(config.joint[0].abs_pos_achieved, 103);
}

/* do_steps: joint disabling with non-zero velocity -> continues stepping while decelerating. */
static void test_do_steps_disabling_decelerates(void **state) {
    (void)state;
    /* Prime last_velocity_q at 10 steps/period via an enabled cycle. */
    config.update_time_us              = 1000;
    config.joint[0].enabled            = 1;
    config.joint[0].cmd_type           = JOINT_CMD_VELOCITY;
    config.joint[0].updated_from_c0    = 1;
    config.joint[0].velocity_requested = 10000.0;  /* 10 steps/period at 1000µs */
    config.joint[0].max_velocity       = 50000.0;
    config.joint[0].max_accel          = 5000000.0; /* 5 steps/period/period: 5e6 × (1e-3)² = 5 */
    mock_tx_fifo_empty                 = 1;
    do_steps(0);  /* enable snap: last_velocity_q = 10 steps/period */

    /* Disable: clamp brings velocity 10 -> 5 (not zero yet) -> steps still issued. */
    config.joint[0].enabled         = 0;
    config.joint[0].updated_from_c0 = 1;
    last_pio_put_value               = 0;
    pio_put_call_count               = 0;
    mock_tx_fifo_empty               = 1;
    uint8_t result = do_steps(0);

    assert_int_equal(result, 0);
    assert_true(last_pio_put_value != 0);  /* still decelerating, not hard-stopped */
}

/* do_steps: joint disabling with velocity equal to one max_accel step -> reaches zero -> hard stop. */
static void test_do_steps_disabling_stops_when_zero(void **state) {
    (void)state;
    /* Prime last_velocity_q at exactly 5 steps/period (= max_accel_q). */
    config.update_time_us              = 1000;
    config.joint[0].enabled            = 1;
    config.joint[0].cmd_type           = JOINT_CMD_VELOCITY;
    config.joint[0].updated_from_c0    = 1;
    config.joint[0].velocity_requested = 5000.0;   /* 5 steps/period */
    config.joint[0].max_velocity       = 50000.0;
    config.joint[0].max_accel          = 5000000.0; /* 5 steps/period/period: 5e6 × (1e-3)² = 5 */
    mock_tx_fifo_empty                 = 1;
    do_steps(0);  /* enable snap: last_velocity_q = 5 steps/period */

    /* Disable: clamp_accel(0, 5, 5) = 0 -> velocity_q == 0 -> hard stop. */
    config.joint[0].enabled          = 0;
    config.joint[0].updated_from_c0  = 1;
    last_pio_put_value                = 0xDEADBEEF;
    pio_put_call_count                = 0;
    mock_tx_fifo_empty                = 1;
    uint8_t result = do_steps(0);

    assert_int_equal(result, 0);
    assert_int_equal(last_pio_put_value, 0);  /* hard-stopped */
}

/* do_steps: network lost (updated==0) while disabled and still moving -> continues decelerating. */
static void test_do_steps_network_loss_decelerates(void **state) {
    (void)state;
    /* Prime last_velocity_q at 10 steps/period via an enabled cycle. */
    config.update_time_us              = 1000;
    config.joint[0].enabled            = 1;
    config.joint[0].cmd_type           = JOINT_CMD_VELOCITY;
    config.joint[0].updated_from_c0    = 1;
    config.joint[0].velocity_requested = 10000.0;
    config.joint[0].max_velocity       = 50000.0;
    config.joint[0].max_accel          = 5000.0;
    mock_tx_fifo_empty                 = 1;
    do_steps(0);  /* enable snap: last_velocity_q = 10 steps/period */

    /* Simulate network loss: disabled, no new packet. */
    config.joint[0].enabled          = 0;
    config.joint[0].updated_from_c0  = 0;  /* no new Core0 data */
    last_pio_put_value               = 0;
    pio_put_call_count               = 0;
    mock_tx_fifo_empty               = 1;
    uint8_t result = do_steps(0);

    assert_int_equal(result, 0);
    assert_true(last_pio_put_value != 0);  /* decelerating, not hard-stopped */
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
     * With update_period_us=1000, max_accel=5000000 (steps/s²) normalises to
     * 5000000 × (1e-3)² = 5.0 steps/period.
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
    config.joint[0].max_accel          = 5000000.0;  /* 5e6 steps/s² → 5.0 steps/period/period */

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

/* --- compute_velocity_cmd unit tests --- */

/* Velocity mode, enabled, updated: returns velocity_requested unchanged. */
static void test_compute_velocity_cmd_velmode_passthrough(void **state) {
    (void)state;
    double result = compute_velocity_cmd(
        JOINT_CMD_VELOCITY, 5000.0, 0.0, 0, /*enabled=*/1, /*updated=*/1, 1000, 0.0);
    assert_float_equal(result, 5000.0, 1e-6);
}

/* Position mode, zero error: returns vel_ff with no correction. */
static void test_compute_velocity_cmd_posmode_at_target(void **state) {
    (void)state;
    double result = compute_velocity_cmd(
        JOINT_CMD_POSITION, 1000.0, 100.0, 100, /*enabled=*/1, /*updated=*/1, 1000, 0.0);
    assert_float_equal(result, 1000.0, 1e-6);
}

/* Position mode, dead zone (|error| < 1 step): no correction applied. */
static void test_compute_velocity_cmd_posmode_dead_zone(void **state) {
    (void)state;
    double result = compute_velocity_cmd(
        JOINT_CMD_POSITION, 500.0, 100.4, 100, /*enabled=*/1, /*updated=*/1, 1000, 0.0);
    assert_float_equal(result, 500.0, 1e-6);
}

/* Position mode, positive error: vel_ff + Kp*error*rate.
 * error=10 steps, period=1000µs: correction = 10*(1e6/1000)*0.5 = 5000 steps/s. */
static void test_compute_velocity_cmd_posmode_forward_correction(void **state) {
    (void)state;
    double result = compute_velocity_cmd(
        JOINT_CMD_POSITION, 1000.0, 110.0, 100, /*enabled=*/1, /*updated=*/1, 1000, 0.0);
    assert_float_equal(result, 1000.0 + 5000.0, 1e-6);
}

/* Position mode, negative error: vel_ff + negative correction. */
static void test_compute_velocity_cmd_posmode_reverse_correction(void **state) {
    (void)state;
    double result = compute_velocity_cmd(
        JOINT_CMD_POSITION, 1000.0, 90.0, 100, /*enabled=*/1, /*updated=*/1, 1000, 0.0);
    assert_float_equal(result, 1000.0 - 5000.0, 1e-6);
}

/* Disabled: returns 0 regardless of mode and error. */
static void test_compute_velocity_cmd_disabled_returns_zero(void **state) {
    (void)state;
    double result = compute_velocity_cmd(
        JOINT_CMD_POSITION, 1000.0, 200.0, 100, /*enabled=*/0, /*updated=*/1, 1000, 0.0);
    assert_float_equal(result, 0.0, 1e-6);
}

/* Underrun (updated=0): returns 0 regardless of mode. */
static void test_compute_velocity_cmd_underrun_returns_zero(void **state) {
    (void)state;
    double result = compute_velocity_cmd(
        JOINT_CMD_VELOCITY, 5000.0, 0.0, 0, /*enabled=*/1, /*updated=*/0, 1000, 0.0);
    assert_float_equal(result, 0.0, 1e-6);
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

/* do_steps: underrun (no new data from Core0) with max_accel=0 -> velocity snaps
 * to 0 immediately -> writes 0 to PIO.  With max_accel>0 it decelerates instead
 * (see test_do_steps_underrun_while_enabled_decelerates). */
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

/* do_steps: underrun (no new data) while enabled and moving with max_accel>0 ->
 * decelerates instead of hard-stopping.  This is the key network-loss scenario:
 * Core0 is blocked at get_UDP(), updated==0, but the joint is still enabled and
 * last_velocity_q is non-zero.  Motor must keep stepping (decelerate) rather than
 * crash-stopping before handle_network_timeout() fires. */
static void test_do_steps_underrun_while_enabled_decelerates(void **state) {
    (void)state;
    /* Prime last_velocity_q at 10 steps/period via an enabled cycle. */
    config.update_time_us              = 1000;
    config.joint[0].enabled            = 1;
    config.joint[0].cmd_type           = JOINT_CMD_VELOCITY;
    config.joint[0].updated_from_c0    = 1;
    config.joint[0].velocity_requested = 10000.0;  /* 10 steps/period at 1000µs */
    config.joint[0].max_velocity       = 50000.0;
    config.joint[0].max_accel          = 5000000.0; /* 5e6 steps/s² → 5.0 steps/period/period */
    mock_tx_fifo_empty                 = 1;
    do_steps(0);  /* enable snap: last_velocity_q = 10 steps/period */

    /* Underrun while still enabled (Core0 blocked on network). */
    config.joint[0].updated_from_c0 = 0;
    last_pio_put_value               = 0;
    pio_put_call_count               = 0;
    mock_tx_fifo_empty               = 1;
    uint8_t result = do_steps(0);

    assert_int_equal(result, 0);
    assert_true(last_pio_put_value != 0);  /* still decelerating, not hard-stopped */
}

/* Integration test: position-mode stationary hold at fractional step position.
 *
 * Uses pico-eth-cnc-3axis.ini parameters:
 *   period=1ms, scale=1280 steps/mm, max_vel=25mm/s=32000 steps/s,
 *   max_accel=375mm/s²=480000 steps/s².
 *
 * Scenario: joint at rest (vel_ff=0), abs_pos_requested=100.5, abs_pos_achieved=100.
 *
 *   Kp correction = (100.5-100) * (1e6/1000) * 0.5 = 250 steps/s = 0.25 steps/period
 *   velocity_q = (250/1000) * 65536 = 16384
 *   step_len   = (133000*65536/32768) - 9 = 265991 ticks  (spans ~2 servo periods)
 *
 * dunk_stepgen_refactor_involved: 265991 >= period_ticks → calculate_step_len returns 0
 *   → plan_steps: max_steps=0 → no step fires → stable hold.
 *
 * dunk_fix_cmd_pos: 265991 > max_len → capped to 66491 → plan_steps fires 1 step every
 *   4 periods → visible back-and-forth jitter on hardware when machine is stationary.
 *
 * Expected: zero steps fired over 20 servo periods. */
static void test_do_steps_position_mode_no_jitter_at_rest(void **state) {
    (void)state;

    config.update_time_us              = 1000;
    config.joint[0].enabled            = 1;
    config.joint[0].cmd_type           = JOINT_CMD_POSITION;
    config.joint[0].velocity_requested = 0.0;     /* vel_ff=0: machine at rest */
    config.joint[0].abs_pos_requested  = 100.5;   /* fractional: 0.5-step Kp error */
    config.joint[0].abs_pos_achieved   = 100;
    config.joint[0].max_velocity       = 32000.0; /* 25 mm/s * 1280 steps/mm */
    config.joint[0].max_accel          = 480000.0; /* 375 mm/s² * 1280 steps/mm */
    mock_rx_fifo_level                 = 0;  /* PIO counter unchanged: no steps from hardware */
    mock_tx_fifo_empty                 = 1;

    int steps_fired = 0;
    for (int i = 0; i < 20; i++) {
        config.joint[0].updated_from_c0 = 1;
        last_pio_put_value = 0;
        do_steps(0);
        if (last_pio_put_value != 0) {
            steps_fired++;
        }
    }

    assert_int_equal(steps_fired, 0);
}

/* ── Velocity-mode position-accuracy integration tests ────────────────────
 *
 * Physical model: when do_steps writes a non-zero word to the PIO FIFO, the
 * step_gen programme loops at rate step_period = 2*(step_len+9) ticks.  In one
 * servo period (133000 ticks @ 1ms/133MHz) it generates
 *   max_steps = 133000 / step_period
 * physical steps.  pio_word_steps() converts a FIFO word to that signed count.
 *
 * Accuracy analysis:
 *
 *   v < 1 step/period:
 *     calculate_step_len caps step_len at max_len=66491 → step_period=133000 →
 *     max_steps=1.  Bresenham schedules 0 or 1 step each period; long-run
 *     average equals v exactly.  Position is EXACT.
 *
 *   v = integer steps/period:
 *     step_len sized so max_steps = v exactly; Bresenham desired = v always.
 *     Position is EXACT.
 *
 *   v = non-integer, v > 1 step/period:
 *     calculate_step_len sizes step_len for ceil(v) → max_steps = ceil(v).
 *     Bresenham alternates floor(v) and ceil(v) steps per period; long-run
 *     average equals v exactly.  Position is EXACT.
 */

/* Convert one PIO FIFO word to the physical step count it causes.
 * Returns negative for direction=0 (reverse). */
static int32_t pio_word_steps(uint32_t word) {
    if (word == 0) return 0;
    int32_t step_len  = (int32_t)(word >> 1);
    int32_t max_steps = 133000 / (2 * (step_len + 9));
    return (word & 1) ? max_steps : -max_steps;
}

/* Run n servo periods in JOINT_CMD_VELOCITY mode; return accumulated position.
 * No acceleration limit so the commanded velocity takes effect immediately.
 * Uses pico-eth-cnc-3axis parameters: 1ms period, max_vel=32000 steps/s. */
static int32_t run_velocity_periods(double vel_steps_per_s, int n) {
    config.update_time_us              = 1000;
    config.joint[0].cmd_type           = JOINT_CMD_VELOCITY;
    config.joint[0].enabled            = 1;
    config.joint[0].velocity_requested = vel_steps_per_s;
    config.joint[0].abs_pos_requested  = 0.0;
    config.joint[0].abs_pos_achieved   = 0;
    config.joint[0].max_velocity       = 32000.0;
    config.joint[0].max_accel          = 0.0;
    mock_tx_fifo_empty                 = 1;

    int32_t sim_pos = 0;
    for (int i = 0; i < n; i++) {
        mock_rx_values[0]  = sim_pos;
        mock_rx_fifo_level = 1;
        mock_rx_index      = 0;
        config.joint[0].updated_from_c0 = 1;
        last_pio_put_value = 0;
        do_steps(0);
        sim_pos += pio_word_steps(last_pio_put_value);
    }
    return sim_pos;
}

/* 0.75 steps/period (750 steps/s): sub-1 fraction, exact.
 * step_len capped → max_steps=1.  Bresenham: 0,1,1,1 per 4 periods.
 * 25 complete cycles of 4 → 75 steps. */
static void test_do_steps_velmode_0_75(void **state) {
    (void)state;
    assert_int_equal(run_velocity_periods(750.0, 100), 75);
}

/* 0.25 steps/period (250 steps/s): sub-1 fraction, exact.
 * Bresenham: 0,0,0,1 per 4 periods → 25 steps in 100 periods. */
static void test_do_steps_velmode_0_25(void **state) {
    (void)state;
    assert_int_equal(run_velocity_periods(250.0, 100), 25);
}

/* 1.0 steps/period (1000 steps/s): integer boundary, exact.
 * step_len=66491 (=max_len), max_steps=1, desired=1 every period → 100 steps. */
static void test_do_steps_velmode_int_1(void **state) {
    (void)state;
    assert_int_equal(run_velocity_periods(1000.0, 100), 100);
}

/* 10.0 steps/period (10000 steps/s): integer, exact.
 * step_len=6641, max_steps=10, desired=10 every period → 1000 steps. */
static void test_do_steps_velmode_int_10(void **state) {
    (void)state;
    assert_int_equal(run_velocity_periods(10000.0, 100), 1000);
}

/* 1.5 steps/period (1500 steps/s): non-integer above 1, now exact.
 * v_ceil=2, step_len=33241, max_steps=2.
 * Bresenham alternates 1,2 → 150 steps in 100 periods. */
static void test_do_steps_velmode_frac_1_5(void **state) {
    (void)state;
    assert_int_equal(run_velocity_periods(1500.0, 100), 150);
}

/* 10.5 steps/period (10500 steps/s): non-integer above 10, now exact.
 * v_ceil=11, step_len=6036, max_steps=11.
 * Bresenham alternates 10,11 → 1050 steps in 100 periods. */
static void test_do_steps_velmode_frac_10_5(void **state) {
    (void)state;
    assert_int_equal(run_velocity_periods(10500.0, 100), 1050);
}

/* -0.75 steps/period (-750 steps/s): reverse direction.
 * Identical Bresenham schedule to +0.75 but direction bit=0.
 * 75 reverse steps → position -75. */
static void test_do_steps_velmode_reverse(void **state) {
    (void)state;
    assert_int_equal(run_velocity_periods(-750.0, 100), -75);
}

/* ── clamp_accel oscillation tests ──────────────────────────────────────────
 *
 * clamp_accel alone is monotonic: with a fixed target of 0 it can only reduce
 * |velocity|, never overshoot.  The oscillation bug lives in the *interaction*
 * between the position-mode correction term (which recomputes a new target each
 * period based on position error) and clamp_accel.
 *
 * When the motor overshoots abs_pos_requested the error flips sign, the
 * correction flips sign, and clamp_accel chases a reversed target — causing
 * the velocity to swing to approximately the original jogging speed in the
 * opposite direction.  The loop is indefinite.
 *
 * Two tests below establish this:
 *   1. clamp_accel with a fixed zero target never overshoots (PASSES — control).
 *   2. position-correction + clamp_accel loop oscillates indefinitely (FAILS —
 *      proves the bug; fix it so this test passes).
 */

/* Simulate position-mode correction + clamp_accel for max_periods iterations.
 * Motor starts at position 0 (= abs_pos_requested) moving at init_velocity_q.
 * max_accel is steps/s² — mirrors the bang-bang cap in compute_velocity_cmd.
 * Returns the number of velocity-sign reversals that occurred. */
static int posmode_decel_sign_changes(
    int32_t init_velocity_q,
    int32_t max_accel_q,
    int     max_periods)
{
    const double period_us  = 1000.0;
    /* max_accel_q = max_accel × period_s² × 65536 → max_accel = max_accel_q / (period_s² × 65536) */
    const double period_s   = period_us * 1e-6;
    const double max_accel  = (double)max_accel_q / (period_s * period_s * 65536.0);

    double  float_pos  = 0.0;
    int32_t velocity_q = init_velocity_q;
    int     sign_changes = 0;
    int32_t prev_sign    = (velocity_q >= 0) ? 1 : -1;

    for (int i = 0; i < max_periods; i++) {
        float_pos += (double)velocity_q / 65536.0;
        int32_t pos_int = (int32_t)float_pos;

        double error = 0.0 - (double)pos_int;
        double correction = 0.0;
        if (error >= 1.0 || error <= -1.0) {
            correction = error * (1e6 / period_us) * 0.5;
            if (max_accel > 0.0) {
                double max_correction = sqrt(2.0 * max_accel * fabs(error));
                if (correction >  max_correction) correction =  max_correction;
                if (correction < -max_correction) correction = -max_correction;
            }
        }

        int32_t target_vel_q = (int32_t)(correction / period_us * 65536.0);
        velocity_q = clamp_accel(target_vel_q, velocity_q, max_accel_q);

        /* Hard velocity cap — mirrors do_steps: cap when approaching target. */
        if (max_accel_q > 0 && pos_int != 0 && (int64_t)velocity_q * (-pos_int) > 0) {
            int32_t max_vel_q = (int32_t)sqrt(
                2.0 * (double)max_accel_q * (double)abs(pos_int) * 65536.0);
            if (velocity_q >  max_vel_q) velocity_q =  max_vel_q;
            if (velocity_q < -max_vel_q) velocity_q = -max_vel_q;
        }

        if (velocity_q != 0) {
            int32_t sign = (velocity_q > 0) ? 1 : -1;
            if (sign != prev_sign) {
                sign_changes++;
                prev_sign = sign;
            }
        }
    }
    return sign_changes;
}

/* clamp_accel with a fixed target of 0 must never overshoot zero.
 * Velocity decreases monotonically and sign never changes. */
static void test_clamp_accel_fixed_zero_target_never_overshoots(void **state) {
    (void)state;
    int32_t velocity_q  = 10 * 65536;  /* 10 steps/period */
    int32_t max_accel_q = 32768;        /* 0.5 steps/period/period */
    int32_t prev = velocity_q;

    for (int i = 0; i < 100000; i++) {
        velocity_q = clamp_accel(0, velocity_q, max_accel_q);
        assert_true(velocity_q >= 0);   /* must not go negative */
        assert_true(velocity_q <= prev); /* must not increase */
        prev = velocity_q;
        if (velocity_q == 0) break;
    }
    assert_int_equal(velocity_q, 0);
}

/* Position-mode correction + clamp_accel must decelerate to rest without
 * indefinite oscillation.
 *
 * Parameters: 10 steps/period jog, 500 000 steps/s² accel limit (real machine).
 * max_accel_q = 500 000 × (1e-3)² × 65536 = 32 768.
 *
 * Bug: correction for 1-step error = 32 768 Q16.16 = exactly max_accel_q.
 * For any overshoot > 1 step the correction saturates clamp_accel every period,
 * producing a limit cycle: motor never stops, direction reverses repeatedly. */
static void test_clamp_accel_posmode_decel_does_not_oscillate(void **state) {
    (void)state;
    /* 10 steps/period; 0.5 step/period/period accel limit */
    int sign_changes = posmode_decel_sign_changes(10 * 65536, 32768, 500);

    /* Motor must decelerate to rest: at most one direction change
     * (forward → stopped is not a reversal; stopped → backward is). */
    assert_true(sign_changes <= 1);
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
        cmocka_unit_test_setup(test_compute_velocity_cmd_velmode_passthrough,        test_setup),
        cmocka_unit_test_setup(test_compute_velocity_cmd_posmode_at_target,          test_setup),
        cmocka_unit_test_setup(test_compute_velocity_cmd_posmode_dead_zone,          test_setup),
        cmocka_unit_test_setup(test_compute_velocity_cmd_posmode_forward_correction, test_setup),
        cmocka_unit_test_setup(test_compute_velocity_cmd_posmode_reverse_correction, test_setup),
        cmocka_unit_test_setup(test_compute_velocity_cmd_disabled_returns_zero,      test_setup),
        cmocka_unit_test_setup(test_compute_velocity_cmd_underrun_returns_zero,      test_setup),
        cmocka_unit_test_setup(test_do_steps_zero_period,               test_setup),
        cmocka_unit_test_setup(test_do_steps_disabled,                  test_setup),
        cmocka_unit_test_setup(test_do_steps_disabled_drains_rx_fifo,  test_setup),
        cmocka_unit_test_setup(test_do_steps_disabling_decelerates,    test_setup),
        cmocka_unit_test_setup(test_do_steps_disabling_stops_when_zero,  test_setup),
        cmocka_unit_test_setup(test_do_steps_network_loss_decelerates,   test_setup),
        cmocka_unit_test_setup(test_do_steps_no_update,                  test_setup),
        cmocka_unit_test_setup(test_do_steps_normal_step,               test_setup),
        cmocka_unit_test_setup(test_do_steps_accel_clamped,                      test_setup),
        cmocka_unit_test_setup(test_do_steps_no_motion,                          test_setup),
        cmocka_unit_test_setup(test_do_steps_underrun_stops_pio,                      test_setup),
        cmocka_unit_test_setup(test_do_steps_underrun_while_enabled_decelerates,      test_setup),
        cmocka_unit_test_setup(test_do_steps_position_mode_drives_forward,       test_setup),
        cmocka_unit_test_setup(test_do_steps_position_mode_reverses,             test_setup),
        cmocka_unit_test_setup(test_do_steps_position_mode_at_target,            test_setup),
        cmocka_unit_test_setup(test_do_steps_position_mode_no_jitter_at_rest,    test_setup),
        cmocka_unit_test_setup(test_do_steps_velmode_0_75,      test_setup),
        cmocka_unit_test_setup(test_do_steps_velmode_0_25,      test_setup),
        cmocka_unit_test_setup(test_do_steps_velmode_int_1,     test_setup),
        cmocka_unit_test_setup(test_do_steps_velmode_int_10,    test_setup),
        cmocka_unit_test_setup(test_do_steps_velmode_frac_1_5,  test_setup),
        cmocka_unit_test_setup(test_do_steps_velmode_frac_10_5, test_setup),
        cmocka_unit_test_setup(test_do_steps_velmode_reverse,   test_setup),
        cmocka_unit_test_setup(test_clamp_accel_fixed_zero_target_never_overshoots, test_setup),
        cmocka_unit_test_setup(test_clamp_accel_posmode_decel_does_not_oscillate,   test_setup),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
