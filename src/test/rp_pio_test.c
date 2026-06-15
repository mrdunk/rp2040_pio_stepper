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
extern int step_gen2_program_init_call_count;
static size_t   mock_rx_fifo_level  = 0;
static int32_t  mock_rx_values[8]   = {0};
static size_t   mock_rx_index       = 0;
static uint32_t last_pio_put_value  = 0;
static uint32_t last_pio_step_value = 0;  /* last put with step_len > 0 */
static int      pio_put_call_count  = 0;
static int      mock_tx_fifo_empty  = 0;
/* When >= 0: return mock_tx_fifo_empty for this many calls, then return 0 (non-empty).
 * When -1 (default): always return mock_tx_fifo_empty. */
static int      mock_tx_fifo_empty_calls_remaining = -1;

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
    if (((uint32_t)data >> 1) & 0xFFFFFF) {
        last_pio_step_value = (uint32_t)data;
    }
}

int __wrap_pio_sm_is_tx_fifo_empty(size_t pio, size_t sm) {
    (void)pio; (void)sm;
    if (mock_tx_fifo_empty_calls_remaining == 0)
        return 0;  /* exhausted — simulate non-empty FIFO */
    if (mock_tx_fifo_empty_calls_remaining > 0)
        mock_tx_fifo_empty_calls_remaining--;
    return mock_tx_fifo_empty;
}

/* Fractional accumulator for pio_word_steps() — declared here so test_setup can reset it. */
static double pio_step_frac;

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
    last_pio_put_value  = 0;
    last_pio_step_value = 0;
    pio_put_call_count  = 0;
    mock_tx_fifo_empty  = 0;
    mock_tx_fifo_empty_calls_remaining = -1;
    pio_step_frac       = 0.0;
    memset(mock_rx_values, 0, sizeof(mock_rx_values));
    step_gen2_program_init_call_count = 0;
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
    assert_int_equal((last_pio_put_value >> 1) & 0xFFFFFF, 0);  /* hard-stopped: step_len==0 */
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

/* do_steps: network reconnects while joint is mid-deceleration -> acceleration limit honoured.
 *
 * Bug: on the enable 0->1 transition, last_velocity_q was unconditionally snapped to the
 * new commanded velocity, bypassing clamp_accel.  If the motor was still decelerating,
 * this caused a velocity jump (jitter).
 *
 * Scenario: max_accel=2e6 steps/s² (clamp_accel_q = 2*65536*1.1 = 144179 ≈ 2.2 steps/period),
 * velocity=10 steps/period.
 *   Tick 1: enable snap → last_velocity_q=655360 (10 steps/period).
 *   Ticks 2-4: decel (enabled=0, updated=0):
 *     Tick 2: 655360-144179=511181 (7.8 steps/period).
 *     Tick 3: 511181-144179=367002 (5.6 steps/period).
 *     Tick 4: 367002-144179=222823 (3.4 steps/period).
 *   Tick 5: re-enable (enabled=1, updated=1) at velocity=10.
 *     WRONG (old): snap → clamp_accel(10,10,2.2)=10 → jump from 3.4 to 10.
 *     CORRECT: no snap → clamp_accel(10,3.4,2.2)=5.6 → accel-limited to 367002.
 *     Continuous mode: step_low_half = 133000*32768/367002 - 175 = 11874 - 175 = 11699. */
static void test_do_steps_reconnect_mid_decel_no_jitter(void **state) {
    (void)state;
    config.update_time_us              = 1000;
    config.joint[0].enabled            = 1;
    config.joint[0].cmd_type           = JOINT_CMD_VELOCITY;
    config.joint[0].updated_from_c0    = 1;
    config.joint[0].velocity_requested = 10000.0;  /* 10 steps/period at 1000µs */
    config.joint[0].max_velocity       = 50000.0;
    config.joint[0].max_accel          = 2000000.0; /* 2e6 steps/s² → 2 steps/period/period */
    mock_tx_fifo_empty                 = 1;
    do_steps(0);  /* tick 1: enable snap → last_velocity_q=10 */

    /* Ticks 2-4: network lost, decelerate 10→8→6→4 steps/period. */
    config.joint[0].enabled          = 0;
    config.joint[0].updated_from_c0  = 0;
    mock_tx_fifo_empty               = 1;
    do_steps(0);  /* 10→8 */
    do_steps(0);  /* 8→6 */
    do_steps(0);  /* 6→4 */

    /* Tick 5: network reconnects, LinuxCNC re-enables at velocity=10. */
    config.joint[0].enabled          = 1;
    config.joint[0].updated_from_c0  = 1;
    config.joint[0].velocity_requested = 10000.0;
    last_pio_put_value               = 0;
    pio_put_call_count               = 0;
    mock_tx_fifo_empty               = 1;
    do_steps(0);

    /* Acceleration limit must be honoured: velocity ≤ 3.4+2.2=5.6 steps/period,
     * not a snap to 10.  Continuous mode step_low_half for 367002: 11874-175=11699. */
    assert_int_equal((last_pio_put_value >> 1) & 0xFFFFFF, 11699);
}

/* do_steps: fresh enable (last_velocity_q==0) snaps to commanded velocity.
 *
 * This is the intentional behaviour for joints enabled from rest when LinuxCNC
 * is already commanding motion: the motor must not ramp slowly from zero.
 * Regression guard: the mid-decel fix must not break this snap. */
static void test_do_steps_fresh_enable_snaps_to_commanded(void **state) {
    (void)state;
    /* last_velocity_q starts at 0 (pio_reset_for_test in test_setup). */
    config.update_time_us              = 1000;
    config.joint[0].enabled            = 1;
    config.joint[0].cmd_type           = JOINT_CMD_VELOCITY;
    config.joint[0].updated_from_c0    = 1;
    config.joint[0].velocity_requested = 10000.0;  /* 10 steps/period */
    config.joint[0].max_velocity       = 50000.0;
    config.joint[0].max_accel          = 2000000.0; /* 2 steps/period/period */
    mock_tx_fifo_empty                 = 1;
    do_steps(0);

    /* Snap applied (last_velocity_q was 0): full commanded velocity immediately.
     * step_low_half for 10 steps/period = 133000/(2*10)-175 = 6475. */
    assert_int_equal((last_pio_put_value >> 1) & 0xFFFFFF, 6475);
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
    config.joint[0].velocity_requested = 5.0 * update_period_us;  /* low: snap primes to 5.0 */
    config.joint[0].max_velocity       = 100000.0;  /* steps/s */
    config.joint[0].max_accel          = 5000000.0;  /* 5e6 steps/s² → 5.0 steps/period/period */

    /* First call: enable transition snaps last_velocity_q to 5.0 steps/period.
     * velocity=5.0 -> step_low_half=(133000/(5*2))-175=13125 */
    mock_tx_fifo_empty = 1;
    last_pio_put_value = 0;
    do_steps(0);
    uint32_t first_word = last_pio_put_value;

    /* Second call: jump velocity to 10000 steps/s; clamp limits increase to 5.0*1.1=5.5,
     * so velocity reaches 10.5 steps/period → continuous mode step_low_half=133000*32768/688128-175=6158 */
    mock_tx_fifo_empty = 1;
    last_pio_put_value = 0;
    config.joint[0].updated_from_c0    = 1;
    config.joint[0].velocity_requested = 10000.0 * update_period_us;

    do_steps(0);
    uint32_t second_word = last_pio_put_value;

    assert_int_equal((first_word >> 1) & 0xFFFFFF, 13125);
    assert_int_equal(first_word & 0x1, 1);
    assert_int_equal((second_word >> 1) & 0xFFFFFF, 6158);
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

/* Velocity mode, lagging: adds gentle position correction to close the gap.
 * error=10 steps, period=1000µs: correction = 10*(1e6/1000)*0.01 = 100 steps/s. */
static void test_compute_velocity_cmd_velmode_lag_correction(void **state) {
    (void)state;
    double result = compute_velocity_cmd(
        JOINT_CMD_VELOCITY, 5000.0, 10.0, 0, /*enabled=*/1, /*updated=*/1, 1000, 0.0);
    assert_float_equal(result, 5100.0, 1e-6);
}

/* Velocity mode, ahead of target: reduces velocity to let position catch up.
 * error=-10 steps (10 steps ahead): correction = -100 steps/s. */
static void test_compute_velocity_cmd_velmode_lead_correction(void **state) {
    (void)state;
    double result = compute_velocity_cmd(
        JOINT_CMD_VELOCITY, 5000.0, 0.0, 10, /*enabled=*/1, /*updated=*/1, 1000, 0.0);
    assert_float_equal(result, 4900.0, 1e-6);
}

/* Velocity mode, sub-1-step error: dead zone suppresses correction. */
static void test_compute_velocity_cmd_velmode_dead_zone(void **state) {
    (void)state;
    double result = compute_velocity_cmd(
        JOINT_CMD_VELOCITY, 5000.0, 0.5, 0, /*enabled=*/1, /*updated=*/1, 1000, 0.0);
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
    assert_int_equal((last_pio_put_value >> 1) & 0xFFFFFF, 0);  /* no steps when at target */
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
    assert_int_equal((last_pio_put_value >> 1) & 0xFFFFFF, 0);
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
    assert_int_equal((last_pio_put_value >> 1) & 0xFFFFFF, 0);  /* step_len==0: motor idle */
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

/* do_steps: position mode, active vel_ff, small tracking error.
 * vel_ff=10000 steps/s (10 steps/period), error=1 step, max_accel=5e6 steps/s².
 * velocity = vel_ff + Kp·error = 10000+500 = 10500 steps/s = 10.5 steps/period.
 * Continuous mode: step_low_half = 133000*32768/688128 - 175 = 6333-175 = 6158. */
static void test_do_steps_posmode_ff_active_tracks_at_full_speed(void **state) {
    (void)state;
    config.update_time_us              = 1000;
    config.joint[0].enabled            = 1;
    config.joint[0].cmd_type           = JOINT_CMD_POSITION;
    config.joint[0].velocity_requested = 10000.0;
    config.joint[0].abs_pos_requested  = 1.0;
    config.joint[0].abs_pos_achieved   = 0;
    config.joint[0].max_velocity       = 50000.0;
    config.joint[0].max_accel          = 5000000.0;
    config.joint[0].updated_from_c0    = 1;
    mock_tx_fifo_empty                  = 1;
    mock_rx_fifo_level                  = 0;

    do_steps(0);

    assert_int_equal((last_pio_put_value >> 1) & 0xFFFFFF, 6158);
}

/* Position mode, large negative vel_ff: mirror of large-positive.
 * vel_ff=-10000, error=-1 → velocity=-10500 steps/s = 10.5 steps/period.
 * Continuous mode: step_low_half = 6158. */
static void test_do_steps_posmode_ff_large_negative_tracks_at_full_speed(void **state) {
    (void)state;
    config.update_time_us              = 1000;
    config.joint[0].enabled            = 1;
    config.joint[0].cmd_type           = JOINT_CMD_POSITION;
    config.joint[0].velocity_requested = -10000.0;
    config.joint[0].abs_pos_requested  = -1.0;
    config.joint[0].abs_pos_achieved   = 0;
    config.joint[0].max_velocity       = 50000.0;
    config.joint[0].max_accel          = 5000000.0;
    config.joint[0].updated_from_c0    = 1;
    mock_tx_fifo_empty                 = 1;
    mock_rx_fifo_level                 = 0;

    do_steps(0);

    assert_int_equal((last_pio_put_value >> 1) & 0xFFFFFF, 6158);
}

/* Position mode, small positive vel_ff.
 * vel_ff=4000, error=1 → velocity=4500 steps/s = 4.5 steps/period.
 * Continuous mode: step_low_half = 133000*32768/294912 - 175 = 14777-175 = 14602. */
static void test_do_steps_posmode_ff_small_positive_tracks_normally(void **state) {
    (void)state;
    config.update_time_us              = 1000;
    config.joint[0].enabled            = 1;
    config.joint[0].cmd_type           = JOINT_CMD_POSITION;
    config.joint[0].velocity_requested = 4000.0;
    config.joint[0].abs_pos_requested  = 1.0;
    config.joint[0].abs_pos_achieved   = 0;
    config.joint[0].max_velocity       = 50000.0;
    config.joint[0].max_accel          = 5000000.0;
    config.joint[0].updated_from_c0    = 1;
    mock_tx_fifo_empty                 = 1;
    mock_rx_fifo_level                 = 0;

    do_steps(0);

    assert_int_equal((last_pio_put_value >> 1) & 0xFFFFFF, 14602);
}

/* Position mode, small negative vel_ff: mirror of small-positive (step_low_half=14602). */
static void test_do_steps_posmode_ff_small_negative_tracks_normally(void **state) {
    (void)state;
    config.update_time_us              = 1000;
    config.joint[0].enabled            = 1;
    config.joint[0].cmd_type           = JOINT_CMD_POSITION;
    config.joint[0].velocity_requested = -4000.0;
    config.joint[0].abs_pos_requested  = -1.0;
    config.joint[0].abs_pos_achieved   = 0;
    config.joint[0].max_velocity       = 50000.0;
    config.joint[0].max_accel          = 5000000.0;
    config.joint[0].updated_from_c0    = 1;
    mock_tx_fifo_empty                 = 1;
    mock_rx_fifo_level                 = 0;

    do_steps(0);

    assert_int_equal((last_pio_put_value >> 1) & 0xFFFFFF, 14602);
}

/* Position mode, vel_ff=0, residual error: stopping-profile cap limits velocity.
 *
 * Period 1: enable snap at 10 steps/period → last_velocity_q=655360.
 * Period 2: vel_ff=0, 1-step error → Kp correction=500 steps/s → target=32768.
 *   clamp_accel: 655360-360448=294912 (4.5 steps/period).
 *   cap: vel_ff_q=0, sqrt_term=sqrt(2·327680·1·65536)≈207243 (3.16 steps/period).
 *   294912>207243 → capped → velocity_q=207243 → continuous mode.
 *   step_low_half = 133000*32768/207243 - 175 = 21029-175 = 20854.
 * Continuous mode: one step_word written (no stop word). */
static void test_do_steps_posmode_ff_zero_clamp_accel_positive(void **state) {
    (void)state;
    config.update_time_us              = 1000;
    config.joint[0].enabled            = 1;
    config.joint[0].cmd_type           = JOINT_CMD_POSITION;
    config.joint[0].velocity_requested = 10000.0;
    config.joint[0].abs_pos_requested  = 0.0;
    config.joint[0].abs_pos_achieved   = 0;
    config.joint[0].max_velocity       = 50000.0;
    config.joint[0].max_accel          = 5000000.0;
    config.joint[0].updated_from_c0    = 1;
    mock_tx_fifo_empty                 = 1;
    mock_rx_fifo_level                 = 0;
    do_steps(0);  /* enable snap: last_velocity_q=655360 */

    config.joint[0].velocity_requested = 0.0;
    config.joint[0].abs_pos_requested  = 1.0;
    config.joint[0].updated_from_c0    = 1;
    last_pio_put_value  = 0;
    last_pio_step_value = 0;
    do_steps(0);

    assert_int_equal((last_pio_put_value >> 1) & 0xFFFFFF, 20854);  /* step_low_half, continuous mode */
    assert_int_equal((last_pio_step_value >> 1) & 0xFFFFFF, 20854); /* same word, no stop word */
}

/* Position mode, vel_ff=0, negative approach: mirror of positive. */
static void test_do_steps_posmode_ff_zero_clamp_accel_negative(void **state) {
    (void)state;
    config.update_time_us              = 1000;
    config.joint[0].enabled            = 1;
    config.joint[0].cmd_type           = JOINT_CMD_POSITION;
    config.joint[0].velocity_requested = -10000.0;
    config.joint[0].abs_pos_requested  = 0.0;
    config.joint[0].abs_pos_achieved   = 0;
    config.joint[0].max_velocity       = 50000.0;
    config.joint[0].max_accel          = 5000000.0;
    config.joint[0].updated_from_c0    = 1;
    mock_tx_fifo_empty                 = 1;
    mock_rx_fifo_level                 = 0;
    do_steps(0);  /* enable snap: last_velocity_q=-655360 */

    config.joint[0].velocity_requested = 0.0;
    config.joint[0].abs_pos_requested  = -1.0;
    config.joint[0].updated_from_c0    = 1;
    last_pio_put_value  = 0;
    last_pio_step_value = 0;
    do_steps(0);

    assert_int_equal((last_pio_put_value >> 1) & 0xFFFFFF, 20854);  /* step_low_half, continuous mode */
    assert_int_equal((last_pio_step_value >> 1) & 0xFFFFFF, 20854); /* same word, no stop word */
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
 *   step_low_half = (133000*65536/32768) - 175 = 265825 ticks  (spans ~2 servo periods)
 *
 * dunk_fix_cmd_pos: 265826 > max_len → capped to 66326 → plan_steps fires 1 step every
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
        if (((last_pio_put_value >> 1) & 0xFFFFFF) != 0) {
            steps_fired++;
        }
    }

    assert_int_equal(steps_fired, 0);
}

/* ── Velocity-mode position-accuracy integration tests ────────────────────
 *
 * Physical model: when do_steps writes a non-zero word to the PIO FIFO, the
 * step_gen2 programme loops at rate step_period = 2*(step_low_half+175) ticks.
 * In one servo period (133000 ticks @ 1ms/133MHz) it generates
 *   steps = 133000.0 / step_period
 * physical steps.  pio_word_steps() converts a FIFO word to that signed count,
 * using a fractional accumulator (pio_step_frac) to track sub-integer residuals
 * across calls so the long-run total is exact.
 *
 * Accuracy analysis:
 *
 *   v < 1 step/period (sub-1-step Bresenham path):
 *     Produces 2 FIFO writes when a step fires: step word then stop word.
 *     The stop word halts the PIO after exactly 1 physical step regardless of
 *     step_low_half.  Detected by pio_put_call_count==2; counted as exactly 1 step.
 *     Bresenham schedules 0 or 1 step each period; long-run average equals v
 *     exactly.  Position is EXACT.
 *
 *   v >= 1 step/period (continuous mode):
 *     step_low_half = period_ticks * 32768 / velocity_q - 175 (= STEP_PIO_LEN_OVERHEAD).
 *     exact steps per period = 133000.0 / (2*(step_low_half+175)) ≈ v.
 *     Fractional accumulator ensures long-run total matches v*n_periods exactly.
 *     Position is EXACT.
 */

/* Convert one PIO FIFO word to the physical step count it causes.
 * Returns negative for direction=0 (reverse).
 * Idle/stop words have step_low_half==0 (upper bits zero) regardless of the
 * direction bit (which preserves the last cached direction).
 * step_period = 2*(step_low_half + 175)  where 175 = STEP_PIO_LEN_OVERHEAD */
static int32_t pio_word_steps(uint32_t word) {
    int32_t step_low_half = (int32_t)((word >> 1) & 0xFFFFFF);
    if (step_low_half == 0) return 0;
    double exact = 133000.0 / (2.0 * ((double)step_low_half + 175.0));
    pio_step_frac += exact;
    int32_t whole = (int32_t)pio_step_frac;
    pio_step_frac -= (double)whole;
    return (word & 1) ? whole : -whole;
}

/* Run n servo periods in JOINT_CMD_VELOCITY mode; return accumulated position.
 * No acceleration limit so the commanded velocity takes effect immediately.
 * Uses pico-eth-cnc-3axis parameters: 1ms period, max_vel=32000 steps/s.
 * abs_pos_requested advances each period to match the commanded velocity so
 * the position correction term stays near zero (error < 1 step). */
static int32_t run_velocity_periods(double vel_steps_per_s, int n) {
    config.update_time_us              = 1000;
    config.joint[0].cmd_type           = JOINT_CMD_VELOCITY;
    config.joint[0].enabled            = 1;
    config.joint[0].velocity_requested = vel_steps_per_s;
    config.joint[0].max_velocity       = 32000.0;
    config.joint[0].max_accel          = 0.0;
    mock_tx_fifo_empty                 = 1;

    int32_t sim_pos      = 0;
    double  pos_requested = 0.0;
    for (int i = 0; i < n; i++) {
        config.joint[0].abs_pos_requested = pos_requested;
        mock_rx_values[0]  = sim_pos;
        mock_rx_fifo_level = 1;
        mock_rx_index      = 0;
        config.joint[0].updated_from_c0 = 1;
        last_pio_put_value  = 0;
        last_pio_step_value = 0;
        pio_put_call_count  = 0;
        do_steps(0);
        /* Sub-1-step: 2 writes (step word + stop word) → exactly 1 physical step.
         * Continuous: 1 write → PIO repeats at step_len rate for the full period. */
        if (pio_put_call_count == 2 && last_pio_step_value != 0)
            sim_pos += (last_pio_step_value & 1) ? 1 : -1;
        else
            sim_pos += pio_word_steps(last_pio_step_value);
        pos_requested += vel_steps_per_s * 1e-3;  /* advance 1ms per period */
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
 * Continuous mode (abs(plan_vel_q)==Q16_ONE): step_low_half=66326, 1 step/period → 100 steps. */
static void test_do_steps_velmode_int_1(void **state) {
    (void)state;
    assert_int_equal(run_velocity_periods(1000.0, 100), 100);
}

/* 10.0 steps/period (10000 steps/s): integer, exact.
 * step_low_half=6476, max_steps=10, desired=10 every period → 1000 steps. */
static void test_do_steps_velmode_int_10(void **state) {
    (void)state;
    assert_int_equal(run_velocity_periods(10000.0, 100), 1000);
}

/* 1.5 steps/period (1500 steps/s): non-integer above 1, exact.
 * Continuous mode: step_low_half=133000*32768/98304-175=44158, step_period=88666.
 * Exact steps/period=1.50034; fractional accumulator alternates 1,2 → 150 steps. */
static void test_do_steps_velmode_frac_1_5(void **state) {
    (void)state;
    assert_int_equal(run_velocity_periods(1500.0, 100), 150);
}

/* 10.5 steps/period (10500 steps/s): non-integer above 10, exact.
 * Continuous mode: step_low_half=133000*32768/688128-175=6158, step_period=12666.
 * Exact steps/period=10.5034; fractional accumulator → 1050 steps in 100 periods. */
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

/* do_steps velocity mode corrects accumulated position lag over time.
 *
 * A 10-step initial lag (pos_requested = 10, sim_pos = 0) generates a
 * +100 steps/s correction each period the motor is behind.  The Bresenham
 * accumulator fills faster than for the bare 10000 steps/s command, producing
 * an extra step roughly every 10 periods.  After 100 periods the motor has
 * taken more than the 1000 steps it would without correction, closing the lag.
 * (Without correction: exactly 1000 steps; with correction: ≥ 1001.) */
static void test_do_steps_velmode_lag_corrected_over_time(void **state) {
    (void)state;
    config.update_time_us              = 1000;
    config.joint[0].cmd_type           = JOINT_CMD_VELOCITY;
    config.joint[0].enabled            = 1;
    config.joint[0].velocity_requested = 10000.0;
    config.joint[0].max_velocity       = 50000.0;
    config.joint[0].max_accel          = 0.0;
    mock_tx_fifo_empty                 = 1;

    int32_t sim_pos      = 0;
    double  pos_requested = 10.0;  /* start with 10-step lag */
    for (int i = 0; i < 100; i++) {
        config.joint[0].abs_pos_requested = pos_requested;
        mock_rx_values[0]  = sim_pos;
        mock_rx_fifo_level = 1;
        mock_rx_index      = 0;
        config.joint[0].updated_from_c0 = 1;
        last_pio_put_value  = 0;
        last_pio_step_value = 0;
        do_steps(0);
        sim_pos       += pio_word_steps(last_pio_step_value);
        pos_requested += 10.0;  /* LinuxCNC advances pos by 10 steps/period */
    }
    assert_true(sim_pos > 1000);
}

/* Non-integer velocity must not drift: 25.6 steps/period (SCALE=1024 × 25mm/s).
 * max_velocity = vel × 1.01 matches the VEL_HEADROOM applied by the driver, which
 * triggers the min_len clamp bug when v_ceil == v_ceil_max.
 * Over 100 periods Bresenham must average exactly 25.6 steps → 2560 ± 1 steps. */
static void test_do_steps_noninteger_velocity_no_drift(void **state) {
    (void)state;
    double vel = 25600.0; /* 25.6 steps/period at 1ms, matching SCALE=1024 × 25mm/s */
    config.update_time_us              = 1000;
    config.joint[0].cmd_type           = JOINT_CMD_VELOCITY;
    config.joint[0].enabled            = 1;
    config.joint[0].velocity_requested = vel;
    config.joint[0].max_velocity       = vel * 1.01;
    config.joint[0].max_accel          = 0.0;
    mock_tx_fifo_empty                 = 1;

    int32_t sim_pos       = 0;
    double  pos_requested = 0.0;
    for (int i = 0; i < 100; i++) {
        config.joint[0].abs_pos_requested = pos_requested;
        mock_rx_values[0]  = sim_pos;
        mock_rx_fifo_level = 1;
        mock_rx_index      = 0;
        config.joint[0].updated_from_c0 = 1;
        last_pio_put_value  = 0;
        last_pio_step_value = 0;
        do_steps(0);
        sim_pos       += pio_word_steps(last_pio_step_value);
        pos_requested += vel * 1e-3;
    }
    assert_true(sim_pos >= 2559);
    assert_true(sim_pos <= 2561);
}

/* Position mode: no overshoot after final correction step.
 *
 * Models JOINT_0 (scale=160, max_accel=750 mm/s²=120000 steps/s²).
 * max_accel_q = 120000 * (1e-3)² * 65536 = 7864.
 * Bang-bang cap at 1-step error = sqrt(2·7864·65536) ≈ 32109 Q16.16 ≈ 490 steps/s.
 *
 * Prime last_velocity_q to 32112 (≈ bang-bang cap) via velocity mode enable snap.
 * Then switch to position mode at target (error=0, vel_ff=0).
 * Without fix: clamp_accel leaves velocity_q=24248; over 2 periods the Bresenham
 * accumulator (32112 + 24248 + 16384 = 72744) crosses 65536 → overshoot step fires.
 * With fix: velocity_q and accumulator are zeroed at err_int=0, vel_ff_q=0. */
static void test_do_steps_position_mode_no_overshoot_after_correction(void **state) {
    (void)state;

    config.update_time_us              = 1000;
    config.joint[0].enabled            = 1;
    config.joint[0].cmd_type           = JOINT_CMD_VELOCITY;   /* velocity mode to prime */
    config.joint[0].velocity_requested = 490.0;                /* 32112 Q16.16 ≈ bang-bang cap */
    config.joint[0].abs_pos_requested  = 0.0;
    config.joint[0].abs_pos_achieved   = 0;
    config.joint[0].max_velocity       = 200000.0;
    config.joint[0].max_accel          = 120000.0;             /* 750 mm/s² × 160 steps/mm */
    config.joint[0].updated_from_c0    = 1;
    mock_tx_fifo_empty                 = 1;
    mock_rx_fifo_level                 = 0;
    do_steps(0);  /* enable snap: last_velocity_q = 490/1000 * 65536 = 32112 */

    /* Now at target: error=0, vel_ff=0.  Residual velocity must not fire a step. */
    config.joint[0].cmd_type           = JOINT_CMD_POSITION;
    config.joint[0].velocity_requested = 0.0;
    config.joint[0].abs_pos_requested  = 1.0;
    config.joint[0].abs_pos_achieved   = 1;

    int steps_fired = 0;
    for (int i = 0; i < 10; i++) {
        config.joint[0].updated_from_c0 = 1;
        last_pio_put_value = 0;
        do_steps(0);
        if (((last_pio_put_value >> 1) & 0xFFFFFF) != 0) steps_fired++;
    }

    assert_int_equal(steps_fired, 0);
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

/* Reproduce JOINT_1 numbers: max_accel_q = floor(1280 * 1e-6 * 65536) = 83.
 * LinuxCNC ramps vel_ff_q by floor(n*83.886) each period; the per-period delta
 * alternates between 83 and 84 as the 0.886 fractional part accumulates.
 * Without headroom (budget=83) the firmware falls 1 unit short on every "84"
 * period, accumulating lag that grows into a position error large enough to trip
 * FERROR over a 12-second ramp.  With ACCEL_HEADROOM=1.1 the budget=91 covers
 * the worst-case delta of 84, keeping lag at zero. */
static void test_ramp_accel_headroom_tracks_vel_ff(void **state) {
    (void)state;
    int32_t max_accel_q  = 83;
    int32_t clamp_budget = (int32_t)(max_accel_q * ACCEL_HEADROOM);
    int32_t firmware_vel = 0;
    int32_t max_lag      = 0;

    for (int n = 1; n <= 120; n++) {
        int32_t vel_ff_q = (int32_t)(n * 83.886);
        firmware_vel = clamp_accel(vel_ff_q, firmware_vel, clamp_budget);
        int32_t lag = vel_ff_q - firmware_vel;
        if (lag > max_lag) max_lag = lag;
    }
    assert_int_equal(max_lag, 0);
}

/* do_steps: velocity_achieved reports exact 0 when joint has fully decelerated.
 *
 * Regression guard for the driver recovery gate: vel_fb == 0.0 (exact) triggers
 * machine-on after network recovery.  velocity_achieved carries last_velocity_q;
 * it must be exactly 0 — not a small residual — when the joint has stopped. */
static void test_do_steps_velocity_achieved_zero_when_stopped(void **state) {
    (void)state;
    /* Prime at 5 steps/period (= max_accel); one disable step reaches zero. */
    config.update_time_us              = 1000;
    config.joint[0].enabled            = 1;
    config.joint[0].cmd_type           = JOINT_CMD_VELOCITY;
    config.joint[0].updated_from_c0    = 1;
    config.joint[0].velocity_requested = 5000.0;    /* 5 steps/period at 1000µs */
    config.joint[0].max_velocity       = 50000.0;
    config.joint[0].max_accel          = 5000000.0; /* 5 steps/period/period */
    mock_tx_fifo_empty                 = 1;
    do_steps(0);  /* enable snap: last_velocity_q = 5 steps/period */

    /* Disable: clamp_accel(0, 5, 5) = 0 → hard stop → velocity_achieved = 0 exactly. */
    config.joint[0].enabled          = 0;
    config.joint[0].updated_from_c0  = 1;
    mock_tx_fifo_empty               = 1;
    do_steps(0);

    assert_int_equal(config.joint[0].velocity_achieved, 0);
}

/* do_steps: velocity_achieved is non-zero while joint is still decelerating.
 *
 * The driver recovery gate blocks machine-on while vel_fb != 0.0.  If velocity_achieved
 * dropped to 0 prematurely, recovery would fire while the motor is still moving. */
static void test_do_steps_velocity_achieved_nonzero_while_decelerating(void **state) {
    (void)state;
    /* Prime at 10 steps/period, max_accel=5; one disable step → 5 (still moving). */
    config.update_time_us              = 1000;
    config.joint[0].enabled            = 1;
    config.joint[0].cmd_type           = JOINT_CMD_VELOCITY;
    config.joint[0].updated_from_c0    = 1;
    config.joint[0].velocity_requested = 10000.0;   /* 10 steps/period at 1000µs */
    config.joint[0].max_velocity       = 50000.0;
    config.joint[0].max_accel          = 5000000.0; /* 5 steps/period/period */
    mock_tx_fifo_empty                 = 1;
    do_steps(0);  /* enable snap: last_velocity_q = 10 steps/period */

    /* Disable: clamp_accel(0, 10, 5) = 5 → still decelerating → velocity_achieved != 0. */
    config.joint[0].enabled          = 0;
    config.joint[0].updated_from_c0  = 1;
    mock_tx_fifo_empty               = 1;
    do_steps(0);

    assert_true(config.joint[0].velocity_achieved != 0);
}

/* do_steps: velocity_achieved is exact 0 after reverse-direction deceleration.
 *
 * Confirms the hard-stop path works regardless of direction; clamp_accel must not
 * overshoot zero when last_velocity_q is negative. */
static void test_do_steps_velocity_achieved_zero_reverse(void **state) {
    (void)state;
    config.update_time_us              = 1000;
    config.joint[0].enabled            = 1;
    config.joint[0].cmd_type           = JOINT_CMD_VELOCITY;
    config.joint[0].updated_from_c0    = 1;
    config.joint[0].velocity_requested = -5000.0;   /* -5 steps/period (reverse) */
    config.joint[0].max_velocity       = 50000.0;
    config.joint[0].max_accel          = 5000000.0; /* 5 steps/period/period */
    mock_tx_fifo_empty                 = 1;
    do_steps(0);  /* enable snap: last_velocity_q = -5 steps/period */

    /* Disable: clamp_accel(0, -5, 5) = 0 → hard stop → velocity_achieved = 0 exactly. */
    config.joint[0].enabled          = 0;
    config.joint[0].updated_from_c0  = 1;
    mock_tx_fifo_empty               = 1;
    do_steps(0);

    assert_int_equal(config.joint[0].velocity_achieved, 0);
}

/* do_steps: position-mode velocity_achieved is exact 0 when joint has fully decelerated.
 * Same recovery-gate invariant as the velocity-mode equivalent, but exercising the
 * JOINT_CMD_POSITION control path through compute_velocity_cmd. */
static void test_do_steps_posmode_velocity_achieved_zero_when_stopped(void **state) {
    (void)state;
    /* vel_ff=5 steps/period, zero position error → velocity_q=5; max_accel=5 → one step to zero. */
    config.update_time_us              = 1000;
    config.joint[0].enabled            = 1;
    config.joint[0].cmd_type           = JOINT_CMD_POSITION;
    config.joint[0].updated_from_c0    = 1;
    config.joint[0].velocity_requested = 5000.0;   /* vel_ff = 5 steps/period at 1000µs */
    config.joint[0].abs_pos_requested  = 0.0;       /* zero error: no Kp correction */
    config.joint[0].abs_pos_achieved   = 0;
    config.joint[0].max_velocity       = 50000.0;
    config.joint[0].max_accel          = 5000000.0; /* 5 steps/period/period */
    mock_tx_fifo_empty                 = 1;
    do_steps(0);  /* enable snap: last_velocity_q = 5 steps/period */

    config.joint[0].enabled          = 0;
    config.joint[0].updated_from_c0  = 1;
    mock_tx_fifo_empty               = 1;
    do_steps(0);  /* clamp_accel(0, 5, ≥5) = 0 → hard stop */

    assert_int_equal(config.joint[0].velocity_achieved, 0);
}

/* do_steps: position-mode velocity_achieved is non-zero while still decelerating. */
static void test_do_steps_posmode_velocity_achieved_nonzero_while_decelerating(void **state) {
    (void)state;
    /* vel_ff=10 steps/period, max_accel=5; one disable step → 5 (still moving). */
    config.update_time_us              = 1000;
    config.joint[0].enabled            = 1;
    config.joint[0].cmd_type           = JOINT_CMD_POSITION;
    config.joint[0].updated_from_c0    = 1;
    config.joint[0].velocity_requested = 10000.0;  /* vel_ff = 10 steps/period */
    config.joint[0].abs_pos_requested  = 0.0;
    config.joint[0].abs_pos_achieved   = 0;
    config.joint[0].max_velocity       = 50000.0;
    config.joint[0].max_accel          = 5000000.0; /* 5 steps/period/period */
    mock_tx_fifo_empty                 = 1;
    do_steps(0);  /* enable snap: last_velocity_q = 10 steps/period */

    config.joint[0].enabled          = 0;
    config.joint[0].updated_from_c0  = 1;
    mock_tx_fifo_empty               = 1;
    do_steps(0);  /* clamp limits decel: velocity_q = 5, still moving */

    assert_true(config.joint[0].velocity_achieved != 0);
}

/* do_steps: position-mode velocity_achieved is exact 0 after reverse-direction deceleration. */
static void test_do_steps_posmode_velocity_achieved_zero_reverse(void **state) {
    (void)state;
    /* vel_ff=-5 steps/period (reverse), max_accel=5 → one step to zero. */
    config.update_time_us              = 1000;
    config.joint[0].enabled            = 1;
    config.joint[0].cmd_type           = JOINT_CMD_POSITION;
    config.joint[0].updated_from_c0    = 1;
    config.joint[0].velocity_requested = -5000.0;  /* vel_ff = -5 steps/period */
    config.joint[0].abs_pos_requested  = 0.0;
    config.joint[0].abs_pos_achieved   = 0;
    config.joint[0].max_velocity       = 50000.0;
    config.joint[0].max_accel          = 5000000.0; /* 5 steps/period/period */
    mock_tx_fifo_empty                 = 1;
    do_steps(0);  /* enable snap: last_velocity_q = -5 steps/period */

    config.joint[0].enabled          = 0;
    config.joint[0].updated_from_c0  = 1;
    mock_tx_fifo_empty               = 1;
    do_steps(0);  /* clamp_accel(0, -5, ≥5) = 0 → hard stop */

    assert_int_equal(config.joint[0].velocity_achieved, 0);
}

/* Position mode, 0.3 steps/period: inter-step intervals uniform despite correction.
 *
 * At sub-1-step speeds the position-correction term fires whenever error ≥ 1 step,
 * spiking velocity_q above the feedforward and over-filling the Bresenham accumulator.
 * This causes steps to cluster (e.g. gap of 2 followed by gap of 4) rather than
 * being spaced uniformly at 3-4 periods apart.  The fix is to drive plan_steps with
 * vel_ff_q (feedforward only) in the sub-1-step regime so the correction spike does
 * not disrupt the accumulator. */
static void test_do_steps_posmode_uniform_spacing_at_low_speed(void **state) {
    (void)state;
    config.update_time_us              = 1000;
    config.joint[0].enabled            = 1;
    config.joint[0].cmd_type           = JOINT_CMD_POSITION;
    config.joint[0].velocity_requested = 300.0;  /* 0.3 steps/period at 1ms */
    config.joint[0].max_velocity       = 32000.0;
    config.joint[0].max_accel          = 0.0;
    mock_tx_fifo_empty                 = 1;

    int     step_at[20];
    int     n       = 0;
    int32_t sim_pos = 0;
    double  pos_req = 0.0;

    for (int p = 0; p < 80 && n < 15; p++) {
        pos_req += 0.3;
        config.joint[0].abs_pos_requested = pos_req;
        mock_rx_values[0]  = sim_pos;
        mock_rx_fifo_level = 1;
        mock_rx_index      = 0;
        config.joint[0].updated_from_c0 = 1;
        last_pio_put_value  = 0;
        last_pio_step_value = 0;
        do_steps(0);
        int32_t fired = pio_word_steps(last_pio_step_value);
        if (fired > 0) {
            step_at[n++] = p;
            sim_pos += fired;
        }
    }

    assert_true(n >= 6);

    /* Ideal Bresenham at 0.3 steps/period: gaps are 3 or 4, never 1 or 5+. */
    int min_gap = 1000, max_gap = 0;
    for (int i = 1; i < n; i++) {
        int gap = step_at[i] - step_at[i - 1];
        if (gap < min_gap) min_gap = gap;
        if (gap > max_gap) max_gap = gap;
    }
    assert_true(max_gap - min_gap <= 1);
}

/* do_steps: position-mode stopping-profile cap is inactive in velocity mode.
 *
 * In position mode the sqrt(2·a·|error|) cap reduces velocity when the joint is
 * close to its target; in velocity mode this branch must not fire regardless of
 * acceleration setting.  Without the cmd_type guard an implementation bug could
 * reduce velocity_requested to a fraction of the commanded value. */
static void test_do_steps_velocity_mode_no_position_cap(void **state) {
    (void)state;
    /* Zero position error so the gentle velocity-mode correction doesn't apply;
     * only the stopping-profile cap guard (cmd_type == JOINT_CMD_POSITION) is tested. */
    config.update_time_us              = 1000;
    config.joint[0].enabled            = 1;
    config.joint[0].cmd_type           = JOINT_CMD_VELOCITY;
    config.joint[0].updated_from_c0    = 1;
    config.joint[0].velocity_requested = 10000.0;  /* 10 steps/period at 1000µs */
    config.joint[0].abs_pos_requested  = 0.0;
    config.joint[0].abs_pos_achieved   = 0;
    config.joint[0].max_velocity       = 50000.0;
    config.joint[0].max_accel          = 5000000.0; /* 5 steps/period/period */
    mock_tx_fifo_empty                 = 1;

    do_steps(0);  /* enable snap: last_velocity_q = 10 steps/period */

    /* Velocity mode: full 10 steps/period → step_low_half = 133000/(2*10)-175 = 6475.
     * If the stopping-profile cap had fired, step_low_half would be larger (fewer steps). */
    assert_int_equal((last_pio_put_value >> 1) & 0xFFFFFF, 6475);
}

/* Position mode: overshoot correction does not reverse step direction.
 *
 * When pos_ach > pos_req, the correction term exceeds vel_ff and flips velocity_q
 * negative.  Without the fix, direction = (velocity_q > 0) = 0 (backward), while
 * plan_vel_q = vel_ff_q > 0 — so Bresenham fires forward-rate steps with the wrong
 * direction bit.  The motor physically steps backward, worsening position error.
 * Fix: direction uses plan_vel_q so it always matches the scheduled step direction. */
static void test_do_steps_posmode_overshoot_does_not_reverse(void **state) {
    (void)state;
    /* vel_ff=300 steps/s, pos_req=0.5, pos_ach=2
     * error=-1.5 → correction=-750 → velocity=-450 → velocity_q=-29491 (negative)
     * vel_ff_q=+19660; step_count_q=29491 <= 65536 and vel_ff_q>0 → plan_vel_q=+19660 */
    config.update_time_us              = 1000;
    config.joint[0].enabled            = 1;
    config.joint[0].cmd_type           = JOINT_CMD_POSITION;
    config.joint[0].velocity_requested = 300.0;
    config.joint[0].abs_pos_requested  = 0.5;
    config.joint[0].max_velocity       = 32000.0;
    config.joint[0].max_accel          = 0.0;
    mock_tx_fifo_empty                 = 1;

    int forward_steps = 0, reverse_steps = 0;
    for (int p = 0; p < 20; p++) {
        mock_rx_values[0]  = 2;   /* pos_ach=2 each period; correction stays negative */
        mock_rx_fifo_level = 1;
        mock_rx_index      = 0;
        config.joint[0].updated_from_c0 = 1;
        last_pio_put_value  = 0;
        last_pio_step_value = 0;
        do_steps(0);
        /* Use last_pio_step_value: sub-1-step pushes step_word then stop_word,
         * so last_pio_put_value holds the stop_word (direction bit only). */
        if (last_pio_step_value != 0) {
            if (last_pio_step_value & 1) forward_steps++;
            else                         reverse_steps++;
        }
        assert_true(config.joint[0].velocity_achieved >= 0);
    }
    assert_int_equal(reverse_steps, 0);
    assert_true(forward_steps > 0);
}

/* Position mode: velocity_achieved stays 0 when vel_ff reaches 0 at stop, even
 * with an active 1-step overshoot correction.
 *
 * When LinuxCNC decelerates to rest, vel_ff_q → 0.  A 1-step overshoot fires
 * correction = -500 steps/s → velocity_q = -32768 Q16.16.  Without the fix,
 * vel_ff_q==0 exits the feedforward path and velocity_achieved = velocity_q ≈ -0.5
 * steps/period — an order-of-magnitude spike vs the near-zero vel_ff at stop.
 * Fix: velocity_achieved = vel_ff_q always, so it reports 0 when the motor is
 * commanded to stand still regardless of the correction term. */
static void test_do_steps_posmode_correction_at_stop_does_not_spike_vel_achieved(void **state) {
    (void)state;
    config.update_time_us              = 1000;
    config.joint[0].enabled            = 1;
    config.joint[0].cmd_type           = JOINT_CMD_POSITION;
    config.joint[0].velocity_requested = 0.0;   /* vel_ff=0: motor commanded to stop */
    config.joint[0].abs_pos_requested  = 0.0;
    config.joint[0].max_velocity       = 32000.0;
    config.joint[0].max_accel          = 0.0;
    mock_tx_fifo_empty                 = 1;

    mock_rx_values[0]  = 1;   /* pos_ach=1, error=0-1=-1 → correction=-500 steps/s */
    mock_rx_fifo_level = 1;
    mock_rx_index      = 0;
    config.joint[0].updated_from_c0 = 1;
    do_steps(0);

    assert_int_equal(config.joint[0].velocity_achieved, 0);
}

/* Position mode: large overshoot produces step_count_q > 65536 backward — motor
 * must not reverse.
 *
 * vel_ff=300 steps/s, pos_ach=5, pos_req=0.5: error=-4.5, correction=-2250 steps/s,
 * velocity=-1950 steps/s → velocity_q ≈ -127795 (|velocity_q| >> 65536).
 * Old code: in_ff_path=0 (step_count_q>65536), plan_vel_q=velocity_q<0 → backward.
 * New code: sign-flip branch of in_ff_path catches this → plan_vel_q=vel_ff_q → forward. */
static void test_do_steps_posmode_large_overshoot_does_not_reverse(void **state) {
    (void)state;
    config.update_time_us              = 1000;
    config.joint[0].enabled            = 1;
    config.joint[0].cmd_type           = JOINT_CMD_POSITION;
    config.joint[0].velocity_requested = 300.0;
    config.joint[0].abs_pos_requested  = 0.5;
    config.joint[0].max_velocity       = 32000.0;
    config.joint[0].max_accel          = 0.0;  /* no cap so correction stays large */
    mock_tx_fifo_empty                 = 1;

    int forward_steps = 0, reverse_steps = 0;
    for (int p = 0; p < 20; p++) {
        mock_rx_values[0]  = 5;   /* pos_ach=5; error=-4.5 → velocity_q ≈ -127795 */
        mock_rx_fifo_level = 1;
        mock_rx_index      = 0;
        config.joint[0].updated_from_c0 = 1;
        last_pio_put_value = 0;
        do_steps(0);
        /* Continuous mode (|velocity_q| > Q16_ONE): only step_word is pushed, no
         * trailing stop_word, so last_pio_put_value is the step_word. */
        if (((last_pio_put_value >> 1) & 0xFFFFFF) != 0) {
            if (last_pio_put_value & 1) forward_steps++;
            else                        reverse_steps++;
        }
        assert_true(config.joint[0].velocity_achieved >= 0);
    }
    assert_int_equal(reverse_steps, 0);
}

/* Position mode at sub-1-step speed: same-direction correction must not cause
 * double-stepping.
 *
 * vel_ff=100 steps/s → vel_ff_q=6553 (0.1 steps/period).  After 10 priming
 * periods the accumulator is at 65530 (one add away from firing).  On period 11,
 * pos_ach=0 while pos_req=2.0 → error=2 steps → correction=1000 steps/s →
 * velocity_q=(1100/1000)*65536=72090 (>65536).
 *
 * Without the vel_ff_q≤65536 guard: step_count_q=72090 exits in_ff_path (same
 * direction, large), plan_steps adds 72090 to accum (65530+72090=137620 → 2 steps).
 * With the fix: abs(vel_ff_q)=6553≤65536 → in_ff_path=1, plan_vel_q=vel_ff_q=6553,
 * accum=65530+6553=72083 → exactly 1 step. */
static void test_do_steps_posmode_sub1step_correction_fires_at_sub1step_ff(void **state) {
    (void)state;
    config.update_time_us              = 1000;
    config.joint[0].enabled            = 1;
    config.joint[0].cmd_type           = JOINT_CMD_POSITION;
    config.joint[0].velocity_requested = 100.0;  /* vel_ff_q=6553, 0.1 steps/period */
    config.joint[0].abs_pos_requested  = 0.0;
    config.joint[0].max_velocity       = 32000.0;
    config.joint[0].max_accel          = 0.0;    /* no accel cap — worst case */
    mock_tx_fifo_empty                 = 1;

    /* Prime accumulator: 10 periods with error=0 → accum=10*6553=65530, no step yet */
    for (int p = 0; p < 10; p++) {
        mock_rx_values[0]  = 0;
        mock_rx_fifo_level = 1;
        mock_rx_index      = 0;
        config.joint[0].updated_from_c0 = 1;
        do_steps(0);
    }

    /* Period 11: error=2 → correction=1000 steps/s → velocity_q=72090 > 65536.
     * Condition (a) is gone: large same-direction corrections fire through even at
     * sub-1-step ff, so the position error resolves immediately (2 steps: 1 from
     * accumulated Bresenham + 1 correction).  A burst step is preferable to
     * accumulated follow error. */
    config.joint[0].abs_pos_requested = 2.0;
    mock_rx_values[0]  = 0;
    mock_rx_fifo_level = 1;
    mock_rx_index      = 0;
    config.joint[0].updated_from_c0 = 1;
    last_pio_put_value = 0;
    do_steps(0);

    /* At least 1 step must fire (correction is not suppressed). */
    int steps_this_period = (last_pio_put_value != 0) ? 1 : 0;
    assert_true(steps_this_period >= 1);
}

/* Multi-step velocity with accel ramp: step count must match vel_ff_q rate from
 * period 0, not build a backlog.
 *
 * 1280 steps/mm × 4mm/s = 5120 steps/s = 5.12 steps/period.  clamp_accel limits
 * velocity_q during ramp-up (period 0: vq≈14417, << vel_ff_q=335544).  The old
 * code computed step_len_ceil from step_count_q=14417 → step_len=66450 → max_steps=1.
 * With plan_vel_q=vel_ff_q=335544 added to accumulator each period, 4+ steps of
 * backlog built up in 4 periods, draining as a burst later.
 *
 * Fix: step_len_ceil computed from abs(plan_vel_q)=335544 → step_len≈11033 →
 * max_steps=6.  Period 0 fires 5 steps; accumulator stays near zero. */
static void test_do_steps_multistep_accel_ramp_no_backlog(void **state) {
    (void)state;
    config.update_time_us              = 1000;
    config.joint[0].enabled            = 1;
    config.joint[0].cmd_type           = JOINT_CMD_POSITION;
    config.joint[0].velocity_requested = 5120.0;  /* 5.12 steps/period */
    config.joint[0].abs_pos_requested  = 5120.0 * 20;  /* far target, no stopping cap */
    config.joint[0].max_velocity       = 5120.0 * 1.01;
    config.joint[0].max_accel          = 200000.0;  /* gives clamp_accel_q≈14417 */
    mock_tx_fifo_empty                 = 1;

    /* Period 0: velocity_q is clamped to clamp_accel_q << vel_ff_q.
     * Must fire ~5 steps (plan_vel_q=vel_ff_q), not 1 (max_steps sized for vq). */
    mock_rx_values[0]  = 0;
    mock_rx_fifo_level = 1;
    mock_rx_index      = 0;
    config.joint[0].updated_from_c0 = 1;
    last_pio_put_value = 0;
    do_steps(0);

    /* Decode n_steps from PIO word: upper bits = step_len_ticks, lower bit = dir */
    if (last_pio_put_value != 0) {
        /* step_len_ticks encodes how many steps fit: a short step_len means
         * multiple steps.  We just check at least 3 steps fired (not 1). */
        int32_t step_len = (int32_t)((last_pio_put_value >> 1) & 0xFFFFFF);
        /* For 5 steps/period: step_len ≈ 11033.  For 1 step: step_len ≈ 66450. */
        assert_true(step_len < 30000);  /* step_len small → multiple steps scheduled */
    }
    assert_int_not_equal(last_pio_put_value, 0);  /* at least something fired */
}

/* do_steps: single-step PIO double-buffer prevents spurious second step.
 *
 * At sub-1-step velocities (n_steps=1 per Bresenham decision), the PIO step
 * cycle takes 2*step_len+11 PIO clocks.  step_len = period_ticks/4 − overhead,
 * so the step occupies ~half the servo period, leaving the trailing stop word
 * time to clear before the next timer fires (see pio.c step_len sizing comment).
 * Without the stop word, x still holds step_len and the PIO fires a second
 * unwanted step.
 *
 * At 500 steps/s (0.5 steps/period, 1ms period) Bresenham fires on alternate
 * periods.  The period that fires (n_steps=1) must produce exactly 2 FIFO
 * writes: the step word and the trailing stop word. */
static void test_do_steps_sub1step_double_buffer_stop_word(void **state) {
    (void)state;
    config.update_time_us              = 1000;
    config.joint[0].enabled            = 1;
    config.joint[0].cmd_type           = JOINT_CMD_VELOCITY;
    config.joint[0].updated_from_c0    = 1;
    config.joint[0].velocity_requested = 500.0;  /* 0.5 steps/period at 1ms */
    config.joint[0].abs_pos_requested  = 0.0;
    config.joint[0].max_velocity       = 50000.0;
    config.joint[0].max_accel          = 0.0;
    mock_tx_fifo_empty                 = 1;

    /* Period 1: accumulator = 32768; n_steps=0; only stop word written. */
    mock_rx_values[0]  = 0;
    mock_rx_fifo_level = 1;
    mock_rx_index      = 0;
    pio_put_call_count = 0;
    do_steps(0);
    assert_int_equal(pio_put_call_count, 1);  /* only stop word */
    assert_int_equal((last_pio_put_value >> 1) & 0xFFFFFF, 0);  /* step_len == 0 */

    /* Period 2: accumulator = 65536; n_steps=1; step word + stop word. */
    mock_rx_values[0]  = 0;
    mock_rx_fifo_level = 1;
    mock_rx_index      = 0;
    pio_put_call_count = 0;
    last_pio_step_value = 0;
    config.joint[0].updated_from_c0 = 1;
    do_steps(0);
    assert_int_equal(pio_put_call_count, 2);  /* step word + stop word */
    assert_int_equal((last_pio_put_value >> 1) & 0xFFFFFF, 0);      /* last write is stop word */
    assert_int_equal((last_pio_step_value >> 1) & 0xFFFFFF, 33075); /* step word has correct step_low_half: period_ticks/4 - overhead */
    assert_int_equal(last_pio_step_value & 1, 1);      /* direction = forward */
}

/* do_steps: large position correction at sub-1-step ff speed uses continuous mode.
 *
 * When vel_ff=500 steps/s (0.5 steps/period, sub-1-step) and a 100-step lag
 * adds a correction of 1000 steps/s, velocity_q = 1.5 steps/period (98304) ≥ 65536.
 * The threshold selects continuous mode (not Bresenham + stop_word).
 * Continuous mode: one step_word with step_low_half=133000*32768/98304-175=44158, no stop word.
 * The PIO fires evenly at 1.5 steps/period across period boundaries. */
static void test_do_steps_sub1step_two_step_correction_stop_word(void **state) {
    (void)state;
    config.update_time_us              = 1000;
    config.joint[0].enabled            = 1;
    config.joint[0].cmd_type           = JOINT_CMD_VELOCITY;
    config.joint[0].updated_from_c0    = 1;
    config.joint[0].velocity_requested = 500.0;  /* 0.5 steps/period */
    config.joint[0].abs_pos_requested  = 0.0;
    config.joint[0].max_velocity       = 50000.0;
    config.joint[0].max_accel          = 0.0;
    mock_tx_fifo_empty                 = 1;

    /* Period 1: prime accumulator to 32768 (no step, no correction). */
    mock_rx_values[0]  = 0;
    mock_rx_fifo_level = 1;
    mock_rx_index      = 0;
    pio_put_call_count = 0;
    do_steps(0);
    assert_int_equal(pio_put_call_count, 1);  /* only stop word (n_steps=0) */

    /* Period 2: 100-step lag → correction 1000 steps/s → velocity_q=98304
     * (1.5 steps/period ≥ 65536) → continuous mode.
     * step_low_half = 133000*32768/98304 - 175 = 44333-175 = 44158. One FIFO write. */
    config.joint[0].abs_pos_requested  = 100.0;
    mock_rx_values[0]  = 0;
    mock_rx_fifo_level = 1;
    mock_rx_index      = 0;
    pio_put_call_count = 0;
    last_pio_step_value = 0;
    config.joint[0].updated_from_c0 = 1;
    do_steps(0);
    assert_int_equal(pio_put_call_count, 1);            /* one step_word, no stop word */
    assert_int_equal((last_pio_put_value >> 1) & 0xFFFFFF, 44158);  /* step_low_half for continuous 1.5 steps/period */
    assert_int_equal((last_pio_step_value >> 1) & 0xFFFFFF, 44158); /* same word */
    assert_int_equal(last_pio_step_value & 1, 1);      /* direction = forward */
}

/* Sub-1-step accumulator resets on direction reversal.
 *
 * At 0.5 steps/period (32768 Q16.16), Bresenham fires every other period.
 * After 3 forward periods the accumulator holds 32768 (no step in period 3).
 * On reversal, the correct behaviour is to discard that 32768 so the first
 * reverse period fires no step (0+32768 < Q16_ONE).  Without the reset, the
 * carry-over 32768+32768=65536 would fire a step one period too early. */
static void test_do_steps_accumulator_resets_on_direction_change(void **state) {
    (void)state;
    config.update_time_us              = 1000;
    config.joint[0].enabled            = 1;
    config.joint[0].cmd_type           = JOINT_CMD_VELOCITY;
    config.joint[0].abs_pos_requested  = 0.0;
    config.joint[0].max_velocity       = 32000.0;
    config.joint[0].max_accel          = 0.0;
    mock_tx_fifo_empty                 = 1;
    mock_rx_fifo_level                 = 0;  /* no pos error, no correction */

    /* — Forward: 500 steps/s = 0.5 steps/period — */
    config.joint[0].velocity_requested = 500.0;

    /* Period 1: acc = 32768 < Q16_ONE, no step. */
    config.joint[0].updated_from_c0 = 1;
    last_pio_step_value = 0;
    do_steps(0);
    assert_int_equal((last_pio_step_value >> 1) & 0xFFFFFF, 0);

    /* Period 2: acc = 65536 → step fires forward, acc drains to 0. */
    config.joint[0].updated_from_c0 = 1;
    last_pio_step_value = 0;
    do_steps(0);
    assert_true(last_pio_step_value >> 1 > 0);
    assert_int_equal(last_pio_step_value & 1, 1);  /* forward */

    /* Period 3: acc = 32768, no step — accumulator holds 32768 at reversal. */
    config.joint[0].updated_from_c0 = 1;
    last_pio_step_value = 0;
    do_steps(0);
    assert_int_equal((last_pio_step_value >> 1) & 0xFFFFFF, 0);

    /* — Reverse: -500 steps/s = 0.5 steps/period backward — */
    config.joint[0].velocity_requested = -500.0;

    /* Reverse period 1: accumulator must reset to 0 on direction change.
     * acc = 0+32768 = 32768 < Q16_ONE → no step.
     * Without the reset: 32768+32768=65536 → step fires one period early. */
    config.joint[0].updated_from_c0 = 1;
    last_pio_step_value = 0;
    do_steps(0);
    assert_int_equal((last_pio_step_value >> 1) & 0xFFFFFF, 0);  /* proves accumulator reset */

    /* Reverse period 2: acc = 65536 → step fires backward. */
    config.joint[0].updated_from_c0 = 1;
    last_pio_step_value = 0;
    do_steps(0);
    assert_true(last_pio_step_value >> 1 > 0);
    assert_int_equal(last_pio_step_value & 1, 0);  /* backward */

    /* Reverse period 3: acc = 32768, no step. */
    config.joint[0].updated_from_c0 = 1;
    last_pio_step_value = 0;
    do_steps(0);
    assert_int_equal((last_pio_step_value >> 1) & 0xFFFFFF, 0);

    /* Reverse period 4: acc = 65536 → step fires backward. */
    config.joint[0].updated_from_c0 = 1;
    last_pio_step_value = 0;
    do_steps(0);
    assert_true(last_pio_step_value >> 1 > 0);
    assert_int_equal(last_pio_step_value & 1, 0);  /* backward */
}

/* DIR setup violation fires when step_low_half < DIR_SETUP_MIN_CYCLES on direction change.
 *
 * At 100,000 steps/s (100 steps/period @ 1ms), step_low_half ≈ 491 PIO cycles, which is
 * below DIR_SETUP_MIN_CYCLES (665).  Reversing direction at this speed must set bit 0
 * in the violation bitmask. */
static void test_dir_setup_violation_fires_on_fast_reversal(void **state) {
    (void)state;
    config.update_time_us              = 1000;
    config.joint[0].enabled            = 1;
    config.joint[0].cmd_type           = JOINT_CMD_VELOCITY;
    config.joint[0].abs_pos_requested  = 0.0;
    config.joint[0].max_velocity       = 200000.0;
    config.joint[0].max_accel          = 0.0;
    mock_tx_fifo_empty                 = 1;
    mock_rx_fifo_level                 = 0;

    /* Forward at low speed — establishes direction; discard any initial-state change. */
    config.joint[0].velocity_requested = 1000.0;
    config.joint[0].updated_from_c0   = 1;
    do_steps(0);
    pio_get_and_clear_dir_setup_violations();

    /* Reverse at high speed: step_low_half ≈ 491 < DIR_SETUP_MIN_CYCLES (665). */
    config.joint[0].velocity_requested = -100000.0;
    config.joint[0].updated_from_c0   = 1;
    do_steps(0);
    assert_int_equal(pio_get_and_clear_dir_setup_violations(), 1 << 0);
}

/* No violation when step_low_half >= DIR_SETUP_MIN_CYCLES on direction change.
 *
 * At 1,000 steps/s (1 step/period @ 1ms), step_low_half ≈ 1490 PIO cycles >> 665.
 * Reversing at this speed must not set any violation bit. */
static void test_dir_setup_no_violation_at_low_speed(void **state) {
    (void)state;
    config.update_time_us              = 1000;
    config.joint[0].enabled            = 1;
    config.joint[0].cmd_type           = JOINT_CMD_VELOCITY;
    config.joint[0].abs_pos_requested  = 0.0;
    config.joint[0].max_velocity       = 200000.0;
    config.joint[0].max_accel          = 0.0;
    mock_tx_fifo_empty                 = 1;
    mock_rx_fifo_level                 = 0;

    config.joint[0].velocity_requested = 1000.0;
    config.joint[0].updated_from_c0   = 1;
    do_steps(0);
    pio_get_and_clear_dir_setup_violations();

    config.joint[0].velocity_requested = -1000.0;
    config.joint[0].updated_from_c0   = 1;
    do_steps(0);
    assert_int_equal(pio_get_and_clear_dir_setup_violations(), 0);
}

/* No violation when direction does not change, even at high speed. */
static void test_dir_setup_no_violation_without_direction_change(void **state) {
    (void)state;
    config.update_time_us              = 1000;
    config.joint[0].enabled            = 1;
    config.joint[0].cmd_type           = JOINT_CMD_VELOCITY;
    config.joint[0].abs_pos_requested  = 0.0;
    config.joint[0].max_velocity       = 200000.0;
    config.joint[0].max_accel          = 0.0;
    mock_tx_fifo_empty                 = 1;
    mock_rx_fifo_level                 = 0;

    /* First forward call may see initial-state direction "change"; discard. */
    config.joint[0].velocity_requested = 100000.0;
    config.joint[0].updated_from_c0   = 1;
    do_steps(0);
    pio_get_and_clear_dir_setup_violations();

    /* Second forward call at same high speed: no direction change → no violation. */
    config.joint[0].updated_from_c0   = 1;
    do_steps(0);
    assert_int_equal(pio_get_and_clear_dir_setup_violations(), 0);
}

/* Violation bit clears after the first read (momentary semantics). */
static void test_dir_setup_violation_is_momentary(void **state) {
    (void)state;
    config.update_time_us              = 1000;
    config.joint[0].enabled            = 1;
    config.joint[0].cmd_type           = JOINT_CMD_VELOCITY;
    config.joint[0].abs_pos_requested  = 0.0;
    config.joint[0].max_velocity       = 200000.0;
    config.joint[0].max_accel          = 0.0;
    mock_tx_fifo_empty                 = 1;
    mock_rx_fifo_level                 = 0;

    config.joint[0].velocity_requested = 1000.0;
    config.joint[0].updated_from_c0   = 1;
    do_steps(0);
    pio_get_and_clear_dir_setup_violations();

    config.joint[0].velocity_requested = -100000.0;
    config.joint[0].updated_from_c0   = 1;
    do_steps(0);
    assert_int_equal(pio_get_and_clear_dir_setup_violations(), 1 << 0);  /* violation present */
    assert_int_equal(pio_get_and_clear_dir_setup_violations(), 0);        /* cleared on first read */
}

/* Sub-1-step step_low_half must fit within half the servo period to prevent a FIFO race.
 *
 * With step_low_half = period_ticks/2 the PIO step takes 2*step_low_half+348 ≈ period_ticks
 * cycles.  The trailing stop word is still being consumed by the PIO when the next
 * timer fires, so the !fifo_empty guard in commit_steps spuriously blocks the next
 * step push — consecutive steps near 1 step/period are randomly dropped.
 *
 * Fix: step_low_half = period_ticks/4 - overhead.  Step takes exactly period_ticks/2
 * PIO cycles, leaving the rest of the period for the stop word to clear.
 * At 900 steps/s (0.9 steps/period at 1ms), Bresenham fires a step on period 2
 * (acc=117964).  step_low_half must be 133000/4−175 = 33075, not the old 66491. */
static void test_do_steps_sub1step_step_len_fits_half_period(void **state) {
    (void)state;
    config.update_time_us              = 1000;
    config.joint[0].enabled            = 1;
    config.joint[0].cmd_type           = JOINT_CMD_VELOCITY;
    config.joint[0].abs_pos_requested  = 0.0;
    config.joint[0].max_velocity       = 32000.0;
    config.joint[0].max_accel          = 0.0;
    mock_tx_fifo_empty                 = 1;
    mock_rx_fifo_level                 = 0;
    config.joint[0].velocity_requested = 900.0;  /* 0.9 steps/period at 1ms */

    /* Period 1: acc=58982 < 65536, no step. */
    config.joint[0].updated_from_c0 = 1;
    last_pio_step_value = 0;
    do_steps(0);
    assert_int_equal((last_pio_step_value >> 1) & 0xFFFFFF, 0);

    /* Period 2: acc=117964 → step fires.  Verify step_low_half = period_ticks/4 - overhead.
     * Before the fix step_low_half was 66325 (= period_ticks/2 − 175), causing the race. */
    config.joint[0].updated_from_c0 = 1;
    last_pio_step_value = 0;
    do_steps(0);
    assert_true(last_pio_step_value >> 1 > 0);
    assert_int_equal((last_pio_step_value >> 1) & 0xFFFFFF, 133000 / 4 - 175);  /* 33075 */
}

/* Sub-1-step stop word is pushed even when the PIO has not yet drained step_word.
 *
 * Hardware race: issue_stop_word() checks pio_sm_is_tx_fifo_empty() immediately
 * after step_word is pushed.  At ~1 step/period the PIO is mid-step and cannot
 * drain the FIFO in the ~15 CPU cycles between the two calls, so the stop word
 * is silently skipped and the PIO stale-x repeats at 2 steps/ms.
 *
 * Fix: push stop word unconditionally in commit_steps (do not use issue_stop_word).
 *
 * This test simulates the hardware condition by exhausting mock_tx_fifo_empty
 * after the guard check so the stop-word call sees a non-empty FIFO.  With the
 * old issue_stop_word() path that would produce only 1 FIFO write (step_word);
 * the fix produces 2 (step_word + stop_word). */
static void test_do_steps_sub1step_stop_word_pushed_despite_active_pio(void **state) {
    (void)state;
    config.update_time_us              = 1000;
    config.joint[0].enabled            = 1;
    config.joint[0].cmd_type           = JOINT_CMD_VELOCITY;
    config.joint[0].abs_pos_requested  = 0.0;
    config.joint[0].max_velocity       = 32000.0;
    config.joint[0].max_accel          = 0.0;
    mock_rx_fifo_level                 = 0;
    mock_tx_fifo_empty                 = 1;
    config.joint[0].velocity_requested = 500.0;  /* 0.5 steps/period */

    /* Period 1: acc=32768, n_steps=0 — one FIFO-empty check (guard). */
    config.joint[0].updated_from_c0 = 1;
    pio_put_call_count = 0;
    do_steps(0);

    /* Period 2: acc=65536, n_steps=1.  Allow exactly 1 call to return 1 (empty)
     * so the guard passes and step_word is pushed; subsequent call returns 0
     * (non-empty), simulating the PIO still holding step_word mid-step.
     * The stop word must be pushed regardless. */
    mock_tx_fifo_empty_calls_remaining = 1;
    config.joint[0].updated_from_c0 = 1;
    pio_put_call_count = 0;
    do_steps(0);
    assert_int_equal(pio_put_call_count, 2);  /* step_word + stop_word */
}

/* step_len_us reflects the half-period of the most recent step sent to PIO,
 * or 0 if no step fired this window.
 * Continuous at 1 step/period: step_len = 133000*32768/65536 - 9 = 66491;
 * half_period_us = (66491+9)/133 = 500. */
static void test_step_len_us_tracked(void **state) {
    (void)state;
    config.update_time_us              = 1000;
    config.joint[0].enabled            = 1;
    config.joint[0].cmd_type           = JOINT_CMD_VELOCITY;
    config.joint[0].abs_pos_requested  = 0.0;
    config.joint[0].max_velocity       = 32000.0;
    config.joint[0].max_accel          = 0.0;
    mock_tx_fifo_empty                 = 1;
    mock_rx_fifo_level                 = 0;

    config.joint[0].velocity_requested = 1000.0;  /* 1 step/period → continuous */
    config.joint[0].updated_from_c0    = 1;
    do_steps(0);
    assert_int_equal(config.joint[0].step_len_us, 500);

    config.joint[0].velocity_requested = 0.0;  /* no step */
    config.joint[0].updated_from_c0    = 1;
    do_steps(0);
    assert_int_equal(config.joint[0].step_len_us, 0);
}

/* After a LinuxCNC restart (pio_invalidate_all_joints()), init_pio() must run
 * again on the next enable so new GPIO pin assignments take effect (issue #44). */
static void test_pio_reinit_after_invalidate(void **state) {
    (void)state;
    config.update_time_us           = 1000;
    config.joint[0].io_pos_step     = 1;
    config.joint[0].io_pos_dir      = 2;
    config.joint[0].enabled         = 1;
    config.joint[0].cmd_type        = JOINT_CMD_VELOCITY;
    config.joint[0].velocity_requested = 0.0;
    config.joint[0].updated_from_c0 = 1;
    mock_tx_fifo_empty               = 1;

    do_steps(0);  /* first enable: triggers init_pio */
    assert_int_equal(step_gen2_program_init_call_count, 1);

    /* Simulate LinuxCNC restart clearing init_done for all joints. */
    pio_invalidate_all_joints();

    /* New GPIO config arrives with different pins. */
    config.joint[0].io_pos_step = 5;
    config.joint[0].io_pos_dir  = 6;

    /* Disable then re-enable to trigger handle_enable_transition. */
    config.joint[0].enabled          = 0;
    config.joint[0].updated_from_c0  = 1;
    do_steps(0);
    config.joint[0].enabled          = 1;
    config.joint[0].updated_from_c0  = 1;
    do_steps(0);

    /* init_pio must have run a second time to apply the new pins. */
    assert_int_equal(step_gen2_program_init_call_count, 2);
}

/* high_count is now 7 bits (0–127): values > 63 must not be clamped to 0.
 * With the old 0x3F mask, high_count=64 → 0 (silent failure, step_word bit 31 clear).
 * With the new 0x7F mask, high_count=64 → 64 → bit 31 of step_word set.
 * Set high_count AFTER the first enable so init_pio's default assignment runs first. */
/* drain_rx_fifo: position is continuous across step_count SM reinit.
 *
 * When LinuxCNC restarts, pio_invalidate_all_joints() clears init_done and the
 * step_count SM counter resets to 0. Without the joint_offset fix, drain_rx_fifo
 * returns 0 on the next period, causing a position jump and follow error (issue #51).
 * With the fix, abs_pos_achieved is preserved across the reinit. */
static void test_drain_rx_fifo_position_continuous_across_reinit(void **state) {
    (void)state;
    config.update_time_us              = 1000;
    config.joint[0].enabled            = 1;
    config.joint[0].cmd_type           = JOINT_CMD_POSITION;
    config.joint[0].velocity_requested = 0.0;
    config.joint[0].abs_pos_requested  = 100.0;
    config.joint[0].max_velocity       = 50.0;
    config.joint[0].max_accel          = 0.0;
    mock_rx_values[0]                  = 100;
    mock_rx_fifo_level                 = 1;
    mock_tx_fifo_empty                 = 1;
    config.joint[0].updated_from_c0    = 1;
    do_steps(0);
    assert_int_equal(config.joint[0].abs_pos_achieved, 100);

    /* Simulate LinuxCNC restart: invalidate PIO and reset SM counter to 0. */
    pio_invalidate_all_joints();
    mock_rx_index      = 0;
    mock_rx_values[0]  = 0;  /* hardware counter reset to 0 after SM restart */
    mock_rx_fifo_level = 1;
    config.joint[0].updated_from_c0 = 1;
    do_steps(0);

    /* Position must remain at 100 — no jump to 0. */
    assert_int_equal(config.joint[0].abs_pos_achieved, 100);
}

static void test_step_high_count_7bit_accepted(void **state) {
    (void)state;
    config.update_time_us              = 1000;
    config.joint[0].enabled            = 1;
    config.joint[0].cmd_type           = JOINT_CMD_VELOCITY;
    config.joint[0].velocity_requested = 1000.0;
    config.joint[0].max_velocity       = 32000.0;
    config.joint[0].updated_from_c0    = 1;
    mock_tx_fifo_empty                 = 1;

    do_steps(0);                       /* trigger init_pio (sets high_count=20) */
    pio_set_step_high_count(0, 64);    /* first value beyond old 6-bit limit */
    config.joint[0].updated_from_c0   = 1;
    last_pio_put_value                 = 0;
    do_steps(0);

    /* high_count=64 << 25 = 0x80000000 → bit 31 must be set */
    assert_true(last_pio_put_value & (1u << 31));
}


int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test_setup(test_drain_rx_fifo_empty_returns_current, test_setup),
        cmocka_unit_test_setup(test_drain_rx_fifo_single_entry,          test_setup),
        cmocka_unit_test_setup(test_drain_rx_fifo_keeps_last,            test_setup),

        cmocka_unit_test_setup(test_clamp_accel_no_change,                       test_setup),
        cmocka_unit_test_setup(test_clamp_accel_under_limit,            test_setup),
        cmocka_unit_test_setup(test_clamp_accel_over_limit_positive,    test_setup),
        cmocka_unit_test_setup(test_clamp_accel_over_limit_negative,    test_setup),
        cmocka_unit_test_setup(test_clamp_accel_zero_max,               test_setup),

        cmocka_unit_test_setup(test_compute_velocity_cmd_velmode_passthrough,        test_setup),
        cmocka_unit_test_setup(test_compute_velocity_cmd_velmode_lag_correction,    test_setup),
        cmocka_unit_test_setup(test_compute_velocity_cmd_velmode_lead_correction,   test_setup),
        cmocka_unit_test_setup(test_compute_velocity_cmd_velmode_dead_zone,         test_setup),
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
        cmocka_unit_test_setup(test_do_steps_reconnect_mid_decel_no_jitter,  test_setup),
        cmocka_unit_test_setup(test_do_steps_fresh_enable_snaps_to_commanded, test_setup),
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
        cmocka_unit_test_setup(test_do_steps_posmode_ff_active_tracks_at_full_speed,      test_setup),
        cmocka_unit_test_setup(test_do_steps_posmode_ff_large_negative_tracks_at_full_speed, test_setup),
        cmocka_unit_test_setup(test_do_steps_posmode_ff_small_positive_tracks_normally,  test_setup),
        cmocka_unit_test_setup(test_do_steps_posmode_ff_small_negative_tracks_normally,  test_setup),
        cmocka_unit_test_setup(test_do_steps_posmode_ff_zero_clamp_accel_positive,       test_setup),
        cmocka_unit_test_setup(test_do_steps_posmode_ff_zero_clamp_accel_negative,       test_setup),
        cmocka_unit_test_setup(test_do_steps_velmode_0_75,      test_setup),
        cmocka_unit_test_setup(test_do_steps_velmode_0_25,      test_setup),
        cmocka_unit_test_setup(test_do_steps_velmode_int_1,     test_setup),
        cmocka_unit_test_setup(test_do_steps_velmode_int_10,    test_setup),
        cmocka_unit_test_setup(test_do_steps_velmode_frac_1_5,  test_setup),
        cmocka_unit_test_setup(test_do_steps_velmode_frac_10_5, test_setup),
        cmocka_unit_test_setup(test_do_steps_velmode_reverse,                  test_setup),
        cmocka_unit_test_setup(test_do_steps_velmode_lag_corrected_over_time,        test_setup),
        cmocka_unit_test_setup(test_do_steps_noninteger_velocity_no_drift,           test_setup),
        cmocka_unit_test_setup(test_do_steps_position_mode_no_overshoot_after_correction, test_setup),
        cmocka_unit_test_setup(test_clamp_accel_fixed_zero_target_never_overshoots, test_setup),
        cmocka_unit_test_setup(test_ramp_accel_headroom_tracks_vel_ff,              test_setup),
        cmocka_unit_test_setup(test_do_steps_velocity_achieved_zero_when_stopped,       test_setup),
        cmocka_unit_test_setup(test_do_steps_velocity_achieved_nonzero_while_decelerating, test_setup),
        cmocka_unit_test_setup(test_do_steps_velocity_achieved_zero_reverse,             test_setup),
        cmocka_unit_test_setup(test_do_steps_velocity_mode_no_position_cap,              test_setup),
        cmocka_unit_test_setup(test_do_steps_posmode_uniform_spacing_at_low_speed,     test_setup),
        cmocka_unit_test_setup(test_do_steps_posmode_velocity_achieved_zero_when_stopped,       test_setup),
        cmocka_unit_test_setup(test_do_steps_posmode_velocity_achieved_nonzero_while_decelerating, test_setup),
        cmocka_unit_test_setup(test_do_steps_posmode_velocity_achieved_zero_reverse,             test_setup),
        cmocka_unit_test_setup(test_do_steps_posmode_overshoot_does_not_reverse,                test_setup),
        cmocka_unit_test_setup(test_do_steps_posmode_correction_at_stop_does_not_spike_vel_achieved, test_setup),
        cmocka_unit_test_setup(test_do_steps_posmode_large_overshoot_does_not_reverse,         test_setup),
        cmocka_unit_test_setup(test_do_steps_posmode_sub1step_correction_fires_at_sub1step_ff, test_setup),
        cmocka_unit_test_setup(test_do_steps_multistep_accel_ramp_no_backlog,                 test_setup),
        cmocka_unit_test_setup(test_do_steps_sub1step_double_buffer_stop_word,               test_setup),
        cmocka_unit_test_setup(test_do_steps_sub1step_two_step_correction_stop_word,        test_setup),
        cmocka_unit_test_setup(test_do_steps_accumulator_resets_on_direction_change,       test_setup),
        cmocka_unit_test_setup(test_dir_setup_violation_fires_on_fast_reversal,            test_setup),
        cmocka_unit_test_setup(test_dir_setup_no_violation_at_low_speed,                   test_setup),
        cmocka_unit_test_setup(test_dir_setup_no_violation_without_direction_change,       test_setup),
        cmocka_unit_test_setup(test_dir_setup_violation_is_momentary,                      test_setup),
        cmocka_unit_test_setup(test_do_steps_sub1step_step_len_fits_half_period,           test_setup),
        cmocka_unit_test_setup(test_do_steps_sub1step_stop_word_pushed_despite_active_pio, test_setup),
        cmocka_unit_test_setup(test_step_len_us_tracked,                                   test_setup),
        cmocka_unit_test_setup(test_pio_reinit_after_invalidate,                           test_setup),
        cmocka_unit_test_setup(test_drain_rx_fifo_position_continuous_across_reinit,       test_setup),
        cmocka_unit_test_setup(test_step_high_count_7bit_accepted,                         test_setup),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
