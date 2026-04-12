#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <stdint.h>
#include <cmocka.h>
#include <stdio.h>

#include "../rp2040/config.h"
#include "mocks/rp_mocks.h"
#include "../rp2040/timing.h"

/* timing_test.c does not compile config.c — define tick here. */
volatile uint32_t tick = 0;

/* Configurable time mock: each call returns the previous value + mock_time_step.
 * A step of 1000 simulates steady 1 kHz packets, keeping the EMA stable.
 * Changing mock_time_step between recover_clock() calls simulates varying
 * packet inter-arrival times. Note: because time_us_64() is called once per
 * recover_clock() call, the sample computed on call N is the step set for
 * call N-1 (one-call lag). */
static uint64_t mock_time       = 0;
static uint32_t mock_time_step  = 1000;

uint64_t __wrap_time_us_64(void) {
    uint64_t t = mock_time;
    mock_time += mock_time_step;
    return t;
}

/* Capture the callback and delay so tests can fire the callback manually
 * and verify the period passed to the timer. */
static repeating_timer_callback_t captured_callback = NULL;
static repeating_timer_t          captured_timer;
static int32_t                    captured_timer_delay = 0;

bool __wrap_add_repeating_timer_us(int32_t delay_us,
                                    repeating_timer_callback_t callback,
                                    void *user_data,
                                    repeating_timer_t *out) {
    captured_timer_delay = delay_us;
    captured_callback    = callback;
    return true;
}

bool __wrap_cancel_repeating_timer(repeating_timer_t *timer) {
    return true;
}

/* Configurable id_diff mock. Default is 1 (normal single-packet arrival).
 * Tests that exercise id_diff != 1 set mock_id_diff directly. */
static int32_t mock_id_diff = 1;

int32_t __wrap_get_last_id_diff(void) {
    return mock_id_diff;
}

static int      update_period_call_count = 0;
static uint32_t last_update_period_arg   = 0;
static uint32_t min_update_period_arg    = UINT32_MAX;
static uint32_t max_update_period_arg    = 0;

void __wrap_update_period(uint32_t update_time_us) {
    update_period_call_count++;
    last_update_period_arg = update_time_us;
    if(update_time_us < min_update_period_arg) min_update_period_arg = update_time_us;
    if(update_time_us > max_update_period_arg) max_update_period_arg = update_time_us;
}

static int setup(void **state) {
    (void) state;
    mock_time                = 0;
    mock_time_step           = 1000;
    update_period_call_count = 0;
    last_update_period_arg   = 0;
    min_update_period_arg    = UINT32_MAX;
    max_update_period_arg    = 0;
    tick                     = 0;
    captured_callback        = NULL;
    captured_timer_delay     = 0;
    mock_id_diff             = 1;
    timing_reset_for_test();
    return 0;
}

/* ---- Structural tests ---- */

/* timing_init() must register a callback via add_repeating_timer_us. */
static void test_timing_init__registers_callback(void **state) {
    (void) state;
    timing_init();
    assert_non_null(captured_callback);
}

/* The registered callback must increment tick when called. */
static void test_tick_callback__increments_tick(void **state) {
    (void) state;
    timing_init();
    assert_non_null(captured_callback);

    uint32_t tick_before = tick;
    captured_callback(&captured_timer);
    assert_int_equal(tick_before + 1, tick);
}

/* Each callback invocation increments tick exactly once. */
static void test_tick_callback__increments_tick_multiple_times(void **state) {
    (void) state;
    timing_init();

    uint32_t tick_before = tick;
    captured_callback(&captured_timer);
    captured_callback(&captured_timer);
    captured_callback(&captured_timer);
    assert_int_equal(tick_before + 3, tick);
}

/* timing_init() must start the hardware timer at the default 1000 µs period.
 * The pico SDK convention is a negative delay_us for periodic timers. */
static void test_timing_init__sets_initial_period(void **state) {
    (void) state;
    timing_init();
    assert_int_equal(-1000, captured_timer_delay);
}

/* With steady 1000 µs packets the EMA stays at 1000 µs.
 * update_period() must not be called when the period is stable. */
static void test_recover_clock__does_not_call_update_period_repeatedly(void **state) {
    (void) state;
    timing_init();
    recover_clock();   /* seeds time_last */
    int count_after_first = update_period_call_count;

    recover_clock();
    recover_clock();
    recover_clock();
    assert_int_equal(count_after_first, update_period_call_count);
}

/* recover_clock() must return — no busy-wait, no hang. */
static void test_recover_clock__does_not_hang(void **state) {
    (void) state;
    timing_init();
    recover_clock();
    assert_true(1);
}

/* ---- EMA convergence tests ---- */

/* Helper: seed time_last and stabilise the EMA at 1000 µs.
 * Two calls are needed: the first only sets time_last (time_last==0 skips EMA);
 * the second provides the first 1000 µs sample. */
static void seed_at_1000us(void) {
    timing_init();
    mock_time_step = 1000;
    recover_clock();   /* sets time_last, no EMA update (time_last was 0) */
    recover_clock();   /* first 1000 µs sample, EMA stays at 1000 */
}

/* After the EMA has converged on 1000 µs, sustained early packets (900 µs)
 * must drive the EMA — and therefore the timer period — down to 900 µs.
 *
 * The EMA (alpha=1/64, stored ×64) requires ~300 samples to reach within
 * 1 µs of the new rate.  We run 400 to guarantee integer convergence.
 *
 * Note: the first sample after the step change reads as 1000 µs due to the
 * one-call lag in the mock (mock_time_step affects the next call's interval,
 * not the current one).  This matches real-world behaviour where the first
 * "early" packet is measured relative to the last "normal" packet. */
static void test_recover_clock__early_packets_converge(void **state) {
    (void) state;
    seed_at_1000us();

    mock_time_step = 900;
    for(int i = 0; i < 400; i++) {
        recover_clock();
    }

    assert_int_equal(900, last_update_period_arg);
}

/* After the EMA has converged on 1000 µs, sustained late packets (1100 µs)
 * must drive the EMA up to 1100 µs. */
static void test_recover_clock__late_packets_converge(void **state) {
    (void) state;
    seed_at_1000us();

    mock_time_step = 1100;
    for(int i = 0; i < 400; i++) {
        recover_clock();
    }

    assert_int_equal(1100, last_update_period_arg);
}

/* After the EMA has converged on 1000 µs, erratic packets that alternate
 * between 800 µs and 1200 µs (average 1000 µs) must not push the timer
 * period outside ±5 µs of 1000 µs.
 *
 * Verified by simulation: with alpha=1/64 the EMA oscillates in [998, 1003].
 * The ±5 µs bound (995–1005) is a conservative engineering tolerance.
 *
 * Mock note: setting mock_time_step=1200 for even iterations and 800 for odd
 * produces actual EMA samples of [1000, 1200, 800, 1200, 800, …] due to the
 * one-call lag.  The average of the non-lag samples is exactly 1000 µs. */
static void test_recover_clock__erratic_packets_stay_bounded(void **state) {
    (void) state;
    seed_at_1000us();

    for(int i = 0; i < 200; i++) {
        mock_time_step = (i % 2 == 0) ? 1200 : 800;
        recover_clock();
    }

    assert_true(update_period_call_count > 0);
    assert_true(min_update_period_arg >= 995);
    assert_true(max_update_period_arg <= 1005);
}

/* ---- EMA id_diff normalisation tests ---- */

/* When a packet arrives 2000 µs late because one slot was missed (id_diff=2),
 * the normalized sample is 2000/2=1000 µs and the EMA must not change.
 * Strategy: after seed, call recover_clock() once with step=2000 and id_diff=1
 * to flush the one-call lag (sample=1000 µs, EMA stable), then switch to
 * id_diff=2 and call again — now sample=2000 µs, normalized=1000 µs. */
static void test_recover_clock__missed_packet_does_not_distort_ema(void **state) {
    (void) state;
    seed_at_1000us();

    /* Flush the lag carry-over with a normal id_diff=1, step=2000. */
    mock_time_step = 2000;
    mock_id_diff   = 1;
    recover_clock();     /* sample=1000 µs (lag), EMA stays at 1000 */

    /* Now the inflated gap: sample=2000 µs, id_diff=2, normalized=1000 µs. */
    int count_before = update_period_call_count;
    mock_id_diff = 2;
    recover_clock();     /* sample=2000 µs / 2 = 1000 µs → EMA unchanged */

    assert_int_equal(count_before, update_period_call_count);
}

/* When 50 packets are missed, time gap=50000 µs, id_diff=50.
 * Normalized sample=1000 µs → EMA must not change. */
static void test_recover_clock__large_gap_normalizes_correctly(void **state) {
    (void) state;
    seed_at_1000us();

    /* Flush the lag carry-over with id_diff=1, step=50000. */
    mock_time_step = 50000;
    mock_id_diff   = 1;
    recover_clock();     /* sample=1000 µs (lag), EMA stays at 1000 */

    /* Now the 50-packet gap: sample=50000 µs / 50 = 1000 µs → EMA unchanged. */
    int count_before = update_period_call_count;
    mock_id_diff = 50;
    recover_clock();

    assert_int_equal(count_before, update_period_call_count);
}

/* id_diff=0 must cause the EMA update to be skipped entirely.
 * A very large time step would distort the EMA if the guard were absent. */
static void test_recover_clock__skips_ema_on_zero_id_diff(void **state) {
    (void) state;
    seed_at_1000us();

    int count_before   = update_period_call_count;
    mock_id_diff       = 0;
    mock_time_step     = 5000;
    recover_clock();   /* EMA update skipped — count must not change */
    recover_clock();   /* second call with same large step — still skipped */

    assert_int_equal(count_before, update_period_call_count);
}

/* When the EMA is converging (each packet shifts the integer period by 1 µs),
 * the time guard must prevent a restart on every packet.
 *
 * Scenario: packets at 990 µs (10 µs early).  The guard allows at most one
 * restart per ~1000 µs elapsed, so over 20 × 990 µs ≈ 19 800 µs there can be
 * at most ≈ 19 restarts.  Without the guard all 20 EMA changes would each
 * restart the timer, resetting the countdown each time and starving Core1.
 * With the guard the count must be ≤ 11 (roughly one per two packets). */
static void test_recover_clock__rate_limits_timer_restarts(void **state) {
    (void) state;
    seed_at_1000us();

    mock_time_step = 990;
    int count_before = update_period_call_count;
    for(int i = 0; i < 20; i++) {
        recover_clock();
    }

    int restarts = update_period_call_count - count_before;
    assert_true(restarts <= 11);
}

/* Negative id_diff (e.g. LinuxCNC restart wrapping the sequence counter) must
 * also cause the EMA update to be skipped, protecting against huge normalized
 * samples being fed into the accumulator.  After the guard fires, a second
 * skipped call flushes the time gap (resetting time_last), then normal packets
 * at 1000 µs leave the EMA undisturbed.
 *
 * Mock timing note: the call with step=5000000 advances mock_time by 5000000
 * but time_last is set to the *returned* time (before the increment).  A second
 * skipped call consumes that gap so the subsequent normal sample is small. */
static void test_recover_clock__skips_ema_on_negative_id_diff(void **state) {
    (void) state;
    seed_at_1000us();

    int count_before = update_period_call_count;

    /* First skipped call: time gap of 5 seconds (simulates restart). */
    mock_id_diff  = -50000;
    mock_time_step = 5000000;
    recover_clock();   /* EMA update skipped */
    assert_int_equal(count_before, update_period_call_count);

    /* Second skipped call at normal step: advances time_last to near mock_time
     * so the subsequent normal sample is ≈ 1000 µs, not 5 seconds. */
    mock_time_step = 1000;
    recover_clock();   /* EMA update still skipped */
    assert_int_equal(count_before, update_period_call_count);

    /* Normal packet after recovery — the gap is now consumed, sample ≈ 1000 µs.
     * EMA stays at 1000 µs so the timer period does not change. */
    mock_id_diff   = 1;
    mock_time_step = 1000;
    recover_clock();
    assert_int_equal(count_before, update_period_call_count);
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test_setup(test_timing_init__registers_callback, setup),
        cmocka_unit_test_setup(test_tick_callback__increments_tick, setup),
        cmocka_unit_test_setup(test_tick_callback__increments_tick_multiple_times, setup),
        cmocka_unit_test_setup(test_timing_init__sets_initial_period, setup),
        cmocka_unit_test_setup(test_recover_clock__does_not_call_update_period_repeatedly, setup),
        cmocka_unit_test_setup(test_recover_clock__does_not_hang, setup),
        cmocka_unit_test_setup(test_recover_clock__early_packets_converge, setup),
        cmocka_unit_test_setup(test_recover_clock__late_packets_converge, setup),
        cmocka_unit_test_setup(test_recover_clock__erratic_packets_stay_bounded, setup),
        cmocka_unit_test_setup(test_recover_clock__missed_packet_does_not_distort_ema, setup),
        cmocka_unit_test_setup(test_recover_clock__large_gap_normalizes_correctly, setup),
        cmocka_unit_test_setup(test_recover_clock__skips_ema_on_zero_id_diff, setup),
        cmocka_unit_test_setup(test_recover_clock__skips_ema_on_negative_id_diff, setup),
        cmocka_unit_test_setup(test_recover_clock__rate_limits_timer_restarts, setup),
    };

    return cmocka_run_group_tests(tests, NULL, NULL);
}
