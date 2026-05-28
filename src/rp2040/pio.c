#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifdef BUILD_TESTS

#include "../test/mocks/rp_mocks.h"
#include "../test/mocks/pio_mocks.h"

#else  // BUILD_TESTS

#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "pico/stdlib.h"
#include "pico_stepper.pio.h"

#endif  // BUILD_TESTS

#include "pio.h"
#include "config.h"

/* PIO instruction cycles consumed by the state machine loop itself (derived
 * from pico_stepper.pio); subtracted when converting step period to PIO len. */
#define STEP_PIO_LEN_OVERHEAD  9
#define RP2040_CLOCK_MHZ       133
#define Q16_ONE                65536  /* 1.0 in Q16.16 fixed-point */

/* Remaining SMs after step_gen, capped at MAX_JOINT (can't count more joints
 * than we move). For MAX_JOINT=4: 4 feedback SMs (current behaviour).
 * For MAX_JOINT=8: 0 feedback SMs (open-loop). */
#if MAX_JOINT > 8
  #error "MAX_JOINT must be 1-8"
#endif
#define NUM_FEEDBACK  ((8 - MAX_JOINT) < MAX_JOINT ? (8 - MAX_JOINT) : MAX_JOINT)

/* Which PIO block and program offset a joint's step_gen SM lives on.
 * Joints 0-3 always use PIO0; joints 4-7 use PIO1 (MAX_JOINT > 4 only). */
#if MAX_JOINT > 4
  #define JOINT_PIO(j)        ((j) < 4 ? pio0 : pio1)
  #define JOINT_GEN_OFFSET(j) ((j) < 4 ? offset_pio0 : offset_pio1_gen)
#else
  #define JOINT_PIO(j)        pio0
  #define JOINT_GEN_OFFSET(j) offset_pio0
#endif

typedef struct {
    uint32_t sm_gen;    /* step_gen SM on JOINT_PIO(joint) */
    uint32_t sm_count;  /* step_count SM on PIO1; valid only for joint < NUM_FEEDBACK */
    bool     init_done;
    int32_t  last_pos_achieved;
    uint32_t last_enabled;
    int32_t  last_velocity_q;
    int32_t  step_accumulator_q;
    uint32_t last_direction;
} JointPioState;

static JointPioState joint_state[MAX_JOINT];
static uint32_t offset_pio0       = 0;  /* step_gen on PIO0 */
static uint32_t offset_pio1_gen   = 0;  /* step_gen on PIO1 (MAX_JOINT > 4 only) */
static uint32_t offset_pio1_count = 0;  /* step_count on PIO1 (NUM_FEEDBACK > 0 only) */
static uint8_t  programs_loaded   = 0;

void init_pio(const uint32_t joint)
{

  if(joint_state[joint].init_done) {
    return;
  }

  int8_t io_pos_step;
  int8_t io_pos_dir;
  uint8_t invert_step;
  uint8_t invert_dir;
  get_joint_config(
      joint,
      CORE1,
      NULL,
      &io_pos_step,
      &io_pos_dir,
      NULL,
      NULL,
      NULL,
      NULL,
      NULL,
      NULL,
      NULL,
      &invert_step,
      &invert_dir
      );

  if(io_pos_step < 0 || io_pos_step >= 32) {
    printf("WARN: Joint %u step io pin is out of range: %i\n", joint, io_pos_step);
    return;
  }
  if(io_pos_dir < 0 || io_pos_dir >= 32) {
    printf("WARN: Joint %u dir io pin is out of range: %i\n", joint, io_pos_dir);
    return;
  }
#ifdef VERBOSE_CONFIG_LOG
  printf("\tio-step: %i\tio-dir: %i\tinv-step: %u\tinv-dir: %u\n",
         io_pos_step, io_pos_dir, invert_step, invert_dir);
#endif
  gpio_init(io_pos_step);
  gpio_init(io_pos_dir);
  gpio_set_dir(io_pos_step, GPIO_OUT);
  gpio_set_dir(io_pos_dir, GPIO_OUT);
  gpio_put(io_pos_step, 0);
  gpio_put(io_pos_dir, 0);
  gpio_set_outover(io_pos_step, invert_step ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);
  gpio_set_outover(io_pos_dir,  invert_dir  ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);

  if(programs_loaded == 0)
  {
    offset_pio0 = pio_add_program(pio0, &step_gen_program);

#if MAX_JOINT > 4
    offset_pio1_gen = pio_add_program(pio1, &step_gen_program);
#endif
#if NUM_FEEDBACK > 0
    offset_pio1_count = pio_add_program(pio1, &step_count_program);
#endif

    /* Claim step_gen SMs: joints 0-3 on PIO0, joints 4+ on PIO1. */
    for(int8_t a = 0; a < MAX_JOINT && a < 4; a++) {
      joint_state[a].sm_gen = pio_claim_unused_sm(pio0, true);
    }
    for(int8_t a = 4; a < MAX_JOINT; a++) {
      joint_state[a].sm_gen = pio_claim_unused_sm(pio1, true);
    }

    /* Claim step_count SMs on PIO1 for the first NUM_FEEDBACK joints. */
    for(int8_t a = 0; a < NUM_FEEDBACK; a++) {
      joint_state[a].sm_count = pio_claim_unused_sm(pio1, true);
    }

    programs_loaded = 1;
  }

  /* Initialise the step_gen state machine for this joint. */
  pio_sm_set_enabled(JOINT_PIO(joint), joint_state[joint].sm_gen, false);
  step_gen_program_init(JOINT_PIO(joint), joint_state[joint].sm_gen,
                        JOINT_GEN_OFFSET(joint), io_pos_step, io_pos_dir);
  pio_sm_set_enabled(JOINT_PIO(joint), joint_state[joint].sm_gen, true);

  if(joint_state[joint].sm_gen != joint % 4) {
    printf("ERROR: Incorrect PIO initialization order for step_gen. joint: %u  sm_gen[joint]: %u",
        joint, joint_state[joint].sm_gen);
  }

  /* Initialise the step_count state machine for joints that have feedback. */
  if(joint < NUM_FEEDBACK) {
    pio_sm_set_enabled(pio1, joint_state[joint].sm_count, false);
    step_count_program_init(pio1, joint_state[joint].sm_count,
                            offset_pio1_count, io_pos_step, io_pos_dir);
    pio_sm_set_enabled(pio1, joint_state[joint].sm_count, true);

    /* step_count SMs are claimed after step_gen SMs on PIO1; expected index
     * is joint + number of step_gen SMs already on PIO1. */
    uint32_t expected_sm_count = joint + (MAX_JOINT > 4 ? MAX_JOINT - 4 : 0);
    if(joint_state[joint].sm_count != expected_sm_count) {
      printf("ERROR: Incorrect PIO init order for step_count. joint: %u  sm_count: %u  expected: %u",
          joint, joint_state[joint].sm_count, expected_sm_count);
    }
  }

  joint_state[joint].init_done = true;
}

/* Read the latest feedback position from PIO1's RX FIFO.
 * Snapshots the FIFO level, then drains exactly that many entries, keeping
 * only the last.  Intermediate values are discarded — only the current
 * position matters.  Returns current_pos unchanged if the FIFO is empty. */
int32_t drain_rx_fifo(uint32_t sm, int32_t current_pos) {
    uint8_t fifo_len = pio_sm_get_rx_fifo_level(pio1, sm);
    while (fifo_len > 0) {
        current_pos = pio_sm_get_blocking(pio1, sm);
        fifo_len--;
    }
    return current_pos;
}

/* Compute the PIO step-timer length in clock ticks.
 * Returns 0 only for step_count_q <= 0 (no motion).
 * Velocities below 1 step/period are capped at max_len so the step fits within
 * one servo period; plan_steps spaces them out via its accumulator.
 * max_vel_q <= 0 means "no max-velocity configured yet"; min_len clamping is
 * skipped so the default config does not block stepping before the first
 * MSG_SET_JOINT_CONFIG packet arrives. */
int32_t calculate_step_len(int32_t step_count_q, int32_t period_ticks, int32_t max_vel_q) {
    if (step_count_q <= 0) {
        return 0;
    }
    /* Longest step_len that keeps a step within one servo period.
     * A step > 1 period blocks the FIFO for the following Core1 tick. */
    int32_t max_len = period_ticks / 2 - STEP_PIO_LEN_OVERHEAD;
    /* Size step_len for ceil(v) steps per period so that max_steps = ceil(v).
     * The Bresenham accumulator can then alternate floor(v)/ceil(v) to achieve
     * exactly v steps/period on average, including non-integer velocities. */
    int32_t v_ceil = (step_count_q + 65535) >> 16;
    int32_t len    = period_ticks / (2 * v_ceil) - STEP_PIO_LEN_OVERHEAD;
    if (len > max_len) len = max_len;
    /* Only enforce the velocity ceiling when the commanded v_ceil exceeds the
     * max-velocity ceiling. For in-range velocities (v_ceil <= v_ceil_max),
     * clamping would shrink step_len below what Bresenham needs to produce
     * v_ceil steps/period, preventing non-integer velocities from averaging
     * correctly and causing cumulative position error. */
    if (max_vel_q > 0) {
        int32_t v_ceil_max = (max_vel_q + 65535) >> 16;
        if (v_ceil > v_ceil_max) {
            /* Compute min_len from v_ceil_max (integer ceiling) not from max_vel_q
             * (fractional).  The fractional formula gives a min_len that only fits
             * floor(max_vel_q/65536) = v_ceil_max-1 steps, so the clamp itself would
             * reduce max_steps below v_ceil_max and re-introduce the deficit.
             * Using v_ceil_max guarantees max_steps == v_ceil_max after clamping. */
            int32_t min_len = period_ticks / (2 * v_ceil_max) - STEP_PIO_LEN_OVERHEAD;
            int32_t clamped = len < min_len ? min_len : len;
            return clamped > max_len ? max_len : clamped;
        }
    }
    return len;
}

/* Clamp velocity change to at most max_accel_q per period.
 * Returns velocity unchanged if max_accel_q <= 0 (no limiting). */
int32_t clamp_accel(int32_t velocity_q, int32_t last_velocity_q, int32_t max_accel_q) {
    if (max_accel_q <= 0) {
        return velocity_q;
    }
    int32_t delta = velocity_q - last_velocity_q;
    if (delta > max_accel_q) {
        return last_velocity_q + max_accel_q;
    }
    if (delta < -max_accel_q) {
        return last_velocity_q - max_accel_q;
    }
    return velocity_q;
}

/* Bresenham step scheduler.
 *
 * Accumulates fractional desired steps (velocity_q is Q16.16 steps/period) and
 * returns how many steps to issue this period.  Excess desired steps are
 * returned to the accumulator for the next period.
 *
 * max_steps = floor(period_ticks / step_period): the most complete steps the
 * PIO can produce in one period at the given step_len.  calculate_step_len
 * guarantees step_len <= period_ticks/2 - OVERHEAD, so max_steps >= 1
 * whenever step_len > 0.  step_len == 0 (no motion) gives max_steps = 0;
 * the accumulator is preserved so no desired steps are lost. */
int32_t plan_steps(int32_t velocity_q, uint8_t joint,
                   int32_t period_ticks, int32_t step_len) {
    joint_state[joint].step_accumulator_q += abs(velocity_q);
    int32_t n_steps_desired = joint_state[joint].step_accumulator_q >> 16;
    joint_state[joint].step_accumulator_q -= n_steps_desired << 16;

    int32_t step_period = 2 * (step_len + STEP_PIO_LEN_OVERHEAD);
    int32_t max_steps = (step_len > 0) ? period_ticks / step_period : 0;

    int32_t n_steps = n_steps_desired < max_steps ? n_steps_desired : max_steps;
    joint_state[joint].step_accumulator_q += (n_steps_desired - n_steps) << 16;

    return n_steps;
}

/* Write a packed step command to the joint's step_gen TX FIFO if it is empty.
 * Encoding: lower bit = direction, upper bits = half-period in ticks.
 * When step_len_ticks > 0 the passed direction is cached; when 0 the cached
 * direction is reused so the DIR pin does not toggle unnecessarily.
 *
 * The PIO auto-repeats using stale x when the TX FIFO is empty (the `nop [3]`
 * path falls through to `data_acquired:`).  After firing its intended steps the
 * PIO idles until Core1 writes the next period's command, but if steps finish
 * before that write the stale x fires a spurious extra step.
 *
 * The sub-1-step Bresenham path produces n_steps of 0 or 1 only (max_steps==1
 * when abs(velocity_q) < 65536).  For n_steps==1 a stop word follows the step
 * word so the PIO does not spuriously re-fire via stale-x mid-period:
 *   n_steps == 0: [stop_word]          — idle, step_len=0
 *   n_steps == 1: [step_word, stop_word]
 *
 * Used only by the sub-1-step Bresenham path (commit_steps below).
 * The ≥1-step/period continuous path uses issue_pio_steps() instead. */
static void issue_pio_step(uint32_t joint, int32_t step_len_ticks, uint32_t direction,
                           int32_t n_steps) {
    if (!pio_sm_is_tx_fifo_empty(JOINT_PIO(joint), joint_state[joint].sm_gen)) {
        return;
    }
    if (step_len_ticks > 0) {
        joint_state[joint].last_direction = direction;
    }
    uint32_t step_word = ((uint32_t)step_len_ticks << 1) | joint_state[joint].last_direction;
    uint32_t stop_word = joint_state[joint].last_direction;
    pio_sm_put(JOINT_PIO(joint), joint_state[joint].sm_gen, step_word);
    if (n_steps >= 1) {
        pio_sm_put(JOINT_PIO(joint), joint_state[joint].sm_gen, stop_word);
    }
}

/* Write a single step word to the joint's step_gen TX FIFO for continuous mode.
 * No stop word is appended: the PIO auto-repeats via stale-x across period
 * boundaries, producing a uniform step rate at the commanded velocity. */
static void issue_pio_steps(uint32_t joint, int32_t step_len_ticks,
                                      uint32_t direction) {
    if (!pio_sm_is_tx_fifo_empty(JOINT_PIO(joint), joint_state[joint].sm_gen)) {
        return;
    }
    if (step_len_ticks > 0) {
        joint_state[joint].last_direction = direction;
    }
    uint32_t step_word = ((uint32_t)step_len_ticks << 1) | joint_state[joint].last_direction;
    pio_sm_put(JOINT_PIO(joint), joint_state[joint].sm_gen, step_word);
}

/* Compute the commanded velocity (steps/s) for this period.
 *
 * In position mode: vel_ff + Kp*error, with a 1-step dead zone.
 * The correction is capped to sqrt(2*max_accel*|error|) steps/s so the motor
 * can always decelerate to rest within the remaining error distance (bang-bang
 * stopping profile).  Without this cap, large errors after emergency decel
 * produce a correction that saturates clamp_accel every period — limit cycle.
 * Returns 0.0 when disabled or no new Core0 data (underrun/network loss). */
double compute_velocity_cmd(
    uint8_t  cmd_type,
    double   velocity_requested,
    double   abs_pos_requested,
    int32_t  abs_pos_achieved,
    uint8_t  enabled,
    uint32_t updated,
    uint32_t update_period_us,
    double   max_accel)
{
  if (!enabled || updated == 0) {
    return 0.0;
  }
  if (cmd_type == JOINT_CMD_POSITION) {
    double error_steps = abs_pos_requested - (double)abs_pos_achieved;
    double correction  = 0.0;
    if (error_steps >= 1.0 || error_steps <= -1.0) {
      correction = error_steps * (1.0e6 / (double)update_period_us) * 0.5;
      if (max_accel > 0.0) {
        double max_correction = sqrt(2.0 * max_accel * fabs(error_steps));
        if (correction >  max_correction) correction =  max_correction;
        if (correction < -max_correction) correction = -max_correction;
      }
    }
    velocity_requested += correction;
  } else {
    /* Velocity mode: gentle position correction to prevent drift accumulation.
     * Pure velocity mode has no feedback — any systematic step-rate undershoot
     * (e.g. from update_period_us bias or dropped periods) accumulates without
     * bound.  Kp = 0.01× of position-mode gain limits steady-state lag to
     * ~100× the per-period undershoot (E = U/Kp_vel = U/0.01 vs U/0.5 for
     * position mode) without fighting the trajectory planner. */
    double error_steps = abs_pos_requested - (double)abs_pos_achieved;
    if (error_steps >= 1.0 || error_steps <= -1.0) {
      velocity_requested += error_steps * (1.0e6 / (double)update_period_us) * 0.01;
    }
  }
  return velocity_requested;
}

/* Fixed-point dynamics for one servo period.  All velocity/accel values are
 * Q16.16 steps/period; period_ticks is in RP2040 clock cycles. */
typedef struct {
    int32_t velocity_q;
    int32_t vel_ff_q;
    int32_t max_vel_q;
    int32_t max_accel_q;
    int32_t clamp_accel_q;
    int32_t period_ticks;
} joint_dynamics_q_t;

/* Convert double velocity/accel inputs to fixed-point for one servo period.
 * Use the EMA-measured inter-packet interval so crystal disagreement between
 * host and RP is automatically tracked.  VEL_HEADROOM on max_vel_q gives the
 * correction term room to act at full speed even when update_period_us is
 * biased slightly above SERVO_PERIOD_US by jitter. */
static joint_dynamics_q_t dynamics_to_fixed(
    double velocity_requested, double vel_ff,
    double max_velocity, double max_accel,
    uint32_t update_period_us)
{
    double  period_s    = (double)update_period_us * 1e-6;
    int32_t max_accel_q = (int32_t)(max_accel * period_s * period_s * 65536.0);
    return (joint_dynamics_q_t){
        .velocity_q    = (int32_t)((velocity_requested / (double)update_period_us) * 65536.0),
        .vel_ff_q      = (int32_t)((vel_ff / (double)update_period_us) * 65536.0),
        .max_vel_q     = (int32_t)((max_velocity / (double)update_period_us) * 65536.0 * VEL_HEADROOM),
        .max_accel_q   = max_accel_q,
        .clamp_accel_q = (int32_t)(max_accel_q * ACCEL_HEADROOM),
        .period_ticks  = (int32_t)((int64_t)update_period_us * RP2040_CLOCK_MHZ),
    };
}

/* Handle joint enable/disable transitions.
 * On rising edge: initialise PIO and snap last_velocity_q to the commanded
 * velocity when stopped, so clamp_accel does not ramp from zero when LinuxCNC
 * is already moving.  When last_velocity_q is non-zero the joint is
 * mid-deceleration (network reconnect); preserve it so clamp_accel limits the
 * change and avoids a jitter step. */
static void handle_enable_transition(uint8_t joint, uint8_t enabled, int32_t velocity_q) {
    if (enabled == joint_state[joint].last_enabled)
        return;
    joint_state[joint].last_enabled = enabled;
    if (enabled) {
        printf("J%u enab\n", joint);
        init_pio(joint);
        if (joint_state[joint].last_velocity_q == 0)
            joint_state[joint].last_velocity_q = velocity_q;
    } else {
        printf("J%u disab\n", joint);
    }
}

/* Stopping-profile cap: ensure the motor can decelerate to vel_ff_q within the
 * remaining distance to target.  Formula: |v| ≤ vel_ff + sqrt(2·a·|error|).
 * When vel_ff=0 this is the classic bang-bang stopping guarantee.
 * When vel_ff>0 (active jog or G-code move) the extra headroom prevents the
 * cap from interfering with normal tracking.
 * Uses floating-point error (not truncated integer) so the cap stays active
 * during the final sub-1-step approach and prevents overshoot.
 * When the cap fires, clears the Bresenham accumulator so its residual
 * fraction cannot drain into an extra overshoot step. */
static int32_t apply_stopping_cap(
    uint8_t cmd_type, uint8_t enabled, uint32_t updated,
    int32_t velocity_q, int32_t vel_ff_q, int32_t max_accel_q,
    double abs_pos_requested, int32_t abs_pos_achieved,
    int32_t *step_accumulator_q)
{
    if (!(cmd_type == JOINT_CMD_POSITION && max_accel_q > 0 && enabled && updated))
        return velocity_q;
    double err_f = abs_pos_requested - (double)abs_pos_achieved;
    if (err_f == 0.0 || (velocity_q > 0) != (err_f > 0.0))
        return velocity_q;
    double sqrt_d = sqrt(2.0 * (double)max_accel_q * fabs(err_f) * (double)Q16_ONE);
    int32_t sqrt_term = (sqrt_d > (double)INT32_MAX) ? INT32_MAX : (int32_t)sqrt_d;
    if (err_f > 0.0) {
        int32_t cap_v = vel_ff_q + sqrt_term;
        if (velocity_q > cap_v) { velocity_q = cap_v; *step_accumulator_q = 0; }
    } else {
        int32_t floor_v = vel_ff_q - sqrt_term;
        if (velocity_q < floor_v) { velocity_q = floor_v; *step_accumulator_q = 0; }
    }
    return velocity_q;
}

/* At-target snap: when in the dead zone with no feedforward, zero velocity and
 * accumulator immediately.  Without this, clamp_accel leaves residual velocity
 * after the final correction step; the Bresenham accumulator drains it into an
 * overshoot step. */
static int32_t apply_at_target_snap(
    uint8_t cmd_type, uint8_t enabled, uint32_t updated,
    int32_t velocity_q, int32_t vel_ff_q,
    double abs_pos_requested, int32_t abs_pos_achieved,
    int32_t *step_accumulator_q)
{
    if (!(cmd_type == JOINT_CMD_POSITION && vel_ff_q == 0 && enabled && updated))
        return velocity_q;
    if (fabs(abs_pos_requested - (double)abs_pos_achieved) < 0.5) {
        velocity_q = 0;
        *step_accumulator_q = 0;
    }
    return velocity_q;
}

/* Translate velocity_q → PIO step command and update abs_pos_achieved for
 * open-loop joints (no feedback counter).
 *
 * Two modes, selected by abs(plan_vel_q):
 *
 * Sub-1-step (<65536, i.e. <1 step/period): Bresenham accumulator schedules 0
 * or 1 step per period.  A stop word follows each non-zero command so the PIO
 * does not spuriously re-fire after the step completes mid-period.  Feedforward
 * path (in_ff_path) drives Bresenham with vel_ff_q so position-correction
 * spikes do not disrupt inter-step timing.
 *
 * Continuous (>=65536, i.e. >=1 step/period): step_len is derived directly from
 * velocity so steps are evenly spaced in time.  One step_word is written per
 * period; no stop word — the PIO fires continuously via stale-x across period
 * boundaries.  Feedback joints get position from the step_count PIO (already
 * read by drain_rx_fifo before this call).  Open-loop joints track position via
 * uncapped Bresenham (step_accumulator_q) so the long-run average is exact.
 *
 * Returns vel_ff_q as velocity_achieved so vel-fb tracks the commanded
 * trajectory velocity in both modes.  When vel_ff_q == 0 (pure position mode,
 * no jog), velocity_achieved is zero even if a correction step fires. */
static int32_t commit_steps(
    uint8_t joint, const joint_dynamics_q_t *dq,
    int32_t velocity_q, int32_t *abs_pos_achieved)
{
    int32_t step_count_q = abs(velocity_q);
    /* Feedforward path: use vel_ff_q for sub-1-step scheduling when:
     * (a) step_count_q is sub-1-step — vel_ff_q governs pacing, or
     * (b) correction has sign-flipped velocity_q (near the 2-step boundary or
     *     a large correction spike has reversed sign at higher velocities).
     * At >=1 step/period without a sign flip, plan_vel_q = velocity_q directly. */
    int in_ff_path = abs(dq->vel_ff_q) > 0 && step_count_q > 0 &&
                     (step_count_q <= Q16_ONE ||
                      (abs(dq->vel_ff_q) <= 2*Q16_ONE &&
                       (dq->vel_ff_q > 0 ? velocity_q < 0 : velocity_q > 0)));
    int32_t plan_vel_q = in_ff_path ? dq->vel_ff_q : velocity_q;
    uint32_t direction = (plan_vel_q > 0);

    if (abs(plan_vel_q) >= Q16_ONE) {
        /* Continuous mode: >=1 step/period.
         * step_period = period_ticks * 65536 / abs(plan_vel_q)
         * step_len    = step_period/2 - overhead
         *             = period_ticks * 32768 / abs(plan_vel_q) - overhead
         * int64 prevents overflow: 133000 * 32768 > INT32_MAX. */
        int32_t step_len = (int32_t)((int64_t)dq->period_ticks * 32768 / abs(plan_vel_q))
                           - STEP_PIO_LEN_OVERHEAD;
        if (step_len < 0) step_len = 0;
        issue_pio_steps(joint, step_len, direction);
        if (joint >= NUM_FEEDBACK && joint < MAX_JOINT) {
            /* Open-loop: uncapped Bresenham tracks fractional step accumulation.
             * Mathematically equivalent to period_ticks / step_period per period.
             * joint < MAX_JOINT guard: compiler needs an explicit upper bound to
             * prove joint_state[joint] is in-bounds (do_steps guarantees it). */
            joint_state[joint].step_accumulator_q += abs(plan_vel_q);
            int32_t n_steps_f = joint_state[joint].step_accumulator_q >> 16;
            joint_state[joint].step_accumulator_q -= n_steps_f << 16;
            *abs_pos_achieved += (direction ? 1 : -1) * n_steps_f;
        }
    } else {
        /* Sub-1-step mode: Bresenham + stop word.
         * step_len_ceil sized for plan_vel_q (not step_count_q) so the
         * accumulator doesn't build a backlog during acceleration ramp-up. */
        int32_t step_len_ceil = calculate_step_len(abs(plan_vel_q), dq->period_ticks,
                                                   dq->max_vel_q);
        int32_t n_steps = plan_steps(plan_vel_q, joint, dq->period_ticks, step_len_ceil);
        int32_t step_len_ticks = calculate_step_len(n_steps * Q16_ONE, dq->period_ticks,
                                                    dq->max_vel_q);
        if (joint >= NUM_FEEDBACK)
            *abs_pos_achieved += (direction ? 1 : -1) * n_steps;
        issue_pio_step(joint, step_len_ticks, direction, n_steps);
    }

    return dq->vel_ff_q;
}

/* Generate step counts and send to PIOs. */
uint8_t do_steps(const uint8_t joint) {
  uint32_t update_period_us = get_period();

  uint8_t enabled;
  int32_t abs_pos_achieved = 0;
  double velocity_requested;
  double abs_pos_requested;
  double max_velocity;
  double max_accel;
  uint8_t cmd_type;
  int32_t velocity_achieved = 0;
  uint32_t updated = get_joint_config(
      joint, CORE1, &enabled, NULL, NULL,
      &velocity_requested, &abs_pos_requested, &abs_pos_achieved,
      &max_velocity, &max_accel, NULL, &cmd_type, NULL, NULL);

  if (update_period_us == 0) {
    /* Period unknown: can't compute step timing. */
    issue_pio_step(joint, 0, 0, 0);
    return 0;
  }
  if (updated == 0 && joint_state[joint].last_velocity_q == 0) {
    /* No new Core0 data and already at rest: nothing to compute.
     * last_velocity_q == 0 implies the joint was stationary last period, so
     * the step_count PIO produced no output and its FIFO is empty — safe to
     * skip drain_rx_fifo. */
    issue_pio_step(joint, 0, 0, 0);
    return 0;
  }

  if (joint < NUM_FEEDBACK) {
    /* Read step_count FIFO before computing velocity correction so
     * compute_velocity_cmd sees the current-period position, not the
     * stale value written to config at the end of the previous period. */
    abs_pos_achieved = drain_rx_fifo(joint_state[joint].sm_count, abs_pos_achieved);
  }

  double vel_ff = velocity_requested;  /* save before correction is added */
  velocity_requested = compute_velocity_cmd(
      cmd_type, velocity_requested, abs_pos_requested, abs_pos_achieved,
      enabled, updated, update_period_us, max_accel);

  joint_dynamics_q_t dq = dynamics_to_fixed(
      velocity_requested, vel_ff, max_velocity, max_accel, update_period_us);

  handle_enable_transition(joint, enabled, dq.velocity_q);

  int32_t velocity_q = clamp_accel(
      dq.velocity_q, joint_state[joint].last_velocity_q, dq.clamp_accel_q);

  velocity_q = apply_stopping_cap(
      cmd_type, enabled, updated,
      velocity_q, dq.vel_ff_q, dq.max_accel_q,
      abs_pos_requested, abs_pos_achieved,
      &joint_state[joint].step_accumulator_q);

  velocity_q = apply_at_target_snap(
      cmd_type, enabled, updated,
      velocity_q, dq.vel_ff_q,
      abs_pos_requested, abs_pos_achieved,
      &joint_state[joint].step_accumulator_q);

  joint_state[joint].last_velocity_q = velocity_q;

  if (!enabled && velocity_q == 0) {
    /* Fully decelerated: issue hard stop and keep pos_fb current while disabled.
     * abs_pos_achieved already reflects any in-flight steps drained above. */
    issue_pio_step(joint, 0, 0, 0);
    velocity_achieved = 0;
    update_joint_config(
        joint, CORE1, NULL, NULL, NULL, NULL, NULL,
        &abs_pos_achieved, NULL, NULL, &velocity_achieved, NULL, NULL, NULL);
    joint_state[joint].last_pos_achieved = abs_pos_achieved;
    return 0;
  }

  /* Report Q16.16 internal velocity so the driver can detect velocity_q==0
   * exactly.  Integer step-delta aliased to 0 at low speed (<1 step/period),
   * causing premature network-recovery detection on the driver side. */
  velocity_achieved = commit_steps(joint, &dq, velocity_q, &abs_pos_achieved);

  update_joint_config(
      joint, CORE1, NULL, NULL, NULL, NULL, NULL,
      &abs_pos_achieved, NULL, NULL, &velocity_achieved, NULL, NULL, NULL);
  joint_state[joint].last_pos_achieved = abs_pos_achieved;

  return enabled ? updated : 0;
}

#ifdef BUILD_TESTS
void pio_reset_for_test(void) {
    memset(joint_state, 0, sizeof(joint_state));
    offset_pio0       = 0;
    offset_pio1_gen   = 0;
    offset_pio1_count = 0;
    programs_loaded   = 0;
}
#endif  // BUILD_TESTS
