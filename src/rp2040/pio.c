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

/* Fixed overhead per step cycle in step_gen2 (pico_stepper.pio):
 *   HIGH phase:     1 + (high_count+1)×16 cycles  (mov y,isr + (high_count+1)×(nop[7]+jmp[7] y--))
 *   non-loop:      14 cycles  (FIFO-check path 8 + stop-guard 1 + LOW-setup×2 + LOW-end×2 + HIGH-setup 1)
 *   total:         14 + (high_count+1)×16  — always even (both terms even)
 * STEP_PIO_LEN_OVERHEAD = total/2, subtracted from desired_half_period to get step_low_half:
 *   step_low_half = period_ticks * 32768 / abs(plan_vel_q) - STEP_PIO_LEN_OVERHEAD
 *   step_period   = 2*(step_low_half + STEP_PIO_LEN_OVERHEAD) */
#define STEP_PIO_HIGH_COUNT_DEFAULT   20   /* 1 + 21×16 = 337 cycles ≈ 2.53µs */
#define STEP_PIO_LEN_OVERHEAD         ((14 + (STEP_PIO_HIGH_COUNT_DEFAULT + 1) * 16) / 2)
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
    uint32_t last_commanded_direction;
    uint32_t high_count;    /* HIGH-phase loop iterations; default STEP_PIO_HIGH_COUNT_DEFAULT */
} JointPioState;

static JointPioState joint_state[MAX_JOINT];
static uint32_t offset_pio0       = 0;  /* step_gen on PIO0 */
static uint32_t offset_pio1_gen   = 0;  /* step_gen on PIO1 (MAX_JOINT > 4 only) */
static uint32_t offset_pio1_count = 0;  /* step_count on PIO1 (NUM_FEEDBACK > 0 only) */
static uint8_t  programs_loaded   = 0;
static uint8_t  dir_setup_violation_bits = 0;  /* bit N set when joint N violated DIR setup time */

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

  if(programs_loaded == 0)
  {
    offset_pio0 = pio_add_program(pio0, &step_gen2_program);

#if MAX_JOINT > 4
    offset_pio1_gen = pio_add_program(pio1, &step_gen2_program);
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
  step_gen2_program_init(JOINT_PIO(joint), joint_state[joint].sm_gen,
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

  /* Apply output polarity inversion after all pio_gpio_init() calls — those
   * zero the GPIO CTRL register (via gpio_set_function), wiping any earlier
   * outover setting. */
  gpio_set_outover(io_pos_step, invert_step ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);
  gpio_set_outover(io_pos_dir,  invert_dir  ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);

  if (joint_state[joint].high_count == 0)
    joint_state[joint].high_count = STEP_PIO_HIGH_COUNT_DEFAULT;
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


/* Write a stop word (step_len=0) to the TX FIFO if it is empty.
 * Halts the PIO without toggling the DIR pin. */
static void issue_stop_word(uint32_t joint) {
    if (pio_sm_is_tx_fifo_empty(JOINT_PIO(joint), joint_state[joint].sm_gen))
        pio_sm_put(JOINT_PIO(joint), joint_state[joint].sm_gen,
                   joint_state[joint].last_direction);
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
 * Sub-1-step (<Q16_ONE, i.e. <1 step/period): Bresenham accumulator schedules 0
 * or 1 step per period.  A stop word follows each non-zero command so the PIO
 * does not spuriously re-fire after the step completes mid-period.  Feedforward
 * path (in_ff_path) drives Bresenham with vel_ff_q so position-correction
 * spikes do not disrupt inter-step timing.
 *
 * Continuous (>=Q16_ONE, i.e. >=1 step/period): step_len is derived directly from
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

    // The following section made improvements at one time or another but i have
    // low confidence they they are all (any) still needed.
    // TODO(dunk): Experiment with removing some/all of these.
    // corr_spike is almost certainly redundant.
    int has_ff       = abs(dq->vel_ff_q) > 0 && step_count_q > 0; /* feedforward is active and there's motion */
    int vel_sub1step = step_count_q <= Q16_ONE;                    /* commanded velocity is below 1 step/period */
    int sign_flipped = (dq->vel_ff_q > 0) ? (velocity_q < 0) : (velocity_q > 0); /* correction has reversed the direction of velocity_q */
    int corr_spike   = abs(dq->vel_ff_q) <= 2*Q16_ONE && sign_flipped; /* sign flip near the 2-step boundary */
    int in_ff_path   = has_ff && (vel_sub1step || corr_spike);
    int32_t plan_vel_q = in_ff_path ? dq->vel_ff_q : velocity_q;

    uint32_t direction = (plan_vel_q > 0);
    int direction_changed = (plan_vel_q != 0) &&
                            (direction != joint_state[joint].last_commanded_direction);

    /* Bresenham accumulator — identical for both modes.
     * Continuous (>=1 step/period): n_steps is the full integer count.
     * Sub-1-step (<1 step/period): plan_vel_q < Q16_ONE so the accumulator
     * never reaches 2×Q16_ONE between drains; n_steps is naturally 0 or 1.
     * Reset on direction reversal so the prior direction's fractional accumulation
     * does not cause an early first step after the reversal.  Skipped when
     * plan_vel_q==0 so a brief stop preserves the fraction for same-direction resume. */
    if (plan_vel_q != 0) {
        if (direction != joint_state[joint].last_commanded_direction)
            joint_state[joint].step_accumulator_q = 0;
        joint_state[joint].last_commanded_direction = direction;
    }
    joint_state[joint].step_accumulator_q += (uint32_t)abs(plan_vel_q);
    int32_t n_steps = (int32_t)(joint_state[joint].step_accumulator_q >> 16);
    joint_state[joint].step_accumulator_q -= (uint32_t)(n_steps << 16);

    if (joint >= NUM_FEEDBACK)
        *abs_pos_achieved += (direction ? 1 : -1) * n_steps;

    if (!pio_sm_is_tx_fifo_empty(JOINT_PIO(joint), joint_state[joint].sm_gen))
        return dq->vel_ff_q;

    /* Encoding: lower bit = direction, upper bits = step_low_half in ticks.
     * step_low_half=0 encodes a stop word (PIO idles); direction bit is preserved
     * so the DIR pin does not toggle on idle writes.
     * step_period = 2*(step_low_half + STEP_PIO_LEN_OVERHEAD) */
    int32_t step_low_half;
    int sub1step;
    if (abs(plan_vel_q) >= Q16_ONE) {
        /* Continuous: step_low_half sized for plan_vel_q; PIO auto-repeats via
         * stale-x across period boundaries — no stop word needed.
         * int64 prevents overflow: 133000 * 32768 > INT32_MAX. */
        step_low_half = (int32_t)((int64_t)dq->period_ticks * 32768 / abs(plan_vel_q))
                        - STEP_PIO_LEN_OVERHEAD;
        if (step_low_half < 0) step_low_half = 0;
        sub1step = 0;
    } else {
        /* Sub-1-step: step occupies exactly period_ticks/2 PIO cycles total
         * (2*(period_ticks/4 - 175) + 350 = period_ticks/2).
         * Using period_ticks/2 here would make the step consume nearly the entire
         * period; the trailing stop word would still be in the FIFO when the next
         * timer fires, causing the !fifo_empty guard to spuriously drop the next step.
         * Halving step_low_half leaves ~half the period for the stop word to clear.
         * step_low_half=0 when n_steps=0 — the resulting step_word equals a stop word. */
        step_low_half = n_steps > 0 ? dq->period_ticks / 4 - STEP_PIO_LEN_OVERHEAD : 0;
        sub1step = 1;
    }

    if (direction_changed && step_low_half > 0 && step_low_half < DIR_SETUP_MIN_CYCLES)
        dir_setup_violation_bits |= (1u << joint);

    config.joint[joint].step_len_us = step_low_half > 0
        ? (uint16_t)((step_low_half + STEP_PIO_LEN_OVERHEAD) / RP2040_CLOCK_MHZ) : 0;

    if (step_low_half > 0) joint_state[joint].last_direction = direction;
    uint32_t high_count = joint_state[joint].high_count & 0x3F;
    uint32_t step_word  = (high_count << 25)
                        | (((uint32_t)step_low_half & 0xFFFFFF) << 1)
                        | joint_state[joint].last_direction;
    pio_sm_put(JOINT_PIO(joint), joint_state[joint].sm_gen, step_word);
    if (sub1step && n_steps >= 1) {
        /* Push stop word directly — do NOT use issue_stop_word() here.
         * issue_stop_word() guards on FIFO-empty, which fails immediately after
         * pushing step_word (FIFO has 1 entry).  On hardware the PIO is mid-step
         * and cannot drain the FIFO in the ~15 CPU cycles between push and check,
         * so the stop word would never be sent near 1 step/period, allowing the
         * PIO to stale-x repeat at 2× the commanded rate. */
        pio_sm_put(JOINT_PIO(joint), joint_state[joint].sm_gen,
                   joint_state[joint].last_direction);
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
    issue_stop_word(joint);
    return 0;
  }
  if (updated == 0 && joint_state[joint].last_velocity_q == 0) {
    /* No new Core0 data and already at rest: nothing to compute.
     * last_velocity_q == 0 implies the joint was stationary last period, so
     * the step_count PIO produced no output and its FIFO is empty — safe to
     * skip drain_rx_fifo. */
    issue_stop_word(joint);
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

  /* Clamp to max_velocity. Position-mode Kp corrections can exceed max_velocity
   * when max_accel=0 (no stopping cap) and error is large. Without this clamp,
   * high velocity_q produces step_low_half < 0 which is truncated to a stop word. */
  if (velocity_q >  dq.max_vel_q) velocity_q =  dq.max_vel_q;
  if (velocity_q < -dq.max_vel_q) velocity_q = -dq.max_vel_q;

  joint_state[joint].last_velocity_q = velocity_q;

  if (!enabled && velocity_q == 0) {
    /* Fully decelerated: issue hard stop and keep pos_fb current while disabled.
     * abs_pos_achieved already reflects any in-flight steps drained above. */
    issue_stop_word(joint);
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

void pio_set_step_high_count(uint32_t joint, uint32_t count) {
    if (joint >= MAX_JOINT) return;
    joint_state[joint].high_count = count & 0x3F;
}

uint8_t pio_get_and_clear_dir_setup_violations(void) {
    uint8_t v = dir_setup_violation_bits;
    dir_setup_violation_bits = 0;
    return v;
}

void pio_invalidate_all_joints(void) {
    for (uint8_t j = 0; j < MAX_JOINT; j++)
        joint_state[j].init_done = false;
}

#ifdef BUILD_TESTS
void pio_reset_for_test(void) {
    memset(joint_state, 0, sizeof(joint_state));
    offset_pio0              = 0;
    offset_pio1_gen          = 0;
    offset_pio1_count        = 0;
    programs_loaded          = 0;
    dir_setup_violation_bits = 0;
}
#endif  // BUILD_TESTS
