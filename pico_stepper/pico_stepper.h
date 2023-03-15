#ifndef PICO_STEPPER__H
#define PICO_STEPPER__H

/* Initialise an rp2040 PIO to drive stepper motors.
 *
 * Arguments:
 *  stepper: A stepper motor index in the range 0-7.
 *  pin_step: The STEP rp2040 output pin.
 *  pin_direction: The DIR rp2040 output pin.
 */
void init_pio(
    uint stepper,
    uint pin_step,
    uint pin_direction);

/* Get the step count of a stepper motor. */
uint get_absolute_position(uint stepper);

/* Perform some steps.
 *
 * Arguments:
 *  stepper: A stepper motor index in the range 0-7.
 *  step_count: How many steps.
 *  step_len_us: Step duration in us.
 *  direction: Forwards (>0) or backwards (==0).
 */
uint send_pio_steps(
    uint stepper,
    uint step_count,
    uint step_len_us,
    uint direction);

/* Perform some steps equal to a specified step count.
 *
 * Arguments:
 *  stepper: A stepper motor index in the range 0-7.
 *  position_diff: Number of steps to increase (+ive) or decrease (-ive) the position by.
 *  time_slice_us: The time in us to perform the steps.
 *
 * Returns:
 *  The position of the stepper motor after steps have been performed.
 */
uint set_relative_position(
    uint stepper,
    int position_diff,
    uint time_slice_us);

/* Perform some steps to reach a specified step count.
 *
 * Arguments:
 *  stepper: A stepper motor index in the range 0-7.
 *  position: The target position.
 *  time_slice_us: The time in us to perform the steps.
 *
 * Returns:
 *  The position of the stepper motor after steps have been performed.
 */
uint set_absolute_position(
    uint stepper,
    uint new_position,
    uint time_slice_us);


#endif  // PICO_STEPPER__H
