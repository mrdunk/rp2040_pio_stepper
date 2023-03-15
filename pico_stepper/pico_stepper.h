#ifndef PICO_STEPPER__H
#define PICO_STEPPER__H

/* Initialise an rp2040 PIO.
 *
 * Arguments:
 *  stepper: A stepper motor index in the range 0-8.
 *  pin_step: The STEP rp2040 output pin.
 *  pin_direction: The DIR rp2040 output pin.
 */
void init_pio(
    uint stepper,
    uint pin_step,
    uint pin_direction);

/* Perform some steps.
 *
 * Arguments:
 *  stepper: A stepper motor index in the range 0-8.
 *  time_slice_us: Length of time to perform steps for.
 *  freq: Frequency of steps.
 *  direction: Forwards (>0) or backwards (==0).
 */
void send_pio_steps(
    uint stepper,
    uint time_slice_us,
    uint freq,
    uint direction);

#endif  // PICO_STEPPER__H
