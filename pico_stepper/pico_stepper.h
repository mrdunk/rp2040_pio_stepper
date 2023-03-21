#ifndef PICO_STEPPER__H
#define PICO_STEPPER__H

#define MAX_AXIS 8

#define MESSAGE_SECTIONS 10

#define LED_PIN 25
//#define CORE1_STACK_SIZE 4096  // TODO: I have no idea how big this needs to be.
#define CORE1_STACK_SIZE 0x4000  // TODO: I have no idea how big this needs to be.


#define CMND_SET_POS           1
#define CMND_SET_MIN_STEP_LEN  2
#define CMND_SET_DESIRED_POS   3

struct AxisUpdate {
  uint8_t padding;
  uint8_t command;
  uint8_t updated;
  uint8_t axis;
  int32_t value;
};

struct ConfigAxis {
  uint32_t abs_pos;             // In steps. Default value is UINT_MAX / 2.
  uint32_t min_step_len_ticks;
  int32_t velocity;             // Steps per update_time_us.
};

struct ConfigGlobal {
  // Both the following are really the same thing, portrayed different ways.
  // Update using set_global_update_rate(...).
  uint32_t update_rate;
  uint32_t update_time_us;  // (1,000,000) / update_rate
  uint32_t update_time_ticks;  // clock_speed / update_rate

  struct ConfigAxis axis[MAX_AXIS];
};

void init_core1();

/* Initialise an rp2040 PIO to drive stepper motors.
 *
 * Arguments:
 *  stepper: A stepper motor index in the range 0-7.
 *  pin_step: The STEP rp2040 output pin.
 *  pin_direction: The DIR rp2040 output pin.
 */
void init_pio(
    uint32_t stepper,
    uint32_t pin_step,
    uint32_t pin_direction);

/* Get the step count of a stepper motor in steps. */
uint32_t get_absolute_position(uint32_t stepper);

/* Perform a set number of steps.
 *
 * Arguments:
 *  stepper: A stepper motor index in the range 0-7.
 *  position_diff: The number of steps to be applied in the next config.update_time_us.
 */
uint32_t send_pio_steps(
    uint32_t stepper,
    int32_t position_diff);

/* Perform a specified number of steps within the globally configured update_time_us.
 *
 * Arguments:
 *  stepper: A stepper motor index in the range 0-7.
 *  position_diff: Number of steps to increase (+ive) or decrease (-ive) the position by.
 *
 * Returns:
 *  The position of the stepper motor will be in after steps have been performed.
 */
uint32_t set_relative_position(
    uint32_t stepper,
    int position_diff);

/* Reach a specified step count in the globally configured update_time_us.
 *
 * Arguments:
 *  stepper: A stepper motor index in the range 0-7.
 *  position: The target position in steps.
 *
 * Returns:
 *  The position of the stepper motor will be in after steps have been performed.
 */
uint32_t set_absolute_position(
    uint32_t stepper,
    uint32_t new_position);

/* Set the update rate for position commands that do not include a time window.
 *
 * Arguments:
 *   update_rate: The update rate in Hz.
 *
 * Returns the stored value. Could theoretically be different to that asked due
 *   to rounding errors when converting to config.update_time_us.
 */
uint32_t set_global_update_rate(uint32_t update_rate);

/* Gets summary of global config. */
void get_global_config(
    uint8_t* msg_human,
    size_t msg_human_len_max,
    uint8_t* msg_machine,
    size_t* msg_machine_len,
    size_t msg_machine_len_max);

/* Gets summary of specified axis config.*/
void get_axis_config(
    const uint32_t axis,
    uint8_t* msg_human,
    size_t msg_human_len_max,
    uint8_t* msg_machine,
    size_t* msg_machine_len,
    size_t msg_machine_len_max);

/* Gets the abs_pos paramiter for an axis.*/
void get_axis_pos(
    const uint32_t axis,
    uint8_t* msg_human,
    size_t msg_human_len_max,
    uint8_t* msg_machine,
    size_t* msg_machine_len,
    size_t msg_machine_len_max);

/* Convert a time in us to PIO ticks. */
inline static uint32_t time_to_ticks(const uint32_t time_us);

#endif  // PICO_STEPPER__H
