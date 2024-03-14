#ifndef PIO__H
#define PIO__H

#include <stdint.h>

/* Initialize a pair of PIO programmes.
 * One for step generation on pio0 and one for counting said steps on pio1.
 */
void init_pio(const uint32_t joint);

/* Generate step counts and send to PIOs. */
uint8_t do_steps(const uint8_t joint, const uint32_t update_time_us);

/* One bit per axis to indicate whether a corresponding PIO has been updated */
extern uint8_t axis_updated_bitmask;

#endif  // PIO__H
