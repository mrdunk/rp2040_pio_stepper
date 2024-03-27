#ifndef PIO__H
#define PIO__H

#include <stdint.h>

/* Force PIO to stop a joint. */
void stop_joint(const uint32_t joint);

/* Initialize a pair of PIO programmes.
 * One for step generation on pio0 and one for counting said steps on pio1.
 */
void init_pio(const uint32_t joint);

/* Generate step counts and send to PIOs. */
uint8_t do_steps(const uint8_t joint, const uint32_t update_time_us);

#endif  // PIO__H
