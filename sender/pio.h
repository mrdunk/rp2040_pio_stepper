#ifndef PIO__H
#define PIO__H

#include <stdint.h>

void init_pio(
    const uint32_t axis,
    const uint32_t pin_step,
    const uint32_t pin_direction
    );

uint8_t do_steps(const uint8_t axis, const uint32_t update_time_us);

#endif  // PIO__H
