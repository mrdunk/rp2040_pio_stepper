#include <stddef.h>

size_t pio0;
size_t pio1;

void step_gen_program(size_t pio) {}
void step_count_program(size_t pio ) {}


void step_count_program_init(
    size_t pio, size_t sm, size_t offset, size_t pin_step, size_t pin_direction
) {}

void step_gen_program_init(
    size_t pio, size_t sm, size_t offset, size_t pin_step, size_t pin_direction
) {}

void pio_sm_set_enabled (size_t pio, size_t sm, int enabled) {}

size_t pio_add_program(size_t pio, const void* program) {return 0;}

size_t pio_sm_get_blocking(size_t pio, size_t sm) {return 1;}

int pio_claim_unused_sm(size_t pio, int sm) {return 0;}

int pio_sm_is_tx_fifo_full(size_t pio, size_t sm) {return 0;}

void pio_sm_put(size_t, size_t pio, size_t sm) {}

int pio_sm_is_tx_fifo_empty(size_t pio, size_t sm) {return 0;}

size_t pio_sm_get_rx_fifo_level(size_t pio, size_t sm) {return 1;}

void pio_sm_clear_fifos(size_t pio, size_t sm) {}

void multicore_launch_core1(void(*entry)(void)) {}
