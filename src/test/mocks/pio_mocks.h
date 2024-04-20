#ifndef MOCKS_PIO_MOCKS__H
#define MOCKS_PIO_MOCKS__H

extern size_t pio0;
extern size_t pio1;

void step_gen_program(size_t);
void step_count_program(size_t);

void step_gen_program_init(size_t, size_t, size_t, size_t, size_t);
void step_count_program_init(size_t, size_t, size_t, size_t, size_t);
size_t pio_add_program(size_t, const void*);
int pio_claim_unused_sm(size_t, int);
void pio_sm_set_enabled (size_t pio, size_t sm, int enabled);
int pio_sm_is_tx_fifo_full(size_t, size_t);
void pio_sm_put(size_t, size_t, size_t);
int pio_sm_is_tx_fifo_empty(size_t, size_t);
size_t pio_sm_get_rx_fifo_level(size_t, size_t);
size_t pio_sm_get_blocking(size_t, size_t);


#endif  // MOCKS_PIO_MOCKS__H

