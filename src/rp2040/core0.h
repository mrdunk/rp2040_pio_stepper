#ifndef CORE0__H
#define CORE0__H

#include "buffer.h"

size_t process_received_buffer(
    struct NWBuffer* rx_buf, struct NWBuffer* tx_buf, uint8_t* received_count, uint16_t expected_length);

void core0_main();


#endif  // CORE0__H

