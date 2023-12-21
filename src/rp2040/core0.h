#ifndef CORE0__H
#define CORE0__H

#include "buffer.h"

size_t process_received_buffer( struct NWBuffer* rx_buf, uint8_t* tx_buf, uint8_t* received_count);

void core0_main();


#endif  // CORE0__H

