#ifndef DEBUG_METHODS__H
#define DEBUG_METHODS__H

#include <stdlib.h>

/* Displays a buffer to be sent via UDP.*/
void display_tx_data(void* packet, size_t packet_size);

/* Decodes and displays a raw buffer that has bee received via UDP. */
void display_rx_data(char* buf);


#endif  // DEBUG_METHODS__H
