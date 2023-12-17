#include "buffer.h"

/* checksum(...) must go in it's own file rather than where it is called from  to
 * allow it to be mocked for the calling functions. */

uint16_t checksum(uint16_t checksum, void* new_data, uint16_t new_data_len) {
    for(uint16_t index = 0; index < NW_BUF_LEN / 2; index++) {
        checksum += *((uint16_t*)new_data + index);
    }
    return checksum;
}

