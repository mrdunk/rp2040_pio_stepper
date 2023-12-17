#include "buffer.h"

/* checksum(...) must go in it's own file rather than where it is called from  to
 * allow it to be mocked for the calling functions. */

uint16_t checksum(uint16_t val_in, void* data, uint16_t data_len) {
    for(uint16_t index = 0; index < data_len / 2; index++) {
        val_in += *((uint16_t*)data + index);
    }
    return val_in;
}

