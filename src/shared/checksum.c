#include "buffer.h"

/* checksum(...) must go in it's own file rather than where it is called from  to
 * allow it to be mocked for the calling functions. */

uint16_t checksum(uint16_t checksum_val, size_t pos_in, size_t pos_end, void* data) {
    uint16_t mod;
    for(size_t index = pos_in; index < pos_end; index++) {
        mod = ((uint8_t*)data)[index];
        if(index % 2) {
           mod = mod << 8;
        }
        checksum_val += mod;
    }

    return checksum_val;
}
