#ifndef BUFFER__H
#define BUFFER__H

#include <sys/types.h>
#include <stdint.h>
#include <string.h>

// Must be even number as the checksum is calculated on uint16_t chunks which
// account for 2 bytes.
#define NW_BUF_LEN 256

struct NWBuffer {
    uint16_t length;
    uint16_t checksum;
    uint8_t payload[NW_BUF_LEN];
};

size_t alligned32(size_t input);

size_t pack_nw_buff(struct NWBuffer* buffer, void* new_data, size_t new_data_len);

void* unpack_nw_buff(
    struct NWBuffer* buffer,
    size_t payload_offset,
    size_t* new_payload_offset,
    void* dest_container,
    size_t dest_container_len
);

size_t checkNWBuff(struct NWBuffer* buffer);

void reset_nw_buf(struct NWBuffer* buffer);

//uint16_t checksum(uint16_t val_in, void* data, uint16_t data_len);
uint16_t checksum(uint16_t val_in, size_t pos_in, size_t pos_end, void* data);


#endif  // BUFFER__H
