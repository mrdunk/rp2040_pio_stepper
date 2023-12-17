#ifndef BUFFER__H
#define BUFFER__H

#include <sys/types.h>
#include <stdint.h>
#include <string.h>

// Must be even number as NWBuffer.length is a uint16_t which is 2 bytes..
#define NW_BUF_LEN 256

struct NWBuffer {
    uint16_t length;
    uint16_t checksum;
    uint8_t payload[NW_BUF_LEN];
};

uint16_t packNWBuff(struct NWBuffer* buffer, void* new_data, uint16_t new_data_len);

uint16_t unPackNWBuff(
    struct NWBuffer* buffer,
    uint16_t payload_offset,
    void* dest_container,
    uint16_t dest_container_len
);

uint16_t checksum(uint16_t checksum, void* new_data, uint16_t new_data_len);


#endif  // BUFFER__H
