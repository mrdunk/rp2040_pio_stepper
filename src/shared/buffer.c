#include "buffer.h"

uint16_t packNWBuff(struct NWBuffer* buffer, void* new_data, uint16_t new_data_len) {
  if(buffer->length + new_data_len > NW_BUF_LEN) {
    // Buffer full.
    return 0;
  }

  memcpy(buffer->payload + buffer->length, new_data, new_data_len);

  buffer->length += new_data_len;
  buffer->checksum = checksum(buffer->checksum, new_data, new_data_len);

  return new_data_len;
}

uint16_t unPackNWBuff(
    struct NWBuffer* buffer,
    uint16_t payload_offset,
    void* dest_container,
    uint16_t dest_container_len
) {
  if(payload_offset + dest_container_len > buffer->length) {
    // Requested data overlaps end of buffer.
    // This implies data corruption.
    return 0;
  }

  if(buffer->length > NW_BUF_LEN) {
    // buffer.length value longer than the allocated memory.
    // This implies data corruption of the buffer.length value.
    return 0;
  }

  memcpy(dest_container, buffer->payload + payload_offset, dest_container_len);
  return 1;
}

uint8_t checkNWBuff(struct NWBuffer* buffer) {
  uint16_t cs = 0;
  if(buffer->checksum != checksum(cs, buffer->payload, buffer->length)) {
    // Checksum failure.
    return 0;
  }
  return 1;
}
