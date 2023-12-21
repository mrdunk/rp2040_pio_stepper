#include "buffer.h"
#include <stdio.h>

uint16_t pack_nw_buff(struct NWBuffer* buffer, void* new_data, uint16_t new_data_len) {
  if(buffer->length + new_data_len > NW_BUF_LEN) {
    // Buffer full.
    return 0;
  }

  memcpy(buffer->payload + buffer->length, new_data, new_data_len);

  buffer->length += new_data_len;
  buffer->checksum = checksum(buffer->checksum, new_data, new_data_len);

  return new_data_len;
}

void* unpack_nw_buff(
    struct NWBuffer* buffer,
    uint16_t payload_offset,
    uint16_t* new_payload_offset,   // May be null if final offset not needed.
    void* dest_container,           // May be null if no copy to struct needed.
    uint16_t dest_container_len
) {
  if(payload_offset + dest_container_len > buffer->length) {
    // Requested data overlaps end of buffer.
    // This implies we've reached the end of the valid data.
    return NULL;
  }

  if(buffer->length > NW_BUF_LEN) {
    // buffer.length value longer than the allocated memory.
    // This implies data corruption of the buffer.length value.
    return NULL;
  }

  void* data_p = buffer->payload + payload_offset;

  if(dest_container) {
    memcpy(dest_container, data_p, dest_container_len);
  }

  if(new_payload_offset) {
    *new_payload_offset = payload_offset += dest_container_len;
  }

  return data_p;
}

uint8_t checkNWBuff(struct NWBuffer* buffer) {
  uint16_t cs = 0;
  if(buffer->checksum != checksum(cs, buffer->payload, buffer->length)) {
    // Checksum failure.
    return 0;
  }
  return 1;
}

void reset_nw_buf(struct NWBuffer* buffer) {
  buffer->length = 0;
  buffer->checksum = 0;
}
