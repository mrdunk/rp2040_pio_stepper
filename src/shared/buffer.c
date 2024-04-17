#include "buffer.h"
#include <stdio.h>

size_t alligned32(size_t input) {
  return input + 3 - ((input - 1) % 4);
}

size_t pack_nw_buff(struct NWBuffer* buffer, void* new_data, size_t new_data_len) {
  // 32bit align value.
  size_t new_data_len_alligned = alligned32(new_data_len);

  if(buffer->length + new_data_len > NW_BUF_LEN) {
    // Buffer full.
    return 0;
  }

  if(! new_data) {
    return 0;
  }

  memset(buffer->payload + buffer->length, 0, new_data_len_alligned);
  memcpy(buffer->payload + buffer->length, new_data, new_data_len);

  buffer->checksum = checksum(
      buffer->checksum, buffer->length, buffer->length + new_data_len_alligned, buffer->payload);
  buffer->length += new_data_len_alligned;

  return new_data_len_alligned;
}

/* Returns pointer to struct position within network packet.
 * Updates offset value to point to next struct if required. */
void* unpack_nw_buff(
    struct NWBuffer* buffer,
    size_t payload_offset,
    size_t* new_payload_offset,   // May be null if final offset not needed.
    void* dest_container,           // May be null if no copy to struct needed.
    size_t dest_container_len
) {
  // 32bit align value.
  size_t dest_container_len_alligned = alligned32(dest_container_len);

  if(payload_offset + dest_container_len_alligned > buffer->length) {
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
    *new_payload_offset = payload_offset += dest_container_len_alligned;
  }

  return data_p;
}

/* Calculate checksum and compare to the checksum stored in the NW data. */
size_t checkNWBuff(struct NWBuffer* buffer) {
  uint16_t cs = 0;
  if(buffer->checksum != checksum(cs, 0, buffer->length, buffer->payload)) {
    // Checksum failure.
    return 0;
  }
  return 1;
}

void reset_nw_buf(struct NWBuffer* buffer) {
  buffer->length = 0;
  buffer->checksum = 0;
}
