#ifndef RING_BUFFER__H
#define RING_BUFFER__H


/* A ring buffer that returns the average value of it's contents.
 * Used for calculating average period between incoming network updates. */
#define RING_BUF_AVE_LEN 1000
struct Ring_buf_uint_ave {
  uint32_t buf[RING_BUF_AVE_LEN];
  size_t head;
  uint32_t total;
  size_t count;
};

struct Ring_buf_int_ave {
  int32_t buf[RING_BUF_AVE_LEN];
  size_t head;
  int32_t total;
  size_t count;
};

uint32_t ring_buf_uint_ave(struct Ring_buf_uint_ave* data, const uint32_t new_val);
int32_t ring_buf_int_ave(struct Ring_buf_int_ave* data, const int32_t new_val);


#endif  // RING_BUFFER__H

