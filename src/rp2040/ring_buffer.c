#include "config.h"


/* A ring buffer that returns the average value of it's contents.
 * Used for calculating average period between incoming network updates. */
uint32_t ring_buf_uint_ave(struct Ring_buf_uint_ave* data, const uint32_t new_val) {
  uint32_t tail_val = data->buf[data->head];
  data->buf[data->head] = new_val;
  data->head++;
  if(data->head >= RING_BUF_AVE_LEN) {
    data->head = 0;
  }
  data->total += new_val;
  if(data->count < RING_BUF_AVE_LEN) {
    data->count++;
  } else {
    data->total -= tail_val;
  }

  return data->total / data->count;
}


/* A ring buffer that returns the average value of it's contents.
 * Used for calculating average period between incoming network updates. */
int32_t ring_buf_int_ave(struct Ring_buf_int_ave* data, const int32_t new_val) {
  int32_t tail_val = data->buf[data->head];
  data->buf[data->head] = new_val;
  data->head++;
  if(data->head >= RING_BUF_AVE_LEN) {
    data->head = 0;
  }
  data->total += new_val;
  if(data->count < RING_BUF_AVE_LEN) {
    data->count++;
  } else {
    data->total -= tail_val;
  }

  return data->total / (int32_t)data->count;
}


