#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "config.h"
#include "messages.h"
#include "buffer.h"


#ifdef BUILD_TESTS

#include "../test/mocks/rp_mocks.h"
#include "../test/mocks/network_mocks.h"

#else  // BUILD_TESTS

#include "pico/stdlib.h"
#include "stepper_control.h"
#include "network.h"

#endif  // BUILD_TESTS

/* Called after receiving network packet.
 * Takes the average time delay over the previous 1000 packet receive events and
 * blocks long enough to normalize any network time jitter. */
void recover_clock() {
  static struct Ring_buf_uint_ave period_average_data;
  static struct Ring_buf_uint_ave time_average_data_1;
  static struct Ring_buf_uint_ave time_average_data_2;
  static size_t time_last = 0;
  uint32_t restart_at = 0;
  uint32_t last_ave_period_us = 0;

  // Record the average length of time between packets.
  size_t time_now = time_us_64();
  uint32_t ave_period_us = ring_buf_uint_ave(&period_average_data, time_now - time_last);
  time_last = time_now;

  // Record the average arrival time.
  // Since we can't average modulo values (0us and 1000us would return 500 despite them
  // being adjacent) so we collect 2 averages, offset by 500us.
  uint32_t time_offset_1 = time_now % 1000;
  uint32_t ave_time_offset_us_1 = ring_buf_uint_ave(&time_average_data_1, time_offset_1);
  uint32_t time_offset_2 = (time_now + 500) % 1000;
  uint32_t ave_time_offset_us_2 = ring_buf_uint_ave(&time_average_data_2, time_offset_2);

  // Use whichever average arrival time is furthest from the modulo rollover event.
  uint32_t time_offset;
  uint32_t ave_time_offset_us;
  if(time_offset_1 > 250 && time_offset_1 < 750) {
    time_offset = time_offset_1;
    ave_time_offset_us = ave_time_offset_us_1;
  } else {
    time_offset = time_offset_2;
    ave_time_offset_us = ave_time_offset_us_2;
  }


  int32_t time_diff = (int32_t)time_offset - (int32_t)ave_time_offset_us;

  // Do the busy-wait to synchronise timing.
  restart_at = time_now + 200 - time_diff;
  while(restart_at > time_now) {
    time_now = time_us_64();
    tight_loop_contents();
  }

  // Semaphore to send core1 as the synchronization event.
  // Note that this happens soon after the busy-wait.
  tick++;

  if(last_ave_period_us != ave_period_us) {
    update_period(ave_period_us);
    last_ave_period_us = ave_period_us;
  }
}

bool unpack_timing(
    struct NWBuffer* rx_buf,
    uint16_t* rx_offset,
    uint8_t* tx_buf,
    size_t* tx_buf_len,
    uint8_t* received_count
) {
  void* data_p = unpack_nw_buff(
      rx_buf, *rx_offset, rx_offset, NULL, sizeof(struct Message_timing));

  if(! data_p) {
    return NULL;
  }

  struct Message_timing* message = data_p;
  int32_t id_diff;
  int32_t time_diff;
  update_packet_metrics(message, &id_diff, &time_diff);
  serialise_metrics(tx_buf, tx_buf_len, message->update_id, time_diff);

  (*received_count)++;
  return true;
}

bool unpack_joint_enable(
    struct NWBuffer* rx_buf,
    uint16_t* rx_offset,
    uint8_t* received_count
) {
  void* data_p = unpack_nw_buff(
      rx_buf, *rx_offset, rx_offset, NULL, sizeof(struct Message_joint_enable));

  if(! data_p) {
    return NULL;
  }

  struct Message_joint_enable* message = data_p;
  uint32_t joint = message->axis;
  uint8_t enabled = message->value;

  printf("%u Enabling joint: %u\t%i\n", *received_count, joint, enabled);
  update_axis_config(
      joint, CORE0,
      &enabled, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);

  (*received_count)++;
  return true;
}

bool unpack_joint_abs_pos(
    struct NWBuffer* rx_buf,
    uint16_t* rx_offset,
    uint8_t* received_count
) {
  void* data_p = unpack_nw_buff(
      rx_buf, *rx_offset, rx_offset, NULL, sizeof(struct Message_set_abs_pos));

  if(! data_p) {
    return NULL;
  }

  struct Message_set_abs_pos* message = data_p;
  uint32_t joint = message->axis;
  double abs_pos = message->value;

  update_axis_config(
      joint, CORE0,
      NULL, NULL, NULL, NULL, &abs_pos, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);

  (*received_count)++;
  return true;
}

bool unpack_joint_velocity(
    struct NWBuffer* rx_buf,
    uint16_t* rx_offset,
    uint8_t* received_count
) {
  void* data_p = unpack_nw_buff(
      rx_buf, *rx_offset, rx_offset, NULL, sizeof(struct Message_set_velocity));

  if(! data_p) {
    return NULL;
  }

  struct Message_set_velocity* message = data_p;
  uint32_t joint = message->axis;
  double vel_reques = message->value;

  update_axis_config(
      joint, CORE0,
      NULL, NULL, NULL, &vel_reques, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);

  (*received_count)++;
  return true;
}

bool unpack_joint_max_velocity(
    struct NWBuffer* rx_buf,
    uint16_t* rx_offset,
    uint8_t* received_count
) {
  void* data_p = unpack_nw_buff(
      rx_buf, *rx_offset, rx_offset, NULL, sizeof(struct Message_set_max_velocity));

  if(! data_p) {
    return NULL;
  }

  struct Message_set_max_velocity* message = data_p;
  uint32_t joint = message->axis;
  double max_velocity = message->value;

  update_axis_config(
      joint, CORE0,
      NULL, NULL, NULL, NULL, NULL, NULL, &max_velocity, NULL, NULL, NULL, NULL, NULL, NULL);

  (*received_count)++;
  return true;
}

bool unpack_joint_max_accel(
    struct NWBuffer* rx_buf,
    uint16_t* rx_offset,
    uint8_t* received_count
) {
  void* data_p = unpack_nw_buff(
      rx_buf, *rx_offset, rx_offset, NULL, sizeof(struct Message_set_max_accel));

  if(! data_p) {
    return NULL;
  }

  struct Message_set_max_accel* message = data_p;
  uint32_t joint = message->axis;
  double max_accel = message->value;

  update_axis_config(
      joint, CORE0,
      NULL, NULL, NULL, NULL, NULL, NULL, NULL, &max_accel, NULL, NULL, NULL, NULL, NULL);

  (*received_count)++;
  return true;
}

bool unpack_joint_io_step(
    struct NWBuffer* rx_buf,
    uint16_t* rx_offset,
    uint8_t* received_count
) {
  void* data_p = unpack_nw_buff(
      rx_buf, *rx_offset, rx_offset, NULL, sizeof(struct Message_joint_gpio));

  if(! data_p) {
    return NULL;
  }
  
  struct Message_joint_gpio* message = data_p;
  uint32_t joint = message->axis;
  int8_t io_step = message->value;

  printf("%u Setting axis: %u\tstep-io: %i\n", *received_count, joint, io_step);
  update_axis_config(
      joint, CORE0,
            NULL, &io_step, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);


  (*received_count)++;
  return true;
}

bool unpack_joint_io_dir(
    struct NWBuffer* rx_buf,
    uint16_t* rx_offset,
    uint8_t* received_count
) {
  void* data_p = unpack_nw_buff(
      rx_buf, *rx_offset, rx_offset, NULL, sizeof(struct Message_joint_gpio));

  if(! data_p) {
    return NULL;
  }

  struct Message_joint_gpio* message = data_p;
  uint32_t joint = message->axis;
  int8_t io_dir = message->value;

  printf("%u Setting axis: %u\tdir-io:  %i\n", *received_count, joint, io_dir);
  update_axis_config(
      joint, CORE0,
      NULL, NULL, &io_dir, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);

  (*received_count)++;
  return true;
}

/* Process data received over the network.
 * This consists of serialised structs as defined in src/shared/massages.h
 */
size_t process_received_buffer(struct NWBuffer* rx_buf, uint8_t* tx_buf, uint8_t* received_count) {
  uint16_t rx_offset = 0;
  size_t tx_buf_len = 0;
  union MessageAny message;

  if(rx_buf->length > NW_BUF_LEN) {
    printf("WARN: RX length greater than buffer size. %u\n", *received_count);
    return 0;
  }

  bool unpack_success = checkNWBuff(rx_buf);

  if(! unpack_success) {
    printf("WARN: RX checksum fail. %u\n", *received_count);
  }

  while(unpack_success) {
    unpack_success = unpack_success && unpack_nw_buff(
        rx_buf, rx_offset, NULL, &message, sizeof(struct Message_header));
    if(!unpack_success) {
      // End of data.
      break;
    }

    switch(message.header.type) {
      case MSG_TIMING:
        ;
        unpack_success = unpack_success && unpack_timing(
            rx_buf, &rx_offset, tx_buf, &tx_buf_len, received_count);
        break;
      case MSG_SET_AXIS_ENABLED:
        unpack_success = unpack_success && unpack_joint_enable(
            rx_buf, &rx_offset, received_count);
        break;
      case MSG_SET_AXIS_ABS_POS:
        unpack_success = unpack_success && unpack_joint_abs_pos(
            rx_buf, &rx_offset, received_count);
        break;
      case MSG_SET_AXIS_VELOCITY:
        unpack_success = unpack_success && unpack_joint_velocity(
            rx_buf, &rx_offset, received_count);
        break;
      case MSG_SET_AXIS_MAX_VELOCITY:
        unpack_success = unpack_success && unpack_joint_max_velocity(
            rx_buf, &rx_offset, received_count);
        break;
      case MSG_SET_AXIS_MAX_ACCEL:
        unpack_success = unpack_success && unpack_joint_max_accel(
            rx_buf, &rx_offset, received_count);
        break;
      case MSG_SET_AXIS_IO_STEP:
        unpack_success = unpack_success && unpack_joint_io_step(
            rx_buf, &rx_offset, received_count);
        break;
      case MSG_SET_AXIS_IO_DIR:
        unpack_success = unpack_success && unpack_joint_io_dir(
            rx_buf, &rx_offset, received_count);
        break;
      default:
        printf("WARN: Invalid message type: %u\t%lu\n", *received_count, message.header.type);
        // Implies data corruption.
        tx_buf_len = 0;
        unpack_success = false;
        break;
    }
  }

  if(rx_offset < rx_buf->length) {
    printf("WARN: Unconsumed RX buffer remainder: %u bytes.\t%u\n",
        rx_buf->length - rx_offset, *received_count);
    // Implies data corruption.
    // Received a message type in the header but not enough data in the buffer
    // to populate the struct.
  }

  reset_nw_buf(rx_buf);
  return tx_buf_len;
}

void core0_main() {
  int retval = 0;
  struct NWBuffer rx_buf = {0};
  uint8_t tx_buf[DATA_BUF_SIZE] = {0};
  size_t tx_buf_len = 0;
  uint8_t received_msg_count = 0;
  uint8_t data_received = 0;
  size_t time_now;

  // Need these to store the IP and port.
  // We get the remote values when receiving data.
  // These store them for when we want to reply later.
  uint8_t  destip_machine[4] = {0, 0, 0, 0};
  uint16_t destport_machine = 0;

  while (1) {
    tx_buf_len = 0;
    data_received = 0;
    memset(tx_buf, 0, DATA_BUF_SIZE);

    while(data_received == 0 || retval <= 0) {
      retval = get_UDP(
          SOCKET_NUMBER,
          NW_PORT,
          &rx_buf,
          &data_received,
          destip_machine,
          &destport_machine);
    }

    //static size_t count = 0;
    //printf("%u, %u, %u\t", rx_buf.length, data_received, *((uint16_t*)&rx_buf));
    //if(count++ % 20 == 0) {
    //  printf("\n");
    //}


    tx_buf_len = process_received_buffer(&rx_buf, tx_buf, &received_msg_count);
    if(received_msg_count != 9) {
      // Not the standard number of received packets.
      // This likely was a config update.
      printf("Received msgs: %u\t%u\t%i\n", received_msg_count, data_received, retval);
    }

    if(received_msg_count > 0) {
      recover_clock();

      time_now = time_us_64();
      gpio_put(LED_PIN, (time_now / 1000000) % 2);
      received_msg_count = 0;

      size_t axis_count = 0;
      for(size_t axis = 0; axis < MAX_AXIS; axis++) {
        // Get data from config and put in TX buffer.
        axis_count += serialise_axis_config(axis, tx_buf, &tx_buf_len, true);
      }
    }

    put_UDP(
        SOCKET_NUMBER,
        NW_PORT,
        tx_buf,
        tx_buf_len,
        destip_machine,
        &destport_machine);
  }
}


