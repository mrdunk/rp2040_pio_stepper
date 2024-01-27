#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "config.h"
#include "messages.h"
#include "buffer.h"
#include "gpio.h"
#include "modbus.h"


#ifdef BUILD_TESTS

#include "../test/mocks/rp_mocks.h"
#include "../test/mocks/network_mocks.h"

#else  // BUILD_TESTS

#include "pico/stdlib.h"
#include "stepper_control.h"
#include "network.h"

#endif  // BUILD_TESTS

float req_spindle_frequency = 0;
float act_spindle_frequency = -1000000;

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
  restart_at = time_now + 150 - time_diff;
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
    size_t* rx_offset,
    struct NWBuffer* tx_buf,
    size_t* received_count
) {
  void* data_p = unpack_nw_buff(
      rx_buf, *rx_offset, rx_offset, NULL, sizeof(struct Message_timing));

  if(! data_p) {
    return false;
  }

  struct Message_timing* message = data_p;
  int32_t id_diff;
  int32_t time_diff;
  update_packet_metrics(message, &id_diff, &time_diff);
  serialise_timing(tx_buf, message->update_id, time_diff);

  (*received_count)++;
  return true;
}

bool unpack_joint_enable(
    struct NWBuffer* rx_buf,
    size_t* rx_offset,
    size_t* received_count
) {
  void* data_p = unpack_nw_buff(
      rx_buf, *rx_offset, rx_offset, NULL, sizeof(struct Message_joint_enable));

  if(! data_p) {
    return false;
  }

  struct Message_joint_enable* message = data_p;
  uint8_t joint = message->axis;
  uint8_t enabled = message->value;

  printf("%u Enabling joint: %u\t%i\n", *received_count, joint, enabled);
  update_axis_config(
      joint, CORE0,
      &enabled, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);

  (*received_count)++;
  return true;
}

bool unpack_joint_abs_pos(
    struct NWBuffer* rx_buf,
    size_t* rx_offset,
    size_t* received_count
) {
  void* data_p = unpack_nw_buff(
      rx_buf, *rx_offset, rx_offset, NULL, sizeof(struct Message_set_abs_pos));

  if(! data_p) {
    return false;
  }

  struct Message_set_abs_pos* message = data_p;
  uint8_t joint = message->axis;
  double abs_pos = message->position;
  double vel_reques = message->velocity;

  update_axis_config(
      joint, CORE0,
      NULL, NULL, NULL, &vel_reques, &abs_pos, NULL, NULL, NULL, NULL, NULL, NULL);

  (*received_count)++;
  return true;
}

bool unpack_spindle_config(
    struct NWBuffer* rx_buf,
    size_t* rx_offset,
    struct NWBuffer* tx_buf,
    size_t* received_count
) {
  void* data_p = unpack_nw_buff(
      rx_buf, *rx_offset, rx_offset, NULL, sizeof(struct Message_spindle_config));

  if(! data_p) {
    return false;
  }

  struct Message_spindle_config* message = data_p;
  uint8_t spindle = message->spindle_index;
  if(spindle > 0) {
    printf("ERROR: More than one spindle not yet supported.\n");
    return false;
  }
  vfd_config.address = message->modbus_address;
  vfd_config.bitrate = message->bitrate;
  vfd_config.type = message->vfd_type;


  serialise_spindle_config(spindle, tx_buf);

  (*received_count)++;
  return true;
}

bool unpack_spindle_speed(
    struct NWBuffer* rx_buf,
    size_t* rx_offset,
    size_t* received_count
) {
  void* data_p = unpack_nw_buff(
      rx_buf, *rx_offset, rx_offset, NULL, sizeof(struct Message_spindle_speed));

  if(! data_p) {
    return false;
  }

  struct Message_spindle_speed* message = data_p;
  req_spindle_frequency = message->speed;
  (*received_count)++;
  return true;
}

bool unpack_joint_max_velocity(
    struct NWBuffer* rx_buf,
    size_t* rx_offset,
    size_t* received_count
) {
  void* data_p = unpack_nw_buff(
      rx_buf, *rx_offset, rx_offset, NULL, sizeof(struct Message_set_max_velocity));

  if(! data_p) {
    return false;
  }

  struct Message_set_max_velocity* message = data_p;
  uint8_t joint = message->axis;
  double max_velocity = message->value;

  update_axis_config(
      joint, CORE0,
      NULL, NULL, NULL, NULL, NULL, NULL, &max_velocity, NULL, NULL, NULL, NULL);

  (*received_count)++;
  return true;
}

bool unpack_joint_max_accel(
    struct NWBuffer* rx_buf,
    size_t* rx_offset,
    size_t* received_count
) {
  void* data_p = unpack_nw_buff(
      rx_buf, *rx_offset, rx_offset, NULL, sizeof(struct Message_set_max_accel));

  if(! data_p) {
    return false;
  }

  struct Message_set_max_accel* message = data_p;
  uint8_t joint = message->axis;
  double max_accel = message->value;

  update_axis_config(
      joint, CORE0,
      NULL, NULL, NULL, NULL, NULL, NULL, NULL, &max_accel, NULL, NULL, NULL);

  (*received_count)++;
  return true;
}

bool unpack_joint_io_step(
    struct NWBuffer* rx_buf,
    size_t* rx_offset,
    size_t* received_count
) {
  void* data_p = unpack_nw_buff(
      rx_buf, *rx_offset, rx_offset, NULL, sizeof(struct Message_joint_gpio));

  if(! data_p) {
    return false;
  }

  struct Message_joint_gpio* message = data_p;
  uint8_t joint = message->axis;
  int8_t io_step = message->value;

  printf("%u Setting axis: %u\tstep-io: %i\n", *received_count, joint, io_step);
  update_axis_config(
      joint, CORE0,
            NULL, &io_step, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);


  (*received_count)++;
  return true;
}

bool unpack_joint_io_dir(
    struct NWBuffer* rx_buf,
    size_t* rx_offset,
    size_t* received_count
) {
  void* data_p = unpack_nw_buff(
      rx_buf, *rx_offset, rx_offset, NULL, sizeof(struct Message_joint_gpio));

  if(! data_p) {
    return false;
  }

  struct Message_joint_gpio* message = data_p;
  uint8_t joint = message->axis;
  int8_t io_dir = message->value;

  printf("%u Setting axis: %u\tdir-io:  %i\n", *received_count, joint, io_dir);
  update_axis_config(
      joint, CORE0,
      NULL, NULL, &io_dir, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);

  (*received_count)++;
  return true;
}

bool unpack_joint_config(
    struct NWBuffer* rx_buf,
    size_t* rx_offset,
    struct NWBuffer* tx_buf,
    size_t* received_count
) {
  void* data_p = unpack_nw_buff(
      rx_buf, *rx_offset, rx_offset, NULL, sizeof(struct Message_joint_config));

  if(! data_p) {
    return false;
  }

  struct Message_joint_config* message = data_p;
  uint8_t joint = message->axis;
  uint8_t enabled = message->enable;
  int8_t io_step = message->gpio_step;
  int8_t io_dir = message->gpio_dir;
  double max_velocity = message->max_velocity;
  double max_accel = message->max_accel;


  printf("%u Configuring joint: %u\n", *received_count, joint);
  update_axis_config(
      joint,
      CORE0,
      &enabled,
      &io_step,
      &io_dir,
      NULL,
      NULL,
      NULL,
      &max_velocity,
      &max_accel,
      NULL, NULL, NULL);

  serialise_axis_config(joint, tx_buf);

  (*received_count)++;
  return true;
}

bool unpack_gpio(
    struct NWBuffer* rx_buf,
    size_t* rx_offset,
    size_t* received_count
) {
  void* data_p = unpack_nw_buff(
      rx_buf, *rx_offset, rx_offset, NULL, sizeof(struct Message_gpio));

  if(! data_p) {
    return false;
  }

  struct Message_gpio* message = data_p;
  const uint8_t bank = message->bank;
  uint32_t values = message->values;
  bool confirmation_pending = message->confirmation_pending;

  config.gpio_confirmation_pending[bank] = confirmation_pending;

  gpio_set_values(bank, values);

  (*received_count)++;
  return true;
}

bool unpack_gpio_config(
    struct NWBuffer* rx_buf,
    size_t* rx_offset,
    struct NWBuffer* tx_buf,
    size_t* received_count
) {
  void* data_p = unpack_nw_buff(
      rx_buf, *rx_offset, rx_offset, NULL, sizeof(struct Message_gpio_config));

  if(! data_p) {
    return false;
  }

  struct Message_gpio_config* message = data_p;
  uint8_t gpio_type = message->gpio_type;
  uint8_t gpio_count = message->gpio_count;
  uint8_t index = message->index;
  uint8_t address = message->address;

  printf("%u Configuring gpio: %u\t%u\n", *received_count, gpio_type, gpio_count);

  config.gpio[gpio_count].type = gpio_type;
  config.gpio[gpio_count].index = index;
  config.gpio[gpio_count].address = address;

  switch(gpio_type) {
    case GPIO_TYPE_NATIVE_IN:
    case GPIO_TYPE_NATIVE_IN_DEBUG:
      ;
      // Remember HAL's definition of IN and OUT are opposite of RPs.
      printf("Setting RP native IO to RP OUT: %u\n", index);
      gpio_init(index);
      gpio_set_dir(index, GPIO_OUT);
      break;
    case GPIO_TYPE_NATIVE_OUT:
    case GPIO_TYPE_NATIVE_OUT_DEBUG:
      // Remember HAL's definition of IN and OUT are opposite of RPs.
      printf("Setting RP native IO to RP IN: %u\n", index);
      gpio_init(index);
      gpio_set_dir(index, GPIO_IN);
      gpio_pull_up(index);
      break;
    default:
      break;
  }

  serialise_gpio_config(gpio_count, tx_buf);

  (*received_count)++;
  return true;
}

/* Process data received over the network.
 * This consists of serialised structs as defined in src/shared/massages.h
 */
void process_received_buffer(
    struct NWBuffer* rx_buf,
    struct NWBuffer* tx_buf,
    size_t* received_count,
    size_t expected_length
) {
  size_t rx_offset = 0;

  if(rx_buf->length + sizeof(rx_buf->length) + sizeof(rx_buf->checksum) != expected_length) {
    printf("WARN: RX length not equal to expected. %u\n", *received_count);
    reset_nw_buf(rx_buf);
    return;
  }
  if(rx_buf->length > NW_BUF_LEN) {
    printf("WARN: RX length greater than buffer size. %u\n", *received_count);
    reset_nw_buf(rx_buf);
    return;
  }

  if(!checkNWBuff(rx_buf)) {
    printf("WARN: RX checksum fail.\n");

    reset_nw_buf(rx_buf);
    reset_nw_buf(tx_buf);
    *received_count = 0;
    return;
  }

  bool unpack_success = true;

  while(unpack_success) {
    struct Message_header* header = unpack_nw_buff(
        rx_buf, rx_offset, NULL, NULL, sizeof(struct Message_header));

    if(!header || !header->type) {
      // End of data.
      break;
    }

    switch(header->type) {
      case MSG_TIMING:
        ;
        unpack_success = unpack_success && unpack_timing(
            rx_buf, &rx_offset, tx_buf, received_count);
        break;
      case MSG_SET_AXIS_ENABLED:
        unpack_success = unpack_success && unpack_joint_enable(
            rx_buf, &rx_offset, received_count);
        break;
      case MSG_SET_AXIS_ABS_POS:
        unpack_success = unpack_success && unpack_joint_abs_pos(
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
      case MSG_SET_AXIS_CONFIG:
        unpack_success = unpack_success && unpack_joint_config(
            rx_buf, &rx_offset, tx_buf, received_count);
        break;
      case MSG_SET_SPINDLE_CONFIG:
        unpack_success = unpack_success && unpack_spindle_config(
            rx_buf, &rx_offset, tx_buf, received_count);
        break;
      case MSG_SET_SPINDLE_SPEED:
        unpack_success = unpack_success && unpack_spindle_speed(
            rx_buf, &rx_offset, received_count);
      case MSG_SET_GPIO:
        unpack_success = unpack_success && unpack_gpio(
            rx_buf, &rx_offset, received_count);
        break;
      case MSG_SET_GPIO_CONFIG:
        unpack_success = unpack_success && unpack_gpio_config(
            rx_buf, &rx_offset, tx_buf, received_count);
        break;
      default:
        printf("WARN: Invalid message type: %u\t%lu\n", header->type, *received_count);
        // Implies data corruption.
        unpack_success = false;
        reset_nw_buf(rx_buf);
        reset_nw_buf(tx_buf);
        *received_count = 0;
        return;
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
  return;
}

void core0_main() {
  int retval = 0;
  struct NWBuffer rx_buf = {0};
  struct NWBuffer tx_buf = {0};
  size_t received_msg_count = 0;
  size_t data_received = 0;
  size_t time_now;

  // Need these to store the IP and port.
  // We get the remote values when receiving data.
  // These store them for when we want to reply later.
  uint8_t  destip_machine[4] = {0, 0, 0, 0};
  uint16_t destport_machine = 0;

  modbus_init();

  int count = 0;
  while (1) {
    data_received = 0;

    while(data_received == 0 || retval <= 0) {
      retval = get_UDP(
          SOCKET_NUMBER,
          NW_PORT,
          &rx_buf,
          &data_received,
          destip_machine,
          &destport_machine);
    }

    process_received_buffer(&rx_buf, &tx_buf, &received_msg_count, data_received);
    //if(received_msg_count != 9) {
      // Not the standard number of received packets.
      // This likely was a config update.
    //  printf("Received msgs: %u\t%u\n", received_msg_count, data_received);
    //}

    if(received_msg_count > 0) {
      recover_clock();

      time_now = time_us_64();
      gpio_put(LED_PIN, (time_now / 1000000) % 2);
      received_msg_count = 0;

      for(size_t axis = 0; axis < MAX_AXIS; axis++) {
        // Get data from config and put in TX buffer.
        serialise_axis_movement(axis, &tx_buf, true);
        serialise_axis_metrics(axis, &tx_buf);
      }
      serialise_spindle_speed_out(&tx_buf, act_spindle_frequency, &vfd.stats);
      count++;

      size_t tx_buf_len = 0;
      gpio_serialize(&tx_buf, &tx_buf_len);

      put_UDP(
          SOCKET_NUMBER,
          NW_PORT,
          &tx_buf,
          tx_buf.length + sizeof(tx_buf.length) + sizeof(tx_buf.checksum),
          destip_machine,
          &destport_machine);
      act_spindle_frequency = modbus_loop(req_spindle_frequency);
    }
    reset_nw_buf(&tx_buf);
  }
}


