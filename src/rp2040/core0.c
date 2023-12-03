#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "config.h"
#include "messages.h"


#ifndef BUILD_TESTS

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

/* Process data received over the network.
 * This consists of serialised structs as defined in src/shared/massages.h
 */
size_t process_received_buffer( uint8_t* rx_buf, uint8_t* tx_buf, uint8_t* received_count) {
  uint8_t* rx_itterator = rx_buf;
  size_t tx_buf_len = 0;
  uint32_t msg_type;
  uint32_t axis;
  int8_t io_pos_value = -1;

  while((msg_type = *(uint32_t*)(rx_itterator))) {  // msg_type of 0 indicates end of data.
    switch(msg_type) {
      case MSG_TIMING:
        ;
        uint32_t update_id = ((struct Message_timing*)(rx_itterator))->update_id;
        uint32_t tx_time = ((struct Message_timing*)(rx_itterator))->time;

        rx_itterator += sizeof(struct Message_timing);

        int32_t id_diff;
        int32_t time_diff;
        update_packet_metrics(update_id, tx_time, &id_diff, &time_diff);
        serialise_metrics(tx_buf, &tx_buf_len, update_id, time_diff);
        (*received_count)++;
        break;
      case MSG_SET_AXIS_ENABLED:
        axis = ((struct Message_joint_enable*)(rx_itterator))->axis;
        uint8_t enabled = ((struct Message_joint_enable*)(rx_itterator))->value;
        
        rx_itterator += sizeof(struct Message_joint_enable);

        printf("%u Enabling axis: %u\t%i\n", *received_count, axis, enabled);
        update_axis_config(
            axis, CORE0,
            &enabled, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
        (*received_count)++;
        break;
      case MSG_SET_AXIS_ABS_POS:
        axis = ((struct Message_set_abs_pos*)(rx_itterator))->axis;
        double abs_pos_requested =
          ((struct Message_set_abs_pos*)(rx_itterator))->value;
        
        rx_itterator += sizeof(struct Message_set_abs_pos);

        update_axis_config(
            axis, CORE0,
            NULL, NULL, NULL, NULL, &abs_pos_requested, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
        (*received_count)++;
        break;
      case MSG_SET_AXIS_REL_POS:
        axis = ((struct Message_set_rel_pos*)(rx_itterator))->axis;
        double velocity_requested =
          ((struct Message_set_rel_pos*)(rx_itterator))->value;
        
        rx_itterator += sizeof(struct Message_set_rel_pos);

        update_axis_config(
            axis, CORE0,
            NULL, NULL, NULL, &velocity_requested, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
        (*received_count)++;
        break;
      case MSG_SET_AXIS_MAX_SPEED:
        axis = ((struct Message_set_max_velocity*)(rx_itterator))->axis;
        double max_velocity = 
          ((struct Message_set_max_velocity*)(rx_itterator))->value;

        rx_itterator += sizeof(struct Message_set_max_velocity);

        update_axis_config(
            axis, CORE0,
            NULL, NULL, NULL, NULL, NULL, NULL, &max_velocity, NULL, NULL, NULL, NULL, NULL, NULL);
        (*received_count)++;
        break;
      case MSG_SET_AXIS_MAX_ACCEL:
        axis = ((struct Message_set_max_accel*)(rx_itterator))->axis;
        double max_accel = ((struct Message_set_max_accel*)(rx_itterator))->value;

        rx_itterator += sizeof(struct Message_set_max_accel);

        update_axis_config(
            axis, CORE0,
            NULL, NULL, NULL, NULL, NULL, NULL, NULL, &max_accel, NULL, NULL, NULL, NULL, NULL);
        (*received_count)++;
        break;
      case MSG_SET_AXIS_PID_KP:
        axis = ((struct Message_set_kp*)(rx_itterator))->axis;
        float kp = ((struct Message_set_kp*)(rx_itterator))->value;

        rx_itterator += sizeof(struct Message_set_kp);

        printf("%u Setting axis: %u\tkp:      %f\n", *received_count, axis, kp);
        update_axis_config(
            axis, CORE0,
            NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, &kp);
        (*received_count)++;
        break;
      case MSG_SET_AXIS_IO_STEP:
        axis = ((struct Message_joint_gpio*)(rx_itterator))->axis;
        io_pos_value = ((struct Message_joint_gpio*)(rx_itterator))->value;

        rx_itterator += sizeof(struct Message_joint_gpio);

        printf("%u Setting axis: %u\tstep-io: %i\n", *received_count, axis, io_pos_value);
        update_axis_config(
            axis, CORE0,
            NULL, &io_pos_value, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
        (*received_count)++;
        break;
      case MSG_SET_AXIS_IO_DIR:
        axis = ((struct Message_joint_gpio*)(rx_itterator))->axis;
        io_pos_value = ((struct Message_joint_gpio*)(rx_itterator))->value;

        rx_itterator += sizeof(struct Message_joint_gpio);

        printf("%u Setting axis: %u\tdir-io:  %i\n", *received_count, axis, io_pos_value);
        update_axis_config(
            axis, CORE0,
            NULL, NULL, &io_pos_value, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
        (*received_count)++;
        break;
      default:
        printf("%u Invalid message type: %lu\r\n", *received_count, msg_type);
        *(uint32_t*)(rx_itterator) = 0;  // 0 msg_type stops execution.
        tx_buf_len = 0;
        break;
    }
  }
  memset(rx_buf, '\0', DATA_BUF_SIZE);

  return tx_buf_len;
}

void core0_main() {
  int retval = 0;
  uint8_t rx_buf[DATA_BUF_SIZE] = {0};
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
  memset(rx_buf, 0, DATA_BUF_SIZE);

  while (1) {
    tx_buf_len = 0;
    data_received = 0;
    memset(tx_buf, 0, DATA_BUF_SIZE);

    while(data_received == 0 || retval <= 0) {
      retval = get_UDP(
          SOCKET_NUMBER,
          NW_PORT,
          rx_buf,
          &data_received,
          destip_machine,
          &destport_machine);
    }

    tx_buf_len = process_received_buffer(rx_buf, tx_buf, &received_msg_count);
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
    }

    size_t axis_count = 0;
    for(size_t axis = 0; axis < MAX_AXIS; axis++) {
      // Get data from config and put in TX buffer.
      axis_count += serialise_axis_config(axis, tx_buf, &tx_buf_len, true);
      //axis_count += serialise_axis_config(axis, tx_buf, &tx_buf_len, false);
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


