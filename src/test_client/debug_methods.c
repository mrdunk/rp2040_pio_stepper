#include <stdint.h>

#include <stdio.h>
//#include <stdlib.h>
#include <string.h>

#include "../shared/messages.h"
#include "debug_methods.h"

/* Displays a buffer to be sent via UDP.*/
void display_tx_data(void* packet, size_t packet_size) {
  size_t packet_parsed = 0;
  size_t message_size;
  struct Message_uint message;
  struct Message_uint message_uint;
  //struct Message_uint_uint message_uint_uint;
  struct Message_uint_int message_uint_int;
  struct Message_uint_float message_uint_float;

  printf("\nSending messages:\n");
  printf("Raw:\n");
  printf("  addr | val\n");
  printf("  --------+-------\n");

  for(size_t i = 0; i < packet_size / sizeof(uint32_t); i++) {
    printf("    %lu\t| %u\n", i, ((uint32_t*)packet)[i]);
  }

  printf("Parsed:\n");
  while(packet_parsed < packet_size) {
    uint32_t msg_type = *(uint32_t*)packet;
    switch(msg_type) {
      case MSG_SET_GLOAL_UPDATE_RATE:
        message_size = sizeof(struct Message_uint);
        memcpy(&message_uint, packet, message_size);
        printf("  msg type: %u\tvalue0: %u\n",
            message_uint.type, message_uint.value);
        break;
      case MSG_SET_AXIS_ABS_POS:
        message_size = sizeof(struct Message_uint_uint);
        memcpy(&message_uint_int, packet, message_size);
        printf("  msg type: %u\tvalue0: %u\tvalue1: %u\n",
            message_uint_int.type, message_uint_int.axis, message_uint_int.value);
        break;
      case MSG_SET_AXIS_REL_POS:
        message_size = sizeof(struct Message_uint_uint);
        memcpy(&message_uint_int, packet, message_size);
        printf("  msg type: %u\tvalue0: %u\tvalue1: %i\n",
            message_uint_int.type, message_uint_int.axis, message_uint_int.value);
        break;
      case MSG_SET_AXIS_MAX_SPEED:
      case MSG_SET_AXIS_MAX_ACCEL:
        printf("  Invalid message type: %u\n", msg_type);
        exit(0);
      case MSG_SET_AXIS_PID_KP:
        message_size = sizeof(struct Message_uint_float);
        memcpy(&message_uint_float, packet, message_size);
        printf("  msg type: %u\taxis: %u\tvalue: %f\n",
            message_uint_float.type, message_uint_float.axis, message_uint_float.value);
        break;
      case MSG_GET_GLOBAL_CONFIG:
        message_size = sizeof(struct Message);
        memcpy(&message, packet, message_size);
        printf("  msg type: %u\n", message.type);
        break;
      default:
        printf("  Invalid message type: %u\n", msg_type);
        exit(0);
    }
    packet += message_size;
    packet_parsed += message_size;
  }
  printf("\n");
}

/* Decodes and displays a raw buffer that has bee received via UDP. */
void display_rx_data(char* buf) {
  struct Reply_global_config reply_global_config;
  struct Reply_axis_config reply_axis_config;
  struct Reply_axis_pos reply_axis_pos;
  char* itterator = buf;
  size_t size;
  uint32_t msg_type;
  while((msg_type = *(uint32_t*)itterator)) {
    switch(msg_type) {
      case REPLY_GLOBAL_CONFIG:
        size = sizeof(struct Reply_global_config);
        memcpy(&reply_global_config, itterator, size);
        printf("Reply_global_config\n  "
            "type: %u\n  update_rate: %u\n  update_time_us: %u  update_time_ticks: %u\n",
            msg_type,
            reply_global_config.update_rate,
            reply_global_config.update_time_us,
            reply_global_config.update_rate);
        itterator += size;
        break;
      case REPLY_AXIS_CONFIG:
        size = sizeof(struct Reply_axis_config);
        memcpy(&reply_axis_config, itterator, size);
        printf("Reply_axis_config\n"
            "  type: %u\n  axis: %u\n  abs_pos_acheived: %u\n  min_step_len_ticks: %u\n"
            "  max_accel_ticks: %u\n  velocity: %i\n",
            msg_type,
            reply_axis_config.axis,
            reply_axis_config.abs_pos_acheived,
            reply_axis_config.min_step_len_ticks,
            reply_axis_config.max_accel_ticks,
            reply_axis_config.velocity_acheived);
        itterator += size;

        break;
      case REPLY_AXIS_POS:
        size = sizeof(struct Reply_axis_pos);
        memcpy(&reply_axis_pos, itterator, size);
        printf("Reply_axis_pos\n  type: %u\n  axis: %u\n abs_pos_acheived: %u\n",
            msg_type,
            reply_axis_pos.axis,
            reply_axis_pos.abs_pos_acheived);
        itterator += size;

        break;
      default:
        printf("ERROR: Unexpected reply type: %u\n", msg_type);
        exit(0);
    }
  }
}

