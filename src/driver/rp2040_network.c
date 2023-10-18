#include <stdio.h>
#include <netdb.h>
#include <string.h>

#include "rp2040_defines.h"
#include "../shared/messages.h"

/* Network globals. */
struct sockaddr_in remote_addr[MAX_DEVICES];
int sockfd[MAX_DEVICES] = {-1};


/* Initialise network for UDP. */
int init_eth(int device) {
  char *hostname = "192.168.12.2";
  int portno = 5002;

  /* socket: create the NW socket */
  sockfd[device] = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (sockfd[device] < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "ERROR opening socket\n");
    return -1;
  }

  /* Set zero timeout on socket. */
  struct timeval timeout;
  timeout.tv_sec  = 0;
  timeout.tv_usec = 0;
  int rc = setsockopt(sockfd[device], SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(struct timeval));
  if (rc < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "ERROR setting SOL_SOCKET, SO_RCVTIMEO\n");
    return -1;
  }

  /* SO_REUSEADDR should allow reuse of IP/port combo when quickly stopping and
   * restarting program. */
  int option = 1;
  rc = setsockopt(sockfd[device], SOL_SOCKET, SO_REUSEADDR, (void *)&option, sizeof(option));
  if (rc < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "ERROR setting SOL_SOCKET, SO_REUSEADDR\n");
    return -1;
  }

  /* gethostbyname: get the server's DNS entry */
  struct hostent *server = gethostbyname(hostname);
  if (server == NULL) {
    rtapi_print_msg(RTAPI_MSG_ERR, "ERROR, no such host as %s\n", hostname);
    return -1;
  }

  /* build the server's Internet address */
  memset(&remote_addr[device], 0, sizeof(remote_addr[device]));

  remote_addr[device].sin_family = AF_INET;  // IPv4
  memmove((char *)&remote_addr[device].sin_addr.s_addr, (char *)server->h_addr, server->h_length);
  remote_addr[device].sin_port = htons(portno);

  return sockfd[device];
}

/* Send data via UDP.*/
uint8_t send_data(int device, char* packet, size_t packet_size) {
  int addr_len = sizeof(remote_addr[device]);
  int n = sendto(
      sockfd[device],
      (void*)packet,
      packet_size,
      0,
      (struct sockaddr *)&remote_addr[device],
      addr_len);
  if (n < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "ERROR sending on network port %i\n ", sockfd[device]);
    return 1;
  }
  return 0;
}

/* Get data via UDP. */
size_t get_reply_non_block(int device, char* receive_buffer) {
  int addr_len;
  int flags = MSG_DONTWAIT;
  ssize_t receive_count = recvfrom(
      sockfd[device], receive_buffer, BUFSIZE, flags, &remote_addr[device], (socklen_t *)&addr_len);
  if (receive_count < 0 && errno != EAGAIN) {
    rtapi_print_msg(RTAPI_MSG_ERR, "ERROR receiving on network port %i\n ", sockfd[device]);
    return 0;
  }
  return receive_count;
}

size_t serialize_data(void* values, void** packet, size_t* packet_space) {
  struct Message message;
  struct Message_timing message_timing;
  struct Message_uint message_uint;
  struct Message_uint_uint message_uint_uint;
  struct Message_uint_int message_uint_int;
  struct Message_set_kp message_uint_float;
  size_t message_size = 0;
  uint32_t msg_type = ((uint32_t*)values)[0];
  uint32_t uint_value;
  int32_t int_value;
  float float_value;
  union MessageAny message_any;
  uint32_t axis, update_id, time;

  switch(msg_type) {
    case MSG_TIMING:
      message_size = sizeof(struct Message_set_max_velocity);
      break;
    case MSG_SET_AXIS_ENABLED:
      message_size = sizeof(struct Message_joint_enable);
      break;
    case MSG_SET_AXIS_ABS_POS:
      message_size = sizeof(struct Message_set_abs_pos);
      break;
    case MSG_SET_AXIS_REL_POS:
      message_size = sizeof(struct Message_set_rel_pos);
      break;
    case MSG_SET_AXIS_MAX_SPEED:
      message_size = sizeof(struct Message_set_max_velocity);
      break;
    case MSG_SET_AXIS_MAX_ACCEL:
      message_size = sizeof(struct Message_set_max_accel);
      break;
    case MSG_SET_AXIS_PID_KP:
      message_size = sizeof(struct Message_set_kp);
      break;
    case MSG_SET_AXIS_IO_STEP:
    case MSG_SET_AXIS_IO_DIR:
      message_size = sizeof(struct Message_joint_gpio);
      break;
    default:
      printf("Invalid message type: %u\n", msg_type);
      return 0;
  }

  if(*packet_space < message_size) {
    printf("ERROR: No space left in packet\n");
    return 0;
  }

  memcpy(*packet, values, message_size);

  packet_space -= message_size;
  *packet += message_size;

  return message_size;
}

void process_data(char* buf, skeleton_t* data, int debug) {
  struct Reply_metrics reply_metrics;
  struct Reply_global_config reply_global_config;
  struct Reply_axis_config reply_axis_config;
  struct Reply_axis_pos reply_axis_pos;
  char* itterator = buf;
  size_t size;
  uint32_t msg_type;
  size_t msg_count = 0;
  while((msg_type = *(uint32_t*)itterator)) {  // First uint32_t of each massage is the type.
    switch(msg_type) {
      case REPLY_METRICS:
        size = sizeof(struct Reply_metrics);
        memcpy(&reply_metrics, itterator, size);
        if(debug) {
          printf("Reply_metrics: %i\t%i\n", reply_metrics.update_id, reply_metrics.time_diff);
        }
        *data->metric_update_id = reply_metrics.update_id;
        *data->metric_time_diff = reply_metrics.time_diff;
        *data->metric_rp_update_len = reply_metrics.rp_update_len;
        itterator += size;
        break;
      case REPLY_GLOBAL_CONFIG:
        size = sizeof(struct Reply_global_config);
        memcpy(&reply_global_config, itterator, size);
        if(debug) {
          printf("Reply_global_config\n  "
              "type: %u\n  update_rate: %u\n  update_time_us: %u  update_time_ticks: %u\n",
              msg_type,
              reply_global_config.update_rate,
              reply_global_config.update_time_us,
              reply_global_config.update_rate);
        }
        itterator += size;
        break;
      case REPLY_AXIS_CONFIG:
        size = sizeof(struct Reply_axis_config);
        memcpy(&reply_axis_config, itterator, size);
        if(debug) {
          printf("Reply_axis_config\n"
              "  type: %u\n  axis: %u\n  abs_pos_acheived: %u\n  min_step_len_ticks: %u\n"
              "  max_accel_ticks: %u\n  velocity: %i\n",
              msg_type,
              reply_axis_config.axis,
              reply_axis_config.abs_pos_acheived,
              reply_axis_config.max_velocity,
              reply_axis_config.max_accel_ticks,
              reply_axis_config.velocity_acheived);
        }
        *data->feedback[reply_axis_config.axis] =
          ((double)reply_axis_config.abs_pos_acheived - (UINT_MAX / 2))
          / *data->scale[reply_axis_config.axis];
        *data->calculated_velocity[reply_axis_config.axis] =
          (float)reply_axis_config.velocity_requested;
        *data->fb_velocity[reply_axis_config.axis] =
          (float)reply_axis_config.velocity_acheived;
        itterator += size;

        break;
      case REPLY_AXIS_POS:
        size = sizeof(struct Reply_axis_pos);
        memcpy(&reply_axis_pos, itterator, size);
        if(debug) {
          printf("Reply_axis_pos\n  type: %u\n  axis: %u\n abs_pos_acheived: %u\n",
              msg_type,
              reply_axis_pos.axis,
              reply_axis_pos.abs_pos_acheived);
        }
        itterator += size;

        break;
      default:
        printf("ERROR: Unexpected reply type: %u  msg_count: %u  update_id: %u\n",
            msg_type, msg_count, *data->metric_update_id);
        return;
    }
    msg_count++;
  }
  if(msg_count != 5) {
    printf("msg_count: %u  %u\n", msg_count, *data->metric_update_id);
  }
}

