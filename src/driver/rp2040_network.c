#include <stdio.h>
#include <netdb.h>
#include <string.h>

#include "rp2040_defines.h"
#include "../shared/messages.h"

#ifdef BUILD_TESTS

#include "../test/mocks/driver_mocks.h"

#endif  // BUILD_TESTS


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
      sockfd[device],
      receive_buffer,
      BUFSIZE,
      flags,
      (struct sockaddr *)&remote_addr[device],
      (socklen_t *)&addr_len);
  if (receive_count < 0 && errno != EAGAIN) {
    rtapi_print_msg(RTAPI_MSG_ERR, "ERROR receiving on network port %i\n ", sockfd[device]);
    return 0;
  }
  return receive_count;
}

size_t serialize_data(void* message, size_t message_size, void** packet, size_t* packet_space) {
  if(*packet_space < message_size) {
    printf("ERROR: No space left in packet\n");
    return 0;
  }

  memcpy(*packet, message, message_size);

  *packet_space -= message_size;
  *packet += message_size;

  return message_size;
}

size_t serialize_timing(
    union MessageAny* message,
    uint32_t update_id,
    uint32_t time,
    void** packet,
    size_t* packet_space
) {
  message->timing.type = MSG_TIMING;
  message->timing.update_id = update_id;
  message->timing.time = time;

  return serialize_data(message, sizeof(struct Message_timing), packet, packet_space);
}

size_t serialize_joint_pos(
    union MessageAny* message,
    uint32_t axis,
    double position,
    void** packet,
    size_t* packet_space
) {
  message->set_abs_pos.type = MSG_SET_AXIS_ABS_POS;
  message->set_abs_pos.axis = axis;
  message->set_abs_pos.value = position;

  return serialize_data(message, sizeof(struct Message_set_abs_pos), packet, packet_space);
}

size_t serialize_joint_velocity(
    union MessageAny* message,
    uint32_t axis,
    double velocity,
    void** packet,
    size_t* packet_space
) {
  message->set_velocity.type = MSG_SET_AXIS_VELOCITY;
  message->set_velocity.axis = axis;
  message->set_velocity.value = velocity;

  return serialize_data(message, sizeof(struct Message_set_velocity), packet, packet_space);
}

size_t serialize_joint_max_velocity(
    union MessageAny* message,
    uint32_t axis,
    double velocity,
    void** packet,
    size_t* packet_space
) {
  message->set_velocity.type = MSG_SET_AXIS_MAX_VELOCITY;
  message->set_velocity.axis = axis;
  message->set_velocity.value = velocity;

  return serialize_data(message, sizeof(struct Message_set_max_velocity), packet, packet_space);
}

size_t serialize_joint_max_accel(
    union MessageAny* message,
    uint32_t axis,
    double accel,
    void** packet,
    size_t* packet_space
) {
  message->set_velocity.type = MSG_SET_AXIS_MAX_VELOCITY;
  message->set_velocity.axis = axis;
  message->set_velocity.value = accel;

  return serialize_data(message, sizeof(struct Message_set_max_accel), packet, packet_space);
}

size_t serialize_joint_io_step(
    union MessageAny* message,
    uint32_t axis,
    uint8_t value,
    void** packet,
    size_t* packet_space
) {
  message->joint_gpio.type = MSG_SET_AXIS_IO_STEP;
  message->joint_gpio.axis = axis;
  message->joint_gpio.value = value;

  return serialize_data(message, sizeof(struct Message_joint_gpio), packet, packet_space);
}

size_t serialize_joint_io_dir(
    union MessageAny* message,
    uint32_t axis,
    uint8_t value,
    void** packet,
    size_t* packet_space
) {
  message->joint_gpio.type = MSG_SET_AXIS_IO_DIR;
  message->joint_gpio.axis = axis;
  message->joint_gpio.value = value;

  return serialize_data(message, sizeof(struct Message_joint_gpio), packet, packet_space);
}

size_t serialize_joint_enable(
    union MessageAny* message,
    uint32_t axis,
    uint8_t value,
    void** packet,
    size_t* packet_space
) {
  message->joint_gpio.type = MSG_SET_AXIS_ENABLED;
  message->joint_gpio.axis = axis;
  message->joint_gpio.value = value;

  return serialize_data(message, sizeof(struct Message_joint_enable), packet, packet_space);
}

void process_data(char* buf, skeleton_t* data, int debug) {
  struct Reply_metrics reply_metrics;
  struct Reply_axis_config reply_axis_config;
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
        *data->joint_pos_feedback[reply_axis_config.axis] =
          ((double)reply_axis_config.abs_pos_acheived - (UINT_MAX / 2))
          / *data->joint_scale[reply_axis_config.axis];

        //*data->joint_pos_error[reply_axis_config.axis] =
        //  reply_axis_config.pos_error;

        *data->joint_step_len_ticks[reply_axis_config.axis] =
          reply_axis_config.step_len_ticks;

        *data->joint_velocity_cmd[reply_axis_config.axis] =
          (float)reply_axis_config.velocity_requested;

        *data->joint_velocity_feedback[reply_axis_config.axis] =
          (float)reply_axis_config.velocity_acheived;

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

