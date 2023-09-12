/*
 * BUILD:
 *   gcc ../test_client/simple_udp.c ../test_client/debug_methods.c -lm -o simple_udp
 */
#include <errno.h>
#include <limits.h>
#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "../shared/messages.h"
#include "debug_methods.h"

#define BUFSIZE 1024
#define LOOP_TIME 1000
#define MAX_AXIS 4

/*
 * error - wrapper for perror
 */
void error(char *msg) {
    perror(msg);
    exit(0);
}

/* Take data and serialize into appropriate struct.
 *
 * Arguments:
 *   values: A void* array containing 32bit data.
 *           When dereferenced the first 32bits represents the data type as
 *           defined in shared/messages.h.
 */
size_t serialize_data(void* values, void** packet, size_t* packet_space) {
  struct Message message;
  struct Message_uint message_uint;
  struct Message_uint_uint message_uint_uint;
  struct Message_uint_int message_uint_int;
  struct Message_uint_float message_uint_float;
  size_t message_size = 0;
  uint32_t msg_type = ((uint32_t*)values)[0];
  uint32_t uint_value;
  float float_value;
  uint32_t axis;

  switch(msg_type) {
    case MSG_SET_GLOAL_UPDATE_RATE:
      uint_value = ((uint32_t*)values)[1];
      message_uint = (struct Message_uint){.type=msg_type, .value=uint_value};
      message_size = sizeof(struct Message_uint);
      memcpy(*packet, &message_uint, message_size);
      break;
    case MSG_SET_AXIS_ABS_POS:
      axis = ((uint32_t*)values)[1];
      uint_value = ((uint32_t*)values)[2];
      message_uint_uint = 
        (struct Message_uint_uint){.type=msg_type, .axis=axis, .value=uint_value};
      message_size = sizeof(struct Message_uint_uint);
      memcpy(*packet, &message_uint_uint, message_size);
      break;
    case MSG_SET_AXIS_REL_POS:
      axis = ((uint32_t*)values)[1];
      uint_value = ((uint32_t*)values)[2];
      message_uint_int = 
        (struct Message_uint_int){.type=msg_type, .axis=axis, .value=uint_value};
      message_size = sizeof(struct Message_uint_int);
      memcpy(*packet, &message_uint_int, message_size);
      break;
    case MSG_SET_AXIS_MAX_SPEED:
      // TODO.
    case MSG_SET_AXIS_MAX_ACCEL:
      // TODO.
    case MSG_SET_AXIS_ABS_POS_AT_TIME:
      // TODO.
    case MSG_SET_PID_KP:
    case MSG_SET_PID_KI:
    case MSG_SET_PID_KD:
      axis = ((uint32_t*)values)[1];
      memcpy(&float_value, &((uint32_t*)values)[2], 4);
      message_uint_float = 
        (struct Message_uint_float){.type=msg_type, .axis=axis, .value=float_value};
      message_size = sizeof(struct Message_uint_float);
      memcpy(*packet, &message_uint_float, message_size);
      break;
    case MSG_GET_GLOBAL_CONFIG:
      message = (struct Message){.type=msg_type};
      message_size = sizeof(struct Message);
      memcpy(*packet, &message, message_size);
      break;
    case MSG_GET_AXIS_CONFIG:
      uint_value = ((uint32_t*)values)[1];
      message_uint = (struct Message_uint){.type=msg_type, .value=uint_value};
      message_size = sizeof(struct Message_uint);
      memcpy(*packet, &message_uint, message_size);
      break;
    case MSG_GET_AXIS_POS:
      uint_value = ((uint32_t*)values)[1];
      message_uint = (struct Message_uint){.type=msg_type, .value=uint_value};
      message_size = sizeof(struct Message_uint);
      memcpy(*packet, &message_uint, message_size);
      break;
    default:
      printf("Invalid message type: %u\n", msg_type);
      exit(0);
  }

  if(*packet_space < message_size) {
    printf("ERROR: No space left in packet\n");
    exit(0);
  }

  packet_space -= message_size;
  *packet += message_size;

  return message_size;
}

/* Send data via UDP.*/
uint8_t send_data(
    struct sockaddr_in* serveraddr, int sockfd, char* packet, size_t packet_size)
{
  int addr_len = sizeof(*serveraddr);
  int n = sendto(
      sockfd,
      (void*)packet,
      packet_size,
      0,
      (struct sockaddr *)serveraddr,
      addr_len);
  if (n < 0) {
    error("ERROR in sendto");
    return 1;
  }
  return 0;
}

/* Get data via UDP. */
void get_reply_non_block(struct sockaddr_in* clientaddr, int sockfd) {
  int addr_len;
  char buf[BUFSIZE];
  memset(buf, '\0', BUFSIZE);
  int flags = MSG_DONTWAIT;
  int n = recvfrom(
      sockfd, buf, BUFSIZE, flags, (struct sockaddr *)clientaddr, (socklen_t *)&addr_len);
  if (n < 0 && errno != EAGAIN) {
    error("ERROR in recvfrom");
    //printf("rx error: %i\n", n);
  }
  if(n > 0) {
    display_rx_data(buf);
  }
}

/* Oscillate requested position using different step sizes for each axis. */
size_t populate_data_loop(void* packet, struct sockaddr_in* clientaddr, int sockfd) {
  static uint32_t axis_pos[MAX_AXIS];
  static uint8_t axis_direction[MAX_AXIS];
  static uint32_t axis_period[MAX_AXIS];
  static uint32_t axis_speed[MAX_AXIS];
  static uint32_t axis_destination[MAX_AXIS];
  static uint32_t axis_start = UINT_MAX / 2;
  static uint64_t last_time_us;
  static uint32_t run_count = 0;

  if(run_count == 0) {
    printf("Init\n");
    for(uint8_t axis = 0; axis < MAX_AXIS; axis++) {
      axis_pos[axis] = axis_start;
      axis_direction[axis] = (axis % 2);
      axis_period[axis] = (axis + 1) * 1000;
    }
    axis_speed[0] = 1;
    axis_speed[1] = 10;
    axis_speed[2] = 100;
    axis_speed[3] = 1000;
    axis_destination[0] = axis_pos[0] + 10000;
    axis_destination[1] = axis_pos[1] + 10000;
    axis_destination[2] = axis_pos[2] + 10000;
    axis_destination[3] = axis_pos[3] + 10000;

    struct timespec last_time;
    clock_gettime(CLOCK_REALTIME, &last_time);
    last_time_us = 1000000 * last_time.tv_sec + last_time.tv_nsec / 1000;
  }

  size_t packet_space = BUFSIZE - sizeof(struct Message);
  memset(packet, 0, BUFSIZE);
  //size_t packet_size = 2;  // The first uint16_t will contain the data length.
  size_t packet_size = 0;
  void* packet_itterator = packet + packet_size;

  for(uint8_t axis = 0; axis < MAX_AXIS; axis++) {
    if((run_count % axis_period[axis]) == 0) {
      // flip direction.
      axis_direction[axis] = !axis_direction[axis];
    }

    if(axis_direction[axis] == 0) {
      axis_pos[axis] += axis_speed[axis];
      if(axis_pos[axis] > axis_destination[axis]) {
        axis_pos[axis] = axis_destination[axis];
      }
    } else {
      axis_pos[axis] -= axis_speed[axis];
      if(axis_pos[axis] < axis_start) {
        axis_pos[axis] = axis_start;
      }
    }

    int32_t values[4] = {
      MSG_SET_AXIS_ABS_POS,
      axis,
      axis_pos[axis],
      0
    };
    packet_size += serialize_data(values, &packet_itterator, &packet_space);
  }

  // Note: If there is more than two data transmissions in a single time window,
  // this will not keep up.
  // (Only one expected.)
  get_reply_non_block(clientaddr, sockfd);
  get_reply_non_block(clientaddr, sockfd);

  struct timespec now;
  clock_gettime(CLOCK_REALTIME, &now);
  uint64_t now_time_us = 1000000 * now.tv_sec + now.tv_nsec / 1000;

  // Pause until next transmission time.
  // I tried various graceful ways of sleeping here but none returned with
  // any sort of time precision so we just busy wait.
  usleep(1);
  while(now_time_us - last_time_us < LOOP_TIME) {
    clock_gettime(CLOCK_REALTIME, &now);
    now_time_us = 1000000 * now.tv_sec + now.tv_nsec / 1000;
  }
  printf("td: %40li\n", now_time_us - last_time_us);
  last_time_us = now_time_us;

  run_count++;

  return packet_size;
}

int main(int argc, char **argv) {
  struct sockaddr_in serveraddr;
  struct sockaddr_in clientaddr;
  struct hostent *server;
  char *hostname;
  char packet[BUFSIZE];
  size_t packet_size;
  uint8_t oneshot = 0;
  uint8_t loop = 0;

  /* check command line arguments */
  if (argc != 3) {
    fprintf(stderr, "usage: %s <hostname>   <port>\n", argv[0]);
    fprintf(stderr, "eg:    %s 192.168.11.2 5002\n", argv[0]);
    exit(0);
  }
  hostname = argv[1];
  int portno = atoi(argv[2]);

  /* socket: create the NW socket */
  int sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (sockfd < 0) {
    error("ERROR opening socket");
  }

  /* Set zero timeout on socket. */
  struct timeval timeout;
  timeout.tv_sec  = 0;
  timeout.tv_usec = 0;
  setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(struct timeval));

  /* SO_REUSEADDR should allow reuse of IP/port combo when quickly stopping and
   * restarting program. */
  int option = 1;
  setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, (void *)&option, sizeof(option));

  /* gethostbyname: get the server's DNS entry */
  server = gethostbyname(hostname);
  if (server == NULL) {
    fprintf(stderr,"ERROR, no such host as %s\n", hostname);
    exit(0);
  }

  /* build the server's Internet address */
  memset(&serveraddr, 0, sizeof(serveraddr));
  memset(&clientaddr, 0, sizeof(clientaddr));

  serveraddr.sin_family = AF_INET;  // IPv4
  memmove((char *)&serveraddr.sin_addr.s_addr, (char *)server->h_addr, server->h_length);
  serveraddr.sin_port = htons(portno);

  while(1) {
    packet_size = populate_data_loop(packet, &serveraddr, sockfd);
    // display_tx_data(packet, packet_size);
    send_data(&serveraddr, sockfd, packet, packet_size);
  }

  return 0;
}

