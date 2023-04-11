/* 
 * Based on https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=&cad=rja&uact=8&ved=2ahUKEwi8t8CwxOD9AhWxQ0EAHWFHBVQQFnoECA4QAQ&url=https%3A%2F%2Fwww.cs.cmu.edu%2Fafs%2Fcs%2Facademic%2Fclass%2F15213-f99%2Fwww%2Fclass26%2Fudpclient.c&usg=AOvVaw2MXuxXL8tVHtLjHFYKrEpg
 * udpclient.c - A simple UDP client
 * usage: udpclient <host> <port>
 *
 * BUILD:
 *  gcc ../test_client/udp.c -lm
 */

#include <ctype.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include <limits.h>
#include <time.h>
#include <math.h>
#include <unistd.h>

#include "../shared/messages.h"

#define BUFSIZE 1024
#define MAX_AXIS 6
#define MAX_ACCELERATION 200
#define LOOP_LEN 1000

/* 
 * error - wrapper for perror
 */
void error(char *msg) {
    perror(msg);
    exit(0);
}

uint32_t all_digits(char* buf, uint32_t sign) {
  char* itterator = buf;
  while(*itterator != 0 && *itterator != 10) {
    if(! isdigit(*itterator) && *itterator != '-') {
      printf("Invalid number: %s\n", buf);
      return 0;
    }
    itterator++;
  }
  return 1;
}

uint32_t get_property_uint(const char* msg) {
    char buf[BUFSIZE] = "";
    bzero(buf, BUFSIZE);
    do {
      printf("%s\n > ", msg);
      fgets(buf, BUFSIZE, stdin);
    } while (! all_digits(buf, 0));

    return strtol((char*)buf, (char**)(&buf), 10);
}

uint32_t get_property_int(const char* msg) {
    char buf[BUFSIZE] = "";
    bzero(buf, BUFSIZE);
    do {
      printf("%s\n > ", msg);
      fgets(buf, BUFSIZE, stdin);
    } while (! all_digits(buf, 1));

    return strtol((char*)buf, (char**)(&buf), 10);
}

size_t populate_message(uint32_t type, void** packet, size_t* packet_space) {
  struct Message message = { type };
  size_t message_size = sizeof(struct Message);
  if(*packet_space < message_size) {
    printf("ERROR: No space left in packet\n");
    exit(0);
  }

  memcpy(*packet, &message, message_size);
  *packet_space -= message_size;
  *packet += message_size;
  return message_size;
}

size_t populate_message_uint(char* text, uint32_t type, void** packet, size_t* packet_space) {
  uint32_t value = get_property_uint(text);
  struct Message_uint message = { type, value };
  size_t message_size = sizeof(struct Message_uint);
  if(*packet_space < message_size) {
    printf("ERROR: No space left in packet\n");
    exit(0);
  }

  memcpy(*packet, &message, message_size);
  *packet_space -= message_size;
  *packet += message_size;
  return message_size;
}

size_t populate_message_uint_uint(
    char* text0, char* text1, uint32_t type, void** packet, size_t* packet_space) {
  uint32_t value0 = get_property_uint(text0);
  uint32_t value1 = get_property_uint(text1);
  struct Message_uint_uint message = { type, value0, value1 };
  size_t message_size = sizeof(struct Message_uint_uint);
  if(*packet_space < message_size) {
    printf("ERROR: No space left in packet\n");
    exit(0);
  }

  memcpy(*packet, &message, message_size);
  *packet_space -= message_size;
  *packet += message_size;
  return message_size;
}

size_t populate_message_uint_int(
    char* text0, char* text1, uint32_t type, void** packet, size_t* packet_space) {
  uint32_t value0 = get_property_uint(text0);
  uint32_t value1 = get_property_int(text1);
  struct Message_uint_int message = { type, value0, value1 };
  size_t message_size = sizeof(struct Message_uint_int);
  if(*packet_space < message_size) {
    printf("ERROR: No space left in packet\n");
    exit(0);
  }

  memcpy(*packet, &message, message_size);
  *packet_space -= message_size;
  *packet += message_size;
  return message_size;
}

size_t populate_data(void* packet) {
  uint32_t msg_type;
  size_t message_size = 0;
  size_t packet_size = 0;
  // Leave room for an empty terminating record.
  size_t packet_space = BUFSIZE - sizeof(struct Message);

  memset(packet, 0, BUFSIZE);

  printf("%s", human_help);

  while(msg_type = get_property_uint("\nPacket message type. (Leave empty to finish.): ")) {
    switch(msg_type) {
      case MSG_SET_GLOAL_UPDATE_RATE:
        message_size = populate_message_uint(
            "Set global update rate:", msg_type, &packet, &packet_space);
        break;
      case MSG_SET_AXIS_ABS_POS:
        message_size = populate_message_uint_uint(
            "Axis:", "Set absolute position:", msg_type, &packet, &packet_space);
        break;
      case MSG_SET_AXIS_REL_POS:
        message_size = populate_message_uint_int(
            "Axis:", "Set relative position:", msg_type, &packet, &packet_space);
        break;
      case MSG_SET_AXIS_MAX_SPEED:
        // TODO.
      case MSG_SET_AXIS_MAX_ACCEL:
        // TODO.
      case MSG_SET_AXIS_ABS_POS_AT_TIME:
        // TODO.
      case MSG_GET_GLOBAL_CONFIG:
        message_size = populate_message(msg_type, &packet, &packet_space);
        break;
      case MSG_GET_AXIS_CONFIG:
        message_size = populate_message_uint(
            "Get configuration for axis:", msg_type, &packet, &packet_space);
        break;
      case MSG_GET_AXIS_POS:
        message_size = populate_message_uint(
            "Get absolute position of axis:", msg_type, &packet, &packet_space);
        break;
      default:
        printf("Invalid message type: %u\n", msg_type);
        exit(0);

    }
    packet_size += message_size;
    //printf("  %lu\t%lu\t%lu\n", message_size, packet_space, packet_size);
  }

  return packet_size;
}

size_t store_message(int64_t* values, void** packet, size_t* packet_space) {
  printf("store_message [%li, %li, %li, %li]\n", values[0], values[1], values[2], values[3]);
  struct Message message;
  struct Message_uint message_uint;
  struct Message_uint_uint message_uint_uint;
  struct Message_uint_int message_uint_int;
  size_t message_size = 0;

  switch(values[0]) {
    case MSG_SET_GLOAL_UPDATE_RATE:
      message_uint = (struct Message_uint){.type=values[0], .value=values[1]};
      message_size = sizeof(struct Message_uint);
      memcpy(*packet, &message_uint, message_size);
      break;
    case MSG_SET_AXIS_ABS_POS:
      message_uint_uint = 
        (struct Message_uint_uint){.type=values[0], .axis=values[1], .value=values[2]};
      message_size = sizeof(struct Message_uint_uint);
      memcpy(*packet, &message_uint_uint, message_size);
      break;
    case MSG_SET_AXIS_REL_POS:
      message_uint_int = 
        (struct Message_uint_int){.type=values[0], .axis=values[1], .value=values[2]};
      message_size = sizeof(struct Message_uint_int);
      memcpy(*packet, &message_uint_int, message_size);
      break;
    case MSG_SET_AXIS_MAX_SPEED:
      // TODO.
    case MSG_SET_AXIS_MAX_ACCEL:
      // TODO.
    case MSG_SET_AXIS_ABS_POS_AT_TIME:
      // TODO.
    case MSG_GET_GLOBAL_CONFIG:
      message = (struct Message){.type=values[0]};
      message_size = sizeof(struct Message);
      memcpy(*packet, &message, message_size);
      break;
    case MSG_GET_AXIS_CONFIG:
      message_uint = (struct Message_uint){.type=values[0], .value=values[1]};
      message_size = sizeof(struct Message_uint);
      memcpy(*packet, &message_uint, message_size);
      break;
    case MSG_GET_AXIS_POS:
      message_uint = (struct Message_uint){.type=values[0], .value=values[1]};
      message_size = sizeof(struct Message_uint);
      memcpy(*packet, &message_uint, message_size);
      break;
    default:
      printf("Invalid message type: %lu\n", values[0]);
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

size_t populate_data_oneshot(void* packet) {
  size_t message_size = 0;
  size_t packet_size = 0;
  int64_t values[4] = {0,0,0,0};
  size_t value_num = 0;
  uint32_t val = 0;
  // Leave room for an empty terminating record.
  size_t packet_space = BUFSIZE - sizeof(struct Message);

  memset(packet, 0, packet_space);

  char buf[BUFSIZE] = "";
  bzero(buf, BUFSIZE);
  printf("%s", human_help);
  printf("Use \":\" to separate values.\n"
      "Use \",\" to seperate messages\n"
      "eg: > 3:0:10, 3:1:-10, 8:0, 8:1\n");
  printf(" > ");
  fgets(buf, BUFSIZE, stdin);
  printf("%s\n", buf);

  char* itterate = buf;

  while(*itterate) {
    if (*itterate == ',') {
      // New message.
      packet_size += store_message(values, &packet, &packet_space);
      values[0] = 0;
      values[1] = 0;
      values[2] = 0;
      values[3] = 0;
      value_num = 0;
      itterate++;
    } else if (*itterate == ':') {
      // Separator.
      itterate++;
    } else if (*itterate == 10) {
      // Enter
      packet_size += store_message(values, &packet, &packet_space);
      break;
    } else if (isspace(*itterate)) {
      // Whitespace. Ignore and continue.
      itterate++;
    } else if (isdigit(*itterate) || *itterate == '-') {
      val = strtol(itterate, &itterate, 10);
      if (value_num < 4) {
        values[value_num] = val;
      } else {
        printf("Too many values. %u\n", val);
        memset(packet, '\0', BUFSIZE);
        return 0;
      }
      value_num++;
    } else {
      printf("Unexpected character: %u : %c\n", *itterate, *itterate);
      memset(packet, '\0', BUFSIZE);
      return 0;
    }
  }

  return packet_size;
}

double sampleNormal() {
    double u = ((double) rand() / (RAND_MAX)) * 2 - 1;
    double v = ((double) rand() / (RAND_MAX)) * 2 - 1;
    double r = u * u + v * v;
    if (r == 0 || r > 1) return sampleNormal();
    double c = sqrt(-2 * log(r) / r);
    return u * c;
}

size_t populate_data_loop(void* packet) {
  static uint32_t axis_pos[MAX_AXIS];
  static int32_t axis_velocity[MAX_AXIS];
  static int32_t axis_acceleration[MAX_AXIS];
  static struct timespec last_time;
  static uint8_t first_run = 1;

  if(first_run == 1) {
    for(uint8_t axis = 0; axis < MAX_AXIS; axis++) {
      axis_pos[axis] = UINT_MAX / 2;
      axis_velocity[axis] = 0;
      axis_acceleration[axis] = 0;
    }
    srand(time(NULL));   // Seed random numbers.
    first_run = 0;

    clock_gettime(CLOCK_REALTIME, &last_time);
  }

  size_t packet_space = BUFSIZE - sizeof(struct Message);
  memset(packet, 0, BUFSIZE);
  //size_t packet_size = 2;  // The first uint16_t will contain the data length.
  size_t packet_size = 0;
  void* packet_itterator = packet + packet_size;

  for(uint8_t axis = 0; axis < MAX_AXIS; axis++) {
    int32_t accel_mod = 0;
    if(rand() % 20 == 0) {
      accel_mod = sampleNormal() * 100;
    }

    axis_acceleration[axis] *= 8;
    axis_acceleration[axis] /= 10;
    axis_acceleration[axis] += accel_mod;

    axis_velocity[axis] *= 18;
    axis_velocity[axis] /= 20;
    axis_velocity[axis] += axis_acceleration[axis];
    
    axis_pos[axis] += axis_velocity[axis];


    /*
    if(axis == 0) {
      printf(
          "axis: %6u "
          "accel_mod: %6i "
          "axis_acceleration: %6i "
          "axis_velocity: %6i "
          "axis_pos: %6u\n", 
          axis,
          accel_mod,
          axis_acceleration[axis],
          axis_velocity[axis],
          axis_pos[axis]);
    }
    */

    int64_t values[4] = {
      MSG_SET_AXIS_ABS_POS,
      axis,
      axis_pos[axis],
      0
    };
    packet_size += store_message(values, &packet_itterator, &packet_space);
  }

  struct timespec now;
  clock_gettime(CLOCK_REALTIME, &now);
  uint64_t now_time_us = 1000000 * now.tv_sec + now.tv_nsec / 1000;
  uint64_t then_time_us = 1000000 * last_time.tv_sec + last_time.tv_nsec / 1000;

  while(now_time_us - then_time_us < LOOP_LEN) {
    clock_gettime(CLOCK_REALTIME, &now);
    now_time_us = 1000000 * now.tv_sec + now.tv_nsec / 1000;
    if(now_time_us - then_time_us < LOOP_LEN * 10 / 9) {
      usleep(1);
    }
  }
  printf("%40li%40li%40li\n", then_time_us, now_time_us, now_time_us - then_time_us);
  last_time = now;

  //((uint16_t*)packet)[0] = packet_size;
  //printf("%lu\n", packet_size);
  return packet_size;
}

void display_data(void* packet, size_t packet_size) {
  size_t packet_parsed = 0;
  size_t message_size;
  struct Message_uint message;
  struct Message_uint message_uint;
  struct Message_uint_uint message_uint_uint;
  struct Message_uint_int message_uint_int;

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
      case MSG_SET_AXIS_ABS_POS_AT_TIME:
        printf("  Invalid message type: %u\n", msg_type);
        exit(0);
      case MSG_GET_GLOBAL_CONFIG:
        message_size = sizeof(struct Message);
        memcpy(&message, packet, message_size);
        printf("  msg type: %u\n", message.type);
        break;
      case MSG_GET_AXIS_CONFIG:
      case MSG_GET_AXIS_POS:
        message_size = sizeof(struct Message_uint);
        memcpy(&message_uint, packet, message_size);
        printf("  msg type: %u\tvalue0: %u\n",
            message_uint.type, message_uint.value);
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

void display_reply(char* buf) {
  char* itterator = buf;
  size_t size;
  uint32_t msg_type;
  while(msg_type = *(uint32_t*)itterator) {
    switch(msg_type) {
      case REPLY_GLOBAL_CONFIG:
        struct Reply_global_config reply_global_config;
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
        struct Reply_axis_config reply_axis_config;
        size = sizeof(struct Reply_axis_config);
        memcpy(&reply_axis_config, itterator, size);
        printf("Reply_axis_config\n"
            "  type: %u\n  axis: %u\n  abs_pos: %u\n  min_step_len_ticks: %u\n"
            "  max_accel_ticks: %u\n  velocity: %u\n",
            msg_type,
            reply_axis_config.axis,
            reply_axis_config.abs_pos,
            reply_axis_config.min_step_len_ticks,
            reply_axis_config.max_accel_ticks,
            reply_axis_config.velocity);
        itterator += size;
        break;
      case REPLY_AXIS_POS:
        struct Reply_axis_pos reply_axis_pos;
        size = sizeof(struct Reply_axis_pos);
        memcpy(&reply_axis_pos, itterator, size);
        printf("Reply_axis_pos\n  type: %u\n  axis: %u\n abs_pos: %u\n",
            msg_type,
            reply_axis_pos.axis,
            reply_axis_pos.abs_pos);
        itterator += size;
        break;
      default:
        printf("ERROR: Unexpected reply type: %u\n", msg_type);
        exit(0);
    }
  }
}

uint8_t send_data(
    struct sockaddr_in* serveraddr, int sockfd, char* packet, size_t packet_size)
{
  /* send the message to the server */
  int serverlen = sizeof(*serveraddr);
  int n = sendto(
      sockfd,
      (void*)packet,
      packet_size,
      0,
      (struct sockaddr *)serveraddr,
      serverlen);
  if (n < 0) {
    error("ERROR in sendto");
    return 1;
  }
  return 0;
}

void get_reply(struct sockaddr_in* serveraddr, int sockfd) {
  /* print the server's reply */
  int serverlen;
  char buf[BUFSIZE];
  int flags = MSG_DONTWAIT;
  while(1) {
    int n = recvfrom(sockfd, buf, BUFSIZE, flags, (struct sockaddr *)&serveraddr, &serverlen);
    if (n < 0 && errno != EAGAIN) {
      error("ERROR in recvfrom");
    }
    if(n > 0) {
      //printf("Echo from server:\r\n %s\r\n", buf);
      printf("Binary reply received.\n");
      display_reply(buf);
    }
    memset(buf, '\0', BUFSIZE);
  }
}

int main(int argc, char **argv) {
  struct sockaddr_in serveraddr;
  struct hostent *server;
  char *hostname;
  char packet[BUFSIZE];
  size_t packet_size;
  uint8_t oneshot = 0;
  uint8_t loop = 0;

  /* check command line arguments */
  if (argc < 3 || argc > 4) {
    fprintf(stderr,"usage: %s <hostname> <port> [-oneshot] [-loop]\n", argv[0]);
    exit(0);
  }
  hostname = argv[1];
  int portno = atoi(argv[2]);

  if(argc == 4) {
    if(strcmp(argv[3], "-oneshot") == 0) {
      oneshot = 1;
    } else if(strcmp(argv[3], "-loop") == 0) {
      loop = 1;
    } else {
      fprintf(stderr,"usage: %s <hostname> <port> [-oneshot] [-loop]\n", argv[0]);
      exit(0);
    }
  }

  /* socket: create the socket */
  int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  if (sockfd < 0) {
    error("ERROR opening socket");
  }

  /* gethostbyname: get the server's DNS entry */
  server = gethostbyname(hostname);
  if (server == NULL) {
    fprintf(stderr,"ERROR, no such host as %s\n", hostname);
    exit(0);
  }

  /* build the server's Internet address */
  bzero((char *) &serveraddr, sizeof(serveraddr));
  serveraddr.sin_family = AF_INET;
  bcopy((char *)server->h_addr, 
      (char *)&serveraddr.sin_addr.s_addr, server->h_length);
  serveraddr.sin_port = htons(portno);


  if(oneshot) {
    packet_size = populate_data_oneshot(packet);
    display_data(packet, packet_size);
    send_data(&serveraddr, sockfd, packet, packet_size);
    get_reply(&serveraddr, sockfd);
  } else if(loop) {
    while(1) {
      packet_size = populate_data_loop(packet);
      //display_data(packet, packet_size);
      send_data(&serveraddr, sockfd, packet, packet_size);
    }
  } else {
    packet_size = populate_data(packet);
    display_data(packet, packet_size);
    send_data(&serveraddr, sockfd, packet, packet_size);
    get_reply(&serveraddr, sockfd);
  }

  return 0;
}
