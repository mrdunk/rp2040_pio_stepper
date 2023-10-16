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
#include <limits.h>
#include <math.h>
#include <netdb.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <signal.h>

#include "../shared/messages.h"

#define BUFSIZE 1024
#define MAX_AXIS 4
#define MAX_ACCELERATION 200
#define LOOP_LEN 1000

/* Forward defines. */
void get_reply_non_block(struct sockaddr_in* serveraddr, int sockfd);
size_t serialize_data(void* values, void** packet, size_t* packet_space);
uint8_t send_data(
    struct sockaddr_in* serveraddr, int sockfd, char* packet, size_t packet_size);
void display_data(void* packet, size_t packet_size);
/* --- */

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

/* Get user input. */
uint32_t get_property_uint(const char* msg) {
    char buf[BUFSIZE] = "";
    bzero(buf, BUFSIZE);
    do {
      printf("%s\n > ", msg);
      fgets(buf, BUFSIZE, stdin);
    } while (! all_digits(buf, 0));

    return strtol((char*)buf, (char**)(&buf), 10);
}

/* Get user input. */
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

void log_data_init() {
  // Create named pipes.
  const char * fifo_c_to_py = "/tmp/fifo_c_to_py";
  const char * fifo_py_to_c = "/tmp/fifo_py_to_c";
  mkfifo(fifo_c_to_py, 0666);
  mkfifo(fifo_py_to_c, 0666);

  sigaction(SIGPIPE, &(struct sigaction){SIG_IGN}, NULL);
}

/* Send debug information to the Linux FIFO for drawing graphs elsewhere. */
void log_data_write(const uint8_t axis, const uint8_t data_type, const int32_t value) {
  static uint32_t count = 0;
  const char * fifo_c_to_py = "/tmp/fifo_c_to_py";

  static FILE* fifo_write;
  if(!fifo_write) {
    fifo_write = fopen(fifo_c_to_py, "wb");
    //setbuf(fifo_write, NULL); // make it unbuffered

    if(fifo_write == NULL) {
      printf("ERROR: Could not open %s\n", fifo_c_to_py);
      return;
    }
  }

  char tx_buffer[11] = {0};
  memcpy(tx_buffer, &count, 4);
  tx_buffer[4] = axis;
  tx_buffer[5] = data_type;
  memcpy(tx_buffer + 6, &value, 4);
  tx_buffer[10] = 0;

  int rc = -1;
  while(rc < 0) {
    rc = fwrite(tx_buffer, 1, 10, fifo_write);
  }

  if(count % 500 == 0) {
    fclose( fifo_write );
    fifo_write = 0;
  }
  count++;
}

/* Get user feedback from the Linux FIFO for transmission to the RP2040. */
void log_data_read(struct sockaddr_in* serveraddr, int sockfd) {
  static uint32_t count = 0;
  const char * fifo_py_to_c = "/tmp/fifo_py_to_c";

  uint32_t axis = 1;
  size_t packet_size = 0;
  int32_t values[4] = {0,0,0,0};
  // Leave room for an empty terminating record.
  size_t packet_space = BUFSIZE - sizeof(struct Message);

  static int fifo_read;
  if(!fifo_read) {
    fifo_read = open(fifo_py_to_c, O_RDONLY | O_NONBLOCK);

    if(fifo_read == 0) {
      printf("ERROR: Could not open %s\n", fifo_py_to_c);
      return;
    }
  }

  char buffer[8] = {0};
  ssize_t r = read(fifo_read, buffer, 8);
  if(r > 0) {
    uint32_t data_type;
    float value;
    memcpy(&data_type, buffer, 4);
    memcpy(&value, buffer + 4, 4);

    values[0] = data_type + MSG_SET_AXIS_PID_KP;
    values[1] = axis;
    memcpy(&((uint32_t*)values)[2], &value, 4);

    char packet[BUFSIZE];
    void* packet_itterator = packet;
    memset(packet, 0, packet_space);
    packet_size += serialize_data(values, &packet_itterator, &packet_space);
    packet_itterator = packet;
    //display_data(packet_itterator, packet_size);
    send_data(serveraddr, sockfd, packet, packet_size);
  }

  if(count % 500 == 0) {
    close( fifo_read );
    fifo_read = 0;
  }
  count++;
}

size_t populate_data(void* packet) {
  uint32_t msg_type;
  size_t message_size = 0;
  size_t packet_size = 0;
  // Leave room for an empty terminating record.
  size_t packet_space = BUFSIZE - sizeof(struct Message);

  memset(packet, 0, BUFSIZE);

  while((msg_type = get_property_uint("\nPacket message type. (Leave empty to finish.): "))) {
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
      case MSG_GET_GLOBAL_CONFIG:
        message_size = populate_message(msg_type, &packet, &packet_space);
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

/* Serialize data and write to packet for later transmission. */
size_t serialize_data(void* values, void** packet, size_t* packet_space) {
  //printf("serialize_data [%i, %i, %i, %i]\n",
  //    ((uint32_t*)values)[0], ((uint32_t*)values)[1],
  //      ((uint32_t*)values)[2], ((uint32_t*)values)[3]);
  struct Message message;
  struct Message_timing message_timing;
  struct Message_uint message_uint;
  struct Message_uint_uint message_uint_uint;
  struct Message_uint_int message_uint_int;
  struct Message_set_kp message_set_kp;
  size_t message_size = 0;
  uint32_t msg_type = ((uint32_t*)values)[0];
  uint32_t uint_value;
  uint32_t update_id;
  float float_value;
  uint32_t axis;
  uint32_t time;
  int32_t int_value;

  switch(msg_type) {
    case MSG_TIMING:
      update_id = ((uint32_t*)values)[1];
      time = ((uint32_t*)values)[2];
      uint_value = ((uint32_t*)values)[1];
      message_timing = (struct Message_timing){.type=msg_type, .update_id=update_id, .time=time};
      message_size = sizeof(struct Message_timing);
      memcpy(*packet, &message_timing, message_size);
      break;
    case MSG_SET_GLOAL_UPDATE_RATE:
      uint_value = ((uint32_t*)values)[1];
      message_uint = (struct Message_uint){.type=msg_type, .value=uint_value};
      message_size = sizeof(struct Message_uint);
      memcpy(*packet, &message_uint, message_size);
      break;
    case MSG_SET_AXIS_ENABLED:
      axis = ((uint32_t*)values)[1];
      uint_value = ((uint32_t*)values)[2];
      message_uint_uint = 
        (struct Message_uint_uint){.type=msg_type, .axis=axis, .value=uint_value};
      message_size = sizeof(struct Message_uint_uint);
      memcpy(*packet, &message_uint_uint, message_size);
      break;
    case MSG_SET_AXIS_ABS_POS:
      axis = ((uint32_t*)values)[1];
      uint_value = ((uint32_t*)values)[2];
      message_uint_uint = 
        (struct Message_uint_uint){.type=msg_type, .axis=axis, .value=uint_value};
      message_size = sizeof(struct Message_uint_uint);
      memcpy(*packet, &message_uint_uint, message_size);
      log_data_write(axis, 1, uint_value);
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
    case MSG_SET_AXIS_PID_KP:
      axis = ((uint32_t*)values)[1];
      memcpy(&float_value, &((uint32_t*)values)[2], 4);
      message_set_kp = 
        (struct Message_set_kp){.type=msg_type, .axis=axis, .value=float_value};
      message_size = sizeof(struct Message_set_kp);
      memcpy(*packet, &message_set_kp, message_size);
      break;
    case MSG_SET_AXIS_IO_STEP:
    case MSG_SET_AXIS_IO_DIR:
      axis = ((uint32_t*)values)[1];
      int_value = ((int32_t*)values)[2];
      message_uint_int = 
        (struct Message_uint_int){.type=msg_type, .axis=axis, .value=int_value};
      message_size = sizeof(struct Message_uint_int);
      memcpy(*packet, &message_uint_int, message_size);
      break;
    case MSG_GET_GLOBAL_CONFIG:
      message = (struct Message){.type=msg_type};
      message_size = sizeof(struct Message);
      memcpy(*packet, &message, message_size);
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

size_t populate_data_oneshot(void* packet) {
  size_t packet_size = 0;
  int32_t values[4] = {0,0,0,0};
  size_t value_num = 0;
  uint32_t val = 0;
  // Leave room for an empty terminating record.
  size_t packet_space = BUFSIZE - sizeof(struct Message);

  memset(packet, 0, packet_space);

  char buf[BUFSIZE] = "";
  bzero(buf, BUFSIZE);
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
      packet_size += serialize_data(values, &packet, &packet_space);
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
      packet_size += serialize_data(values, &packet, &packet_space);
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

/* Set IO pins and set enable flag on the RP. */
size_t initialize_rp(void* packet, int sockfd) {
  size_t packet_space = BUFSIZE - sizeof(struct Message);
  memset(packet, 0, BUFSIZE);
  //size_t packet_size = 2;  // The first uint16_t will contain the data length.
  size_t packet_size = 0;
  void* packet_itterator = packet + packet_size;

  uint32_t io_pos_step[MAX_AXIS] = {0, 2, 4, 6};
  uint32_t io_pos_dir[MAX_AXIS] = {1, 3, 5, 7};

  for(uint32_t axis = 0; axis < MAX_AXIS; axis++) {
    struct Message_uint_int v;
    v.type = MSG_SET_AXIS_IO_STEP;
    v.axis = axis;
    v.value = io_pos_step[axis];
    packet_size += serialize_data(&v, &packet_itterator, &packet_space);
  }

  for(uint32_t axis = 0; axis < MAX_AXIS; axis++) {
    struct Message_uint_int v;
    v.type = MSG_SET_AXIS_IO_DIR;
    v.axis = axis;
    v.value = io_pos_dir[axis];
    packet_size += serialize_data(&v, &packet_itterator, &packet_space);
  }

  for(uint32_t axis = 0; axis < MAX_AXIS; axis++) {
    struct Message_uint_uint v;
    v.type = MSG_SET_AXIS_ENABLED;
    v.axis = axis;
    v.value = 1;
    packet_size += serialize_data(&v, &packet_itterator, &packet_space);
  }

  return packet_size;
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
  struct timespec now;

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
    axis_speed[4] = 10000;
    axis_speed[5] = 10000;
    axis_destination[0] = axis_pos[0] + 10000;
    axis_destination[1] = axis_pos[1] + 10000;
    axis_destination[2] = axis_pos[2] + 10000;
    axis_destination[3] = axis_pos[3] + 10000;
    axis_destination[4] = axis_pos[4] + 10000;
    axis_destination[5] = axis_pos[5] + 10000;

    struct timespec last_time;
    clock_gettime(CLOCK_REALTIME, &last_time);
    last_time_us = 1000000 * last_time.tv_sec + last_time.tv_nsec / 1000;
  }

  size_t packet_space = BUFSIZE - sizeof(struct Message);
  memset(packet, 0, BUFSIZE);
  //size_t packet_size = 2;  // The first uint16_t will contain the data length.
  size_t packet_size = 0;
  void* packet_itterator = packet + packet_size;

  // Put metrics message in packet.
  clock_gettime(CLOCK_REALTIME, &now);
  uint64_t now_time_ns = 1000000000 * now.tv_sec + now.tv_nsec;
  uint32_t values[4] = {
    MSG_TIMING,
    run_count,
    now_time_ns,
    0
  };
  packet_size += serialize_data(values, &packet_itterator, &packet_space);

  // Put axis position messages in packet.
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

    //printf("%u\t%u\t%u\n", axis, axis_direction[axis], axis_pos[axis]);
  }

  // Note: If there is more than two data transmissions in a single time window,
  // this will not keep up.
  // (Only one expected.)
  get_reply_non_block(clientaddr, sockfd);
  get_reply_non_block(clientaddr, sockfd);

  clock_gettime(CLOCK_REALTIME, &now);
  uint64_t now_time_us = 1000000 * now.tv_sec + now.tv_nsec / 1000;

  usleep(1);
  while(now_time_us - last_time_us < LOOP_LEN) {
    clock_gettime(CLOCK_REALTIME, &now);
    now_time_us = 1000000 * now.tv_sec + now.tv_nsec / 1000;
  }
  printf("td: %40li\n", now_time_us - last_time_us);
  last_time_us = now_time_us;

  run_count++;

  return packet_size;
}

void display_data(void* packet, size_t packet_size) {
  size_t packet_parsed = 0;
  size_t message_size;
  struct Message_timing message_timing;
  struct Message_uint message;
  struct Message_uint message_uint;
  //struct Message_uint_uint message_uint_uint;
  struct Message_uint_int message_uint_int;
  struct Message_set_kp message_set_kp;

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
      case MSG_TIMING:
        message_size = sizeof(struct Message_timing);
        memcpy(&message_timing, packet, message_size);
        printf("  msg type: %u\tupdate_id: %u\ttime: %u\n",
            message_timing.type, message_timing.update_id, message_timing.time);
        break;
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
        message_size = sizeof(struct Message_set_kp);
        memcpy(&message_set_kp, packet, message_size);
        printf("  msg type: %u\taxis: %u\tvalue: %f\n",
            message_set_kp.type, message_set_kp.axis, message_set_kp.value);
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

void display_reply(char* buf) {
  struct Reply_metrics reply_metrics;
  struct Reply_global_config reply_global_config;
  struct Reply_axis_config reply_axis_config;
  struct Reply_axis_pos reply_axis_pos;
  char* itterator = buf;
  size_t size;
  uint32_t msg_type;
  while((msg_type = *(uint32_t*)itterator)) {
    switch(msg_type) {
      case REPLY_METRICS:
        size = sizeof(struct Reply_metrics);
        memcpy(&reply_metrics, itterator, size);
        /*
        printf("Reply_metrics: %i\t%i\n", reply_metrics.update_id, reply_metrics.time_diff);
        */
        itterator += size;
        break;
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
        /*
        printf("Reply_axis_config\n"
            "  type: %u\n  axis: %u\n  abs_pos_acheived: %u\n  min_step_len_ticks: %u\n"
            "  max_accel_ticks: %u\n  velocity: %i\n",
            msg_type,
            reply_axis_config.axis,
            reply_axis_config.abs_pos_acheived,
            reply_axis_config.min_step_len_ticks,
            reply_axis_config.max_accel_ticks,
            reply_axis_config.velocity_acheived);
        */
        itterator += size;

        log_data_write(reply_axis_config.axis, 0, reply_axis_config.abs_pos_acheived);
        log_data_write(reply_axis_config.axis, 2, reply_axis_config.velocity_acheived);

        break;
      case REPLY_AXIS_POS:
        size = sizeof(struct Reply_axis_pos);
        memcpy(&reply_axis_pos, itterator, size);
        printf("Reply_axis_pos\n  type: %u\n  axis: %u\n abs_pos_acheived: %u\n",
            msg_type,
            reply_axis_pos.axis,
            reply_axis_pos.abs_pos_acheived);
        itterator += size;

        log_data_write(reply_axis_config.axis, 0, reply_axis_config.abs_pos_acheived);

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

void get_reply(struct sockaddr_in* clientaddr, int sockfd) {
  /* print the server's reply */
  int addr_len;
  char buf[BUFSIZE];
  int flags = 0;  //MSG_DONTWAIT;
  while(1) {
    memset(buf, '\0', BUFSIZE);
    int n = recvfrom(
        sockfd, buf, BUFSIZE, flags, (struct sockaddr *)&clientaddr, (socklen_t *)&addr_len);
    if (n < 0 && errno != EAGAIN) {
      error("ERROR in recvfrom");
    }
    if(n > 0) {
      //printf("Echo from server:\r\n %s\r\n", buf);
      printf("Binary reply received.\n");
      display_reply(buf);

      if(clientaddr) {
        //printf("Received from %s:%d\n",
        //    inet_ntoa(clientaddr->sin_addr), ntohs(clientaddr->sin_port));
      }
    }
  }
}

void get_reply_non_block(struct sockaddr_in* clientaddr, int sockfd) {
  int addr_len;
  char buf[BUFSIZE];
  memset(buf, '\0', BUFSIZE);
  int flags = MSG_DONTWAIT;
  int n = recvfrom(
      sockfd, buf, BUFSIZE, flags, (struct sockaddr *)clientaddr, (socklen_t *)&addr_len);
  if (n < 0 && errno != EAGAIN) {
    //error("ERROR in recvfrom");
    printf("rx error: %i\n", n);
  }
  if(n > 0) {
    //printf("Echo from server:\r\n %s\r\n", buf);
    //printf("Binary reply received.\n");
    display_reply(buf);
  }
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

  // Set up FIFOs to stream data to graphs.
  log_data_init();

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

  // Make socket non-blocking.
  /*
  int on = 1;
  if(ioctl(sockfd, FIONBIO, (char *)&on) < 0) {
    perror("ioctl() failed");
    close(sockfd);
    exit(-1);
  }
  */

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

  packet_size = initialize_rp(packet, sockfd);
  send_data(&serveraddr, sockfd, packet, packet_size);

  if(oneshot) {
    packet_size = populate_data_oneshot(packet);
    display_data(packet, packet_size);
    send_data(&serveraddr, sockfd, packet, packet_size);
    get_reply(&serveraddr, sockfd);
  } else if(loop) {
    while(1) {
      packet_size = populate_data_loop(packet, &serveraddr, sockfd);
      //display_data(packet, packet_size);
      send_data(&serveraddr, sockfd, packet, packet_size);
      log_data_read(&serveraddr, sockfd);
    }
  } else {
    packet_size = populate_data(packet);
    display_data(packet, packet_size);
    send_data(&serveraddr, sockfd, packet, packet_size);
    get_reply(&clientaddr, sockfd);
  }

  return 0;
}
