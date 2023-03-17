/* 
 * Based on https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=&cad=rja&uact=8&ved=2ahUKEwi8t8CwxOD9AhWxQ0EAHWFHBVQQFnoECA4QAQ&url=https%3A%2F%2Fwww.cs.cmu.edu%2Fafs%2Fcs%2Facademic%2Fclass%2F15213-f99%2Fwww%2Fclass26%2Fudpclient.c&usg=AOvVaw2MXuxXL8tVHtLjHFYKrEpg
 * udpclient.c - A simple UDP client
 * usage: udpclient <host> <port>
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

#include "../shared/messages.h"

#define BUFSIZE 1024
#define MAX_DATA 16
#define MAX_AXIS 8
#define MAX_TARGETS (MAX_AXIS + 1)
#define TARGET_GLOBAL = (MAX_TARGETS - 1)


/* 
 * error - wrapper for perror
 */
void error(char *msg) {
    perror(msg);
    exit(0);
}

uint all_digits(char* buf) {
  char* itterator = buf;
  while(*itterator != 0 && *itterator != 10) {
    if(! isdigit(*itterator)) {
      printf("Invalid number: %s\n", buf);
      return 0;
    }
    itterator++;
  }
  return 1;
}

uint get_property_uint(const char* msg) {
    char buf[BUFSIZE] = "";
    bzero(buf, BUFSIZE);
    do {
      printf("%s\n > ", msg);
      fgets(buf, BUFSIZE, stdin);
    } while (! all_digits(buf));

    return strtol((char*)buf, (char**)(&buf), 10);
}

size_t populate_message(uint type, void** packet, size_t* packet_space) {
  struct Message_uint message = { type };
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

size_t populate_message_uint(char* text, uint type, void** packet, size_t* packet_space) {
  uint value0 = get_property_uint(text);
  struct Message_uint message = { type, value0 };
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
    char* text0, char* text1, uint type, void** packet, size_t* packet_space) {
  uint value0 = get_property_uint(text0);
  uint value1 = get_property_uint(text1);
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

size_t populate_data(void* packet, size_t* packet_space) {
  uint msg_type;
  size_t message_size = 0;
  size_t packet_size = 0;

  memset(packet, 0, *packet_space);

  while(msg_type = get_property_uint("Packet message type. (Leave empty to finish.): ")) {
    switch(msg_type) {
      case MSG_SET_GLOAL_UPDATE_RATE:
        message_size = populate_message_uint(
            "Set global update rate:", msg_type, &packet, packet_space);
        break;
      case MSG_SET_AXIS_ABS_POS:
        message_size = populate_message_uint_uint(
            "Axis:", "Set absolute position:", msg_type, &packet, packet_space);
        break;
      case MSG_SET_AXIS_REL_POS:
        // TODO.
      case MSG_SET_AXIS_MAX_SPEED:
        // TODO.
      case MSG_SET_AXIS_MAX_ACCEL:
        // TODO.
      case MSG_SET_AXIS_ABS_POS_AT_TIME:
        // TODO.
      case MSG_GET_GLOBAL_CONFIG:
        message_size = populate_message(msg_type, &packet, packet_space);
        break;
      case MSG_GET_AXIS_CONFIG:
        message_size = populate_message_uint(
            "Get configuration for axis:", msg_type, &packet, packet_space);
        break;
      case MSG_GET_AXIS_POS:
        message_size = populate_message_uint(
            "Get absolute position of axis:", msg_type, &packet, packet_space);
        break;
      default:
        printf("Invalid message type: %u\n", msg_type);
        exit(0);

    }
    packet_size += message_size;
    printf("  %lu\t%lu\t%lu\n", message_size, *packet_space, packet_size);
  }

  return packet_size;
}

void display_data(void* packet, size_t packet_size) {
  size_t packet_parsed = 0;
  size_t message_size;

  while(packet_parsed < packet_size) {
    uint msg_type = *(uint*)packet;
    switch(msg_type) {
      case MSG_SET_GLOAL_UPDATE_RATE:
        struct Message_uint message_1;
        message_size = sizeof(struct Message_uint);
        memcpy(&message_1, packet, message_size);
        printf("msg type: %u\tvalue0: %u\n",
            message_1.type, message_1.value0);
        break;
      case MSG_SET_AXIS_ABS_POS:
        struct Message_uint_uint message_2;
        message_size = sizeof(struct Message_uint_uint);
        memcpy(&message_2, packet, message_size);
        printf("msg type: %u\tvalue0: %u\tvalue1: %u\n",
            message_2.type, message_2.value0, message_2.value1);
        break;
      case MSG_SET_AXIS_REL_POS:
      case MSG_SET_AXIS_MAX_SPEED:
      case MSG_SET_AXIS_MAX_ACCEL:
      case MSG_SET_AXIS_ABS_POS_AT_TIME:
      case MSG_GET_GLOBAL_CONFIG:
      case MSG_GET_AXIS_CONFIG:
      case MSG_GET_AXIS_POS:
      default:
        printf("Invalid message type: %u\n", msg_type);
        exit(0);
    }
    packet += message_size;
    packet_parsed += message_size;
  }
}


int main(int argc, char **argv) {
    int sockfd, portno, n;
    int serverlen;
    struct sockaddr_in serveraddr;
    struct hostent *server;
    char *hostname;
    char buf[BUFSIZE];
    char packet[BUFSIZE];
    // Leave room for an empty terminating record.
    size_t packet_space = BUFSIZE - sizeof(struct Message);

    /* check command line arguments */
    if (argc != 3) {
       fprintf(stderr,"usage: %s <hostname> <port>\n", argv[0]);
       exit(0);
    }
    hostname = argv[1];
    portno = atoi(argv[2]);

    /* socket: create the socket */
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
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


    size_t packet_size = populate_data(packet, &packet_space);
    display_data(packet, packet_size);


    /* send the message to the server */
    serverlen = sizeof(serveraddr);
    n = sendto(
        sockfd,
        (void*)packet,
        packet_size,
        0,
        (struct sockaddr *)&serveraddr,
        serverlen);
    if (n < 0) {
      error("ERROR in sendto");
    }
    
    /* print the server's reply */
    int flags = MSG_DONTWAIT;
    while(1) {
      n = recvfrom(sockfd, buf, BUFSIZE, flags, (struct sockaddr *)&serveraddr, &serverlen);
      if (n < 0 && errno != EAGAIN) {
        error("ERROR in recvfrom");
      }
      if(n > 0) {
        printf("Echo from server:\r\n %s\r\n", buf);
      }
      memset(buf, '\0', BUFSIZE);
    }

    return 0;
}
