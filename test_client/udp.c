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

#define BUFSIZE 1024
#define MAX_DATA 16
#define MAX_TARGETS 8


struct DataEntry {
    uint target;               // Identifies target resource. Eg: axis.
    uint sequence;             // Non zero if following data is related to this one..
    uint key;                  // Identifies data purpose.
    uint value;
};

struct Data {
  uint count;                  // How many DataEntry items are populated.
  struct DataEntry entry[MAX_DATA];
};

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

uint get_property(const char* msg) {
    char buf[BUFSIZE] = "";
    bzero(buf, BUFSIZE);
    do {
      printf("%s\n", msg);
      fgets(buf, BUFSIZE, stdin);
    } while (! all_digits(buf));

    return strtol((char*)buf, (char**)(&buf), 10);
}

void populate_data(struct Data* send_data) {
  struct DataEntry* data_entry;
  uint sequence = 0;
  uint target = UINT_MAX;

  for(uint data_count = 0; data_count < MAX_DATA; data_count++) {
    data_entry = &(send_data->entry[data_count]);
    if(sequence == 0) {
      printf("\n");
      target = get_property("Packet target: ");
      data_entry->target = target;
      if (target >= MAX_TARGETS) {
        data_entry->target = UINT_MAX;
        break;
      }
      sequence = get_property("Number in sequence: ");
      if(sequence <= 1) {
        sequence = 0;
      }
    }

    if(sequence > 0) {
      printf("sequence %u\n", sequence - 1);
      data_entry->sequence = sequence - 1;
      data_entry->target = target;
      data_entry->key = get_property("\tPacket key: ");
      data_entry->value = get_property("\tPacket value: ");
      sequence--;
    } else {
      data_entry->sequence = 0;
      data_entry->key = get_property("Packet key: ");
      data_entry->value = get_property("Packet value: ");
    }

    send_data->count = data_count + 1;
  }
}

void display_data(const struct Data* send_data) {
  const struct DataEntry* data_entry;
  printf("Displaying data. Expecting %u entries.\n", send_data->count);
  for(uint data_count = 0; data_count < MAX_DATA; data_count++) {
    data_entry = &(send_data->entry[data_count]);
    if(data_entry->target >= MAX_TARGETS) {
      break;
    }
    printf("  %u\n", data_count);
    printf("    target:   %u\n", data_entry->target);
    printf("    sequence: %u\n", data_entry->sequence);
    printf("    key:      %u\n", data_entry->key);
    printf("    value:    %u\n", data_entry->value);
  }
}


int main(int argc, char **argv) {
    int sockfd, portno, n;
    int serverlen;
    struct sockaddr_in serveraddr;
    struct hostent *server;
    char *hostname;
    char buf[BUFSIZE];
    struct Data send_data;

    /* check command line arguments */
    if (argc != 3) {
       fprintf(stderr,"usage: %s <hostname> <port>\n", argv[0]);
       exit(0);
    }
    hostname = argv[1];
    portno = atoi(argv[2]);

    /* socket: create the socket */
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) 
        error("ERROR opening socket");

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


    populate_data(&send_data);
    display_data(&send_data);


    /* send the message to the server */
    serverlen = sizeof(serveraddr);
    n = sendto(sockfd, (void*)&send_data, sizeof(send_data), 0, (struct sockaddr *)&serveraddr, serverlen);
    if (n < 0) 
      error("ERROR in sendto");
    
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
