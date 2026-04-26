#include <stdint.h>
#include <stdio.h>
#include <stddef.h>

#ifdef BUILD_TESTS
#include "../test/mocks/socket_mocks.h"
#else
// w5x00 related.
#include "socket.h"
#endif

#include "config.h"
#include "buffer.h"

/* Get data over UDP.
 * $ nc -u <host> <port>
 */
int32_t get_UDP(
    uint8_t socket_num,
    uint16_t port,
    struct NWBuffer* rx_buf,
    size_t* data_received,
    uint8_t* destip,
    uint16_t* destport)
{
   int32_t ret = 0;
   size_t size;

   //printf("NW: %u %u.%u.%u.%u : %u\r\n",
   //    socket_num, destip[0], destip[1], destip[2], destip[3], *destport);

   switch(getSn_SR(socket_num)) {
     case SOCK_UDP :
       if((size = getSn_RX_RSR(socket_num)) > 0) {
         // Receiving data.
         if(size > sizeof(struct NWBuffer)) {
           size = sizeof(struct NWBuffer);
         }

         ret = recvfrom(socket_num, (void*)rx_buf, (uint16_t)size, destip, destport);
         //printf("RECEIVED: %u %u.%u.%u.%u : %u\r\n",
         //    socket_num, destip[0], destip[1], destip[2], destip[3], *destport);
         if(ret <= 0) {
           printf("%d: recvfrom error. %ld\r\n", socket_num, ret);
           return ret;
         }
         size = ret;
         (*data_received) += size;
       }
       break;
     case SOCK_CLOSED:
       if((ret = socket(socket_num, Sn_MR_UDP, port, 0x00)) != socket_num) {
         return ret;
       }
       break;
     default :
       break;
   }

   return ret;
}

/* Send data over UDP. */
int32_t put_UDP(
    uint8_t socket_num,
    uint16_t port,
    void* tx_buf,
    size_t tx_buf_len,
    uint8_t* destip,
    uint16_t* destport
    ) {
  int32_t ret;
  size_t sentsize;

  switch(getSn_SR(socket_num)) {
    case SOCK_UDP :
      if(tx_buf_len > 0) {
        // Sending data.
        // printf("SENDING: %u %u.%u.%u.%u : %u\r\n",
        //    socket_num, destip[0], destip[1], destip[2], destip[3], *destport);
        sentsize = 0;
        while(sentsize < tx_buf_len) {
          ret = sendto(
              socket_num, tx_buf + sentsize, tx_buf_len - sentsize, destip, *destport);
          if(ret < 0) {
            // printf("%d: sendto error. %ld\r\n", socket_num, ret);
            return ret;
          }
          sentsize += ret; // Don't care SOCKERR_BUSY, because it is zero.
        }
      }
      break;
    case SOCK_CLOSED:
      if((ret = socket(socket_num, Sn_MR_UDP, port, 0x00)) != socket_num) {
        return ret;
      }
      printf("%d:Opened, UDP connection, port [%d]\r\n", socket_num, port);
      break;
    default :
      break;
  }
  return 1;
}
