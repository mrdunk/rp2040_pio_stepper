#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include "pico/stdlib.h"

// w5x00 related.
#include "port_common.h"
#include "wizchip_conf.h"
#include "w5x00_spi.h"
#include "socket.h"

#include "stepper_control.h"
#include "messages.h"
#include "config.h"
#include "core1.h"


/* Network */
static wiz_NetInfo g_net_info =
    {
        .mac = {0x00, 0x08, 0xDC, 0x12, 0x34, 0x56}, // MAC address
        .ip = {192, 168, 12, 2},                     // IP address
        .sn = {255, 255, 255, 0},                    // Subnet Mask
        .gw = {192, 168, 12, 1},                     // Gateway
        .dns = {8, 8, 8, 8},                         // DNS server
        .dhcp = NETINFO_STATIC                       // DHCP enable/disable
};

static void set_clock_khz(void)
{
    // set a system clock frequency in khz
    set_sys_clock_khz(PLL_SYS_KHZ, true);

    // configure the specified clock
    clock_configure(
        clk_peri,
        0,                                                // No glitchless mux
        CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, // System PLL on AUX mux
        PLL_SYS_KHZ * 1000,                               // Input frequency
        PLL_SYS_KHZ * 1000                                // Output (must be same as no divider)
    );
}

struct Message* process_msg(char** rx_buf) {
  struct Message* message = (struct Message*)(*rx_buf);

  // printf("NW UPD message on port %u:\n\ttype: %lu\r\n", NW_PORT, message->type);

  *rx_buf += sizeof(struct Message);

  return message;
}

struct Message_uint* process_msg_uint(char** rx_buf) {
  struct Message_uint* message = (struct Message_uint*)(*rx_buf);

  // printf("NW UPD message on port %u:\n\ttype: %lu\n\tvalue0: %lu\r\n",
  //    NW_PORT, message->type, message->value0);

  *rx_buf += sizeof(struct Message_uint);

  return message;
}

struct Message_uint_uint* process_msg_uint_uint(char** rx_buf) {
  struct Message_uint_uint* message = (struct Message_uint_uint*)(*rx_buf);

  // printf("NW UPD message on port %u:\n\ttype: %lu\n\tvalue0: %lu\n\tvalue1: %lu\r\n",
  //    NW_PORT, message->type, message->value0, message->value1);

  *rx_buf += sizeof(struct Message_uint_uint);

  return message;
}

struct Message_timing* process_msg_timing(char** rx_buf) {
  struct Message_timing* message = (struct Message_timing*)(*rx_buf);

  // printf("NW UPD message on port %u:\n\ttype: %lu\n\tvalue0: %lu\n\tvalue1: %lu\r\n",
  //    NW_PORT, message->type, message->value0, message->value1);

  *rx_buf += sizeof(struct Message_timing);

  return message;
}

struct Message_uint_int* process_msg_uint_int(char** rx_buf) {
  struct Message_uint_int* message = (struct Message_uint_int*)(*rx_buf);

  // printf("NW UPD message on port %u:\n\ttype: %lu\n\tvalue0: %lu\n\tvalue1: %li\r\n",
  //    NW_PORT, message->type, message->value0, message->value1);

  *rx_buf += sizeof(struct Message_uint_int);

  return message;
}

struct Message_uint_float* process_msg_uint_float(char** rx_buf) {
  struct Message_uint_float* message = (struct Message_uint_float*)(*rx_buf);

  //printf("NW UPD message on port %u:\n\ttype: %u\n\taxis: %u\n\tvalue: %f\r\n",
  //    NW_PORT, message->type, message->axis, message->value);

  *rx_buf += sizeof(struct Message_uint_float);

  return message;
}

size_t process_received_buffer(uint8_t* rx_buf, uint8_t* tx_buf, uint8_t* return_data) {
  char* rx_itterator = rx_buf;
  size_t tx_buf_len = 0;
  uint32_t msg_type;
  size_t tx_buf_mach_len_max = DATA_BUF_SIZE - sizeof(uint32_t);
  struct Message* msg;
  struct Message_timing* msg_timing;
  struct Message_uint* msg_uint;
  struct Message_uint_uint* msg_uint_uint;
  struct Message_uint_int* msg_uint_int;
  struct Message_uint_float* msg_uint_float;

  uint32_t axis, update_id, tx_time;
  uint32_t abs_pos_requested;

  while(msg_type = *(uint32_t*)(rx_itterator)) {  // msg_type of 0 indicates end of data.
    switch(msg_type) {
      case MSG_TIMING:
        msg_timing = process_msg_timing(&rx_itterator);
        update_id = msg_timing->update_id;
        tx_time = msg_timing->time;
        //printf("%u\t%u\n", count, tx_time);
        int32_t id_diff;
        int32_t time_diff;
        update_packet_metrics(update_id, tx_time, &id_diff, &time_diff);
        serialise_metrics(tx_buf, &tx_buf_len, update_id, time_diff);
      case MSG_SET_GLOAL_UPDATE_RATE:
        //msg_uint = process_msg_uint(&rx_itterator);
        //set_global_update_rate(msg_uint->value);
        //get_global_config(
        //    tx_buf,
        //    &tx_buf_len,
        //    tx_buf_mach_len_max
        //    );
        break;
      case MSG_SET_AXIS_ABS_POS:
        msg_uint_uint = process_msg_uint_uint(&rx_itterator);
        axis = msg_uint_uint->axis;
        abs_pos_requested = msg_uint_uint->value;
        update_axis_config(
            axis, CORE0, &abs_pos_requested, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
        (*return_data)++;
        break;
      case MSG_SET_AXIS_REL_POS:
        msg_uint_int = process_msg_uint_int(&rx_itterator);
        //set_relative_position(msg_uint_int->axis, msg_uint_int->value);
        //(*return_data)++;
        break;
      case MSG_SET_AXIS_MAX_SPEED:
        // TODO.
      case MSG_SET_AXIS_MAX_ACCEL:
        // TODO.
      case MSG_SET_AXIS_ABS_POS_AT_TIME:
        // TODO.
        msg_uint_uint = process_msg_uint_uint(&rx_itterator);
        break;
      case MSG_SET_PID_KP:
        msg_uint_float = process_msg_uint_float(&rx_itterator);
        axis = msg_uint_float->axis;
        update_axis_config(
            axis, CORE0, NULL, NULL, NULL, NULL, NULL, &msg_uint_float->value, NULL, NULL);
        break;
      case MSG_SET_PID_KI:
        msg_uint_float = process_msg_uint_float(&rx_itterator);
        axis = msg_uint_float->axis;
        update_axis_config(
            axis, CORE0, NULL, NULL, NULL, NULL, NULL, NULL, &msg_uint_float->value, NULL);
        break;
      case MSG_SET_PID_KD:
        msg_uint_float = process_msg_uint_float(&rx_itterator);
        axis = msg_uint_float->axis;
        update_axis_config(
            axis, CORE0, NULL, NULL, NULL, NULL, NULL, NULL, NULL, &msg_uint_float->value);
        break;
      case MSG_GET_GLOBAL_CONFIG:
        msg = process_msg(&rx_itterator);
        //get_global_config(
        //    tx_buf,
        //    &tx_buf_len,
        //    tx_buf_mach_len_max
        //    );
        break;
      case MSG_GET_AXIS_CONFIG:
        msg_uint = process_msg_uint(&rx_itterator);
        axis = msg_uint->value;
        // TODO: Test this works.
        serialise_axis_config(axis, tx_buf, &tx_buf_len, true);
        //get_axis_config(
        //    msg_uint->value,
        //    tx_buf,
        //    &tx_buf_len,
        //    tx_buf_mach_len_max
        //    );
        break;
      case MSG_GET_AXIS_POS:
        msg_uint = process_msg_uint(&rx_itterator);
        //get_axis_pos(
        //    msg_uint->value,
        //    tx_buf,
        //    &tx_buf_len,
        //    tx_buf_mach_len_max
        //    );
        break;
      default:
        printf("Invalid message type: %lu\r\n", msg_type);
    }
  }
  memset(rx_buf, '\0', DATA_BUF_SIZE);

  return tx_buf_len;
}


/* Get data over UDP.
 * $ nc -u <host> <port>
 */
int32_t get_UDP(
    uint8_t socket_num,
    uint16_t port,
    uint8_t* rx_buf,
    uint8_t* data_received,
    uint8_t* destip,
    uint16_t* destport)
{
   int32_t  ret;
   uint16_t size;
   uint16_t val1;

   //printf("NW: %u %u.%u.%u.%u : %u\r\n",
   //    socket_num, destip[0], destip[1], destip[2], destip[3], *destport);

   switch(getSn_SR(socket_num)) {
     case SOCK_UDP :
       if((size = getSn_RX_RSR(socket_num)) > 0) {
         // Receiving data.
         if(size > DATA_BUF_SIZE) {
           size = DATA_BUF_SIZE;
         }

         ret = recvfrom(socket_num, rx_buf, size, destip, destport);
         //printf("RECEIVED: %u %u.%u.%u.%u : %u\r\n",
         //    socket_num, destip[0], destip[1], destip[2], destip[3], *destport);
         if(ret <= 0) {
           printf("%d: recvfrom error. %ld\r\n", socket_num,ret);
           return ret;
         }
         (*data_received)++;
       }
       break;
     case SOCK_CLOSED:
       if((ret = socket(socket_num, Sn_MR_UDP, port, 0x00)) != socket_num) {
         return ret;
       }
       printf("%d:Closed, UDP connection, port [%d]\r\n", socket_num, port);
       break;
     default :
       break;
   }

   return 1;
}

/* Send data over UDP. */
int32_t put_UDP(
    uint8_t socket_num,
    uint16_t port,
    uint8_t* tx_buf,
    size_t tx_buf_len,
    uint8_t* destip,
    uint16_t* destport
    ) {
  int32_t  ret;
  uint16_t size;
  uint16_t sentsize;

  switch(getSn_SR(socket_num)) {
    case SOCK_UDP :
      size = tx_buf_len;
      if(size > 0) {
        // Sending data.
        // printf("SENDING: %u %u.%u.%u.%u : %u\r\n",
        //    socket_num, destip[0], destip[1], destip[2], destip[3], *destport);
        sentsize = 0;
        while(sentsize < size) {
          ret = sendto(
              socket_num, tx_buf + sentsize, size - sentsize, destip, *destport);
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

int main() {
  static struct Ring_buf_ave period_average_data;
  uint32_t ave_period_us = 0;
  uint32_t last_ave_period_us = 0;

  int retval = 0;
  char rx_buf[DATA_BUF_SIZE] = "";
  char tx_buf[DATA_BUF_SIZE] = {0};
  size_t tx_buf_len = 0;
  uint8_t received_msg_count;
  uint8_t data_received = 0;

  // Need these to store the IP and port.
  // We get the remote values when receiving data.
  // These store them for when we want to reply later.
  uint8_t  destip_machine[4] = {0, 0, 0, 0};
  uint16_t destport_machine = 0;
  memset(rx_buf, '\0', DATA_BUF_SIZE);

  bi_decl(bi_program_description("Ethernet controlled stepper motor controller."));
  bi_decl(bi_1pin_with_name(LED_PIN, "On-board LED"));

  set_clock_khz();

  stdio_init_all();

  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);


  stdio_usb_init();
  setup_default_uart();
  sleep_ms(2000);
  init_config();
  printf("--------------------------------\n");
  printf("UART up.\n");

  wizchip_spi_initialize();
  spi_init(SPI_PORT, 48000 * 1000);
  wizchip_cris_initialize();
  wizchip_reset();
  wizchip_initialize();
  wizchip_check();
  network_initialize(g_net_info);
  /* Get network information */
  print_network_information(g_net_info);

  init_core1();
  printf("--------------------------------\n");

  size_t time_last = time_us_64();
  size_t time_now;
  while (1) {
    tx_buf_len = 0;
    data_received = 0;
		memset(tx_buf, '\0', DATA_BUF_SIZE);

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

    size_t axis_count = 0;
    for(size_t axis = 0; axis < MAX_AXIS; axis++) {
      // Get data from config and put in TX buffer.
      axis_count += serialise_axis_config(axis, tx_buf, &tx_buf_len, false);
    }
#if DEBUG_OUTPUT
    printf("Sending: %lu\n", axis_count);
#endif

    retval = put_UDP(
        SOCKET_NUMBER,
        NW_PORT,
        tx_buf,
        tx_buf_len,
        destip_machine,
        &destport_machine);


    if(received_msg_count > 0) {
      // Have received axis updates.
      // Save the update period to config if appropriate.
      time_now = time_us_64();
      ave_period_us = ring_buf_ave(&period_average_data, time_now - time_last);
      time_last = time_now;
      if(last_ave_period_us != ave_period_us) {
        update_period(ave_period_us);
        last_ave_period_us = ave_period_us;
      }
#if DEBUG_OUTPUT
      printf("Received: %u \t%lu\n", received_msg_count, ave_period_us);
#endif
      received_msg_count = 0;

      gpio_put(LED_PIN, (time_now / 1000000) % 2);
    }
  }
  return 0;
}
