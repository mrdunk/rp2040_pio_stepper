#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"

// w5x00 related.
#include "port_common.h"
#include "wizchip_conf.h"
#include "w5x00_spi.h"
#include "socket.h"

#include "sender.h"
#include "messages.h"
#include "config.h"


/* Network */
static wiz_NetInfo g_net_info =
    {
        .mac = {0x00, 0x08, 0xDC, 0x12, 0x34, 0x56}, // MAC address
        .ip = {192, 168, 11, 2},                     // IP address
        .sn = {255, 255, 255, 0},                    // Subnet Mask
        .gw = {192, 168, 11, 1},                     // Gateway
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

struct Message_uint_int* process_msg_uint_int(char** rx_buf) {
  struct Message_uint_int* message = (struct Message_uint_int*)(*rx_buf);

  // printf("NW UPD message on port %u:\n\ttype: %lu\n\tvalue0: %lu\n\tvalue1: %li\r\n",
  //    NW_PORT, message->type, message->value0, message->value1);

  *rx_buf += sizeof(struct Message_uint_int);

  return message;
}

size_t process_buffer_machine(uint8_t* rx_buf, uint8_t* tx_buf_machine) {
  char* rx_itterator = rx_buf;
  size_t tx_buf_machine_len = 0;
  uint32_t msg_type;
  size_t tx_buf_mach_len_max = DATA_BUF_SIZE - sizeof(uint32_t);
  struct Message* msg;
  struct Message_uint* msg_uint;
  struct Message_uint_uint* msg_uint_uint;
  struct Message_uint_int* msg_uint_int;

  uint32_t axis;
  uint32_t abs_pos;

  while(msg_type = *(uint32_t*)(rx_itterator)) {  // msg_type of 0 indicates end of data.
    switch(msg_type) {
      case MSG_SET_GLOAL_UPDATE_RATE:
        //msg_uint = process_msg_uint(&rx_itterator);
        //set_global_update_rate(msg_uint->value);
        //get_global_config(
        //    tx_buf_machine,
        //    &tx_buf_machine_len,
        //    tx_buf_mach_len_max
        //    );
        break;
      case MSG_SET_AXIS_ABS_POS:
        msg_uint_uint = process_msg_uint_uint(&rx_itterator);
        //set_absolute_position(msg_uint_uint->axis, msg_uint_uint->value);
        axis = msg_uint_uint->axis;
        abs_pos = msg_uint_uint->value;
        update_axis(axis, &abs_pos, NULL, NULL, NULL);
        break;
      case MSG_SET_AXIS_REL_POS:
        msg_uint_int = process_msg_uint_int(&rx_itterator);
        //set_relative_position(msg_uint_int->axis, msg_uint_int->value);
        break;
      case MSG_SET_AXIS_MAX_SPEED:
        // TODO.
      case MSG_SET_AXIS_MAX_ACCEL:
        // TODO.
      case MSG_SET_AXIS_ABS_POS_AT_TIME:
        // TODO.
        break;
      case MSG_GET_GLOBAL_CONFIG:
        msg = process_msg(&rx_itterator);
        //get_global_config(
        //    tx_buf_machine,
        //    &tx_buf_machine_len,
        //    tx_buf_mach_len_max
        //    );
        break;
      case MSG_GET_AXIS_CONFIG:
        msg_uint = process_msg_uint(&rx_itterator);
        //get_axis_config(
        //    msg_uint->value,
        //    tx_buf_machine,
        //    &tx_buf_machine_len,
        //    tx_buf_mach_len_max
        //    );
        break;
      case MSG_GET_AXIS_POS:
        msg_uint = process_msg_uint(&rx_itterator);
        //get_axis_pos(
        //    msg_uint->value,
        //    tx_buf_machine,
        //    &tx_buf_machine_len,
        //    tx_buf_mach_len_max
        //    );
        break;
      default:
        printf("Invalid message type: %lu\r\n", msg_type);
        memset(rx_buf, '\0', DATA_BUF_SIZE);
    }
  }
  memset(rx_buf, '\0', DATA_BUF_SIZE);

  return tx_buf_machine_len;
}


/* Get data over UDP.
 * $ nc -u <host> <port>
 */
int32_t get_UDP(
    uint8_t socket_num,
    uint16_t port,
    uint8_t* nw_rx_buf,
    uint8_t* tx_buf_machine,
    size_t* tx_buf_machine_len,
    size_t (*callback)(uint8_t*, uint8_t*),
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

         ret = recvfrom(socket_num, nw_rx_buf, size, destip, destport);
         //printf("RECEIVED: %u %u.%u.%u.%u : %u\r\n",
         //    socket_num, destip[0], destip[1], destip[2], destip[3], *destport);
         if(ret <= 0) {
           printf("%d: recvfrom error. %ld\r\n", socket_num,ret);
           return ret;
         }

         *tx_buf_machine_len = callback(nw_rx_buf, tx_buf_machine);
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

void core1_main() {
  uint32_t abs_pos;
  uint32_t min_step_len_ticks;
  uint32_t max_accel_ticks;
  uint32_t velocity;
  uint32_t updated;

  uint32_t count = 0;
  while (1) {
    //gpio_put(LED_PIN, (time_us_64() / 1000000) % 2);
    //gpio_put(LED_PIN, 1);
    //sleep_us(100000);
    //gpio_put(LED_PIN, 0);

    //sleep_us(100000);
    //printf(".");
    //if((count++ % 200) == 0) {
    //  printf("\n");
    //}


    //sleep_us(1000);
    for(uint8_t axis = 0; axis < MAX_AXIS; axis++) {
      updated = get_axis(axis, &abs_pos, &min_step_len_ticks, &max_accel_ticks, &velocity);
      if(updated > 0) {
        printf("%u \t %lu \t %lu \t %lu \t %lu \n",
            axis, abs_pos, min_step_len_ticks, max_accel_ticks, velocity);
      }
    }
  }
}

void init_core1_b() {
  printf("core0: Initializing.\n");

  // Launch core1.
  multicore_launch_core1(&core1_main);
}

int main() {
  int retval = 0;
  char nw_rx_buf[DATA_BUF_SIZE] = "";
  char tx_buf_machine[DATA_BUF_SIZE] = {0};
  size_t tx_buf_machine_len = 0;

  // Need these to store the IP and port.
  // We get the remote values when receiving data.
  // These store them for when we want to reply later.
  uint8_t  destip_machine[4] = {0, 0, 0, 0};
  uint16_t destport_machine = 0;
  memset(nw_rx_buf, '\0', DATA_BUF_SIZE);

  bi_decl(bi_program_description("Ethernet controlled stepper motor controller."));
  bi_decl(bi_1pin_with_name(LED_PIN, "On-board LED"));

  set_clock_khz();

  stdio_init_all();

  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);


  stdio_usb_init();
  setup_default_uart();
  sleep_ms(2000);
  init_core1_b();
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

  //init_core1();
  printf("--------------------------------\n");

  size_t time_last_tx = time_us_64();
  size_t time_now;
  while (1) {
    tx_buf_machine_len = 0;

    retval = get_UDP(
        SOCKET_NUMBER,
        NW_PORT,
        nw_rx_buf,
        tx_buf_machine,
        &tx_buf_machine_len,
        &process_buffer_machine,
        destip_machine,
        &destport_machine);

    for(size_t axis = 0; axis < MAX_AXIS; axis++) {
      // Get data from config and put in TX buffer.
      /*
      get_axis_config_if_updated(
          axis,
          tx_buf_machine,
          &tx_buf_machine_len,
          DATA_BUF_SIZE - sizeof(uint32_t)
          );
      */
    }

    retval = put_UDP(
        SOCKET_NUMBER,
        NW_PORT,
        tx_buf_machine,
        tx_buf_machine_len,
        destip_machine,
        &destport_machine);


    //time_now = time_us_64();
    //if(number_axis_updated() > 0) {
    //  time_last_tx = time_now;
      //core0_send_to_core1();
    //}

  }
  return 0;
}
