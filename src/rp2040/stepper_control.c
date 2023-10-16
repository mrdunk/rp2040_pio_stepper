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

  // printf("NW UPD message on port %u:\n\ttype: %u\n\tvalue0: %i\n\tvalue1: %li\r\n",
  //    NW_PORT, message->type, message->axis, message->value);

  *rx_buf += sizeof(struct Message_uint_int);

  return message;
}

struct Message_set_kp* process_msg_uint_float(char** rx_buf) {
  struct Message_set_kp* message = (struct Message_set_kp*)(*rx_buf);

  //printf("NW UPD message on port %u:\n\ttype: %u\n\taxis: %u\n\tvalue: %f\r\n",
  //    NW_PORT, message->type, message->axis, message->value);

  *rx_buf += sizeof(struct Message_set_kp);

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
  struct Message_set_kp* msg_set_kp;
  union MessageAny* message_any;

  uint32_t axis, update_id, tx_time;
  uint32_t abs_pos_requested = 0;
  double abs_pos_requested_float = 0.0;
  uint32_t abs_pos_acheived;
  int32_t velocity_requested = 0;
  int32_t enabled = 0;
  int8_t io_pos_value = -1;

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
      case MSG_SET_AXIS_ENABLED:
        msg_uint_uint = process_msg_uint_uint(&rx_itterator);
        axis = msg_uint_uint->axis;
        enabled = msg_uint_uint->value;
        update_axis_config(
            axis, CORE0,
            (uint8_t*)&enabled, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
        (*return_data)++;
        break;
      case MSG_SET_AXIS_ABS_POS:
        msg_uint_uint = process_msg_uint_uint(&rx_itterator);
        axis = msg_uint_uint->axis;
        abs_pos_requested = msg_uint_uint->value;
        update_axis_config(
            axis, CORE0,
            NULL, NULL, NULL, &abs_pos_requested, NULL, NULL, NULL, NULL, &velocity_requested, NULL, NULL);
        (*return_data)++;
        break;
      case MSG_SET_AXIS_ABS_POS_FLOAT:
        axis = ((struct Message_set_abs_pos*)(rx_itterator))->axis;
        abs_pos_requested_float = ((struct Message_set_abs_pos*)(rx_itterator))->value;
        
        rx_itterator += sizeof(struct Message_set_abs_pos);

        update_axis_config(
            axis, CORE0,
            NULL, NULL, NULL, NULL, &abs_pos_requested_float, NULL, NULL, NULL, NULL, NULL, NULL);
        (*return_data)++;
        break;
      case MSG_SET_AXIS_REL_POS:
        msg_uint_int = process_msg_uint_int(&rx_itterator);
        axis = msg_uint_int->axis;
        velocity_requested = msg_uint_int->value;
        update_axis_config(
            axis, CORE0,
            NULL, NULL, NULL, &abs_pos_requested, NULL, NULL, NULL, NULL, &velocity_requested, NULL, NULL);
        (*return_data)++;
        break;
      case MSG_SET_AXIS_MAX_SPEED:
        // TODO.
      case MSG_SET_AXIS_MAX_ACCEL:
        // TODO.
      case MSG_SET_AXIS_PID_KP:
        msg_set_kp = process_msg_uint_float(&rx_itterator);
        axis = msg_set_kp->axis;
        printf("Setting axis: %u\tkp:      %f\n", axis, msg_set_kp->value);
        update_axis_config(
            axis, CORE0,
            NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, &msg_set_kp->value);
        break;
      case MSG_SET_AXIS_IO_STEP:
        msg_uint_int = process_msg_uint_int(&rx_itterator);
        axis = msg_uint_int->axis;
        io_pos_value = msg_uint_int->value;
        printf("Setting axis: %u\tstep-io: %i\n", axis, io_pos_value);
        update_axis_config(
            axis, CORE0,
            NULL, &io_pos_value, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
        (*return_data)++;
        break;
      case MSG_SET_AXIS_IO_DIR:
        msg_uint_int = process_msg_uint_int(&rx_itterator);
        axis = msg_uint_int->axis;
        io_pos_value = msg_uint_int->value;
        printf("Setting axis: %u\tdir-io:  %i\n", axis, io_pos_value);
        update_axis_config(
            axis, CORE0,
            NULL, NULL, &io_pos_value, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
        (*return_data)++;
        break;
      case MSG_GET_GLOBAL_CONFIG:
        msg = process_msg(&rx_itterator);
        //get_global_config(
        //    tx_buf,
        //    &tx_buf_len,
        //    tx_buf_mach_len_max
        //    );
        break;
      default:
        printf("Invalid message type: %lu\r\n", msg_type);
        *(uint32_t*)(rx_itterator) = 0;  // 0 msg_type stops execution.
        tx_buf_len = 0;
        break;
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

/* Called after receiving network packet.
 * Takes the average time delay over the previous 1000 packet receive events and
 * blocks long enough to normalize any network time jitter. */
void recover_clock() {
  static struct Ring_buf_uint_ave period_average_data;
  static struct Ring_buf_uint_ave time_average_data_1;
  static struct Ring_buf_uint_ave time_average_data_2;
  static size_t time_last = 0;
  uint32_t restart_at = 0;
  uint32_t last_ave_period_us = 0;

  // Record the average length of time between packets.
  size_t time_now = time_us_64();
  uint32_t ave_period_us = ring_buf_uint_ave(&period_average_data, time_now - time_last);
  time_last = time_now;

  // Record the average arrival time.
  // Since we can't average modulo values (0us and 1000us would return 500 despite them
  // being adjacent) so we collect 2 averages, offset by 500us.
  uint32_t time_offset_1 = time_now % 1000;
  uint32_t ave_time_offset_us_1 = ring_buf_uint_ave(&time_average_data_1, time_offset_1);
  uint32_t time_offset_2 = (time_now + 500) % 1000;
  uint32_t ave_time_offset_us_2 = ring_buf_uint_ave(&time_average_data_2, time_offset_2);

  // Use whichever average arrival time is furthest from the modulo rollover event.
  uint32_t time_offset;
  uint32_t ave_time_offset_us;
  if(time_offset_1 > 250 && time_offset_1 < 750) {
    time_offset = time_offset_1;
    ave_time_offset_us = ave_time_offset_us_1;
  } else {
    time_offset = time_offset_2;
    ave_time_offset_us = ave_time_offset_us_2;
  }


  int32_t time_diff = (int32_t)time_offset - (int32_t)ave_time_offset_us;

  // Do the busy-wait to synchronise timing.
  restart_at = time_now + 100 - time_diff;
  while(restart_at > time_now) {
    time_now = time_us_64();
    tight_loop_contents();
  }

  // Semaphore to send core1 as the synchronization event.
  // Note that this happens soon after the busy-wait.
  tick++;

  if(last_ave_period_us != ave_period_us) {
    update_period(ave_period_us);
    last_ave_period_us = ave_period_us;
  }
}

int main() {
  int retval = 0;
  char rx_buf[DATA_BUF_SIZE] = "";
  char tx_buf[DATA_BUF_SIZE] = {0};
  size_t tx_buf_len = 0;
  uint8_t received_msg_count;
  uint8_t data_received = 0;
  size_t time_now;

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

    if(received_msg_count > 0) {
      recover_clock();

      time_now = time_us_64();
      gpio_put(LED_PIN, (time_now / 1000000) % 2);
      received_msg_count = 0;
    }

    size_t axis_count = 0;
    for(size_t axis = 0; axis < MAX_AXIS; axis++) {
      // Get data from config and put in TX buffer.
      axis_count += serialise_axis_config(axis, tx_buf, &tx_buf_len, true);
    }

    retval = put_UDP(
        SOCKET_NUMBER,
        NW_PORT,
        tx_buf,
        tx_buf_len,
        destip_machine,
        &destport_machine);

  }
  return 0;
}
