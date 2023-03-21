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

#include "pico_stepper.h"
#include "sender.h"
#include "messages.h"


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

void help(char* tx_buf) {
    snprintf(
        tx_buf,
        DATA_BUF_SIZE,
        "\r\nUsage:\r\n"
        " > COMMAND[:VALUE[:VALUE[:VALUE]]]\r\n"
        "eg:\r\n"
        " > 3:0:-15\r\n\r\n"
        "List of available commands:\r\n"
        "%s\r\n",
        human_help
        );
}

struct Message* process_msg(char** rx_buf) {
  struct Message* message = (struct Message*)(*rx_buf);

  printf("NW UPD message on port %u:\n\ttype: %lu\r\n", NW_PORT_MACHINE, message->type);

  *rx_buf += sizeof(struct Message);

  return message;
}

struct Message_uint* process_msg_uint(char** rx_buf) {
  struct Message_uint* message = (struct Message_uint*)(*rx_buf);

  printf("NW UPD message on port %u:\n\ttype: %lu\n\tvalue0: %lu\r\n",
      NW_PORT_MACHINE, message->type, message->value0);

  *rx_buf += sizeof(struct Message_uint);

  return message;
}

struct Message_uint_uint* process_msg_uint_uint(char** rx_buf) {
  struct Message_uint_uint* message = (struct Message_uint_uint*)(*rx_buf);

  printf("NW UPD message on port %u:\n\ttype: %lu\n\tvalue0: %lu\n\tvalue1: %lu\r\n",
      NW_PORT_MACHINE, message->type, message->value0, message->value1);

  *rx_buf += sizeof(struct Message_uint_uint);

  return message;
}

struct Message_uint_int* process_msg_uint_int(char** rx_buf) {
  struct Message_uint_int* message = (struct Message_uint_int*)(*rx_buf);

  printf("NW UPD message on port %u:\n\ttype: %lu\n\tvalue0: %lu\n\tvalue1: %li\r\n",
      NW_PORT_MACHINE, message->type, message->value0, message->value1);

  *rx_buf += sizeof(struct Message_uint_uint);

  return message;
}

size_t process_buffer_machine(uint8_t* rx_buf, uint8_t* tx_buf, uint8_t* tx_buf_machine) {
  char* rx_itterator = rx_buf;
  size_t tx_buf_machine_len = 0;
  uint32_t msg_type;
  size_t tx_buf_mach_len_max = DATA_BUF_SIZE - sizeof(uint32_t);
  struct Message* msg;
  struct Message_uint* msg_uint;
  struct Message_uint_uint* msg_uint_uint;
  struct Message_uint_int* msg_uint_int;

  while(msg_type = *(uint32_t*)(rx_itterator)) {  // msg_type of 0 indicates end of data.
    switch(msg_type) {
      case MSG_SET_GLOAL_UPDATE_RATE:
        msg_uint = process_msg_uint(&rx_itterator);
        set_global_update_rate(msg_uint->value0);
        get_global_config(
            tx_buf, DATA_BUF_SIZE, tx_buf_machine, &tx_buf_machine_len, tx_buf_mach_len_max);
        break;
      case MSG_SET_AXIS_ABS_POS:
        msg_uint_uint = process_msg_uint_uint(&rx_itterator);
        set_absolute_position(msg_uint_uint->value0, msg_uint_uint->value1);
        get_axis_pos(
            msg_uint_uint->value0,
            tx_buf,
            DATA_BUF_SIZE,
            tx_buf_machine,
            &tx_buf_machine_len,
            tx_buf_mach_len_max);
        break;
      case MSG_SET_AXIS_REL_POS:
        msg_uint_int = process_msg_uint_int(&rx_itterator);
        set_relative_position(msg_uint_int->value0, msg_uint_int->value1);
        get_axis_pos(
            msg_uint_int->value0,
            tx_buf,
            DATA_BUF_SIZE,
            tx_buf_machine,
            &tx_buf_machine_len,
            tx_buf_mach_len_max);
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
        get_global_config(
            tx_buf, DATA_BUF_SIZE, tx_buf_machine, &tx_buf_machine_len, tx_buf_mach_len_max);
        break;
      case MSG_GET_AXIS_CONFIG:
        msg_uint = process_msg_uint(&rx_itterator);
        get_axis_config(
            msg_uint->value0,
            tx_buf,
            DATA_BUF_SIZE,
            tx_buf_machine,
            &tx_buf_machine_len,
            tx_buf_mach_len_max);
        break;
      case MSG_GET_AXIS_POS:
        msg_uint = process_msg_uint(&rx_itterator);
        get_axis_pos(
            msg_uint->value0,
            tx_buf,
            DATA_BUF_SIZE,
            tx_buf_machine,
            &tx_buf_machine_len,
            tx_buf_mach_len_max);
        break;
      default:
        printf("Invalid message type: %lu\r\n", msg_type);
    }
  }
  memset(rx_buf, '\0', DATA_BUF_SIZE);

  //printf("tx_buf_machine_len: %lu\n", tx_buf_machine_len);
  //for(size_t i = 0; i < tx_buf_machine_len; i += 4) {
  //  printf("**  %lu\t%lu\n", i, ((uint32_t*)tx_buf_machine)[i/4]);
  //}

  return tx_buf_machine_len;
}

size_t process_buffer_human(uint8_t* rx_buf, uint8_t* tx_buf, uint8_t* tx_buf_machine) {
  printf("\nReceived: %s\n", rx_buf);
  int64_t values[4] = {0,0,0,0};
  size_t value_num = 0;
  char* itterate = rx_buf;
  uint32_t val = 0;
  size_t tx_buf_machine_len = 0;
  size_t tx_buf_mach_len_max = DATA_BUF_SIZE - sizeof(uint32_t);

  while(*itterate) {
    if (*itterate == ':') {
      // Separator.
      itterate++;
    } else if (isspace(*itterate)) {
      // Whitespace. Ignore and continue.
      itterate++;
    } else if (isdigit(*itterate) || *itterate == '-') {
      val = strtol(itterate, &itterate, 10);
      if (value_num < 4) {
        values[value_num] = val;
      } else {
        printf("Too many values. %ld\n", val);
        memset(rx_buf, '\0', DATA_BUF_SIZE);
        return 0;
      }
      value_num++;
    } else if (*itterate == '?' && value_num == 0) {
      help(tx_buf);
      memset(rx_buf, '\0', DATA_BUF_SIZE);
      return 0;
    } else {
      printf("Unexpected character: %u : %c\n", *itterate, *itterate);
      memset(rx_buf, '\0', DATA_BUF_SIZE);
      return 0;
    }
  }

  switch(values[0]) {
    case MSG_SET_GLOAL_UPDATE_RATE:
      if(value_num != 2) {
        printf("Wrong number of values for type %lu. Got: %u. Expected: 2\n",
            (uint32_t)values[0], value_num);
        break;
      }
      set_global_update_rate(values[1]);
      get_global_config(
          tx_buf, DATA_BUF_SIZE, tx_buf_machine, &tx_buf_machine_len, tx_buf_mach_len_max);
      break;
    case MSG_SET_AXIS_ABS_POS:
      if(value_num != 3) {
        printf("Wrong number of values for type %lu. Got: %u. Expected: 3\n",
            (uint32_t)values[0], value_num);
        break;
      }
      set_absolute_position(values[1], values[2]);
      get_axis_pos(
          values[1],
          tx_buf,
          DATA_BUF_SIZE,
          tx_buf_machine,
          &tx_buf_machine_len,
          tx_buf_mach_len_max);
      break;
    case MSG_SET_AXIS_REL_POS:
      if(value_num != 3) {
        printf("Wrong number of values for type %lu. Got: %u. Expected: 3\n",
            (uint32_t)values[0], value_num);
        break;
      }
      set_relative_position(values[1], values[2]);
      get_axis_pos(
          values[1],
          tx_buf,
          DATA_BUF_SIZE,
          tx_buf_machine,
          &tx_buf_machine_len,
          tx_buf_mach_len_max);
      break;
    case MSG_SET_AXIS_MAX_SPEED:
      // TODO.
    case MSG_SET_AXIS_MAX_ACCEL:
      // TODO.
    case MSG_SET_AXIS_ABS_POS_AT_TIME:
      // TODO.
      break;
    case MSG_GET_GLOBAL_CONFIG:
      if(value_num != 1) {
        printf("Wrong number of values for type %lu. Got: %u. Expected: 1\n",
            (uint32_t)values[0], value_num);
        break;
      }
      get_global_config(
          tx_buf, DATA_BUF_SIZE, tx_buf_machine, &tx_buf_machine_len, tx_buf_mach_len_max);
      break;
    case MSG_GET_AXIS_CONFIG:
      if(value_num != 2) {
        printf("Wrong number of values for type %lu. Got: %u. Expected: 2\n",
            (uint32_t)values[0], value_num);
        break;
      }
      get_axis_config(
          values[1],
          tx_buf,
          DATA_BUF_SIZE,
          tx_buf_machine,
          &tx_buf_machine_len,
          tx_buf_mach_len_max);
      break;
    case MSG_GET_AXIS_POS:
      if(value_num != 2) {
        printf("Wrong number of values for type %lu. Got: %u. Expected: 2\n",
            (uint32_t)values[0], value_num);
        break;
      }
      get_axis_pos(
          values[1],
          tx_buf,
          DATA_BUF_SIZE,
          tx_buf_machine,
          &tx_buf_machine_len,
          tx_buf_mach_len_max);
      break;
    default:
      printf("Invalid message type: %lu\r\n", values[0]);
      help(tx_buf);
      memset(rx_buf, '\0', DATA_BUF_SIZE);
      return 0;
  }
  memset(rx_buf, '\0', DATA_BUF_SIZE);

  return 0;
}

uint8_t get_uart(uint8_t* uart_rx_buf, uint8_t* tx_buf_human, uint8_t* tx_buf_machine) {
  char* start_update = &uart_rx_buf[strlen(uart_rx_buf)];

  int ch = getchar_timeout_us(0);
  while (ch > 0) {
    size_t pos = strlen(uart_rx_buf);
    if (ch == 13) {
      // Return pressed.
      process_buffer_human(uart_rx_buf, tx_buf_human, tx_buf_machine);
    } else if (pos < DATA_BUF_SIZE - 1 && isprint(ch)) {
      uart_rx_buf[pos] = ch;
      printf("%c", ch);
    }
    ch = getchar_timeout_us(0);
  }

  return 0;
}

void put_uart(char* tx_buf, size_t buffer_len) {
  if(tx_buf[0] == '\0') {
    return;
  }
  puts_raw(tx_buf);
}

/* Get data over UDP.
 * $ nc -u <host> <port>
 */
int32_t get_UDP(
    uint8_t socket_num,
    uint16_t port,
    uint8_t* nw_rx_buf,
    uint8_t* tx_buf_human,
    uint8_t* tx_buf_machine,
    size_t* tx_buf_machine_len,
    size_t (*callback)(uint8_t*, uint8_t*, uint8_t*),
    uint8_t* destip,
    uint16_t* destport
    ) {
   int32_t  ret;
   uint16_t size;

   switch(getSn_SR(socket_num)) {
     case SOCK_UDP :
       if((size = getSn_RX_RSR(socket_num)) > 0) {
         // Receiving data.
         if(size > DATA_BUF_SIZE) {
           size = DATA_BUF_SIZE;
         }
         ret = recvfrom(socket_num, nw_rx_buf, size, destip, destport);
         // printf("RECEIVED: %u %u.%u.%u.%u : %u\r\n",
         //    socket_num, destip[0], destip[1], destip[2], destip[3], *destport);
         if(ret <= 0) {
           printf("%d: recvfrom error. %ld\r\n", socket_num,ret);
           return ret;
         }

         *tx_buf_machine_len = callback(nw_rx_buf, tx_buf_human, tx_buf_machine);
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

/* Send data over UDP. */
int32_t put_UDP(
    uint8_t socket_num,
    uint16_t port,
    uint8_t* tx_buf,
    size_t tx_buf_len,
    uint8_t* destip,
    uint16_t* destport
    ) {
  if(*destport == 0) {
    return 0;
  }
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
            printf("%d: sendto error. %ld\r\n", socket_num, ret);
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

  int retval = 0;
  char uart_rx_buf[DATA_BUF_SIZE] = "";
  char nw_rx_buf[DATA_BUF_SIZE] = "";
  char tx_buf_human[DATA_BUF_SIZE] = "";
  char tx_buf_machine[DATA_BUF_SIZE] = {0};
  size_t tx_buf_machine_len = 0;

  // Need these to store the IP and port.
  // We get the remote values when receiving data.
  // These store them for when we want to reply later.
  uint8_t  destip_human[4] = {0, 0, 0, 0};
  uint8_t  destip_machine[4] = {0, 0, 0, 0};
  uint16_t destport_human = 0;
  uint16_t destport_machine = 0;
  memset(nw_rx_buf, '\0', DATA_BUF_SIZE);

  bi_decl(bi_program_description("Ethernet controlled stepper motor controller."));
  bi_decl(bi_1pin_with_name(LED_PIN, "On-board LED"));

  set_clock_khz();

  stdio_init_all();

  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);

  if(NET_ENABLE > 0) {
    wizchip_spi_initialize();
    wizchip_cris_initialize();
    wizchip_reset();
    wizchip_initialize();
    wizchip_check();
    network_initialize(g_net_info);
    /* Get network information */
    print_network_information(g_net_info);
  }

  stdio_usb_init();
  setup_default_uart();

  sleep_ms(1000);

  // Initialise PIOs.
  // TODO: Set up all pins.
  uint32_t stepper_count = 2; // MAX_AXIS;  // TODO: populate other pins.
  uint32_t pins_step[2] = {0, 2};
  uint32_t pins_direction[2] = {1, 3};
  for (uint32_t stepper = 0; stepper < stepper_count; stepper++) {
    init_pio(stepper, pins_step[stepper], pins_direction[stepper]);
    //while(1);
  }


  //send_pios_steps(0, 10, 500000, 1);
  //send_pios_steps(0, 20, 100000, 0);
  //send_pios_steps(1, 4, 2000000, 0);
  //send_pios_steps(1, 2, 5000000, 1);


  while (1) {
    memset(tx_buf_machine, '\0', DATA_BUF_SIZE);
    memset(tx_buf_human, '\0', DATA_BUF_SIZE);
    tx_buf_machine_len = 0;

    get_uart(uart_rx_buf, tx_buf_human, tx_buf_machine);

    if (NET_ENABLE > 0) {
        retval = get_UDP(
            SOCKET_NUMBER_HUMAN,
            NW_PORT_HUMAN,
            nw_rx_buf,
            tx_buf_human,
            tx_buf_machine,
            &tx_buf_machine_len,
            &process_buffer_human,
            destip_human,
            &destport_human);
        if (retval < 0) {
          printf(" Network error : %d\n", retval);
          while (1);
        }
        retval = get_UDP(
            SOCKET_NUMBER_MACHINE,
            NW_PORT_MACHINE,
            nw_rx_buf,
            tx_buf_human,
            tx_buf_machine,
            &tx_buf_machine_len,
            &process_buffer_machine,
            destip_machine,
            &destport_machine);
        if (retval < 0) {
          printf(" Network error : %d\n", retval);
          while (1);
        }
        retval = put_UDP(
            SOCKET_NUMBER_HUMAN,
            NW_PORT_HUMAN,
            tx_buf_human,
            strlen(tx_buf_human),
            destip_human,
            &destport_human);
        if (retval < 0) {
          printf(" Network error : %d\n", retval);
          while (1);
        }
        retval = put_UDP(
            SOCKET_NUMBER_MACHINE,
            NW_PORT_MACHINE,
            tx_buf_machine,
            tx_buf_machine_len,
            destip_machine,
            &destport_machine);
        if (retval < 0) {
          printf(" Network error : %d\n", retval);
          while (1);
        }

        //for(size_t i = 0; i < tx_buf_machine_len; i += 4) {
        //  printf("*   %lu\t%lu\n", i, ((uint32_t*)tx_buf_machine)[i/4]);
        //}
    }

    put_uart(tx_buf_human, DATA_BUF_SIZE);
  }
}
