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
        "Usage:\r\n"
        " > [stepper_id]                                           |"
        "   Display the target_position of a stepper_id.\r\n"
        " > [stepper_id]:[new_position]:[time_to_arrive_us]        |"
        "   Set absolute position of a stepper_id.\r\n"
        " > [stepper_id]:[step_count]:[step_len_us]:[direction]    |"
        "   Perform some steps on a stepper_id.\r\n"
        );
}

void display_data(const struct Data* received_data) {
  const struct DataEntry* data_entry;
  printf("Received binary data. Expecting %u entries.\n", received_data->count);
  for(uint data_count = 0; data_count < MAX_MACHINE_DATA; data_count++) {
    data_entry = &(received_data->entry[data_count]);
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

void delegete_set_absolute_position(
    char* tx_buf,
    const uint stepper,
    const uint new_position,
    const uint time_slice_us) {
  uint position = set_absolute_position(stepper, new_position, time_slice_us);
  snprintf(
      tx_buf,
      DATA_BUF_SIZE,
      "Parsed:\r\n"
      " stepper: %lu\r\n"
      " new_position: %lu\r\n"
      " time_slice_us: %lu\r\n"
      "Set:\r\n"
      " stepper: %lu\r\n"
      " target_position: %lu\r\n",
      stepper, new_position, time_slice_us, stepper, position);
}

struct Message* process_msg(char** rx_buf) {
  struct Message* message = (struct Message*)(*rx_buf);

  printf("message:\n\ttype: %lu\r\n", message->type);

  *rx_buf += sizeof(struct Message);

  return message;
}

struct Message_uint* process_msg_uint(char** rx_buf) {
  struct Message_uint* message = (struct Message_uint*)(*rx_buf);

  printf("message:\n\ttype: %lu\n\tvalue0: %lu\r\n", message->type, message->value0);

  *rx_buf += sizeof(struct Message_uint);

  return message;
}

struct Message_uint_uint* process_msg_uint_uint(char** rx_buf) {
  struct Message_uint_uint* message = (struct Message_uint_uint*)(*rx_buf);

  printf("message:\n\ttype: %lu\n\tvalue0: %lu\n\tvalue1: %lu\r\n",
      message->type, message->value0, message->value1);

  *rx_buf += sizeof(struct Message_uint_uint);

  return message;
}

void process_buffer_machine_v02(char* rx_buf, char* tx_buf) {
  uint msg_type;
  struct Message* msg;
  struct Message_uint* msg_uint;
  struct Message_uint_uint* msg_uint_uint;


  while(msg_type = *(uint*)(rx_buf)) {
    switch(msg_type) {
      case MSG_SET_GLOAL_UPDATE_RATE:
        msg_uint = process_msg_uint(&rx_buf);
        break;
      case MSG_SET_AXIS_ABS_POS:
        msg_uint_uint = process_msg_uint_uint(&rx_buf);
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
        msg = process_msg(&rx_buf);
        break;
      case MSG_GET_AXIS_CONFIG:
        msg_uint = process_msg_uint(&rx_buf);
        break;
      case MSG_GET_AXIS_POS:
        msg_uint = process_msg_uint(&rx_buf);
        break;
      default:
        printf("Invalid message type: %lu\r\n", msg_type);
        exit(0);

    }

  }
}

void process_buffer_machine(char* rx_buf, char* tx_buf) {
  struct Data received_data;
  struct DataEntry* data_entry;
  memcpy(&received_data, rx_buf, sizeof(struct Data));

  display_data(&received_data);

  struct CollectedValues collected_values;
  struct CollectedValues collected_is_set;

  uint last_sequence = 0;

  for(uint data_count = 0; data_count < received_data.count; data_count++) {
    data_entry = &((received_data.entry)[data_count]);

    if(last_sequence == 0) {
      // Start of a new sequence.
      memset(&collected_values, 0, sizeof(struct CollectedValues));
      memset(&collected_is_set, 0, sizeof(struct CollectedValues));
    }

    if(collected_is_set.target && collected_values.target != data_entry->target) {
      printf("WARNING: Data corruption: Different targets for entries in same sequence. "
          "%lu %lu\r\n",
          collected_values.target, data_entry->target);
      return;
    }

    if(data_entry->target > MAX_TARGETS) {
      printf("WARNING: Data corruption: Invalid target. %lu\r\n", data_entry->target);
      return;
    }

    collected_values.target = data_entry->target;
    collected_is_set.target = 1;

    switch(data_entry->key) {
      case(TIME_WINDOW):
        collected_values.time_window_us = data_entry->value;
        collected_is_set.time_window_us = 1;
        break;
      case(SET_AXIS_ABS_POS):
        collected_values.set_axis_abs_pos = data_entry->value;
        collected_is_set.set_axis_abs_pos = 1;
        break;
      default:
        printf("Unexpected key: %lu\r\n", data_entry->key);
    }

    if(data_entry->sequence == 0) {
      // Have gathered all associated attributes.
      if(collected_is_set.time_window_us && collected_is_set.set_axis_abs_pos) {
        delegete_set_absolute_position(
            tx_buf,
            collected_values.target,
            collected_values.set_axis_abs_pos,
            collected_values.time_window_us);
      }
    }
    last_sequence = data_entry->sequence;
  }
}

void process_buffer_human(char* rx_buf, char* tx_buf) {
  printf("\nReceived: %s\n", rx_buf);
  uint32_t values[4] = {0,0,0,0};
  size_t value_num = 0;
  char* itterate = rx_buf;
  uint32_t val = 0;
  while(*itterate) {
    if (*itterate == ':') {
      // Separator.
      itterate++;
    } else if (isspace(*itterate)) {
      // Whitespace. Ignore and continue.
      itterate++;
    } else if (isdigit(*itterate)) {
      val = strtol(itterate, &itterate, 10);
      if (value_num < 4) {
        values[value_num] = val;
      } else {
        printf("Too many values. %ld\n", val);
        memset(rx_buf, '\0', DATA_BUF_SIZE);
        return;
      }
      value_num++;
    } else if (*itterate == '?' && value_num == 0) {
      help(tx_buf);
      memset(rx_buf, '\0', DATA_BUF_SIZE);
      return;
    } else {
      printf("Unexpected character: %u : %c\n", *itterate, *itterate);
      memset(rx_buf, '\0', DATA_BUF_SIZE);
      return;
    }
  }

  if (value_num == 4) {
    uint position = send_pio_steps(values[0], values[1], values[2], values[3]);
    snprintf(
        tx_buf,
        DATA_BUF_SIZE,
        "Parsed:\r\n"
        " stepper: %lu\r\n"
        " step_count: %lu\r\n"
        " step_len_us: %lu\r\n"
        " direction: %lu\r\n"
        "Set:\r\n"
        " stepper: %ld\r\n"
        " target_position: %lu\r\n",
        values[0], values[1], values[2], values[3], values[0], position);
  } else if (value_num == 3) {
    delegete_set_absolute_position(tx_buf, values[0], values[1], values[2]);
  } else if (value_num == 1) {
    snprintf(
        tx_buf,
        DATA_BUF_SIZE,
        "Set:\r\n"
        " stepper: %ld\r\n"
        " target_position: %lu\r\n",
        values[0], get_absolute_position(values[0]));
  } else if (value_num > 0) {
    printf("Wrong number of values: %i\r\n", value_num);
  } else {
    snprintf(
        tx_buf,
        DATA_BUF_SIZE,
        "Set:\r\n"
        " stepper: 0\r\n"
        " target_position: %lu\r\n"
        " stepper: 1\r\n"
        " target_position: %lu\r\n"
        " stepper: 2\r\n"
        " target_position: %lu\r\n"
        " stepper: 3\r\n"
        " target_position: %lu\r\n"
        " stepper: 4\r\n"
        " target_position: %lu\r\n"
        " stepper: 5\r\n"
        " target_position: %lu\r\n"
        " stepper: 6\r\n"
        " target_position: %lu\r\n"
        " stepper: 7\r\n"
        " target_position: %lu\r\n",
        get_absolute_position(0),
        get_absolute_position(1),
        get_absolute_position(2),
        get_absolute_position(3),
        get_absolute_position(4),
        get_absolute_position(5),
        get_absolute_position(6),
        get_absolute_position(7)
          );
  }
  memset(rx_buf, '\0', DATA_BUF_SIZE);
}

uint8_t get_uart(char* uart_rx_buf, char* tx_buf, size_t buffer_len) {
    char* start_update = &uart_rx_buf[strlen(uart_rx_buf)];

    int ch = getchar_timeout_us(0);
    while (ch > 0) {
      size_t pos = strlen(uart_rx_buf);
      if (ch == 13) {
        // Return pressed.
        process_buffer_human(uart_rx_buf, tx_buf);
        //memset(uart_rx_buf, '\0', buffer_len);
      } else if (pos < buffer_len - 1 && isprint(ch)) {
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

/*
 * $ nc -u <host> <port>
 */
int32_t comm_UDP(
    uint8_t socket_num,
    uint8_t* nw_rx_buf,
    uint8_t* tx_buf,
    uint16_t port,
    void (*callback)(char*, char*)
    ) {
   int32_t  ret;
   uint16_t size;
   uint16_t sentsize;
   uint8_t  destip[4];
   uint16_t destport;

   switch(getSn_SR(socket_num)) {
      case SOCK_UDP :
         if((size = getSn_RX_RSR(socket_num)) > 0) {
           // Receiving data.
           if(size > DATA_BUF_SIZE) {
             size = DATA_BUF_SIZE;
           }
           ret = recvfrom(socket_num, nw_rx_buf, size, destip, (uint16_t*)&destport);
           if(ret <= 0) {
             printf("%d: recvfrom error. %ld\r\n",socket_num,ret);
             return ret;
           }

           if (strlen(nw_rx_buf) > 0) {
             callback(nw_rx_buf, tx_buf);
           }
         }

         size = strlen(tx_buf);
         if(size > 0) {
           // Sending data.
           sentsize = 0;
           while(sentsize < size) {
             ret = sendto(socket_num, tx_buf + sentsize, size - sentsize, destip, destport);
             if(ret < 0) {
               printf("%d: sendto error. %ld\r\n", socket_num, ret);
               return ret;
             }
             sentsize += ret; // Don't care SOCKERR_BUSY, because it is zero.
           }
         }
         break;
      case SOCK_CLOSED:
         if((ret = socket(socket_num, Sn_MR_UDP, port, 0x00)) != socket_num)
            return ret;
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
  char tx_buf[DATA_BUF_SIZE] = "";

  bi_decl(bi_program_description("This is a test binary."));
  bi_decl(bi_1pin_with_name(LED_PIN, "On-board LED"));

  set_clock_khz();

  stdio_init_all();

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

  // Initialise PIOs.
  uint stepper_count = 2;
  uint pins_step[2] = {0, 2};
  uint pins_direction[2] = {1, 3};
  for (uint stepper = 0; stepper < stepper_count; stepper++) {
    init_pio(stepper, pins_step[stepper], pins_direction[stepper]);
  }

  //send_pios_steps(0, 10, 500000, 1);
  //send_pios_steps(0, 20, 100000, 0);
  //send_pios_steps(1, 4, 2000000, 0);
  //send_pios_steps(1, 2, 5000000, 1);

  sleep_ms(1000);

  while (1) {

    get_uart(uart_rx_buf, tx_buf, DATA_BUF_SIZE);

    if (NET_ENABLE) {
        retval = comm_UDP(
            SOCKET_NUMBER_HUMAN,
            nw_rx_buf,
            tx_buf,
            NW_PORT_HUMAN,
            &process_buffer_human);
        if (retval < 0) {
          printf(" Network error : %d\n", retval);
          while (1);
        }
        retval = comm_UDP(
            SOCKET_NUMBER_MACHINE,
            nw_rx_buf,
            tx_buf,
            NW_PORT_MACHINE,
            &process_buffer_machine);
        if (retval < 0) {
          printf(" Network error : %d\n", retval);
          while (1);
        }
        retval = comm_UDP(
            SOCKET_NUMBER_MACHINE_V02,
            nw_rx_buf,
            tx_buf,
            NW_PORT_MACHINE_V02,
            &process_buffer_machine_v02);
        if (retval < 0) {
          printf(" Network error : %d\n", retval);
          while (1);
        }
    }


    put_uart(tx_buf, DATA_BUF_SIZE);

    memset(tx_buf, '\0', DATA_BUF_SIZE);
  }
}
