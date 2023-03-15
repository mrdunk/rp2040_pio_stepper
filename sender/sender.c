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

void help(char* rx_buffer) {
    snprintf(
        rx_buffer,
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

void process_buffer(char* tx_buffer, char* rx_buffer) {
  printf("\nReceived: %s\n", tx_buffer);
  uint32_t values[4] = {0,0,0,0};
  size_t value_num = 0;
  char* itterate = tx_buffer;
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
        memset(tx_buffer, '\0', DATA_BUF_SIZE);
        return;
      }
      value_num++;
    } else if (*itterate == '?' && value_num == 0) {
      help(rx_buffer);
      memset(tx_buffer, '\0', DATA_BUF_SIZE);
      return;
    } else {
      printf("Unexpected character: %u : %c\n", *itterate, *itterate);
      memset(tx_buffer, '\0', DATA_BUF_SIZE);
      return;
    }
  }

  if (value_num == 4) {
    uint position = send_pio_steps(values[0], values[1], values[2], values[3]);
    snprintf(
        rx_buffer,
        DATA_BUF_SIZE,
        "Parsed:\r\n"
        " stepper: %ld\r\n"
        " step_count: %ld\r\n"
        " step_len_us: %ld\r\n"
        " direction: %ld\r\n"
        "Configured:\r\n"
        " stepper: %ld\r\n"
        " target_position: %lu\r\n",
        values[0], values[1], values[2], values[3], values[0], position);
  } else if (value_num == 3) {
    uint position = set_absolute_position(values[0], values[1], values[2]);
    snprintf(
        rx_buffer,
        DATA_BUF_SIZE,
        "Parsed:\r\n"
        " stepper: %ld\r\n"
        " new_position: %ld\r\n"
        " time_slice_us: %ld\r\n"
        "Configured:\r\n"
        " stepper: %ld\r\n"
        " target_position: %lu\r\n",
        values[0], values[1], values[2], values[0], position);
  } else if (value_num == 1) {
    snprintf(
        rx_buffer,
        DATA_BUF_SIZE,
        "Configured:\r\n"
        " stepper: %ld\r\n"
        " target_position: %lu\r\n",
        values[0], get_absolute_position(values[0]));
  } else if (value_num > 0) {
    printf("Wrong number of values: %i\r\n", value_num);
  } else {
    snprintf(
        rx_buffer,
        DATA_BUF_SIZE,
        "Configured:\r\n"
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
  memset(tx_buffer, '\0', DATA_BUF_SIZE);
}

uint8_t get_uart(char* uart_tx_buffer, char* rx_buffer, size_t buffer_len) {
    char* start_update = &uart_tx_buffer[strlen(uart_tx_buffer)];

    int ch = getchar_timeout_us(0);
    while (ch > 0) {
      size_t pos = strlen(uart_tx_buffer);
      if (ch == 13) {
        // Return pressed.
        process_buffer(uart_tx_buffer, rx_buffer);
        //memset(uart_tx_buffer, '\0', buffer_len);
      } else if (pos < buffer_len - 1 && isprint(ch)) {
        uart_tx_buffer[pos] = ch;
        printf("%c", ch);
      }
      ch = getchar_timeout_us(0);
    }

    return 0;
}

void put_uart(char* rx_buffer, size_t buffer_len) {
  if(rx_buffer[0] == '\0') {
    return;
  }
  puts_raw(rx_buffer);
}

/*
 * $ nc -u <host> <port>
 */
int32_t comm_UDP(
    uint8_t socket_num, uint8_t* network_tx_buffer, uint8_t* rx_buffer, uint16_t port) {
   int32_t  ret;
   uint16_t size;
   uint16_t sentsize;
   uint8_t  destip[4];
   uint16_t destport;

   switch(getSn_SR(socket_num)) {
      case SOCK_UDP :
         if((size = getSn_RX_RSR(socket_num)) > 0) {
            if(size > DATA_BUF_SIZE) {
              size = DATA_BUF_SIZE;
            }
            ret = recvfrom(socket_num, network_tx_buffer, size, destip, (uint16_t*)&destport);
            if(ret <= 0) {
               printf("%d: recvfrom error. %ld\r\n",socket_num,ret);
               return ret;
            }

            if (strlen(network_tx_buffer) > 0) {
              process_buffer(network_tx_buffer, rx_buffer);
            }
         }

         size = strlen(rx_buffer);
         if(size > 0) {
            sentsize = 0;
            while(sentsize < size) {
              ret = sendto(socket_num, rx_buffer + sentsize, size - sentsize, destip, destport);
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
  char uart_tx_buffer[DATA_BUF_SIZE] = "";
  char network_tx_buffer[DATA_BUF_SIZE] = "";
  char rx_buffer[DATA_BUF_SIZE] = "";

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

    get_uart(uart_tx_buffer, rx_buffer, DATA_BUF_SIZE);

    if (NET_ENABLE &&
        (retval = comm_UDP(SOCKET_NUMBER, network_tx_buffer, rx_buffer, NW_PORT)) < 0) {
      printf(" Network error : %d\n", retval);
      while (1);
    }

    put_uart(rx_buffer, DATA_BUF_SIZE);

    memset(rx_buffer, '\0', DATA_BUF_SIZE);
  }
}
