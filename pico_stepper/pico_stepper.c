#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "pico/stdlib.h"

// PIO related.
#include "hardware/pio.h"
#include "hardware/clocks.h"
// The compiled .pio.
#include "pico_stepper.pio.h"

// w5x00 related.
#include "port_common.h"
#include "wizchip_conf.h"
#include "w5x00_spi.h"
#include "socket.h"

/* Clock */
#define PLL_SYS_KHZ (133 * 1000)

/* Buffer */
#define ETHERNET_BUF_MAX_SIZE (1024 * 2)

/* Socket */
#define SOCKET_NUMBER 0

/* Port */
#define NW_PORT 5000

#define NET_ENABLE 1
#define DATA_BUF_SIZE 255

const uint LED_PIN = 25;

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

void init_pio(
    PIO pio,
    uint sm,
    uint offset,
    uint pin_step,
    uint pin_direction)
{
  step_program_init(pio, sm, offset, pin_step, pin_direction);
  pio_sm_set_enabled(pio, sm, true);
}

void init_pios(
    uint stepper,
    uint pin_step,
    uint pin_direction
    )
{
  uint offset;
  switch (stepper) {
    case 0:
      offset = pio_add_program(pio0, &step_program);
      init_pio(pio0, 0, offset, pin_step, pin_direction);
      break;
    case 1:
      offset = pio_add_program(pio0, &step_program);
      init_pio(pio0, 1, offset, pin_step, pin_direction);
      break;
    case 2:
      offset = pio_add_program(pio0, &step_program);
      init_pio(pio0, 2, offset, pin_step, pin_direction);
      break;
    case 3:
      offset = pio_add_program(pio0, &step_program);
      init_pio(pio0, 3, offset, pin_step, pin_direction);
      break;
    case 4:
      offset = pio_add_program(pio1, &step_program);
      init_pio(pio1, 0, offset, pin_step, pin_direction);
      break;
    case 5:
      offset = pio_add_program(pio1, &step_program);
      init_pio(pio1, 1, offset, pin_step, pin_direction);
      break;
    case 6:
      offset = pio_add_program(pio1, &step_program);
      init_pio(pio1, 2, offset, pin_step, pin_direction);
      break;
    case 7:
      offset = pio_add_program(pio1, &step_program);
      init_pio(pio1, 3, offset, pin_step, pin_direction);
      break;
  }
}

void send_pio_steps(
    PIO pio,
    uint sm,
    uint time_slice_us,
    uint freq,
    uint direction
    ) {
  uint32_t pulse_len = (clock_get_hz(clk_sys) / (2 * freq));
  uint32_t step_count = time_slice_us * freq / 1000000;

  printf("%ld\t%ld\n", step_count, pulse_len);
  if(step_count == 0) {
    // No steps to add.
    return;
  }
  if(pulse_len <= 9) {
    // Step too short.
    return;
  }

  // PIO program generates 1 more step than it's told to.
  step_count -= 1;

  // PIO program makes steps 9 instructions longer than it's told to.
  pulse_len -= 9;

  pio_sm_put(pio, sm, direction);
  pio_sm_put(pio, sm, pulse_len);
  pio_sm_put(pio, sm, step_count);
}

void send_pios_steps(
    uint stepper,
    uint time_slice_us,
    uint freq,
    uint direction)
{
  switch (stepper) {
    case 0:
      send_pio_steps(pio0, 0, time_slice_us, freq, direction);
      break;
    case 1:
      send_pio_steps(pio0, 1, time_slice_us, freq, direction);
      break;
    case 2:
      send_pio_steps(pio0, 2, time_slice_us, freq, direction);
      break;
    case 3:
      send_pio_steps(pio0, 3, time_slice_us, freq, direction);
      break;
    case 4:
      send_pio_steps(pio1, 0, time_slice_us, freq, direction);
      break;
    case 5:
      send_pio_steps(pio1, 1, time_slice_us, freq, direction);
      break;
    case 6:
      send_pio_steps(pio1, 2, time_slice_us, freq, direction);
      break;
    case 7:
      send_pio_steps(pio1, 3, time_slice_us, freq, direction);
      break;
  }
}

void send_data(uint32_t *values) {
  // printf("%ld:%ld:%ld:%ld\n", values[0], values[1], values[2], values[3]);
  printf("Sending data\n");
  send_pios_steps(values[0], values[1], values[2], values[3]);
}

void process_buffer(char* tx_buffer, char* rx_buffer) {
  printf("\nReceived: %s\n", tx_buffer);
  uint32_t values[4] = {0,0,0};
  size_t value_num = 0;
  char* itterate = tx_buffer;
  uint32_t val = 0;
  while(*itterate) {
    if (*itterate == ':') {
      // Separator.
      printf(":\n");
      itterate++;
    } else if (isspace(*itterate)) {
      // Whitespace. Ignore and continue.
      itterate++;
    } else if (isdigit(*itterate)) {
      val = strtol(itterate, &itterate, 10);
      if (value_num < 4) {
        printf("%ld\n", val);
        values[value_num] = val;
      } else {
        printf("Too many values. %ld\n", val);
        return;
      }
      value_num++;
    } else {
      printf("Unexpected character: %u : %c\n", *itterate, *itterate);
      return;
    }
  }

  if (value_num == 4) {
    snprintf(
        rx_buffer,
        DATA_BUF_SIZE,
        "Parsed: %ld:%ld:%ld:%ld\r\n",
        values[0], values[1], values[2], values[3]);
    send_data(values);
  } else {
    printf("Wrong number of values: %i\n", value_num);
  }
}

uint8_t get_uart(char* uart_tx_buffer, size_t buffer_len) {
    char* start_update = &uart_tx_buffer[strlen(uart_tx_buffer)];

    int ch = getchar_timeout_us(0);
    while (ch > 0) {
      size_t pos = strlen(uart_tx_buffer);
      if (ch == 13) {
        // Return pressed.
        return 1;
      } else if (pos < buffer_len - 1 && isprint(ch)) {
        uart_tx_buffer[pos] = ch;
        printf("%c", ch);
      }
      ch = getchar_timeout_us(0);
    }

    return 0;
}

void put_uart(char* rx_buffer, size_t buffer_len) {
  puts_raw(rx_buffer);
  memset(rx_buffer, '\0', buffer_len);
}

void comm_uart(char* uart_tx_buffer, char* rx_buffer, size_t buffer_len) {
    uint8_t buffer_state = get_uart(uart_tx_buffer, buffer_len);
    if (buffer_state > 0) {
      process_buffer(uart_tx_buffer, rx_buffer);
      memset(uart_tx_buffer, '\0', buffer_len);
    }

    if(rx_buffer[0] != '\0') {
      put_uart(rx_buffer, buffer_len);
    }
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

   switch(getSn_SR(socket_num))
   {
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

            sentsize = 0;
            size = strlen(rx_buffer);
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

int32_t comm_TCP(
    uint8_t socket_num, uint8_t* network_tx_buffer, uint8_t* rx_buffer, uint16_t port) {
  int32_t ret;
  uint16_t size_received = 0;

  uint8_t destip[4];
  uint16_t destport;

  switch(getSn_SR(socket_num))
  {
    case SOCK_ESTABLISHED :
      if(getSn_IR(socket_num) & Sn_IR_CON) {
        getSn_DIPR(socket_num, destip);
        destport = getSn_DPORT(socket_num);

        printf("%d:Connected - %d.%d.%d.%d : %d\r\n",
            socket_num, destip[0], destip[1], destip[2], destip[3], destport);
        setSn_IR(socket_num,Sn_IR_CON);
      }
      if((size_received = getSn_RX_RSR(socket_num)) > 0) {
        if(size_received > DATA_BUF_SIZE) {
          size_received = DATA_BUF_SIZE;
        }
        ret = recv(socket_num, network_tx_buffer, size_received);

        if(ret <= 0) {
          return ret;
        }

        if (strlen(network_tx_buffer) > 0) {
          process_buffer(network_tx_buffer, rx_buffer);
        }
      }
      break;
    case SOCK_CLOSE_WAIT :
      if((ret = disconnect(socket_num)) != SOCK_OK) {
        return ret;
      }
      printf("%d:Socket Closed\r\n", socket_num);
      break;
    case SOCK_INIT :
      printf("%d:Listen, TCP server, port [%d]\r\n", socket_num, port);
      if( (ret = listen(socket_num)) != SOCK_OK) {
        return ret;
      }
      break;
    case SOCK_CLOSED:
      if((ret = socket(socket_num, Sn_MR_TCP, port, 0x00)) != socket_num) {
        return ret;
      }
      break;
    default:
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
    init_pios(stepper, pins_step[stepper], pins_direction[stepper]);
  }

  //send_pios_steps(0, 5000000, 1, 1);
  //send_pios_steps(0, 10000000, 2, 0);
  //send_pios_steps(1, 20000000, 4, 0);
  //send_pios_steps(1, 5000000, 1, 1);

  sleep_ms(1000);

  while (1) {

    if (NET_ENABLE &&
        // (retval = comm_TCP(SOCKET_NUMBER, network_tx_buffer, rx_buffer, NW_PORT)) < 0) {
        (retval = comm_UDP(SOCKET_NUMBER, network_tx_buffer, rx_buffer, NW_PORT)) < 0) {
      printf(" Network error : %d\n", retval);
      while (1);
    }

    comm_uart(uart_tx_buffer, rx_buffer, DATA_BUF_SIZE);

  }
}
