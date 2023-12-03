#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#ifndef BUILD_TESTS

#include "pico/stdlib.h"

// w5x00 related.
#include "port_common.h"
#include "wizchip_conf.h"
#include "w5x00_spi.h"
#include "socket.h"

#else

#include <stdint.h>

#endif  // BUILD_TESTS

#include "stepper_control.h"
#include "config.h"
#include "core0.h"
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

  core0_main();
  return 0;
}
