#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef BUILD_TESTS
  #include "../test/mocks/rp_mocks.h"
#else  // BUILD_TESTS
  #include "pico/stdlib.h"
#endif  // BUILD_TESTS

#include "modbus.h"

uint16_t crc16_table[256];

uint8_t modbus_command[128];
uint8_t modbus_length;
uint16_t modbus_cur_bitrate;
uint16_t modbus_pause;
uint8_t modbus_outstanding;

struct vfd_config vfd_config;
struct vfd_status vfd;

static void precompute_crc16(void) {
  for (int value = 0; value < 256; ++value) {
    uint16_t crc16 = value;
    for (uint8_t j = 0; j < 8; ++j) {
      crc16 = (crc16 >> 1) ^ (((~crc16 & 1) - 1) & 0xA001);
    }
    crc16_table[value] = crc16;
  }
}

uint16_t modbus_crc16(const uint8_t *data, uint8_t size) {
  uint16_t crc16 = 0xFFFF;
  for (uint8_t i = 0; i < size; ++i) {
    crc16 ^= data[i];
    crc16 = (crc16 >> 8) ^ crc16_table[crc16 & 0xFF];
  }
  return crc16;
}

void modbus_transmit(void) {
  gpio_put(MODBUS_DIR_PIN, 1);
  uint16_t crc16 = modbus_crc16((const uint8_t *)modbus_command, modbus_length);
  uart_tx_wait_blocking(MODBUS_UART);
  modbus_command[modbus_length++] = crc16 & 0xFF;
  modbus_command[modbus_length++] = crc16 >> 8;
  uart_write_blocking(MODBUS_UART, (const uint8_t *)modbus_command, modbus_length);
  modbus_outstanding = 1;
}

void modbus_init(void) {
  precompute_crc16();
  gpio_set_function(MODBUS_TX_PIN, GPIO_FUNC_UART);
  gpio_set_function(MODBUS_RX_PIN, GPIO_FUNC_UART);
  gpio_init(MODBUS_DIR_PIN);
  gpio_set_dir(MODBUS_DIR_PIN, GPIO_OUT);
  vfd.cycle = 10000;
  vfd.command_run = 0;
  vfd.req_freq_x10 = 0;
  memset(&vfd.stats, 0, sizeof(vfd.stats));
  modbus_cur_bitrate = 0;
}

int modbus_rtu_encode(uint8_t address, uint8_t function_code, uint16_t v1, uint16_t v2, const uint8_t *data, uint8_t size)
{
  uint8_t *wrptr = modbus_command;
  *wrptr++ = address;
  *wrptr++ = function_code;
  *wrptr++ = (uint8_t)(v1 >> 8);
  *wrptr++ = (uint8_t)v1;
  *wrptr++ = (uint8_t)(v2 >> 8);
  *wrptr++ = (uint8_t)v2;
  if (data && size) {
    *wrptr++ = size;
    while(size > 0) {
      *wrptr++ = *data++;
      size--;
    }
  }
  modbus_pause = 200;
  return wrptr - modbus_command;
}

int modbus_rtu_check(const uint8_t *data, uint8_t size) {
  if (size < 2)
    return 0;
  uint16_t crc16 = modbus_crc16(data, size - 2);
  if (data[size - 2] != (crc16 & 0xFF) || data[size - 1] != (crc16 >> 8))
    return 0;
  return 1;
}

static inline void modbus_encode_twoints(uint8_t address, uint8_t command, uint16_t v1, uint16_t v2)
{
  modbus_length = modbus_rtu_encode(address, command, v1, v2, NULL, 0);
}

void modbus_read_holding_registers(uint8_t address, uint16_t reg_to_read, uint16_t num_regs)
{
  modbus_encode_twoints(address, 3, reg_to_read, num_regs);
}

void modbus_write_holding_register(uint8_t address, uint16_t reg_to_write, uint16_t value)
{
  modbus_encode_twoints(address, 6, reg_to_write, value);
}

int modbus_check_config(void)
{
  if (modbus_cur_bitrate != vfd_config.bitrate) {
    if (modbus_cur_bitrate)
      uart_deinit(MODBUS_UART);
    uart_init(MODBUS_UART, vfd_config.bitrate);
    modbus_cur_bitrate = vfd_config.bitrate;
    modbus_outstanding = 0;
  }
  return modbus_cur_bitrate && vfd_config.address;
}

int modbus_get_data(void)
{
  int len = 0;
  for (len = 0; len < sizeof(modbus_command) && uart_is_readable(MODBUS_UART); ++len)
    modbus_command[len] = uart_getc(MODBUS_UART);
  while(uart_is_readable(MODBUS_UART))
    uart_getc(MODBUS_UART);
  return len;
}

void dump_buffer(const char *name, const uint8_t *buffer, int bytes)
{
  if (name) {
    printf("%s:", name);
  }
  for (int i = 0; i < bytes; ++i) {
    printf(" %02x", buffer[i]);
  }
  printf("\n");
}

float modbus_loop(float frequency) {
  switch(vfd_config.type) {
    case MODBUS_TYPE_HUANYANG:
      return modbus_loop_huanyang(frequency);
    case MODBUS_TYPE_FULING:
      return modbus_loop_fuling(frequency);
    case MODBUS_TYPE_WEIKEN:
      return modbus_loop_weiken(frequency);
    case MODBUS_TYPE_LOOPBACK:
      return frequency;
    default:
      return 0.f;
  }
}
