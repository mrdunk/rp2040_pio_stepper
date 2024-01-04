#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"

#define MODBUS_UART uart1
#define MODBUS_TX_PIN 8
#define MODBUS_RX_PIN 9
#define MODBUS_DIR_PIN 10

uint8_t modbus_command[64];
uint8_t modbus_address;
uint8_t modbus_length;
uint16_t modbus_bitrate;
uint16_t modbus_cur_bitrate;
int modbus_pause;
int modbus_cycle;
int modbus_last_control;

static uint16_t crc16_table[256];

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

void huanyang_setfreq(unsigned hertz_x100) {
  modbus_command[0] = modbus_address;
  modbus_command[1] = 5;
  modbus_command[2] = 2;
  modbus_command[3] = hertz_x100 >> 8;
  modbus_command[4] = hertz_x100 & 0xFF;
  modbus_pause = 50;
  modbus_length = 5;
}

void huanyang_read_status(uint8_t status) {
  modbus_command[0] = modbus_address;
  modbus_command[1] = 4;
  modbus_command[2] = 1;
  modbus_command[3] = status;
  modbus_pause = 50;
  modbus_length = 4;
}

void huanyang_control(uint8_t control) {
  modbus_command[0] = modbus_address;
  modbus_command[1] = 3;
  modbus_command[2] = 1;
  modbus_command[3] = control;
  modbus_pause = 50;
  modbus_length = 4;
  modbus_last_control = control;
}

void huanyang_poll_control_status(void) {
  huanyang_control(modbus_last_control);
}

void huanyang_start_forward(void) {
  huanyang_control(1);
}

void huanyang_start_reverse(void) {
  // XXXKF doesn't work!
  huanyang_control(1 | 16);
}

void huanyang_stop(void) {
  huanyang_control(8);
}

void modbus_transmit(void) {
  uint16_t crc16 = modbus_crc16((const uint8_t *)modbus_command, modbus_length);
  gpio_put(MODBUS_DIR_PIN, 1);
  uart_tx_wait_blocking(MODBUS_UART);
  modbus_command[modbus_length++] = crc16 & 0xFF;
  modbus_command[modbus_length++] = crc16 >> 8;
  uart_write_blocking(MODBUS_UART, (const uint8_t *)modbus_command, modbus_length);
}

struct vfd_status {
  uint32_t cycle;
  uint32_t last_status_update;
  uint32_t last_set_freq_update;
  uint32_t last_act_freq_update;
  int16_t set_freq_x100;
  int16_t act_freq_x100;
  uint16_t amps_x10;
  uint16_t rpm;
  uint16_t status_run:1;
  uint16_t status_reverse:1;
  uint16_t status_running:1;
  uint16_t spindle_at_speed:1;
  uint16_t command_run:1;
  uint16_t command_reverse:1;
  int16_t req_freq_x100;
} vfd;

void modbus_init(void) {
  precompute_crc16();
  gpio_set_function(MODBUS_TX_PIN, GPIO_FUNC_UART);
  gpio_set_function(MODBUS_RX_PIN, GPIO_FUNC_UART);
  gpio_init(MODBUS_DIR_PIN);
  gpio_set_dir(MODBUS_DIR_PIN, GPIO_OUT);
  vfd.cycle = 10000;
  vfd.command_run = 0;
  vfd.req_freq_x100 = 0;
  modbus_cur_bitrate = modbus_bitrate;
}

// #define modbus_printf printf
#define modbus_printf(...)

void modbus_huanyang_receive(void) {
  int len = 0;
  for (len = 0; len < sizeof(modbus_command) && uart_is_readable(MODBUS_UART); ++len)
    modbus_command[len] = uart_getc(MODBUS_UART);
  while(uart_is_readable(MODBUS_UART))
    uart_getc(MODBUS_UART);
  if (len) {
    uint16_t crc16 = modbus_crc16(modbus_command, len - 2);
    for (int i = 0; i < len; ++i) {
      modbus_printf("%02x", modbus_command[i]);
    }
    modbus_printf(" (%d): ", len);
    if (crc16 == modbus_command[len - 2] + 256 * modbus_command[len - 1]) {
      modbus_printf("Reply type: %02x len: %02x - ", modbus_command[1], modbus_command[2]);
      switch(modbus_command[1]) {
      case 3:
        vfd.last_status_update = vfd.cycle;
        modbus_printf("Status bits %02x\n", modbus_command[3]);
        vfd.status_run = modbus_command[3] & 0x01 ? 1 : 0;
        vfd.status_reverse = modbus_command[3] & 0x04 ? 1 : 0;
        vfd.status_running = modbus_command[3] & 0x08 ? 1 : 0;
        break;
      case 4:
        if (len >= 6) {
          int value = modbus_command[4] * 256  + modbus_command[5];
          modbus_printf("Status type %02x, value %d\n", modbus_command[3], value);
          switch(modbus_command[3]) {
          case 0:
            vfd.last_set_freq_update = vfd.cycle;
            vfd.set_freq_x100 = value;
            break;
          case 1:
            vfd.last_act_freq_update = vfd.cycle;
            vfd.act_freq_x100 = value;
            break;
          case 2:
            vfd.amps_x10 = value;
            break;
          case 3:
            vfd.rpm = value;
            break;
          default:
            break;
          }
        }
        break;
      case 5:
        if (len >= 6) {
          int value = modbus_command[3] * 256  + modbus_command[4];
          modbus_printf("Frequency value %d\n", value);
          vfd.last_set_freq_update = vfd.cycle;
          vfd.set_freq_x100 = value;
        }
        break;
      default:
        printf("Unrecognized response %02x\n", modbus_command[1]);
      }
    } else {
      printf("Response CRC error\n");
    }
  }
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
    while(size > 0)
      *wrptr++ = *data++;
  }
  uint16_t crc16 = modbus_crc16(modbus_command, wrptr - modbus_command);
  *wrptr++ = (uint8_t)crc16;
  *wrptr++ = (uint8_t)(crc16 >> 8);
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

static inline int modbus_encode_twoints(uint8_t address, uint8_t command, uint16_t v1, uint16_t v2)
{
    return modbus_rtu_encode(address, command, v1, v2, NULL, 0);
}

int modbus_read_coils(uint8_t address, uint16_t coil_to_read, uint16_t num_coils)
{
  return modbus_encode_twoints(address, 1, coil_to_read, num_coils);
}

int modbus_read_discrete_inputs(uint8_t address, uint16_t reg_to_read, uint16_t num_regs)
{
  return modbus_encode_twoints(address, 2, reg_to_read, num_regs);
}

int modbus_read_holding_registers(uint8_t address, uint16_t reg_to_read, uint16_t num_regs)
{
  return modbus_encode_twoints(address, 3, reg_to_read, num_regs);
}

int modbus_read_input_registers(uint8_t address, uint16_t reg_to_read, uint16_t num_regs)
{
  return modbus_encode_twoints(address, 4, reg_to_read, num_regs);
}

int modbus_write_coil(uint8_t address, uint16_t coil_address, uint8_t value)
{
  return modbus_encode_twoints(address, 5, coil_address, (value ? 0xFF : 0));
}

int modbus_write_holding_register(uint8_t address, uint16_t reg_to_write, uint16_t value)
{
  return modbus_encode_twoints(address, 6, reg_to_write, value);
}

int modbus_write_multiple_coils(uint8_t address, uint16_t coil_to_write, uint16_t num_coils, const uint8_t *values)
{
  return modbus_rtu_encode(address, 15, coil_to_write, num_coils, values, (num_coils + 7) >> 3);
}

int modbus_write_multiple_holding_registers(uint8_t address, uint16_t reg_to_write, uint16_t num_regs, const uint16_t *values)
{
  uint8_t encoded[2 * num_regs];
  uint8_t *wrptr = encoded;
  for (int i = 0; i < num_regs; ++i) {
    *wrptr++ = (uint8_t)(values[i] >> 8);
    *wrptr++ = (uint8_t)values[i];
  }
  return modbus_rtu_encode(address, 16, reg_to_write, num_regs, encoded, wrptr - encoded);
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

// Fuling inverter uses holding registers for everything (commands 3 and 6), read up to 5 words
#define FULING_DZB_CMD_CONTROL 0x1000
#define FULING_DZB_CMD_CONTROL_RUN_FWD 1
#define FULING_DZB_CMD_CONTROL_RUN_REV 2
#define FULING_DZB_CMD_CONTROL_JOG_FWD 3
#define FULING_DZB_CMD_CONTROL_JOG_REV 4
#define FULING_DZB_CMD_CONTROL_STOP 5
#define FULING_DZB_CMD_CONTROL_E_STOP 6
#define FULING_DZB_CMD_CONTROL_FAULT_RESET 7
#define FULING_DZB_CMD_CONTROL_JOG_STOP 8

#define FULING_DZB_CMD_STATE 0x1001
#define FULING_DZB_CMD_STATE_RUN_FWD 1
#define FULING_DZB_CMD_STATE_RUN_REV 2
#define FULING_DZB_CMD_STATE_RUN_STANDBY 3
#define FULING_DZB_CMD_STATE_RUN_FAULT 4

// Technically, this is the 'communication setting', and the range is -10000 to 10000 (corresponding to -100%..100% of the maximum frequency)
#define FULING_DZB_CMD_SPEED 0x2000

#define FULING_DZB_CMD_STATUS_SET_FREQ 0x3000
#define FULING_DZB_CMD_STATUS_RUN_FREQ 0x3001
#define FULING_DZB_CMD_STATUS_OUT_CURRENT 0x3002
#define FULING_DZB_CMD_STATUS_OUT_VOLTAGE 0x3003
#define FULING_DZB_CMD_STATUS_RUN_SPEED 0x3004
#define FULING_DZB_CMD_STATUS_OUT_POWER 0x3005
#define FULING_DZB_CMD_STATUS_OUT_TORQUE 0x3006
#define FULING_DZB_CMD_STATUS_DC_BUS_VOLTAGE 0x3007
#define FULING_DZB_CMD_STATUS_IN_TERM_STATUS 0x300A
#define FULING_DZB_CMD_STATUS_OUT_TERM_STATUS 0x300B

#define FULING_DZB_CMD_INVERTER_FAULT 0x5000
#define FULING_DZB_CMD_MODBUS_FAULT 0x5001
#define FULING_DZB_CMD_MODBUS_FAULT_NONE 0
#define FULING_DZB_CMD_MODBUS_FAULT_PASSWORD 1
#define FULING_DZB_CMD_MODBUS_FAULT_CMD_CODE 2
#define FULING_DZB_CMD_MODBUS_FAULT_CRC_ERROR 3
#define FULING_DZB_CMD_MODBUS_FAULT_INVALID_ADDRESS 4
#define FULING_DZB_CMD_MODBUS_FAULT_INVALID_DATA 5
#define FULING_DZB_CMD_MODBUS_FAULT_CHANGE_INVALID 6
#define FULING_DZB_CMD_MODBUS_FAULT_LOCKED 7
#define FULING_DZB_CMD_MODBUS_FAULT_EEPROM_BUSY 8

void modbus_fuling_receive(void) {
  int len = 0;
  for (len = 0; len < sizeof(modbus_command) && uart_is_readable(MODBUS_UART); ++len)
    modbus_command[len] = uart_getc(MODBUS_UART);
  while(uart_is_readable(MODBUS_UART))
    uart_getc(MODBUS_UART);
  if (len) {
    uint16_t crc16 = modbus_crc16(modbus_command, len - 2);
    for (int i = 0; i < len; ++i) {
      modbus_printf("%02x", modbus_command[i]);
    }
    modbus_printf(" (%d): ", len);
    if (crc16 == modbus_command[len - 2] + 256 * modbus_command[len - 1]) {
      modbus_printf("Reply type: %02x len: %02x - ", modbus_command[1], modbus_command[2]);
      switch(modbus_command[1]) {
      case 3:
        switch(modbus_command[2]) { // tell by byte count, 2 = status, 4 = data
        case 2:
          vfd.status_run = modbus_command[4] == FULING_DZB_CMD_STATE_RUN_FWD || modbus_command[4] == FULING_DZB_CMD_STATE_RUN_REV;
          vfd.status_reverse = modbus_command[4] == FULING_DZB_CMD_STATE_RUN_REV;
          vfd.status_running = vfd.status_run;
          break;
        case 4:
          vfd.set_freq_x100 = modbus_command[4] * 256  + modbus_command[5];
          vfd.act_freq_x100 = modbus_command[6] * 256  + modbus_command[7];
          break;
        }
        break;
      case 6: // write
        break;
      default:
        printf("Unrecognized response %02x\n", modbus_command[1]);
      }
    } else {
      printf("Response CRC error\n");
    }
  }
}

float modbus_loop_fuling(float frequency) {
  if (modbus_cur_bitrate != modbus_bitrate) {
    if (modbus_cur_bitrate)
      uart_deinit(MODBUS_UART);
    uart_init(MODBUS_UART, modbus_bitrate);
    modbus_cur_bitrate = modbus_bitrate;
  }
  if (!modbus_cur_bitrate || !modbus_address)
    return 2000000.0;
  vfd.cycle++;
  vfd.command_run = frequency != 0;
  vfd.command_reverse = frequency < 0;
  vfd.req_freq_x100 = abs((int16_t)(frequency * 100));
  if (modbus_pause > 0) {
    if ((uart_get_hw(MODBUS_UART)->fr & UART_UARTFR_BUSY_BITS)) {
      goto do_pause;
    }
    gpio_put(MODBUS_DIR_PIN, 0);
    modbus_pause--;
    goto do_pause;
  }
  
  modbus_fuling_receive();
  int refresh_delay = 1000; // delay between polls for the same value
  int freshness_limit = 10000; // No updates after this time means that the data are stale
  if (vfd.cycle - vfd.last_status_update > refresh_delay) {
    modbus_read_holding_registers(modbus_address, FULING_DZB_CMD_STATE, 1);
    modbus_transmit();
  } else if (vfd.cycle - vfd.last_set_freq_update > refresh_delay) {
    modbus_read_holding_registers(modbus_address, FULING_DZB_CMD_STATUS_SET_FREQ, 2);
    modbus_transmit();
  } else {
    if (vfd.req_freq_x100 != vfd.set_freq_x100) {
      modbus_write_holding_register(modbus_address, FULING_DZB_CMD_SPEED, vfd.req_freq_x100 * 10000 / 800);
      modbus_transmit();
    } else if (vfd.command_run != vfd.status_run || vfd.command_reverse != vfd.status_reverse) {
      if (vfd.status_run && !vfd.command_run) {
        modbus_write_holding_register(modbus_address, FULING_DZB_CMD_CONTROL, FULING_DZB_CMD_CONTROL_STOP);
      } else {
        if (vfd.command_reverse) {
          modbus_write_holding_register(modbus_address, FULING_DZB_CMD_CONTROL, FULING_DZB_CMD_CONTROL_RUN_REV);
        } else {
          modbus_write_holding_register(modbus_address, FULING_DZB_CMD_CONTROL, FULING_DZB_CMD_CONTROL_RUN_FWD);
        }
      }
      modbus_transmit();
    }
  }
do_pause:
  if (!((vfd.cycle - vfd.last_status_update < freshness_limit) &&
    (vfd.cycle - vfd.last_set_freq_update < freshness_limit))) {
    return 1000000.0;
  }

  if (!vfd.status_running)
    return 0;
  return (vfd.status_reverse ? -1 : 1) * vfd.act_freq_x100 / 100.0;
}

float modbus_loop_huanyang(float frequency) {
  if (modbus_cur_bitrate != modbus_bitrate) {
    if (modbus_cur_bitrate)
      uart_deinit(MODBUS_UART);
    uart_init(MODBUS_UART, modbus_bitrate);
    modbus_cur_bitrate = modbus_bitrate;
  }
  if (!modbus_cur_bitrate || !modbus_address)
    return 2000000.0;
  vfd.cycle++;
  vfd.command_run = frequency != 0;
  vfd.command_reverse = frequency < 0;
  vfd.req_freq_x100 = abs((int16_t)(frequency * 100));
  if (modbus_pause > 0) {
    if ((uart_get_hw(MODBUS_UART)->fr & UART_UARTFR_BUSY_BITS)) {
      goto do_pause;
    }
    gpio_put(MODBUS_DIR_PIN, 0);
    modbus_pause--;
    goto do_pause;
  }
  
  modbus_huanyang_receive();
  int refresh_delay = 1000; // delay between polls for the same value
  int freshness_limit = 10000; // No updates after this time means that the data are stale
  if (vfd.cycle - vfd.last_status_update > refresh_delay) {
    huanyang_poll_control_status();
    modbus_transmit();
  } else if (vfd.cycle - vfd.last_set_freq_update > refresh_delay) {
    huanyang_read_status(0);
    modbus_transmit();
  } else if (vfd.cycle - vfd.last_act_freq_update > refresh_delay) {
    huanyang_read_status(1);
    modbus_transmit();
  } else {
    if (vfd.req_freq_x100 != vfd.set_freq_x100) {
      huanyang_setfreq(vfd.req_freq_x100);
      modbus_transmit();
    } else if (vfd.command_run != vfd.status_run || vfd.command_reverse != vfd.status_reverse) {
      if (vfd.status_run && !vfd.command_run) {
        huanyang_stop();
      } else {
        if (vfd.command_reverse) {
          huanyang_start_reverse();
        } else {
          huanyang_start_forward();
        }
      }
      modbus_transmit();
    }
  }
do_pause:
  if (!((vfd.cycle - vfd.last_status_update < freshness_limit) &&
    (vfd.cycle - vfd.last_set_freq_update < freshness_limit) &&
    (vfd.cycle - vfd.last_act_freq_update < freshness_limit))) {
    return 1000000.0;
  }

  if (!vfd.status_running)
    return 0;
  return (vfd.status_reverse ? -1 : 1) * vfd.act_freq_x100 / 100.0;
}

float modbus_loop(float frequency) {
  return modbus_loop_fuling(frequency);
}