#include "modbus.h"
#include <stdlib.h>

#ifdef BUILD_TESTS
  #include "../test/mocks/rp_mocks.h"
#else  // BUILD_TESTS
  #include "pico/stdlib.h"
#endif  // BUILD_TESTS

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

static void modbus_fuling_receive(void)
{
  int len = modbus_get_data();
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
          modbus_printf("Status %x\n", modbus_command[4]);
          vfd.last_status_update = vfd.cycle;
          vfd.status_run = modbus_command[4] == FULING_DZB_CMD_STATE_RUN_FWD || modbus_command[4] == FULING_DZB_CMD_STATE_RUN_REV;
          vfd.status_reverse = modbus_command[4] == FULING_DZB_CMD_STATE_RUN_REV;
          vfd.status_running = vfd.status_run;
          break;
        case 4:
          vfd.last_set_freq_update = vfd.cycle;
          vfd.set_freq_x10 = modbus_command[3] * 256U + modbus_command[4];
          vfd.act_freq_x10 = modbus_command[5] * 256U + modbus_command[6];
          modbus_printf("Freqs %d %d (%d)\n", (int)vfd.set_freq_x10, (int)vfd.act_freq_x10, (int)(vfd.req_freq_x10));
          break;
        default:
          ++vfd.stats.unknown;
          break;
        }
        break;
      case 6: // write
        if (modbus_command[2] == 0x50 && modbus_command[3] == 0x01) {
          modbus_printf("Fault %x\n", modbus_command[4] * 256 + modbus_command[5]);
        } else {
          modbus_printf("Write\n");
        }
        break;
      default:
        ++vfd.stats.unknown;
        modbus_printf("Unrecognized response %02x\n", modbus_command[1]);
      }
    } else {
      ++vfd.stats.crc_errors;
    }
  } else if (modbus_outstanding) {
    ++vfd.stats.unanswered;
  }
  modbus_outstanding = 0;
}

float modbus_loop_fuling(float frequency)
{
  if (!modbus_check_config())
    return MODBUS_RESULT_NOT_CONFIGURED;
  int refresh_delay = 1000; // delay between polls for the same value
  int freshness_limit = 10000; // No updates after this time means that the data are stale
  vfd.cycle++;
  vfd.command_run = frequency != 0;
  vfd.command_reverse = frequency < 0;
  vfd.req_freq_x10 = abs((int16_t)(frequency * 10));
  if (modbus_pause > 0) {
    if ((uart_get_hw(MODBUS_UART)->fr & UART_UARTFR_BUSY_BITS)) {
      goto do_pause;
    }
    gpio_put(MODBUS_DIR_PIN, 0);
    modbus_pause--;
    goto do_pause;
  }
  
  modbus_fuling_receive();
  if (vfd.cycle - vfd.last_status_update > refresh_delay) {
    modbus_read_holding_registers(vfd_config.address, FULING_DZB_CMD_STATE, 1);
    modbus_transmit();
  } else if (vfd.cycle - vfd.last_set_freq_update > refresh_delay) {
    modbus_read_holding_registers(vfd_config.address, FULING_DZB_CMD_STATUS_SET_FREQ, 2);
    modbus_transmit();
  } else {
    // This assumes top frequency of 800 Hz (4-pole motor with max RPM of 24000).
    // Long term, it should either ask the inverter (P0.04 or 4) or get it from the configuration.
    // 5 / 4 because of the rate range of -10000..10000 and the frequency being in 0.1 Hz
    // units.
    int rate = (vfd.req_freq_x10 * 5 + 2) / 4;
    // Value received is not always exactly the value sent.
    if (abs((rate * 4 + 2) / 5 - vfd.set_freq_x10) > 1) {
      modbus_write_holding_register(vfd_config.address, FULING_DZB_CMD_SPEED, rate);
      modbus_transmit();
    } else if (vfd.command_run != vfd.status_run || vfd.command_reverse != vfd.status_reverse) {
      if (vfd.status_run && !vfd.command_run) {
        modbus_write_holding_register(vfd_config.address, FULING_DZB_CMD_CONTROL, FULING_DZB_CMD_CONTROL_STOP);
      } else {
        if (vfd.command_reverse) {
          modbus_write_holding_register(vfd_config.address, FULING_DZB_CMD_CONTROL, FULING_DZB_CMD_CONTROL_RUN_REV);
        } else {
          modbus_write_holding_register(vfd_config.address, FULING_DZB_CMD_CONTROL, FULING_DZB_CMD_CONTROL_RUN_FWD);
        }
      }
      modbus_transmit();
    }
  }
do_pause:
  vfd.stats.got_status = (vfd.cycle - vfd.last_status_update < freshness_limit);
  vfd.stats.got_set_frequency = (vfd.cycle - vfd.last_set_freq_update < freshness_limit);
  vfd.stats.got_act_frequency = vfd.stats.got_set_frequency; // a single update does both
  if (!(vfd.stats.got_status && vfd.stats.got_set_frequency && vfd.stats.got_act_frequency)) {
    return MODBUS_RESULT_NOT_READY;
  }

  if (!vfd.status_running)
    return 0;
  return (vfd.status_reverse ? -1 : 1) * vfd.act_freq_x10 / 10.0;
}

