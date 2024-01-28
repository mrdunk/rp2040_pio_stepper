#include "modbus.h"
#include <stdlib.h>
#include "pico/stdlib.h"

#define WEIKEN_CMD_CONTROL 0x2000
#define WEIKEN_CMD_CONTROL_RUN_FWD 1
#define WEIKEN_CMD_CONTROL_RUN_REV 2
#define WEIKEN_CMD_CONTROL_JOG_FWD 3
#define WEIKEN_CMD_CONTROL_JOG_REV 4
#define WEIKEN_CMD_CONTROL_COAST 5
#define WEIKEN_CMD_CONTROL_STOP 6

#define WEIKEN_CMD_STATE 0x7008
#define WEIKEN_CMD_STATE_RUN_FWD 1
#define WEIKEN_CMD_STATE_RUN_REV 2

// Technically, this is the 'communication setting', and the range is -10000 to 10000 (corresponding to -100%..100% of the maximum frequency)
#define WEIKEN_CMD_SPEED 0x1000

#define WEIKEN_CMD_STATUS_SET_FREQ 0x7000
#define WEIKEN_CMD_STATUS_RUN_FREQ 0x7001

static void modbus_weiken_receive(void)
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
      case 3: {
        int bytes = modbus_command[2] * 256 + modbus_command[3];
        switch(bytes) { // tell by byte count, 2 = status, 4 = data
        case 2:
          modbus_printf("Status %x\n", modbus_command[4]);
          vfd.last_status_update = vfd.cycle;
          vfd.status_run = modbus_command[4] == WEIKEN_CMD_STATE_RUN_FWD || modbus_command[4] == WEIKEN_CMD_STATE_RUN_REV;
          vfd.status_reverse = modbus_command[4] == WEIKEN_CMD_STATE_RUN_REV;
          vfd.status_running = vfd.status_run;
          break;
        case 4:
          vfd.last_set_freq_update = vfd.cycle;
          vfd.set_freq_x10 = (4 * (modbus_command[4] * 256U + modbus_command[5]) + 5) / 10; // freq setting -10000...10000
          vfd.act_freq_x10 = modbus_command[6] * 256U + modbus_command[7];
          modbus_printf("Freqs %d %d (%d)\n", (int)vfd.set_freq_x10, (int)vfd.act_freq_x10, (int)(vfd.req_freq_x10));
          break;
        default:
          modbus_printf("Unknown read byte count %d\n", (int)modbus_command[2]);
          ++vfd.stats.unknown;
          break;
        }
        break;
      }
      case 6: { // write 
        int reg = modbus_command[2] * 256 + modbus_command[3];
        int value = modbus_command[4] * 256 + modbus_command[5];
        if (reg == WEIKEN_CMD_CONTROL) {
          modbus_printf("Control value ack=%d\n", value);
          vfd.last_status_update = vfd.cycle;
          vfd.status_run = value == WEIKEN_CMD_STATE_RUN_FWD || value == WEIKEN_CMD_STATE_RUN_REV;
          break;
        } else {
          modbus_printf("Write\n");
        }
        break;
      }
      case 0x83: // read failure
        ++vfd.stats.unknown;
        modbus_printf("Read failure len=%d\n", modbus_command[2]);
        break;
      case 0x86: // write failure
        ++vfd.stats.unknown;
        modbus_printf("Write failure len=%d\n", modbus_command[2]);
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

float modbus_loop_weiken(float frequency)
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
  
  modbus_weiken_receive();
  if (vfd.cycle - vfd.last_status_update > refresh_delay) {
    if (!vfd.command_run) {
      modbus_write_holding_register(vfd_config.address, WEIKEN_CMD_CONTROL, WEIKEN_CMD_CONTROL_STOP);
    } else {
      if (vfd.command_reverse) {
        modbus_write_holding_register(vfd_config.address, WEIKEN_CMD_CONTROL, WEIKEN_CMD_CONTROL_RUN_REV);
      } else {
        modbus_write_holding_register(vfd_config.address, WEIKEN_CMD_CONTROL, WEIKEN_CMD_CONTROL_RUN_FWD);
      }
    }
    modbus_transmit();
  } else if (vfd.cycle - vfd.last_set_freq_update > refresh_delay) {
    modbus_read_holding_registers(vfd_config.address, WEIKEN_CMD_STATUS_SET_FREQ, 2);
    modbus_transmit();
  } else {
    // This assumes top frequency of 400 Hz (2-pole motor with max RPM of 24000).
    // Scales the -4000 to 4000 range to -10000 to 10000 expected by the inverter.
    int rate = (vfd.req_freq_x10 * 5 + 1) / 2;
    if (rate < -10000) rate = -10000;
    if (rate > 10000) rate = 10000;
    // Value received is not always exactly the value sent.
    if (abs((rate * 2 + 2) / 5 - vfd.set_freq_x10) > 1) {
      modbus_write_holding_register(vfd_config.address, WEIKEN_CMD_SPEED, rate);
      modbus_transmit();
    } else if (vfd.command_run != vfd.status_run || vfd.command_reverse != vfd.status_reverse) {
    }
  }
do_pause:
  vfd.stats.got_status = (vfd.cycle - vfd.last_status_update < freshness_limit);
  vfd.stats.got_set_frequency = (vfd.cycle - vfd.last_set_freq_update < freshness_limit);
  vfd.stats.got_act_frequency = vfd.stats.got_set_frequency; // a single update does both
  if (!(vfd.stats.got_status && vfd.stats.got_set_frequency && vfd.stats.got_act_frequency)) {
    return MODBUS_RESULT_NOT_READY;
  }

  //if (!vfd.status_running)
  //  return 0;
  return (vfd.status_reverse ? -1 : 1) * ((uint16_t)vfd.act_freq_x100 / 100.0);
}

