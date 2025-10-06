#include <stdio.h>
#include <stdlib.h>

#ifdef BUILD_TESTS
  #include "../test/mocks/rp_mocks.h"
#else  // BUILD_TESTS
  #include "pico/stdlib.h"
#endif  // BUILD_TESTS

#include "modbus.h"

uint8_t modbus_last_control;

void huanyang_setfreq(unsigned hertz_x100) {
  modbus_command[0] = vfd_config.address;
  modbus_command[1] = 5;
  modbus_command[2] = 2;
  modbus_command[3] = hertz_x100 >> 8;
  modbus_command[4] = hertz_x100 & 0xFF;
  modbus_pause = 50;
  modbus_length = 5;
}

void huanyang_read_status(uint8_t status) {
  modbus_command[0] = vfd_config.address;
  modbus_command[1] = 4;
  modbus_command[2] = 1;
  modbus_command[3] = status;
  modbus_pause = 50;
  modbus_length = 4;
}

void huanyang_control(uint8_t control) {
  modbus_command[0] = vfd_config.address;
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

void modbus_huanyang_receive(void) {
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
        vfd.last_status_update = vfd.cycle;
        modbus_printf("Status bits %02x\n", modbus_command[3]);
        vfd.status_run = modbus_command[3] & 0x01 ? 1 : 0;
        vfd.status_reverse = modbus_command[3] & 0x04 ? 1 : 0;
        vfd.status_running = modbus_command[3] & 0x08 ? 1 : 0;
        break;
      case 4:
        if (len >= 6) {
          int value = modbus_command[4] * 256 + modbus_command[5];
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
            ++vfd.stats.unknown;
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
        ++vfd.stats.unknown;
        printf("Unrecognized response %02x\n", modbus_command[1]);
      }
    } else {
      printf("Response CRC error\n");
      ++vfd.stats.crc_errors;
    }
  } else if (modbus_outstanding) {
    ++vfd.stats.unanswered;
  }
}

float modbus_loop_huanyang(float frequency) {
  if (!modbus_check_config())
    return MODBUS_RESULT_NOT_CONFIGURED;
  int refresh_delay = 1000; // delay between polls for the same value
  int freshness_limit = 10000; // No updates after this time means that the data are stale
  vfd.req_freq_x100 = abs((int16_t)(frequency * 100));
  if (modbus_check_receive()) {
    modbus_huanyang_receive();
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
      vfd.command_run = frequency != 0;
      vfd.command_reverse = frequency < 0;
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
  }
  vfd.stats.got_status = (vfd.cycle - vfd.last_status_update < freshness_limit);
  vfd.stats.got_set_frequency = (vfd.cycle - vfd.last_set_freq_update < freshness_limit);
  vfd.stats.got_act_frequency = (vfd.cycle - vfd.last_act_freq_update < freshness_limit);
  if (!(vfd.stats.got_status && vfd.stats.got_set_frequency && vfd.stats.got_act_frequency)) {
    return MODBUS_RESULT_NOT_READY;
  }

  if (!vfd.status_running)
    return 0;
  return (vfd.status_reverse ? -1 : 1) * vfd.act_freq_x100 / 100.0;
}

