#pragma once

#define MODBUS_UART uart1
#define MODBUS_TX_PIN 8
#define MODBUS_RX_PIN 9
#define MODBUS_DIR_PIN 10

// #define modbus_printf printf
#define modbus_printf(...)

extern uint16_t crc16_table[256];

extern uint8_t modbus_command[128];
extern uint8_t modbus_length;
extern uint16_t modbus_cur_bitrate;
extern int modbus_pause;
extern int modbus_cycle;
extern int modbus_last_control;

#define MODBUS_TYPE_HUANYANG 1
#define MODBUS_TYPE_FULING 2

struct vfd_config {
  uint8_t address;
  uint8_t type;
  uint16_t bitrate;
};

struct vfd_status {
  uint32_t cycle;
  uint32_t last_status_update;
  uint32_t last_set_freq_update;
  uint32_t last_act_freq_update;
  union {
    struct { // Huanyang
      int16_t set_freq_x100;
      int16_t act_freq_x100;
      int16_t req_freq_x100;
    };
    struct { // Fuling
      int16_t set_freq_x10;
      int16_t act_freq_x10;
      int16_t req_freq_x10;
    };
  };
  uint16_t amps_x10;
  uint16_t rpm;
  uint16_t status_run:1;
  uint16_t status_reverse:1;
  uint16_t status_running:1;
  uint16_t spindle_at_speed:1;
  uint16_t command_run:1;
  uint16_t command_reverse:1;
};

extern struct vfd_config vfd_config;
extern struct vfd_status vfd;

uint16_t modbus_crc16(const uint8_t *data, uint8_t size);
void modbus_transmit(void);

extern void modbus_init(void);
extern float modbus_loop(float);

float modbus_loop_huanyang(float frequency);
