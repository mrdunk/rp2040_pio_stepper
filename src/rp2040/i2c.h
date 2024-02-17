#include "messages.h"
#include "hardware/i2c.h"
#include "pico/timeout_helper.h"

#pragma once

enum i2c_engine_phase {
  I2CES_IDLE,
  I2CES_START_WRITE,
  I2CES_WRITE_BYTE,
  I2CES_WRITE_END,
  I2CES_READ_BYTE,
  I2CES_READ_BYTE2,
  I2CES_READ_END,
  I2CES_FAIL,
  I2CES_FAIL_WAIT1,
  I2CES_FAIL_WAIT2,
  I2CES_FAIL_RECOVER,
  I2CES_WAIT,
};

struct i2c_engine_state {
  i2c_inst_t *i2c;

  enum i2c_engine_phase phase;
  uint8_t data_pos, data_len:7, is_read:1;
  uint8_t i2c_addr, abort_reason;
  uint8_t reset_pin;

  const uint8_t *data_src, *data_ptr;
  uint8_t *dst_ptr;
  timeout_state_t timeout_state;
  check_timeout_fn timeout_check;
  uint32_t timeout_us;
};

extern void i2c_engine_init(struct i2c_engine_state *state, i2c_inst_t *i2c, int reset_pin, uint32_t timeout_us);
extern void i2c_engine_reset(struct i2c_engine_state *state);
extern bool i2c_engine_run(struct i2c_engine_state *state);
extern void i2c_engine_set_sequence(struct i2c_engine_state *state, uint8_t addr, const uint8_t *src, uint8_t *dst);

static inline bool i2c_engine_is_idle(struct i2c_engine_state *state) {
  return state->phase == I2CES_IDLE && !state->data_ptr;
}

static inline bool i2c_engine_is_fault(struct i2c_engine_state *state) {
  return state->phase == I2CES_FAIL_RECOVER;
}

static inline void i2c_engine_clear_fault(struct i2c_engine_state *state) {
  state->phase = I2CES_IDLE;
}

#define I2CGPIO_TYPE_NONE 0
#define I2CGPIO_TYPE_MCP23017 1

struct i2c_gpio_config {
  uint8_t input_bitmask[2], pullup_bitmask[2];
  uint8_t input_data[2], output_data[2];
  uint8_t i2c_address;
  uint8_t type;
  uint8_t needs_config;
};

struct i2c_gpio_state {
  struct i2c_engine_state engine;
  struct i2c_gpio_config config[MAX_I2C_MCP];
  uint8_t cur_chip;
};

extern struct i2c_gpio_state i2c_gpio;

extern void i2c_gpio_init(struct i2c_gpio_state *gpio);
extern void i2c_gpio_poll(struct i2c_gpio_state *gpio);
extern void i2c_gpio_set_pin_config(struct i2c_gpio_state *gpio, uint8_t device, uint8_t index, int type);
