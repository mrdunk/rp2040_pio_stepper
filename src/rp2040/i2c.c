#include <stdio.h>

#include "i2c.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"

void i2c_engine_init(struct i2c_engine_state *state, i2c_inst_t *i2c, int reset_pin, uint32_t timeout_us) {
  state->i2c = i2c;
  state->data_pos = state->data_len = state->i2c_addr = 0;
  state->timeout_us = timeout_us;
  state->reset_pin = reset_pin;
  i2c_engine_reset(state);
}

void i2c_engine_reset(struct i2c_engine_state *state) {
  state->phase = I2CES_FAIL;
}

void i2c_engine_set_sequence(struct i2c_engine_state *state, uint8_t addr, const uint8_t *src, uint8_t *dst) {
  state->i2c_addr = addr;
  state->data_ptr = state->data_src = src;
  state->dst_ptr = dst;
}

bool i2c_engine_run(struct i2c_engine_state *state) {
  switch(state->phase) {
  case I2CES_IDLE:
    if (!state->data_ptr) {
      return false;
    }
    if (!*state->data_ptr) {
      state->data_ptr = NULL;
      return false;
    }
    state->data_pos = 0;
    state->data_len = state->data_ptr[0] & 0x7F;
    state->is_read = state->data_ptr[0] >> 7;
    state->phase = I2CES_START_WRITE;
    state->data_ptr++;
    return false;
  case I2CES_START_WRITE:
    state->i2c->hw->enable = 0;
    state->i2c->hw->tar = state->i2c_addr;
    state->i2c->hw->enable = 1;
    state->phase = I2CES_WRITE_BYTE;
    state->timeout_check = init_single_timeout_until(&state->timeout_state, make_timeout_time_us(state->timeout_us));
    return true;
  case I2CES_WRITE_BYTE:
    if (state->i2c->hw->raw_intr_stat & I2C_IC_RAW_INTR_STAT_TX_EMPTY_BITS) {
      bool first = state->data_pos == 0;
      bool last = state->data_pos == state->data_len - 1 && !state->is_read;
      state->i2c->hw->data_cmd =
        bool_to_bit(first && state->i2c->restart_on_next) << I2C_IC_DATA_CMD_RESTART_LSB |
        bool_to_bit(last) << I2C_IC_DATA_CMD_STOP_LSB |
        state->data_ptr[state->data_pos++];
      if (state->data_pos == state->data_len) {
        if (state->is_read) {
          state->data_ptr += state->data_len;            
          state->data_len = *state->data_ptr++;
          state->data_pos = 0;
          state->phase = I2CES_READ_BYTE;
        } else {
          state->phase = I2CES_WRITE_END;
        }
      }
      state->timeout_check = init_single_timeout_until(&state->timeout_state, make_timeout_time_us(state->timeout_us));
      return true;
    }
    else if (state->timeout_check(&state->timeout_state)) {
      state->phase = I2CES_FAIL;
      state->abort_reason = 0; // timeout
      return true;
    }
    state->abort_reason = state->i2c->hw->tx_abrt_source;
    if (state->abort_reason) {
      state->phase = I2CES_FAIL;
      state->i2c->hw->clr_stop_det;
      return true;
    }
    return false;
  case I2CES_WRITE_END:
    if (state->i2c->hw->raw_intr_stat & I2C_IC_RAW_INTR_STAT_STOP_DET_BITS) {
      // stop condition - all done
      state->i2c->hw->clr_stop_det;
      state->phase = I2CES_IDLE;
      state->data_ptr += state->data_len;            
      return true;
    }
    else if (state->timeout_check(&state->timeout_state)) {
      // timeout
      state->phase = I2CES_FAIL;
      state->abort_reason = 0; // timeout
      return true;
    }
    return false;
  case I2CES_READ_BYTE:
    if (i2c_get_write_available(state->i2c)) {
      bool first = state->data_pos == 0;
      bool last = state->data_pos == state->data_len - 1;
      state->i2c->hw->data_cmd =
        bool_to_bit(first) << I2C_IC_DATA_CMD_RESTART_LSB |
        bool_to_bit(last) << I2C_IC_DATA_CMD_STOP_LSB |
        I2C_IC_DATA_CMD_CMD_BITS;
      state->phase = I2CES_READ_BYTE2;
      return true;
    }
    else if (state->timeout_check(&state->timeout_state)) {
      state->phase = I2CES_FAIL;
      state->abort_reason = 0; // timeout
      return true;
    }
    state->abort_reason = state->i2c->hw->tx_abrt_source;
    if (state->abort_reason) {
      state->phase = I2CES_FAIL;
      state->i2c->hw->clr_stop_det;
      return true;
    }
    return false;
  case I2CES_READ_BYTE2:
    if (i2c_get_read_available(state->i2c)) {
      state->dst_ptr[state->data_pos++] = (uint8_t)state->i2c->hw->data_cmd;
      if (state->data_pos == state->data_len) {
        state->phase = I2CES_READ_END;
      } else {
        state->phase = I2CES_READ_BYTE;
      }
      state->timeout_check = init_single_timeout_until(&state->timeout_state, make_timeout_time_us(state->timeout_us));
      return true;
    }
    else if (state->timeout_check(&state->timeout_state)) {
      state->phase = I2CES_FAIL;
      state->abort_reason = 0; // timeout
      return true;
    }
    state->abort_reason = state->i2c->hw->tx_abrt_source;
    if (state->abort_reason) {
      state->phase = I2CES_FAIL;
      state->i2c->hw->clr_stop_det;
      return true;
    }
    return false;
  case I2CES_READ_END:
    if (state->i2c->hw->raw_intr_stat & I2C_IC_RAW_INTR_STAT_STOP_DET_BITS) {
      // stop condition - all done
      state->i2c->hw->clr_stop_det;
      state->phase = I2CES_IDLE;
      return true;
    }
    else if (state->timeout_check(&state->timeout_state)) {
      // timeout
      state->phase = I2CES_FAIL;
      state->abort_reason = 0; // timeout
      return true;
    }
    return false;
  case I2CES_FAIL:
    gpio_put(state->reset_pin, 0);
    state->i2c->hw->enable = 3; // abort
    state->i2c->hw->clr_intr;
    state->phase = I2CES_FAIL_WAIT1;
    state->timeout_check = init_single_timeout_until(&state->timeout_state, make_timeout_time_us(20000));
    return false;
  case I2CES_FAIL_WAIT1:
    state->i2c->hw->enable = 1; // clear abort
    state->i2c->hw->clr_intr;
    state->i2c->hw->tx_abrt_source;
    if (state->timeout_check(&state->timeout_state)) {
      gpio_put(state->reset_pin, 1);
      state->timeout_check = init_single_timeout_until(&state->timeout_state, make_timeout_time_us(20000));
      state->phase = I2CES_FAIL_WAIT2;
      return true;
    }
    return false;
  case I2CES_FAIL_WAIT2:
    state->i2c->hw->clr_intr;
    state->i2c->hw->tx_abrt_source;
    if (state->timeout_check(&state->timeout_state)) {
      state->phase = I2CES_FAIL_RECOVER;
    }
    return false;  
  case I2CES_FAIL_RECOVER:
    return false;
  default:
    return false;
  }
}

