#include "i2c.h"
#include "hardware/gpio.h"

#define PLACEHOLDER 0x00
#define I2C_WRITE_CMD(write_len, write_data...) (write_len), write_data
#define I2C_WRITE_AND_READ_CMD(read_len, write_len, write_data...) (0x80 | (write_len)), write_data, (read_len)
#define I2C_END_CMD 0

uint8_t mcp23017_setup_sequence[] = {
  I2C_WRITE_CMD(3, 0x00, PLACEHOLDER, PLACEHOLDER), // reg 0x0/0x1 = pin direction (1 = input)
  I2C_WRITE_CMD(3, 0x0C, PLACEHOLDER, PLACEHOLDER), // reg 0xc/0xd = pin pullup (1 = active)
  I2C_END_CMD
};

uint8_t mcp23017_run_sequence[] = {
  I2C_WRITE_CMD(3, 0x12, PLACEHOLDER, PLACEHOLDER), // reg 0x12 = GPIO data (output)
  I2C_WRITE_AND_READ_CMD(2, 1, 0x12),               // reg 0x12 = GPIO data (input)
  I2C_END_CMD
};

void mcp23017_init(struct i2c_engine_state *state, i2c_inst_t *i2c, int sda_pin, int scl_pin, int reset_pin) {
  i2c_init(i2c, 400 * 1000);
  gpio_set_function(sda_pin, GPIO_FUNC_I2C);
  gpio_set_function(scl_pin, GPIO_FUNC_I2C);
  gpio_pull_up(sda_pin);
  gpio_pull_up(scl_pin);

  i2c_engine_init(state, i2c, reset_pin, 5000);

  gpio_init(reset_pin);
  gpio_set_dir(reset_pin, GPIO_OUT);
  gpio_put(reset_pin, 0);
  sleep_ms(20);
  gpio_put(reset_pin, 1);
  sleep_ms(20);
}

void mcp23017_reset(struct i2c_engine_state *state, int addr, uint8_t inputs_port0, uint8_t inputs_port1, uint8_t pullups_port0, uint8_t pullups_port1) {
  i2c_engine_set_sequence(state, addr, mcp23017_setup_sequence, NULL);
}

#define I2C_ADDR 32

#define I2C_RESET_PIN 22
#define I2C_SDA_PIN 26
#define I2C_SCL_PIN 27

void i2c_gpio_init(struct i2c_gpio_state *gpio) {
  for (int i = 0; i < MAX_I2C_MCP; ++i) {
    struct i2c_gpio_config *cfg = &gpio->config[i];
    cfg->type = I2CGPIO_TYPE_NONE;
    cfg->i2c_address = 0xFF;
    cfg->input_data[0] = 0;
    cfg->input_data[1] = 0;
    cfg->output_data[0] = 0;
    cfg->output_data[1] = 0;
    cfg->input_bitmask[0] = 0x7F;
    cfg->input_bitmask[1] = 0x7F;
    cfg->pullup_bitmask[0] = 0x7F;
    cfg->pullup_bitmask[1] = 0x7F;
  }
  gpio->cur_chip = MAX_I2C_MCP - 1;
  gpio->config[0].type = I2CGPIO_TYPE_MCP23017;
  gpio->config[0].i2c_address = 32;
  
  mcp23017_init(&gpio->engine, i2c1, I2C_SDA_PIN, I2C_SCL_PIN, I2C_RESET_PIN);
}

static void i2c_gpio_next_chip(struct i2c_gpio_state *gpio) {
  gpio->cur_chip = (gpio->cur_chip + 1) % MAX_I2C_MCP;
  struct i2c_gpio_config *cfg = &gpio->config[gpio->cur_chip];
  switch(cfg->type) {
  case I2CGPIO_TYPE_MCP23017:
    mcp23017_run_sequence[2] = cfg->output_data[0];
    mcp23017_run_sequence[3] = cfg->output_data[1];
    mcp23017_setup_sequence[2] = cfg->input_bitmask[0];
    mcp23017_setup_sequence[3] = cfg->input_bitmask[1];
    mcp23017_setup_sequence[6] = cfg->pullup_bitmask[0];
    mcp23017_setup_sequence[7] = cfg->pullup_bitmask[1];
    i2c_engine_set_sequence(&gpio->engine, cfg->i2c_address, mcp23017_run_sequence, &cfg->input_data[0]);
    break;
  default:
    break;
  }
}

void i2c_gpio_poll(struct i2c_gpio_state *gpio) {
  struct i2c_gpio_config *cfg = &gpio->config[gpio->cur_chip];
  if (i2c_engine_run(&gpio->engine))
    return;
  if (i2c_engine_is_idle(&gpio->engine)) {
    i2c_gpio_next_chip(gpio);
    return;
  }
  if (i2c_engine_is_fault(&gpio->engine)) {
    // reinitialize - at this point the device is reset
    i2c_engine_set_sequence(&gpio->engine, cfg->i2c_address, mcp23017_setup_sequence, NULL);
    i2c_engine_clear_fault(&gpio->engine);
    i2c_engine_run(&gpio->engine);
  }
}
