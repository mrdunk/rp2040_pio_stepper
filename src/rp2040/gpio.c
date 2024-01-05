#include <stdio.h>
#include <string.h>

#include "gpio.h"
#include "config.h"
#include "messages.h"

#ifdef BUILD_TESTS

#include "../test/mocks/rp_mocks.h"

#else  // BUILD_TESTS

#include "port_common.h"

#endif  // BUILD_TESTS


uint32_t gpio_i2c_mcp_indexes[MAX_I2C_MCP] = {0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff};
uint8_t gpio_i2c_mcp_addresses[MAX_I2C_MCP] = {0xff, 0xff, 0xff, 0xff};

void update_gpio_config(
    const uint8_t gpio,
    const uint8_t* type,
    const uint8_t* index,
    const uint8_t* address
)
{
  if(gpio >= MAX_GPIO) {
    printf("ERROR: Higher than MAX_GPIO requested. %u\n", gpio);
    return;
  }

  if(type != NULL) {
    config.gpio[gpio].type = *type;
  }
  if(index != NULL) {
    if(*index > 32) {
      printf("WARN: GPIO index out of range. Add: %u  Index: %u\n", *address, *index);
    } else {
      config.gpio[gpio].index = *index;
    }
  }
  if(address != NULL) {
    config.gpio[gpio].address = *address;
  }
}

void get_gpio_config(
    const uint8_t gpio,
    uint8_t* type,
    uint8_t* index,
    uint8_t* address
)
{
  if(gpio >= MAX_GPIO) {
    printf("ERROR: Higher than MAX_GPIO requested. %u\n", gpio);
    return;
  }

  if(type != NULL) {
    *type = config.gpio[gpio].type;
  }
  if(index != NULL) {
    *index = config.gpio[gpio].index;
  }
  if(address != NULL) {
    *address = config.gpio[gpio].address;
  }
}

void gpio_set_values(const uint8_t bank, uint32_t values) {
  for(uint8_t gpio = bank * 32; gpio < (bank + 1) * 32; gpio++) {
    bool value = ((values & 0x1) == 1);
    uint8_t type;
    uint8_t index;
    uint8_t address;
    get_gpio_config(gpio, &type, &index, &address);

    switch(config.gpio[gpio].type) {
      case GPIO_TYPE_NATIVE_OUT:
        gpio_local_set_out_pin(index, value);
        break;
      case GPIO_TYPE_I2C_MCP_OUT:
        gpio_i2c_mcp_set_out_pin(index, address, value);
        break;
      default:
        break;
    }

    values = values >> 1;
  }
}

void gpio_local_set_out_pin(uint8_t index, bool value) {
  // TODO: Filter on valid native GPIO indexes.
  gpio_put(index, value);
}

bool gpio_local_get_pin(uint32_t index) {
  bool value = gpio_get(index);
  if(value) {
    return true;
  }
  return false;
}

/* Happens before iterating through gpio data.
 * Do any i2c required operations here. */
void gpio_i2c_mcp_prepare() {
  for(uint8_t i2c_bucket = 0; i2c_bucket < MAX_I2C_MCP; i2c_bucket++) {
    // uint8_t i2c_address = gpio_i2c_mcp_addresses[i2c_bucket];
    // TODO: Any other MCP implementation here.
  }
}

/* Find all GPIO pins sharing the same i2c address and put 
 */
void gpio_i2c_mcp_set_out_pin(uint8_t index, uint8_t address, bool value) {
  // Find which slot this i2c address is being stored in.
  uint8_t i2c = MAX_I2C_MCP;
  for(i2c = 0; i2c < MAX_I2C_MCP; i2c++) {
    if(gpio_i2c_mcp_addresses[i2c] == address) {
      break;
    }
    if (gpio_i2c_mcp_addresses[i2c] == 0xff) {
      // Haven't found this address yet.
      // Since this buffer slot is empty, use it for this address.
      gpio_i2c_mcp_addresses[i2c] = address;
      break;
    }
  }

  if(i2c >= MAX_I2C_MCP) {
    printf("WARN: Too may i2c addresses. Add: %u  Index: %u\n", address, index);
    return;
  }

  // Add value to buffer to be sent to this i2c address.
  if(value) {
    gpio_i2c_mcp_indexes[i2c] |= (0x1 << index);
  } else {
    gpio_i2c_mcp_indexes[i2c] &= (~(0x1 << index));
  }
}

/* Happens after iterating through GPIO data.
 * Do any i2c required operations here. */
void gpio_i2c_mcp_tranceive() {
  for(uint8_t i2c_bucket = 0; i2c_bucket < MAX_I2C_MCP; i2c_bucket++) {
    // Write gpio_i2c_mcp_indexes[i2c] as output
    // then read inputs into same buffer.
    uint8_t values = gpio_i2c_mcp_indexes[i2c_bucket];
    uint8_t address = gpio_i2c_mcp_addresses[i2c_bucket];
    // TODO: Send values to the i2c connected MCP.
    printf("i2c addr: %u  values: %#04x\n", address, values);

    // TODO: Read i2c data into gpio_i2c_mcp_indexes[MAX_I2C_MCP] for later
    // UDP transmission.
  }
}

bool gpio_i2c_mcp_get_pin(uint8_t index, uint8_t address) {
  for(uint8_t i2c = 0; i2c < MAX_I2C_MCP; i2c++) {
    if(gpio_i2c_mcp_addresses[i2c] == address) {
      return gpio_i2c_mcp_indexes[i2c] | (0x1 << index);
    }
  }
  return 0;
}

/* Pack GPIO inputs in buffer for UDP transmission. */
void gpio_serialize(struct NWBuffer* tx_buf, size_t* tx_buf_len) {
  uint32_t values[MAX_GPIO / 32] = {0};

  for(uint8_t gpio = 0; gpio < MAX_GPIO; gpio++) {
    uint8_t bank = gpio / 32;
    bool value = 0;
    uint8_t type;
    uint8_t index;
    uint8_t address;

    get_gpio_config(gpio, &type, &index, &address);


    switch(config.gpio[gpio].type) {
      case GPIO_TYPE_NATIVE_IN:
        value = gpio_local_get_pin(index);
        break;
      case GPIO_TYPE_I2C_MCP_IN:
        value = gpio_i2c_mcp_get_pin(index, address);
        break;
      default:
        break;
    }
    values[bank] |= (value << (gpio % 32));
  }

  struct Reply_gpio reply;
  reply.type = REPLY_GPIO;

  for(uint8_t bank = 0; bank < MAX_GPIO / 32; bank++) {
      reply.bank = bank;
      reply.values = values[bank];

      uint16_t tx_buf_len = pack_nw_buff(tx_buf, &reply, sizeof(reply));

      if(!tx_buf_len) {
        printf("WARN: TX length greater than buffer size. gpio bank: %u\n", bank);
        return;
      }
  }
}
