#include <stdio.h>
#include <string.h>

#include "config.h"
#include "gpio.h"
#include "i2c.h"
#include "messages.h"

#ifdef BUILD_TESTS

#include "../test/mocks/rp_mocks.h"

#else  // BUILD_TESTS

#include "port_common.h"

#endif  // BUILD_TESTS


//uint32_t gpio_i2c_mcp_indexes[MAX_I2C_MCP] = {0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff};
uint32_t gpio_i2c_mcp_indexes[MAX_I2C_MCP] = {0, 0, 0, 0};
uint8_t gpio_i2c_mcp_addresses[MAX_I2C_MCP] = {0xff, 0xff, 0xff, 0xff};

void update_gpio_config(
    const uint8_t gpio,
    const uint8_t* type,
    const uint8_t* index,
    const uint8_t* address,
    const bool* value
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
  if(value != NULL) {
    config.gpio[gpio].value = *value;
  }
}

void get_gpio_config(
    const uint8_t gpio,
    uint8_t* type,
    uint8_t* index,
    uint8_t* address,
    bool* value
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
  if(value != NULL) {
    *value = config.gpio[gpio].value;
  }
}

void gpio_set_values(const uint8_t bank, uint32_t values) {
  for(uint8_t gpio = bank * 32; gpio < (bank + 1) * 32; gpio++) {
    uint8_t type;
    uint8_t index;
    uint8_t address;
    bool current_value;
    get_gpio_config(gpio, &type, &index, &address, &current_value);
    //printf("%u\t%#10x\t%u\t%u\n", bank, values, &type, &index);

    // Parsed from Message_gpio.
    bool new_value = values & (0x1 << (gpio % 32));

    if(new_value == current_value) {
      continue;
    }
    config.gpio_confirmation_pending[bank] = true;

    switch(config.gpio[gpio].type) {
      case GPIO_TYPE_NATIVE_IN_DEBUG:
        printf("DBG GPIO: %u  IO: %u  val: %u\n", gpio, index, new_value);
        // Note: no break.
      case GPIO_TYPE_NATIVE_IN:
        gpio_local_set_out_pin(index, new_value);
        break;
      case GPIO_TYPE_I2C_MCP_IN:
        gpio_i2c_mcp_set_out_pin(index, address, new_value);
        break;
      default:
        break;
    }

    update_gpio_config(gpio, NULL, NULL, NULL, &new_value);
  }
}

void gpio_local_set_out_pin(uint8_t index, bool new_value) {
  // TODO: Filter on valid native GPIO indexes.
  printf("gpio_put(%u, %u)\n", index, new_value);
  gpio_put(index, new_value);
}

bool gpio_local_get_pin(uint32_t index) {
  return gpio_get(index);
}

int gpio_i2c_mcp_alloc(uint8_t address) {
  int i2c = 0, i2c_free = -1;
  for(i2c = 0; i2c < MAX_I2C_MCP; i2c++) {
    if(gpio_i2c_mcp_addresses[i2c] == address) {
      return i2c;
    }
    if(gpio_i2c_mcp_addresses[i2c] == 0xFF && i2c_free == -1) {
      i2c_free = i2c;
    }
  }
  if (i2c_free != -1) {
    gpio_i2c_mcp_addresses[i2c_free] = address;
    i2c_gpio.config[i2c_free].i2c_address = address;
  }
  return i2c_free;
}

/* Find all GPIO pins sharing the same i2c address and put 
 */
void gpio_i2c_mcp_set_out_pin(uint8_t index, uint8_t address, bool new_value) {
  // Find which slot this i2c address is being stored in.
  int i2c = gpio_i2c_mcp_alloc(address);

  if(i2c == -1) {
    printf("WARN: Too may i2c addresses. Add: %u  Index: %u\n", address, index);
    return;
  }

  // Add new_value to buffer to be sent to this i2c address.
  uint8_t *data = &i2c_gpio.config[i2c].output_data[(index >> 3) & 1];
  uint8_t bitmask = 0x1 << (index & 7);
  if(new_value) {
    *data |= bitmask;
  } else {
    *data &= ~bitmask;
  }
}

bool gpio_i2c_mcp_get_pin(uint8_t index, uint8_t address) {
  int i2c = gpio_i2c_mcp_alloc(address);
  if(i2c == -1) {
    printf("WARN: Too may i2c addresses. Add: %u  Index: %u\n", address, index);
    return false;
  }
  uint8_t data = i2c_gpio.config[i2c].input_data[(index >> 3) & 1];
  return (data >> (index & 7)) & 0x1;
}

/* Pack GPIO inputs in buffer for UDP transmission. */
void gpio_serialize(struct NWBuffer* tx_buf, size_t* tx_buf_len) {
  uint32_t values[MAX_GPIO / 32] = {0};
  bool to_send[MAX_GPIO / 32];

  for(uint8_t bank = 0; bank < MAX_GPIO / 32; bank++) {
    values[bank] = 0;
    to_send[bank] = false;
  }

  for(uint8_t gpio = 0; gpio < MAX_GPIO; gpio++) {
    uint8_t bank = gpio / 32;

    // For inputs will be set to value of pin.
    // For outputs will use value in config that pin was last set to.
    uint8_t type;
    uint8_t index;
    uint8_t address;
    bool previous_value;
    bool new_value;

    get_gpio_config(gpio, &type, &index, &address, &previous_value);

    switch(config.gpio[gpio].type) {
      case GPIO_TYPE_NATIVE_OUT_DEBUG:
      case GPIO_TYPE_NATIVE_OUT:
        new_value = gpio_local_get_pin(index);
        if(previous_value != new_value) {
          to_send[bank] = true;
          config.gpio_confirmation_pending[bank] = true;
          // Do not update config here.
          // Config gets updated on incoming Message_gpio.
        }
        break;
      case GPIO_TYPE_I2C_MCP_OUT:
        new_value = gpio_i2c_mcp_get_pin(index, address);
        if(previous_value != new_value) {
          to_send[bank] = true;
          config.gpio_confirmation_pending[bank] = true;
          // Do not update config here.
          // Config gets updated on incoming Message_gpio.
        }
        break;
      default:
        // GPIO_TYPE_*_IN.
        new_value = previous_value;
        break;
    }

    values[bank] |= (new_value << (gpio % 32));
  }

  struct Reply_gpio reply;
  reply.type = REPLY_GPIO;

  for(uint8_t bank = 0; bank < MAX_GPIO / 32; bank++) {
      if(! config.gpio_confirmation_pending[bank]) {
        continue;
      }

      reply.bank = bank;
      reply.values = values[bank];
      reply.confirmation_pending = to_send[bank];

      *tx_buf_len = pack_nw_buff(tx_buf, &reply, sizeof(reply));

      if(! *tx_buf_len) {
        printf("WARN: TX length greater than buffer size. gpio bank: %u\n", bank);
        return;
      }
  }
}

/* Serialise data stored in gpio config in a format for sending over UDP. */
bool serialise_gpio_config(const uint8_t gpio, struct NWBuffer* tx_buf) {
  if(gpio >= MAX_GPIO) {
    printf("ERROR: Invalid gpio: %u\n", gpio);
    return false;
  }

  struct Reply_gpio_config reply;
  reply.type = REPLY_GPIO_CONFIG;
  reply.gpio_type = config.gpio[gpio].type;
  reply.gpio_count = gpio;
  reply.index = config.gpio[gpio].index;
  reply.address = config.gpio[gpio].address;

  size_t tx_buf_len = pack_nw_buff(tx_buf, &reply, sizeof(reply));

  if(!tx_buf_len) {
    printf("WARN: TX length greater than buffer size.\n");
    return false;
  }

  return true;
}

