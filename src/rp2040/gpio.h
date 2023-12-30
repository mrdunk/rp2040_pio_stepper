#ifndef GPIO__H
#define GPIO__H

#include "config.h"

extern volatile struct ConfigGlobal config;

/* Updates the GPIO config from CORE0 only.
 * NOT SAFE FOR USE ON CORE1. */
void update_gpio_config(
    const uint8_t gpio,      // Unique identifier for a GPIO. Between 0 and MAX_GPIO -1.
    const uint8_t* type,     // See the GPIO_TYPE_XXX defines in shared/messages.h.
    const uint8_t* index,    // GPIO pin number.
    const uint8_t* address   // i2c address if appicable.
);

void get_gpio_config(
    const uint8_t gpio,
    uint8_t* type,
    uint8_t* index,
    uint8_t* address
);

/* Calls handler for all 32 IO pins in the specified bank.
 * Each bit in values represents the value of a pin.
 * Multiple sets of 32 can be set by increasing bank. */
void gpio_set_values(const uint8_t bank, uint32_t values);

void gpio_local_set_out_pin(uint8_t index, bool value);

/* Happens before gpio data destined for i2c has been gathered. */
void gpio_i2c_mcp_prepare();

/* Put i2c data in temporary buffer pending gpio_i2c_mcp_tranceive() being called. */
void gpio_i2c_mcp_set_out_pin(uint8_t index, uint8_t address, bool value);

/* Happens after all gpio data destined for i2c has been gathered. */
void gpio_i2c_mcp_tranceive();

/* Pack GPIO inputs in buffer for UDP transmission. */
void gpio_serialize(struct NWBuffer* tx_buf, size_t* tx_buf_len);

#endif  // GPIO__H
