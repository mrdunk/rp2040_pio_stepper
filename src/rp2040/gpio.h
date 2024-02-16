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
    const uint8_t* address,  // i2c address if appicable.
    const bool* value        // Last value of GPIO.
);

void get_gpio_config(
    const uint8_t gpio,
    uint8_t* type,
    uint8_t* index,
    uint8_t* address,
    bool* value
);

/* Calls handler for all 32 IO pins in the specified bank.
 * Each bit in values represents the value of a pin.
 * Multiple sets of 32 can be set by increasing bank. */
void gpio_set_values(const uint8_t bank, uint32_t values);

void gpio_local_set_out_pin(uint8_t index, bool new_value);

/* Put i2c data in temporary buffer pending gpio_i2c_mcp_tranceive() being called. */
void gpio_i2c_mcp_set_out_pin(uint8_t index, uint8_t address, bool new_value);

/* Pack GPIO inputs in buffer for UDP transmission. */
void gpio_serialize(struct NWBuffer* tx_buf, size_t* tx_buf_len);

/* Serialise data stored in gpio config in a format for sending over UDP. */
bool serialise_gpio_config(const uint8_t gpio, struct NWBuffer* tx_buf);

#endif  // GPIO__H
