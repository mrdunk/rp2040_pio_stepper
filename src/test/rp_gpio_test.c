#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <stdint.h>
#include <cmocka.h>

#include <stdio.h>
#include <string.h>

#include "../shared/messages.h"
//#include "../shared/buffer.h"
//#include "../rp2040/core0.h"
#include "../rp2040/config.h"
//#include "../rp2040/network.h"
//#include "../rp2040/ring_buffer.h"
//#include "mocks/rp_mocks.h"
#include "../rp2040/gpio.h"


extern uint32_t gpio_i2c_mcp_indexes[MAX_I2C_MCP];
extern uint8_t gpio_i2c_mcp_addresses[MAX_I2C_MCP];

void __wrap_gpio_put(int index, int value) {
    check_expected(value);
}

int __wrap_gpio_get(int index) {
    return mock_type(int);
}

static void test_gpio_config(void **state) {
    (void) state; /* unused */

    // Put some values in config.
    uint8_t gpio = 0;
    uint8_t type = GPIO_TYPE_NATIVE_OUT;
    uint8_t index = 0;
    uint8_t address = 1;
    update_gpio_config(gpio, &type, &index, &address);

    gpio = 1;
    type = GPIO_TYPE_NATIVE_IN;
    index = 1;
    address = 42;
    update_gpio_config(gpio, &type, &index, &address);

    // Confirm the correct values are in the config's data structure.
    assert_int_equal(config.gpio[0].type, GPIO_TYPE_NATIVE_OUT);
    assert_int_equal(config.gpio[0].index, 0);
    assert_int_equal(config.gpio[0].address, 1);

    assert_int_equal(config.gpio[1].type, GPIO_TYPE_NATIVE_IN);
    assert_int_equal(config.gpio[1].index, 1);
    assert_int_equal(config.gpio[1].address, 42);

    // Use the proper getter to make sure the retrieved values match.
    get_gpio_config(0, &type, &index, &address);
    assert_int_equal(config.gpio[0].type, type);
    assert_int_equal(config.gpio[0].index, index);
    assert_int_equal(config.gpio[0].address, address);
    
    get_gpio_config(1, &type, &index, &address);
    assert_int_equal(config.gpio[1].type, type);
    assert_int_equal(config.gpio[1].index, index);
    assert_int_equal(config.gpio[1].address, address);
}

/* Output to the RP2030 GPIO. */
static void test_native_gpio_set_output_values(void **state) {
    (void) state; /* unused */

    // Configure the GPIO in the main config.
    for(uint8_t gpio = 0; gpio < MAX_GPIO; gpio++) {
        config.gpio[gpio].type = GPIO_TYPE_NATIVE_OUT;
        config.gpio[gpio].index = gpio % MAX_GPIO;
    }

    // Set a patten of values we expect to be sent to the GPIO.
    uint32_t values = 0xDEADBEEF;

    // Iterate through the bytes that represent the final status of the GPIO.
    for(uint8_t i = 0; i < MAX_GPIO; i++) {
        int bit = i % 32;  // Stay within the bounds of `values`.
        int v = (values >> bit) & 0x1;
        expect_value(__wrap_gpio_put, value, v);
    }

    // Now send `values` to each bank.
    uint8_t banks = MAX_GPIO / 32;
    for(uint8_t bank = 0; bank < banks; bank++) {
        gpio_set_values(bank, values);
    }

}

/* Output to the i2c MPC GPIO. */
static void test_i2c_gpio_set_output_values(void **state) {
    (void) state; /* unused */

    // Muddy the buffer.
    memset(gpio_i2c_mcp_indexes, 0xf0, sizeof(uint32_t) * MAX_I2C_MCP);

    // Configure the GPIO in the main config.
    for(uint8_t gpio = 0; gpio < MAX_GPIO; gpio++) {
        config.gpio[gpio].type = GPIO_TYPE_I2C_MCP_OUT;
        config.gpio[gpio].index = gpio % 31;
        config.gpio[gpio].address = (gpio % MAX_I2C_MCP) * (gpio % MAX_I2C_MCP);
    }

    // Set a patten of values we expect to be sent to the GPIO.
    uint32_t values = 0x5CA1AB1E;

    // Now send `values` to each bank.
    uint8_t banks = MAX_GPIO / 32;
    for(uint8_t bank = 0; bank < banks; bank++) {
        gpio_set_values(bank, values);
    }

    // Check the buffer has the correct values in each i2c_bucket (buffer).
    for(uint8_t gpio = 0; gpio < MAX_GPIO; gpio++) {
        uint8_t i2c_bucket;
        for(i2c_bucket = 0; i2c_bucket < MAX_I2C_MCP; i2c_bucket++) {
            if(gpio_i2c_mcp_addresses[i2c_bucket] == config.gpio[gpio].address) {
                break;
            }
        }
        uint8_t index = config.gpio[gpio].index;
        bool expected_value = (0x1 << (gpio % 32)) & values;
        bool actual_value = (0x1 << index) & gpio_i2c_mcp_indexes[i2c_bucket];

        assert_int_equal(expected_value, actual_value);
    }
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_gpio_config),
        cmocka_unit_test(test_native_gpio_set_output_values),
        cmocka_unit_test(test_i2c_gpio_set_output_values)
    };

    return cmocka_run_group_tests(tests, NULL, NULL);
}

