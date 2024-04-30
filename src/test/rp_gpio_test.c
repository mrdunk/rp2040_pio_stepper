#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <stdint.h>
#include <cmocka.h>

#include <stdio.h>
#include <string.h>

#include "../shared/messages.h"
#include "../rp2040/config.h"
#include "../rp2040/gpio.h"
#include "../rp2040/core0.h"


extern uint32_t gpio_i2c_mcp_indexes[MAX_I2C_MCP];
extern uint8_t gpio_i2c_mcp_addresses[MAX_I2C_MCP];

void __wrap_gpio_put(int index, int value) {
    // printf("__wrap_gpio_put(%i, %i)\n", index, value);
    check_expected(index);
    check_expected(value);
}

int __wrap_gpio_get(int index) {
    printf("__wrap_gpio_get(%i)\n", index);
    check_expected(index);
    return mock_type(int);
}

static void test_gpio_config(void **state) {
    (void) state; /* unused */

    // Put some values in config.
    uint8_t gpio = 0;
    uint8_t type = GPIO_TYPE_NATIVE_OUT;
    uint8_t index = 0;
    uint8_t address = 1;
    update_gpio_config(gpio, &type, &index, &address, NULL);

    gpio = 1;
    type = GPIO_TYPE_NATIVE_IN;
    index = 1;
    address = 42;
    update_gpio_config(gpio, &type, &index, &address, NULL);

    // Confirm the correct values are in the config's data structure.
    assert_int_equal(config.gpio[0].type, GPIO_TYPE_NATIVE_OUT);
    assert_int_equal(config.gpio[0].index, 0);
    assert_int_equal(config.gpio[0].address, 1);

    assert_int_equal(config.gpio[1].type, GPIO_TYPE_NATIVE_IN);
    assert_int_equal(config.gpio[1].index, 1);
    assert_int_equal(config.gpio[1].address, 42);

    // Use the proper getter to make sure the retrieved values match.
    get_gpio_config(0, &type, &index, &address, NULL);
    assert_int_equal(config.gpio[0].type, type);
    assert_int_equal(config.gpio[0].index, index);
    assert_int_equal(config.gpio[0].address, address);
    
    get_gpio_config(1, &type, &index, &address, NULL);
    assert_int_equal(config.gpio[1].type, type);
    assert_int_equal(config.gpio[1].index, index);
    assert_int_equal(config.gpio[1].address, address);
}

/* Output to the RP2030 GPIO. */
static void test_native_gpio_set_output_values(void **state) {
    (void) state; /* unused */

    // Set a patten of values we expect to be sent to the GPIO.
    uint32_t values = 0xDEADBEEF;
    uint32_t values_inverted = ~values;
    uint32_t values_current = (0xFFFF0000 & values) + (values_inverted & 0x0000FFFF);

    // Configure the GPIO in the main config.
    for(uint8_t gpio = 0; gpio < MAX_GPIO; gpio++) {
        config.gpio[gpio].type = GPIO_TYPE_NATIVE_OUT;
        config.gpio[gpio].index = gpio % MAX_GPIO;

        if(gpio < 32) {
            // bank 0;
            // Configured values differ.
            config.gpio[gpio].value = values_current & (0x1 << gpio);
        } else {
            // bank 1;
            // Configured values are same as those sent.
            config.gpio[gpio].value = values & (0x1 << gpio);
        }
    }

    config.gpio_confirmation_pending[0] = false;
    config.gpio_confirmation_pending[1] = false;

    // Set the expected function calls when we eventually call `gpio_set_values(...)`.
    // Iterate through the bytes that represent the final status of the GPIO.
    for(uint8_t gpio = 0; gpio < MAX_GPIO; gpio++) {
        int bit = gpio % 32;  // Stay within the bounds of `values`.
        int v = (values >> bit) & 0x1;

        if(v != config.gpio[gpio].value) {
            expect_value(__wrap_gpio_put, index, gpio);
            expect_value(__wrap_gpio_put, value, v);
        }
    }

    // Now send `values` to each bank.
    uint8_t banks = MAX_GPIO / 32;
    for(uint8_t bank = 0; bank < banks; bank++) {
        gpio_set_values(bank, values);
    }

    for(uint8_t gpio = 0; gpio < MAX_GPIO; gpio++) {
        int bit = gpio % 32;  // Stay within the bounds of `values`.
        int v = (values >> bit) & 0x1;
        assert_int_equal(config.gpio[gpio].value, v);
    }

    // Some current values in bank 0 differed from those sent.
    assert_int_equal(config.gpio_confirmation_pending[0], true);
    // All current values in bank 1 matched from those sent.
    assert_int_equal(config.gpio_confirmation_pending[1], false);
}

/* Output to the i2c MPC GPIO. */
static void test_i2c_gpio_set_output_values(void **state) {
    (void) state; /* unused */

    // Set a patten of values we expect to be sent to the GPIO.
    uint32_t values = 0x5CA1AB1E;
    uint32_t values_inverted = ~values;
    uint32_t values_current = (0xFFFF0000 & values) + (values_inverted & 0x0000FFFF);

    // Configure the GPIO in the main config.
    for(uint8_t gpio = 0; gpio < MAX_GPIO; gpio++) {
        config.gpio[gpio].type = GPIO_TYPE_I2C_MCP_OUT;
        config.gpio[gpio].index = gpio % 31;
        config.gpio[gpio].address = (gpio % MAX_I2C_MCP) * (gpio % MAX_I2C_MCP);

        if(gpio < 32) {
            // bank 0;
            // Configured values are same as those sent.
            config.gpio[gpio].value = values & (0x1 << gpio);
        } else {
            // bank 0;
            // Configured values differ.
            config.gpio[gpio].value = values_current & (0x1 << gpio);
        }

        // The current value of the i2c value buffer.
        gpio_i2c_mcp_set_out_pin(
                config.gpio[gpio].index,
                config.gpio[gpio].address,
                config.gpio[gpio].value
                );
    }

    config.gpio_confirmation_pending[0] = false;
    config.gpio_confirmation_pending[1] = false;

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

        assert_int_equal(expected_value, config.gpio[gpio].value);
        assert_int_equal(expected_value, actual_value);
    }

    // All current values in bank 0 matched from those sent.
    assert_int_equal(config.gpio_confirmation_pending[0], false);
    // Some current values in bank 1 differed from those sent.
    assert_int_equal(config.gpio_confirmation_pending[1], true);
}

/* Parsing Message_gpio on RP. */
static void test_send_PC_to_RP(void **state) {
    (void) state; /* unused */

    struct NWBuffer rx_buf = {0};
    struct NWBuffer tx_buf = {0};
    reset_nw_buf(&rx_buf);
    reset_nw_buf(&tx_buf);
    uint8_t received_msg_count = 0;

    // Configure the GPIO in the main config.
    for(uint8_t gpio = 0; gpio < MAX_GPIO; gpio++) {
        config.gpio[gpio].type = GPIO_TYPE_NOT_SET;
        config.gpio[gpio].index = 0;
        config.gpio[gpio].address = 0;
        config.gpio[gpio].value = false;
    }

    for(uint8_t bank = 0; bank < MAX_GPIO / 32; bank++) {
        config.gpio_confirmation_pending[bank] = false;
    }

    config.gpio[0].type = GPIO_TYPE_NATIVE_IN;
    config.gpio[0].index = 11;
    config.gpio[0].value = false;

    config.gpio[1].type = GPIO_TYPE_NATIVE_IN;
    config.gpio[1].index = 12;
    config.gpio[1].value = false;

    config.gpio[2].type = GPIO_TYPE_NATIVE_OUT;
    config.gpio[2].index = 13;
    config.gpio[2].value = false;

    config.gpio[3].type = GPIO_TYPE_NATIVE_OUT;
    config.gpio[3].index = 14;
    config.gpio[3].value = false;


    // Populate incoming buffer.
    struct Message_gpio message = {
        .type = MSG_SET_GPIO,
        .bank = 0,
        .values = 0b000000000000000000000000000001010,
        .confirmation_pending = false
    };

    memcpy(rx_buf.payload, &message, sizeof(message));
    rx_buf.length = sizeof(message);
    rx_buf.checksum = checksum(0, 0, rx_buf.length, rx_buf.payload);

    // Only config.gpio[3] is an output pin whose value differs from
    // what is being requested.
    expect_value(__wrap_gpio_put, index, 14);      // RP's GPIO pin.
    expect_value(__wrap_gpio_put, value, true);    // GPIO value.

    // Parse Message_gpio.
    process_received_buffer(
            &rx_buf,
            &tx_buf,
            &received_msg_count,
            sizeof(message) + sizeof(rx_buf.length) + sizeof(rx_buf.checksum));

    // Received a single message.
    assert_int_equal(received_msg_count, 1);

    // The GPIO output pins have been set in the config.
    assert_int_equal(config.gpio[2].value, false);
    assert_int_equal(config.gpio[3].value, true);

    // Should have reset the rx_buf.
    assert_int_equal(rx_buf.length, 0);
    assert_int_equal(rx_buf.checksum, 0);

    // One input and one output in incoming packet did not match the current config.
    // Either one should cause a Reply_gpio to be sent.
    assert_int_equal(config.gpio_confirmation_pending[0], true);

    // ********** Sending repeat Message_gpio should do nothing. **********

    reset_nw_buf(&rx_buf);
    received_msg_count = 0;

    memcpy(rx_buf.payload, &message, sizeof(message));
    rx_buf.length = sizeof(message);
    rx_buf.checksum = checksum(0, 0, rx_buf.length, rx_buf.payload);

    // All output GPIO values now match what's being requested so gpio_out(...)
    // will not be called.
    //expect_value(__wrap_gpio_put, index, 14);      // RP's GPIO pin.
    //expect_value(__wrap_gpio_put, value, true);   // GPIO value.

    // Process incoming data.
    process_received_buffer(
            &rx_buf,
            &tx_buf,
            &received_msg_count,
            sizeof(message) + sizeof(rx_buf.length) + sizeof(rx_buf.checksum));

    // Received a single message.
    assert_int_equal(received_msg_count, 1);

    // HAL was last updated with the following values.
    //assert_int_equal(message.values_confirmed, config.gpio_values_confirmed[message.bank]);

    // The GPIO output pins have not changed.
    assert_int_equal(config.gpio[2].value, false);
    assert_int_equal(config.gpio[3].value, true);

    // Should have reset the rx_buf.
    assert_int_equal(rx_buf.length, 0);
    assert_int_equal(rx_buf.checksum, 0);

    // The packet was the same as last time but it now matches the set values.
    // No Reply_gpio should be sent.
    assert_int_equal(config.gpio_confirmation_pending[0], false);
}

/* Parsing Message_gpio on RP. .confirmation_pending flag set on packet causes Reply_gpio. */
static void test_send_PC_to_RP_confirmation_set(void **state) {
    (void) state; /* unused */

    struct NWBuffer rx_buf = {0};
    struct NWBuffer tx_buf = {0};
    reset_nw_buf(&rx_buf);
    reset_nw_buf(&tx_buf);
    uint8_t received_msg_count = 0;

    // Configure the GPIO in the main config.
    for(uint8_t gpio = 0; gpio < MAX_GPIO; gpio++) {
        config.gpio[gpio].type = GPIO_TYPE_NOT_SET;
        config.gpio[gpio].index = 0;
        config.gpio[gpio].address = 0;
        config.gpio[gpio].value = true;
    }

    for(uint8_t bank = 0; bank < MAX_GPIO / 32; bank++) {
        config.gpio_confirmation_pending[bank] = false;
    }

    config.gpio[0].type = GPIO_TYPE_NATIVE_IN;
    config.gpio[0].index = 11;
    config.gpio[0].value = false;

    config.gpio[1].type = GPIO_TYPE_NATIVE_IN;
    config.gpio[1].index = 12;
    config.gpio[1].value = true;

    config.gpio[2].type = GPIO_TYPE_NATIVE_OUT;
    config.gpio[2].index = 13;
    config.gpio[2].value = false;

    config.gpio[3].type = GPIO_TYPE_NATIVE_OUT;
    config.gpio[3].index = 14;
    config.gpio[3].value = true;


    // Populate incoming buffer.
    struct Message_gpio message = {
        .type = MSG_SET_GPIO,
        .bank = 0,
        .values = 0b000000000000000000000000000001010,
        .confirmation_pending = false
    };

    memcpy(rx_buf.payload, &message, sizeof(message));
    rx_buf.length = sizeof(message);
    rx_buf.checksum = checksum(0, 0, rx_buf.length, rx_buf.payload);

    // Parse Message_gpio.
    process_received_buffer(
            &rx_buf,
            &tx_buf,
            &received_msg_count,
            sizeof(message) + sizeof(rx_buf.length) + sizeof(rx_buf.checksum));

    // Received a single message.
    assert_int_equal(received_msg_count, 1);

    // Should have reset the rx_buf.
    assert_int_equal(rx_buf.length, 0);
    assert_int_equal(rx_buf.checksum, 0);

    // Although the received data matches the GPIO state,
    // the confirmation_pending flag was set on the incoming packet so 
    // Reply_gpio should be sent.
    assert_int_equal(config.gpio_confirmation_pending[0], true);
}

/* Either incoming Message_gpio has confirmation_pending set or input pins have changed.
 * Send Reply_gpio. */
static void test_send_RP_to_PC(void **state) {
    (void) state; /* unused */

    struct NWBuffer tx_buf = {0};
    reset_nw_buf(&tx_buf);
    size_t tx_buf_len = 0;

    // Configure the GPIO in the main config.
    for(uint8_t gpio = 0; gpio < MAX_GPIO; gpio++) {
        config.gpio[gpio].type = GPIO_TYPE_NOT_SET;
        config.gpio[gpio].index = 0;
        config.gpio[gpio].address = 0;
        config.gpio[gpio].value = false;
    }

    config.gpio[0].type = GPIO_TYPE_NATIVE_IN;
    config.gpio[0].index = 11;
    config.gpio[0].value = true;

    config.gpio[1].type = GPIO_TYPE_NATIVE_IN;
    config.gpio[1].index = 12;
    config.gpio[1].value = false;

    config.gpio[2].type = GPIO_TYPE_NATIVE_OUT;
    config.gpio[2].index = 13;
    config.gpio[2].value = true;

    config.gpio[3].type = GPIO_TYPE_NATIVE_OUT;
    config.gpio[3].index = 14;
    config.gpio[3].value = false;

    config.gpio_confirmation_pending[0] = true;


    // Do Reply_gpio from RP to PC.

    reset_nw_buf(&tx_buf);

    // gpio_get(...) will be called for all pins configured as inputs.
    expect_value(__wrap_gpio_get, index, 11);      // RP's GPIO pin.
    expect_value(__wrap_gpio_get, index, 12);      // RP's GPIO pin.
    // Specify the gpio_get(...) return values matching the config.
    will_return(__wrap_gpio_get, true);
    will_return(__wrap_gpio_get, false);

    // Make the call under test.
    // Since the GPIO state does match that sent in the Message_gpio but
    // Message_gpio.confirmation_pending was set to send Reply_gpio anyway.
    gpio_serialize(&tx_buf, &tx_buf_len);

    // Out pins values correctly set.
    assert_int_equal(config.gpio[0].value, true);
    assert_int_equal(config.gpio[1].value, false);

    // Proves data was added to tx_buf_len.
    assert_int_equal(tx_buf_len, sizeof(struct Reply_gpio));
    assert_int_equal(tx_buf.length, sizeof(struct Reply_gpio));
    assert_int_not_equal(tx_buf.checksum, 0);

    // Config agrees that Reply_gpio should be sent.
    assert_int_equal(config.gpio_confirmation_pending[0], true);

    struct Reply_gpio* reply_p = (void*)tx_buf.payload;
    assert_int_equal(reply_p->type, REPLY_GPIO);
    assert_int_equal(reply_p->bank, 0);
    // Since the Output pins match the last values we received in Message_gpio,
    // no need to request an update.
    assert_int_equal(reply_p->confirmation_pending, false);
    assert_int_equal(reply_p->values, 0b00000000000000000000000000000101);
}

static void test_send_RP_to_PC_out_gpio_changed(void **state) {
    (void) state; /* unused */

    struct NWBuffer tx_buf = {0};
    reset_nw_buf(&tx_buf);
    size_t tx_buf_len = 0;

    // Configure the GPIO in the main config.
    for(uint8_t gpio = 0; gpio < MAX_GPIO; gpio++) {
        config.gpio[gpio].type = GPIO_TYPE_NOT_SET;
        config.gpio[gpio].index = 0;
        config.gpio[gpio].address = 0;
        config.gpio[gpio].value = false;
    }

    config.gpio[0].type = GPIO_TYPE_NATIVE_IN;
    config.gpio[0].index = 11;
    config.gpio[0].value = true;

    config.gpio[1].type = GPIO_TYPE_NATIVE_IN;
    config.gpio[1].index = 12;
    config.gpio[1].value = false;

    config.gpio[2].type = GPIO_TYPE_NATIVE_OUT;
    config.gpio[2].index = 13;
    config.gpio[2].value = true;

    config.gpio[3].type = GPIO_TYPE_NATIVE_OUT;
    config.gpio[3].index = 14;
    config.gpio[3].value = false;

    config.gpio_confirmation_pending[0] = false;


    // Do Reply_gpio from RP to PC.

    reset_nw_buf(&tx_buf);

    // gpio_get(...) will be called for all pins configured as inputs.
    expect_value(__wrap_gpio_get, index, 11);      // RP's GPIO pin.
    expect_value(__wrap_gpio_get, index, 12);      // RP's GPIO pin.
    // Specify the gpio_get(...) return values /not/ matching the config.
    will_return(__wrap_gpio_get, false);
    will_return(__wrap_gpio_get, false);

    // Make the call under test.
    // Since the GPIO state does not match that sent in the Message_gpio,
    // send Reply_gpio with .confirmation_pending flag set.
    gpio_serialize(&tx_buf, &tx_buf_len);

    // Out pins values should not have been updated yet, even though GPIO pins have changed.
    // It's up to the incoming Message_gpio to set the config values.
    assert_int_equal(config.gpio[0].value, true);
    assert_int_equal(config.gpio[1].value, false);

    // Proves data was added to tx_buf_len.
    assert_int_equal(tx_buf_len, sizeof(struct Reply_gpio));
    assert_int_equal(tx_buf.length, sizeof(struct Reply_gpio));
    assert_int_not_equal(tx_buf.checksum, 0);

    // Config agrees that Reply_gpio should be sent.
    assert_int_equal(config.gpio_confirmation_pending[0], true);

    struct Reply_gpio* reply_p = (void*)tx_buf.payload;
    assert_int_equal(reply_p->type, REPLY_GPIO);
    assert_int_equal(reply_p->bank, 0);
    // Since the Output pins do not match the last values we received in Message_gpio,
    // request an update.
    assert_int_equal(reply_p->confirmation_pending, true);
    assert_int_equal(reply_p->values, 0b00000000000000000000000000000100);
}

/* If GPIO state does not match that in config.gpio_values_confirmed, don't send Reply_gpio. */
static void test_send_RP_to_PC_matching(void **state) {
    (void) state; /* unused */

    struct NWBuffer tx_buf = {0};
    reset_nw_buf(&tx_buf);
    size_t tx_buf_len = 0;

    // Configure the GPIO in the main config.
    for(uint8_t gpio = 0; gpio < MAX_GPIO; gpio++) {
        config.gpio[gpio].type = GPIO_TYPE_NOT_SET;
        config.gpio[gpio].index = 0;
        config.gpio[gpio].address = 0;
        config.gpio[gpio].value = false;
    }

    config.gpio[0].type = GPIO_TYPE_NATIVE_IN;
    config.gpio[0].index = 11;
    config.gpio[0].value = true;

    config.gpio[1].type = GPIO_TYPE_NATIVE_IN;
    config.gpio[1].index = 12;
    config.gpio[1].value = false;

    config.gpio[2].type = GPIO_TYPE_NATIVE_OUT;
    config.gpio[2].index = 13;
    config.gpio[2].value = true;

    config.gpio[3].type = GPIO_TYPE_NATIVE_OUT;
    config.gpio[3].index = 14;
    config.gpio[3].value = false;

    config.gpio_confirmation_pending[0] = false;

    // ********** This time not Reply_gpio from RP to PC will be sent. **********

    reset_nw_buf(&tx_buf);

    // gpio_get(...) will be called for all pins configured as inputs.
    expect_value(__wrap_gpio_get, index, 11);      // RP's GPIO pin.
    expect_value(__wrap_gpio_get, index, 12);      // RP's GPIO pin.
    // Specify the gpio_get(...) return values matching the config.
    will_return(__wrap_gpio_get, true);
    will_return(__wrap_gpio_get, false);

    // Make the call under test.
    // Since HAL has sent values_confirmed matching GPIO,
    // and Message_gpio.confirmation_pending not set,
    // no data needs sent.
    gpio_serialize(&tx_buf, &tx_buf_len);

    // Proves data was not added to tx_buf_len.
    assert_int_equal(tx_buf.length, 0);
    assert_int_equal(tx_buf.checksum, 0);

    // Out pins values correctly set.
    assert_int_equal(config.gpio[0].value, true);
    assert_int_equal(config.gpio[1].value, false);

    // Config agrees that Reply_gpio has not been sent.
    assert_int_equal(config.gpio_confirmation_pending[0], false);
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_gpio_config),
        cmocka_unit_test(test_native_gpio_set_output_values),
        cmocka_unit_test(test_i2c_gpio_set_output_values),
        cmocka_unit_test(test_send_PC_to_RP),
        cmocka_unit_test(test_send_PC_to_RP_confirmation_set),
        cmocka_unit_test(test_send_RP_to_PC),
        cmocka_unit_test(test_send_RP_to_PC_out_gpio_changed),
        cmocka_unit_test(test_send_RP_to_PC_matching)
    };

    return cmocka_run_group_tests(tests, NULL, NULL);
}

