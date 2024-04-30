#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <stdint.h>
#include <cmocka.h>

#include "../driver/rp2040_network.c"
#include "mocks/driver_mocks.h"
#include "../shared/messages.h"


/* Set up mock skeleton_t struct. */
hal_u32_t metric_update_id;
hal_s32_t metric_time_diff;
hal_u32_t metric_rp_update_len;

hal_bit_t joint_enable[4];
hal_s32_t joint_gpio_step[4];
hal_s32_t joint_gpio_dir[4];
hal_float_t joint_max_velocity[4];
hal_float_t joint_max_accel[4];
hal_s32_t joint_step_len_ticks[4];
hal_float_t joint_pos_feedback[4];
hal_float_t joint_scale[4] = {1000, 1000, 1000, 1000};
hal_float_t joint_velocity_cmd[4];
hal_float_t joint_velocity_feedback[4];

hal_bit_t gpio_data_out[MAX_GPIO];
hal_bit_t gpio_data_out_invert[MAX_GPIO];
hal_bit_t gpio_data_in[MAX_GPIO];
hal_bit_t gpio_data_in_not[MAX_GPIO];
hal_u32_t gpio_type[MAX_GPIO];


void setup_data(skeleton_t* data) {
    data->metric_update_id = &metric_update_id;
    data->metric_time_diff = &metric_time_diff;
    data->metric_rp_update_len = &metric_rp_update_len;

    for(size_t joint = 0; joint < MAX_JOINT; joint++) {
        data->joint_enable[joint] =            &joint_enable[joint];
        data->joint_gpio_step[joint] =         &joint_gpio_step[joint];
        data->joint_gpio_dir[joint] =          &joint_gpio_dir[joint];
        data->joint_max_velocity[joint] =      &joint_max_velocity[joint];
        data->joint_max_accel[joint] =         &joint_max_accel[joint];
        data->joint_pos_feedback[joint] =      &joint_pos_feedback[joint];
        data->joint_scale[joint] =             &joint_scale[joint];
        data->joint_step_len_ticks[joint] =    &joint_step_len_ticks[joint];
        data->joint_velocity_cmd[joint] =      &joint_velocity_cmd[joint];
        data->joint_velocity_feedback[joint] = &joint_velocity_feedback[joint];
    }

    for(size_t gpio = 0; gpio < MAX_GPIO; gpio++) {
        data->gpio_data_out[gpio] = &gpio_data_out[gpio];
        data->gpio_data_out_invert[gpio] = &gpio_data_out_invert[gpio];
        data->gpio_data_in[gpio] = &gpio_data_in[gpio];
        data->gpio_data_in_not[gpio] = &gpio_data_in_not[gpio];
        data->gpio_type[gpio] = &gpio_type[gpio];

        *data->gpio_data_out_invert[gpio] = false;
        *data->gpio_type[gpio] = GPIO_TYPE_NOT_SET;
    }

    for(size_t bank = 0; bank < MAX_GPIO / 32; bank++) {
        data->gpio_data_received[bank] = 0;
        data->gpio_confirmation_pending[bank] = 0;
    }
}

/* Turn a 32 bit value into an array of bool. */
void helper_set_gpio_data(hal_bit_t** gpio_data, uint32_t values, uint8_t bank) {
    int bank_offset = bank * 32;
    for(int i = bank_offset; i < 32 + bank_offset ; i++) {
        bool value = values & (0x1 << i);
        (*gpio_data)[i] = value;
    }
}

/* Compare an array of bool to the same values packed in a 32 bit value. */
bool helper_compare_gpio_data(hal_bit_t** gpio_data, uint32_t values, uint8_t bank) {
    bool return_val = true;
    int bank_offset = bank * 32;
    for(int i = bank_offset; i < 32 + bank_offset ; i++) {
        bool value = values & (0x1 << i);
        return_val &= ((*gpio_data)[i] == value);
    }
    return return_val;
}

/* When an input on the RP changes value, the HAL data should change in response. */
static void test_serialize_gpio_in_change(void **state) {
    (void) state; /* unused */

    skeleton_t data = {0};
    setup_data(&data);

    struct NWBuffer buffer = {0};
    assert_int_equal(buffer.checksum, 0);
    assert_int_equal(buffer.length, 0);

    // Set values.
    // These represent values set on the HAL pins.
    uint32_t hal_values_b0 = 0b10101010110011001111000011111101;
    uint32_t hal_values_b1 = 0b01010101001100110000111100000000;
    helper_set_gpio_data(data.gpio_data_in, hal_values_b0, 0);
    helper_set_gpio_data(data.gpio_data_in_not, ~hal_values_b0, 0);
    helper_set_gpio_data(data.gpio_data_in, hal_values_b1, 1);
    helper_set_gpio_data(data.gpio_data_in_not, ~hal_values_b1, 1);
    helper_set_gpio_data(data.gpio_data_out, hal_values_b0, 0);
    helper_set_gpio_data(data.gpio_data_out, hal_values_b1, 1);

    // Flip bits that are configured as inputs.
    uint32_t hal_values_b0_last_received = hal_values_b0 ^ 0b11;
    // Flip bit that won't be configured as GPIO
    uint32_t hal_values_b1_last_received = hal_values_b1 ^ 0b100;
    data.gpio_data_received[0] = hal_values_b0_last_received;
    data.gpio_data_received[1] = hal_values_b1_last_received;

    // GPIO pin is of input type on a bit with changed data.
    // This will case a bank 0 transmission.
    *data.gpio_type[0] = GPIO_TYPE_NATIVE_IN;
    *data.gpio_type[1] = GPIO_TYPE_NATIVE_IN;

    // GPIO pins on a bits without changed data.
    // This will not case a bank 1 transmission.
    *data.gpio_type[32] = GPIO_TYPE_NATIVE_OUT;
    *data.gpio_type[33] = GPIO_TYPE_NATIVE_IN;

    // Last incoming message did not have confirmation_pending set.
    data.gpio_confirmation_pending[0] = false;
    data.gpio_confirmation_pending[1] = false;

    // Serialize any data that needs sent.
    size_t data_size = serialize_gpio(&buffer, &data);

    struct Message_gpio* message_b0_p = (void*)buffer.payload;

    // Only one message:
    assert_int_equal(data_size, sizeof(struct Message_gpio) * 1);

    assert_int_not_equal(buffer.checksum, 0);
    assert_int_equal(buffer.length, data_size);
    
    assert_int_equal(message_b0_p->type, MSG_SET_GPIO);
    assert_int_equal(message_b0_p->bank, 0);
    assert_int_equal(message_b0_p->values, hal_values_b0_last_received);
    assert_int_equal(message_b0_p->confirmation_pending, true);
    // Has set the HAL state to reflect the received data.
    assert_int_equal((*data.gpio_data_in)[0], false);
    assert_int_equal((*data.gpio_data_in_not)[0], true);
    assert_int_equal((*data.gpio_data_in)[1], true);
    assert_int_equal((*data.gpio_data_in_not)[1], false);
    // These are OUT pins so gpio_data_out is not used or changed.
    assert_int_equal((*data.gpio_data_out)[0], true);
    assert_int_equal((*data.gpio_data_out)[1], false);
}

/* When an output for the RP changes value in HAL, the transmitted data should reflect HAL. */
static void test_serialize_gpio_out_change(void **state) {
    (void) state; /* unused */

    skeleton_t data = {0};
    setup_data(&data);

    struct NWBuffer buffer = {0};
    assert_int_equal(buffer.checksum, 0);
    assert_int_equal(buffer.length, 0);

    // Set values.
    // These represent values set on the HAL pins.
    uint32_t hal_values_b0 = 0b01010101001100110000111100000000;
    uint32_t hal_values_b1 = 0b10101010110011001111000011111101;
    helper_set_gpio_data(data.gpio_data_in, hal_values_b0, 0);
    helper_set_gpio_data(data.gpio_data_in_not, ~hal_values_b0, 0);
    helper_set_gpio_data(data.gpio_data_in, hal_values_b1, 1);
    helper_set_gpio_data(data.gpio_data_in_not, ~hal_values_b1, 1);
    helper_set_gpio_data(data.gpio_data_out, hal_values_b0, 0);
    helper_set_gpio_data(data.gpio_data_out, hal_values_b1, 1);

    // GPIO output configured to invert these bits:
    *data.gpio_data_out_invert[34] = true;
    *data.gpio_data_out_invert[35] = true;
    uint32_t hal_values_b1_with_invert = hal_values_b1 ^ 0b1100;

    // Flip bit that won't be configured as GPIO
    uint32_t hal_values_b0_last_received = hal_values_b0 ^ 0b100;
    // Flip a bit that is configured as output.
    uint32_t hal_values_b1_last_received = hal_values_b1_with_invert ^ 0b100;

    data.gpio_data_received[0] = hal_values_b0_last_received;
    data.gpio_data_received[1] = hal_values_b1_last_received;

    // GPIO pin is of output type on a bit with changed data.
    // This will case a bank 1 transmission.
    *data.gpio_type[32] = GPIO_TYPE_NATIVE_OUT;
    *data.gpio_type[33] = GPIO_TYPE_NATIVE_OUT;
    *data.gpio_type[34] = GPIO_TYPE_NATIVE_OUT;
    *data.gpio_type[35] = GPIO_TYPE_NATIVE_OUT;

    // GPIO pins on bits without changed data.
    // This will not case a bank 0 transmission.
    *data.gpio_type[0] = GPIO_TYPE_NATIVE_OUT;
    *data.gpio_type[1] = GPIO_TYPE_NATIVE_IN;

    // Last incoming message did not have confirmation_pending set.
    data.gpio_confirmation_pending[0] = false;
    data.gpio_confirmation_pending[1] = false;

    // Serialize any data that needs sent.
    size_t data_size = serialize_gpio(&buffer, &data);

    struct Message_gpio* message_b0_p = (void*)buffer.payload;

    // Only one message:
    assert_int_equal(data_size, sizeof(struct Message_gpio) * 1);
    assert_int_not_equal(buffer.checksum, 0);
    assert_int_equal(buffer.length, data_size);
    
    assert_int_equal(message_b0_p->type, MSG_SET_GPIO);
    assert_int_equal(message_b0_p->bank, 1);
    assert_int_equal(message_b0_p->values, hal_values_b1_with_invert);
    assert_int_equal(message_b0_p->confirmation_pending, true);
    // HAL state has not changed since confirmation has not made it back.
    assert_int_equal(*data.gpio_data_in[32], hal_values_b1 & 0x1 << 0);
    assert_int_equal(*data.gpio_data_in_not[32], !(hal_values_b1 & 0x1 << 0));
    assert_int_equal(*data.gpio_data_in[33], hal_values_b1 & 0x1 << 1);
    assert_int_equal(*data.gpio_data_in_not[33], !(hal_values_b1 & 0x1 << 1));
    assert_int_equal(*data.gpio_data_out[32], hal_values_b1 & 0x1 << 0);
    assert_int_equal(*data.gpio_data_out[33], hal_values_b1 & 0x1 << 1);
}

/* No change in data or gpio values since last time but last incoming Reply_gpio
 * has set the confirmation_pending flag. */
static void test_serialize_gpio_confirmation_pending(void **state) {
    (void) state; /* unused */

    skeleton_t data;
    setup_data(&data);

    struct NWBuffer buffer = {0};
    assert_int_equal(buffer.checksum, 0);
    assert_int_equal(buffer.length, 0);

    // Set values.
    // These represent values set on the HAL pins.
    uint32_t hal_values_b0 = 0b10101010110011001111000011111101;
    uint32_t hal_values_b0_last_received = hal_values_b0;
    uint32_t hal_values_b1 = 0b01010101001100110000111100000000;
    uint32_t hal_values_b1_last_received = hal_values_b1;
    helper_set_gpio_data(data.gpio_data_in, hal_values_b0, 0);
    helper_set_gpio_data(data.gpio_data_in_not, ~hal_values_b0, 0);
    helper_set_gpio_data(data.gpio_data_in, hal_values_b1, 1);
    helper_set_gpio_data(data.gpio_data_in_not, ~hal_values_b1, 1);
    helper_set_gpio_data(data.gpio_data_out, hal_values_b0, 0);
    helper_set_gpio_data(data.gpio_data_out, hal_values_b1, 1);
    data.gpio_data_received[0] = hal_values_b0_last_received;
    data.gpio_data_received[1] = hal_values_b1_last_received;

    // GPIO pins on a bits without changed data.
    // This will not case a bank 0 transmission.
    *data.gpio_type[0] = GPIO_TYPE_NATIVE_OUT;
    *data.gpio_type[1] = GPIO_TYPE_NATIVE_IN;

    // Last incoming message had confirmation_pending set on bank 0.
    data.gpio_confirmation_pending[0] = true;
    data.gpio_confirmation_pending[1] = false;

    // Serialize any data that needs sent.
    size_t data_size = serialize_gpio(&buffer, &data);

    // Only one message:
    assert_int_equal(data_size, sizeof(struct Message_gpio));
    assert_int_not_equal(buffer.checksum, 0);
    assert_int_equal(buffer.length, data_size);
    
    struct Message_gpio* message_b0_p = (void*)buffer.payload;
    assert_int_equal(message_b0_p->type, MSG_SET_GPIO);
    assert_int_equal(message_b0_p->bank, 0);
    assert_int_equal(message_b0_p->values, hal_values_b0);
    assert_int_equal(message_b0_p->confirmation_pending, false);
    // HAL state has not changed.
    assert_int_equal(*data.gpio_data_in[0], true);
    assert_int_equal(*data.gpio_data_in_not[0], false);
    assert_int_equal(*data.gpio_data_in[1], false);
    assert_int_equal(*data.gpio_data_in_not[1], true);
    assert_int_equal(*data.gpio_data_out[0], true);
    assert_int_equal(*data.gpio_data_out[1], false);
}

/* No change in data since last time so nothing enqueued on the buffer. */
static void test_serialize_gpio_nothing_to_do(void **state) {
    (void) state; /* unused */

    skeleton_t data;
    setup_data(&data);

    struct NWBuffer buffer = {0};
    assert_int_equal(buffer.checksum, 0);
    assert_int_equal(buffer.length, 0);

    // Set values.
    // These represent values set on the HAL pins.
    uint32_t hal_values_b0 = 0b10101010110011001111000011111101;
    uint32_t hal_values_b0_last_received = hal_values_b0;
    uint32_t hal_values_b1 = 0b01010101001100110000111100000000;
    uint32_t hal_values_b1_last_received = hal_values_b1;
    helper_set_gpio_data(data.gpio_data_in, hal_values_b0, 0);
    helper_set_gpio_data(data.gpio_data_in_not, ~hal_values_b0, 0);
    helper_set_gpio_data(data.gpio_data_in, hal_values_b1, 1);
    helper_set_gpio_data(data.gpio_data_in_not, ~hal_values_b1, 1);
    helper_set_gpio_data(data.gpio_data_out, hal_values_b0, 0);
    helper_set_gpio_data(data.gpio_data_out, hal_values_b1, 1);
    data.gpio_data_received[0] = hal_values_b0_last_received;
    data.gpio_data_received[1] = hal_values_b1_last_received;

    // GPIO pins on a bits without changed data.
    // This will not case a bank 0 transmission.
    *data.gpio_type[0] = GPIO_TYPE_NATIVE_OUT;
    *data.gpio_type[1] = GPIO_TYPE_NATIVE_IN;

    // Last incoming message did not have confirmation_pending set.
    data.gpio_confirmation_pending[0] = false;
    data.gpio_confirmation_pending[1] = false;

    // Serialize any data that needs sent.
    // This is a no-op since no data has changed.
    size_t data_size = serialize_gpio(&buffer, &data);

    assert_int_equal(data_size, 0);
    assert_int_equal(buffer.checksum, 0);
    assert_int_equal(buffer.length, 0);
    // HAL state has not changed.
    assert_int_equal(*data.gpio_data_in[0], true);
    assert_int_equal(*data.gpio_data_in_not[0], false);
    assert_int_equal(*data.gpio_data_in[1], false);
    assert_int_equal(*data.gpio_data_in_not[1], true);
    assert_int_equal(*data.gpio_data_out[0], true);
    assert_int_equal(*data.gpio_data_out[1], false);
}

static void test_unpack_gpio(void **state) {
    (void) state; /* unused */

    skeleton_t data = {0};
    setup_data(&data);

    size_t original_rx_offset = 0;
    size_t rx_offset = original_rx_offset;
    size_t original_received_count = 0;
    size_t received_count = original_received_count;

    struct NWBuffer buffer = {0};
    assert_int_equal(buffer.checksum, 0);
    assert_int_equal(buffer.length, 0);

    struct Reply_gpio reply = {
        .type = REPLY_GPIO,
        .bank = 1,
        .values = 0b10101010110011001111000011111101
    };

    memcpy(buffer.payload, &reply, sizeof(reply));
    buffer.length = sizeof(reply);

    bool result = unpack_gpio(&buffer, &rx_offset, &received_count, &data);

    assert_int_equal(data.gpio_data_received[reply.bank], reply.values);
    assert_int_equal(result, true);
    assert_int_equal(received_count, original_received_count + 1);
    assert_int_equal(rx_offset, original_rx_offset += sizeof(reply));
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_serialize_gpio_out_change),
        cmocka_unit_test(test_serialize_gpio_in_change),
        cmocka_unit_test(test_serialize_gpio_confirmation_pending),
        cmocka_unit_test(test_serialize_gpio_nothing_to_do),
        cmocka_unit_test(test_unpack_gpio)
    };

    return cmocka_run_group_tests(tests, NULL, NULL);
}


