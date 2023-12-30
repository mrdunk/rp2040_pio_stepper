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

hal_bit_t gpio_data[MAX_GPIO];
hal_bit_t gpio_data_received[MAX_GPIO];


void setup_data(skeleton_t* data) {
    data->metric_update_id = &metric_update_id;
    data->metric_time_diff = &metric_time_diff;
    data->metric_rp_update_len = &metric_rp_update_len;
    *data->joint_enable = joint_enable;
    *data->joint_gpio_step = joint_gpio_step;
    *data->joint_gpio_dir = joint_gpio_dir;
    *data->joint_max_velocity = joint_max_velocity;
    *data->joint_max_accel = joint_max_accel;
    *data->joint_pos_feedback = joint_pos_feedback;
    *data->joint_scale = joint_scale;
    *data->joint_step_len_ticks = joint_step_len_ticks;
    *data->joint_velocity_cmd = joint_velocity_cmd;
    *data->joint_velocity_feedback = joint_velocity_feedback;

    *data->gpio_data = gpio_data;
    *data->gpio_data_received = gpio_data_received;
}

void helper_set_gpio_data(hal_bit_t** gpio_data, uint32_t values, uint8_t bank) {
    int bank_offset = bank * 32;
    for(int i = bank_offset; i < 32 + bank_offset ; i++) {
        bool value = values & (0x1 << i);
        //printf("%u", value);
        (*gpio_data)[i] = value;
    }
    //printf("\n");
}


static void test_serialize_gpio(void **state) {
    (void) state; /* unused */

    skeleton_t data;
    setup_data(&data);

    struct NWBuffer buffer = {0};
    assert_int_equal(buffer.checksum, 0);
    assert_int_equal(buffer.length, 0);

    // Set values.
    // These represent the HAL pins.
    uint32_t values_b0 = 0b10101010110011001111000011111111;
    uint32_t values_b0_last_received = values_b0 - 1;
    uint32_t values_b1 = 0b01010101001100110000111100000000;
    uint32_t values_b1_last_received = values_b1 + 0x10000;
    helper_set_gpio_data(data.gpio_data, values_b0, 0);
    helper_set_gpio_data(data.gpio_data, values_b1, 1);
    helper_set_gpio_data(data.gpio_data_received, values_b0_last_received, 0);
    helper_set_gpio_data(data.gpio_data_received, values_b1_last_received, 1);

    size_t data_size = serialize_gpio(&buffer, *data.gpio_data, *data.gpio_data_received);

    struct Message_gpio message_b0 = {
        .type = MSG_SET_GPIO,
        .bank = 0,
        .values = values_b0
    };

    struct Message_gpio message_b1 = {
        .type = MSG_SET_GPIO,
        .bank = 1,
        .values = values_b1
    };

    struct Message_gpio* message_b0_p = (void*)buffer.payload;
    struct Message_gpio* message_b1_p = (void*)(buffer.payload + sizeof(struct Message_gpio));

    assert_int_equal(data_size, sizeof(struct Message_gpio) * 2);
    assert_int_equal(message_b0.type, message_b0_p->type);
    assert_int_equal(message_b0.bank, message_b0_p->bank);
    assert_int_equal(message_b0.values, message_b0_p->values);
    assert_int_equal(message_b1.type, message_b1_p->type);
    assert_int_equal(message_b1.bank, message_b1_p->bank);
    assert_int_equal(message_b1.values, message_b1_p->values);
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_serialize_gpio)
    };

    return cmocka_run_group_tests(tests, NULL, NULL);
}


