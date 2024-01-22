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
}


/* Test the message types. */
static void test_timing(void **state) {
    (void) state; /* unused */

    struct NWBuffer buffer = {0};
    size_t mess_received_count = 0;

    skeleton_t data;
    setup_data(&data);
   
    struct Reply_timing message = {
        .type = REPLY_TIMING,
        .update_id = 1234,
        .time_diff = 5678,
        .rp_update_len = 9012
    };

    memcpy(buffer.payload, &message, sizeof(message));
    buffer.length = sizeof(message);


    process_data(
            &buffer,
            &data,
            &mess_received_count,
            sizeof(message) + sizeof(buffer.length) + sizeof(buffer.checksum),
            NULL,
            NULL
            );


    assert_int_equal(*(data.metric_update_id), message.update_id);
    assert_int_equal(*(data.metric_time_diff), message.time_diff);
    assert_int_equal(*(data.metric_rp_update_len), message.rp_update_len);
}

static void test_axis_movement(void **state) {
    (void) state; /* unused */

    struct NWBuffer buffer = {0};
    size_t mess_received_count = 0;

    skeleton_t data;
    setup_data(&data);
   
    uint32_t axis = 0;
    struct Reply_axis_movement message = {
        .type = REPLY_AXIS_MOVEMENT,
        .axis = axis,
        .abs_pos_acheived = 1234,
        .velocity_acheived = 7890,
    };

    memcpy(buffer.payload, &message, sizeof(message));
    buffer.length = sizeof(message);

    process_data(
            &buffer,
            &data,
            &mess_received_count,
            sizeof(message) + sizeof(buffer.length) + sizeof(buffer.checksum),
            NULL,
            NULL
            );

    assert_double_equal(
            *(data.joint_pos_feedback[axis]),
            (double)message.abs_pos_acheived / *(data.joint_scale[axis]),
            0.0001);
    assert_int_equal(*(data.joint_velocity_feedback[axis]), message.velocity_acheived);
}

static void test_axis_config(void **state) {
    (void) state; /* unused */

    struct NWBuffer buffer = {0};
    size_t mess_received_count = 0;

    skeleton_t data = {0};
    setup_data(&data);
   
    uint32_t axis = 0;
    struct Reply_axis_config message = {
        .type = REPLY_AXIS_CONFIG,
        .axis = axis,
        .enable = 1,
        .gpio_step = 2,
        .gpio_dir = 3,
        .max_velocity = 56.78,
        .max_accel = 90.12,
        //.velocity_requested = 3456,
        //.step_len_ticks = 1357
    };

    struct Message_joint_config last_joint_config;

    memcpy(buffer.payload, &message, sizeof(message));
    buffer.length = sizeof(message);

    process_data(
            &buffer,
            &data,
            &mess_received_count,
            sizeof(message) + sizeof(buffer.length) + sizeof(buffer.checksum),
            &last_joint_config,
            NULL
            );

    assert_int_equal(last_joint_config.enable, message.enable);
    assert_int_equal(last_joint_config.gpio_step, message.gpio_step);
    assert_int_equal(last_joint_config.gpio_dir, message.gpio_dir);
    assert_double_equal(last_joint_config.max_velocity, message.max_velocity, 0.0001);
    assert_double_equal(last_joint_config.max_accel, message.max_accel, 0.0001);
}

static void test_axis_metrics(void **state) {
    (void) state; /* unused */

    struct NWBuffer buffer = {0};
    size_t mess_received_count = 0;

    skeleton_t data = {0};
    setup_data(&data);
   
    uint32_t axis = 0;
    struct Reply_axis_metrics message = {
        .type = REPLY_AXIS_METRICS,
        .axis = axis,
        .velocity_requested = 3456,
        .step_len_ticks = 1357,
    };

    memcpy(buffer.payload, &message, sizeof(message));
    buffer.length = sizeof(message);

    process_data(
            &buffer,
            &data,
            &mess_received_count,
            sizeof(message) + sizeof(buffer.length) + sizeof(buffer.checksum),
            NULL,
            NULL
            );

    assert_int_equal(*(data.joint_step_len_ticks[axis]), message.step_len_ticks);
    assert_int_equal(*(data.joint_velocity_cmd[axis]), message.velocity_requested);
}


int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_timing),
        cmocka_unit_test(test_axis_movement),
        cmocka_unit_test(test_axis_config),
        cmocka_unit_test(test_axis_metrics)
    };

    return cmocka_run_group_tests(tests, NULL, NULL);
}

