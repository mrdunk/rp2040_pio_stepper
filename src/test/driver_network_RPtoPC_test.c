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

  for(size_t joint = 0; joint < MAX_JOINT; joint++) {
    data->joint_enable[joint] = &(joint_enable[joint]);
    data->joint_gpio_step[joint] = &(joint_gpio_step[joint]);
    data->joint_gpio_dir[joint] = &(joint_gpio_dir[joint]);
    data->joint_max_velocity[joint] = &(joint_max_velocity[joint]);
    data->joint_max_accel[joint] = &(joint_max_accel[joint]);
    data->joint_pos_feedback[joint] = &(joint_pos_feedback[joint]);
    data->joint_scale[joint] = &(joint_scale[joint]);
    data->joint_step_len_ticks[joint] = &(joint_step_len_ticks[joint]);
    data->joint_velocity_cmd[joint] = &(joint_velocity_cmd[joint]);
    data->joint_velocity_feedback[joint] = &(joint_velocity_feedback[joint]);
  }
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
    buffer.checksum = checksum(0, 0, buffer.length, buffer.payload);

    process_data(
            &buffer,
            &data,
            &mess_received_count,
            sizeof(message) + sizeof(buffer.length) + sizeof(buffer.checksum),
            NULL,
            NULL,
            NULL
            );


    assert_int_equal(*(data.metric_update_id), message.update_id);
    assert_int_equal(*(data.metric_time_diff), message.time_diff);
    assert_int_equal(*(data.metric_rp_update_len), message.rp_update_len);
}

static void test_joint_movement(void **state) {
    (void) state; /* unused */

    struct NWBuffer buffer = {0};
    size_t mess_received_count = 0;

    skeleton_t data = {0};
    setup_data(&data);
   
    struct Reply_joint_movement message = {
        .type = REPLY_JOINT_MOVEMENT,
        .abs_pos_acheived = {1234, 5678, 9, 10},
        .velocity_acheived = {7890, 1234, 5, 6}
    };

    memcpy(buffer.payload, &message, sizeof(message));
    buffer.length = sizeof(message);
    buffer.checksum = checksum(0, 0, buffer.length, buffer.payload);

    process_data(
            &buffer,
            &data,
            &mess_received_count,
            sizeof(message) + sizeof(buffer.length) + sizeof(buffer.checksum),
            NULL,
            NULL,
            NULL
            );

    for(size_t joint = 0; joint < MAX_JOINT; joint++) {
        assert_double_equal(
                (*data.joint_pos_feedback)[joint],
                (double)message.abs_pos_acheived[joint] / (*data.joint_scale)[joint],
                0.0001);
        assert_int_equal((*data.joint_velocity_feedback)[joint], message.velocity_acheived[joint]);
    }
}

static void test_joint_config(void **state) {
    (void) state; /* unused */

    struct NWBuffer buffer = {0};
    size_t mess_received_count = 0;

    skeleton_t data = {0};
    setup_data(&data);
   
    uint32_t joint = 0;
    struct Reply_joint_config message = {
        .type = REPLY_JOINT_CONFIG,
        .joint = joint,
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
    buffer.checksum = checksum(0, 0, buffer.length, buffer.payload);

    process_data(
            &buffer,
            &data,
            &mess_received_count,
            sizeof(message) + sizeof(buffer.length) + sizeof(buffer.checksum),
            &last_joint_config,
            NULL,
            NULL
            );

    assert_int_equal(last_joint_config.enable, message.enable);
    assert_int_equal(last_joint_config.gpio_step, message.gpio_step);
    assert_int_equal(last_joint_config.gpio_dir, message.gpio_dir);
    assert_double_equal(last_joint_config.max_velocity, message.max_velocity, 0.0001);
    assert_double_equal(last_joint_config.max_accel, message.max_accel, 0.0001);
}

static void test_joint_metrics(void **state) {
    (void) state; /* unused */

    struct NWBuffer buffer = {0};
    size_t mess_received_count = 0;

    skeleton_t data = {0};
    setup_data(&data);
   
    struct Reply_joint_metrics message = {
        .type = REPLY_JOINT_METRICS,
        .velocity_requested = {3456, 7890, 1234},
        .step_len_ticks = {1357, 2468, 3579, 4680}
    };

    memcpy(buffer.payload, &message, sizeof(message));
    buffer.length = sizeof(message);
    buffer.checksum = checksum(0, 0, buffer.length, buffer.payload);

    process_data(
            &buffer,
            &data,
            &mess_received_count,
            sizeof(message) + sizeof(buffer.length) + sizeof(buffer.checksum),
            NULL,
            NULL,
            NULL
            );

    for(size_t joint = 0; joint < MAX_JOINT; joint++) {
      assert_int_equal(*(data.joint_step_len_ticks[joint]), message.step_len_ticks[joint]);
      assert_int_equal(*(data.joint_velocity_cmd[joint]), message.velocity_requested[joint]);
    }
}


int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_timing),
        cmocka_unit_test(test_joint_movement),
        cmocka_unit_test(test_joint_config),
        cmocka_unit_test(test_joint_metrics)
    };

    return cmocka_run_group_tests(tests, NULL, NULL);
}

