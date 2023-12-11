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

hal_s32_t joint_step_len_ticks[4];
hal_float_t joint_pos_feedback[4];
hal_float_t joint_scale[4] = {1000, 1000, 1000, 1000};
hal_float_t joint_velocity_cmd[4];
hal_float_t joint_velocity_feedback[4];

void setup_data(skeleton_t* data) {
    data->metric_update_id = &metric_update_id;
    data->metric_time_diff = &metric_time_diff;
    data->metric_rp_update_len = &metric_rp_update_len;
    *data->joint_pos_feedback = joint_pos_feedback;
    *data->joint_scale = joint_scale;
    *data->joint_step_len_ticks = joint_step_len_ticks;
    *data->joint_velocity_cmd = joint_velocity_cmd;
    *data->joint_velocity_feedback = joint_velocity_feedback;
}


/* Test the message types. */
static void test_metrics(void **state) {
    (void) state; /* unused */

    char buffer[BUFSIZE];
    memset(buffer, '\0', BUFSIZE);

    skeleton_t data;
    setup_data(&data);
   
    struct Reply_metrics message = {
        .type = REPLY_METRICS,
        .update_id = 1234,
        .time_diff = 5678,
        .rp_update_len = 9012
    };

    memcpy(buffer, &message, sizeof(message));


    process_data(buffer, &data, 0);

    assert_int_equal(*(data.metric_update_id), message.update_id);
    assert_int_equal(*(data.metric_time_diff), message.time_diff);
    assert_int_equal(*(data.metric_rp_update_len), message.rp_update_len);
}

static void test_axis_config(void **state) {
    (void) state; /* unused */

    char buffer[BUFSIZE];
    memset(buffer, '\0', BUFSIZE);

    skeleton_t data;
    setup_data(&data);
   
    uint32_t axis = 0;
    struct Reply_axis_config message = {
        .type = REPLY_AXIS_CONFIG,
        .axis = axis,
        .abs_pos_acheived = 1234,
        .max_velocity = 5678,
        .max_accel_ticks = 9012,
        .velocity_requested = 3456,
        .velocity_acheived = 7890,
        .step_len_ticks = 1357
    };

    memcpy(buffer, &message, sizeof(message));


    process_data(buffer, &data, 0);

    assert_double_equal(
            *(data.joint_pos_feedback[axis]),
            (double)message.abs_pos_acheived / *(data.joint_scale[axis]),
            0.0001);
    assert_int_equal(*(data.joint_step_len_ticks[axis]), message.step_len_ticks);
    assert_int_equal(*(data.joint_velocity_cmd[axis]), message.velocity_requested);
    assert_int_equal(*(data.joint_velocity_feedback[axis]), message.velocity_acheived);
}



int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_metrics),
        cmocka_unit_test(test_axis_config)
    };

    return cmocka_run_group_tests(tests, NULL, NULL);
}

