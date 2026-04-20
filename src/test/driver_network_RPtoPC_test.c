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
hal_float_t joint_accel_cmd[4];
hal_float_t joint_velocity_feedback[4];
hal_s32_t joint_pos_error[4];
hal_u32_t joint_overrun_count[4];
hal_u32_t joint_underrun_count[4];
hal_float_t metric_overrun_ratio;
hal_float_t metric_overrun_vs_underrun;

hal_float_t spindle_speed_out[MAX_SPINDLE];
hal_float_t spindle_speed_in[MAX_SPINDLE];
hal_bit_t   spindle_at_speed[MAX_SPINDLE];

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
    data->joint_accel_cmd[joint] = &(joint_accel_cmd[joint]);
    data->joint_velocity_feedback[joint] = &(joint_velocity_feedback[joint]);
    data->joint_pos_error[joint] = &(joint_pos_error[joint]);
    data->joint_overrun_count[joint]  = &(joint_overrun_count[joint]);
    data->joint_underrun_count[joint] = &(joint_underrun_count[joint]);
  }
  data->metric_overrun_ratio       = &metric_overrun_ratio;
  data->metric_overrun_vs_underrun = &metric_overrun_vs_underrun;

  for (size_t s = 0; s < MAX_SPINDLE; s++) {
    data->spindle_speed_out[s] = &spindle_speed_out[s];
    data->spindle_speed_in[s]  = &spindle_speed_in[s];
    data->spindle_at_speed[s]  = &spindle_at_speed[s];
    data->spindle_poles[s]     = 4.0;
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
        .abs_pos_achieved = {1234, 5678, 9, 10},
        .velocity_achieved = {7890, 1234, 5, 6}
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
                (double)message.abs_pos_achieved[joint] / (*data.joint_scale)[joint],
                0.0001);
        assert_int_equal((*data.joint_velocity_feedback)[joint], message.velocity_achieved[joint]);
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
        //.velocity_requested_tm1 = 3456,
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
        .velocity_requested_tm1 = {3456, 7890, 1234, 5678},
        .step_len_ticks = {1357, 2468, 3579, 4680},
        .overrun_count  = {10, 20, 30, 40},
        .underrun_count = {1, 2, 3, 4}
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
      assert_int_equal(*(data.joint_velocity_cmd[joint]), message.velocity_requested_tm1[joint]);
      assert_int_equal(*(data.joint_overrun_count[joint]),  message.overrun_count[joint]);
      assert_int_equal(*(data.joint_underrun_count[joint]), message.underrun_count[joint]);
    }
}


/* overrun/underrun use = so HAL pins show the per-window count, not a running total. */
static void test_joint_metrics_per_window(void **state) {
    (void) state;

    struct NWBuffer buffer = {0};
    size_t mess_received_count = 0;

    memset(joint_overrun_count,  0, sizeof(joint_overrun_count));
    memset(joint_underrun_count, 0, sizeof(joint_underrun_count));
    skeleton_t data = {0};
    setup_data(&data);

    struct Reply_joint_metrics message = {
        .type = REPLY_JOINT_METRICS,
        .overrun_count  = {5, 0, 3, 1},
        .underrun_count = {0, 2, 0, 4}
    };

    /* First servo cycle. */
    memcpy(buffer.payload, &message, sizeof(message));
    buffer.length = sizeof(message);
    buffer.checksum = checksum(0, 0, buffer.length, buffer.payload);
    process_data(&buffer, &data, &mess_received_count,
                 sizeof(message) + sizeof(buffer.length) + sizeof(buffer.checksum),
                 NULL, NULL, NULL);

    /* Second servo cycle with same counts. */
    mess_received_count = 0;
    memcpy(buffer.payload, &message, sizeof(message));
    buffer.length = sizeof(message);
    buffer.checksum = checksum(0, 0, buffer.length, buffer.payload);
    process_data(&buffer, &data, &mess_received_count,
                 sizeof(message) + sizeof(buffer.length) + sizeof(buffer.checksum),
                 NULL, NULL, NULL);

    /* HAL pins reflect only the most recent window — no accumulation. */
    for (size_t joint = 0; joint < MAX_JOINT; joint++) {
        assert_int_equal(*(data.joint_overrun_count[joint]),  message.overrun_count[joint]);
        assert_int_equal(*(data.joint_underrun_count[joint]), message.underrun_count[joint]);
    }
}

/* EMA pins reflect combined overrun/underrun across all joints. */
static void test_joint_metrics_ema_ratios(void **state) {
    (void) state;

    struct NWBuffer buffer = {0};
    size_t mess_received_count = 0;

    memset(joint_overrun_count,  0, sizeof(joint_overrun_count));
    memset(joint_underrun_count, 0, sizeof(joint_underrun_count));
    metric_overrun_ratio       = 0.0;
    metric_overrun_vs_underrun = 0.0;

    skeleton_t data = {0};
    setup_data(&data);

    /* Seed known EMA state: 2.0 overrun, 2.0 underrun → ratio should be 0.5 */
    data.ema_overrun  = 2.0;
    data.ema_underrun = 2.0;

    /* Send message with overrun=4 total (1+1+1+1), underrun=0. */
    struct Reply_joint_metrics message = {
        .type = REPLY_JOINT_METRICS,
        .overrun_count  = {1, 1, 1, 1},
        .underrun_count = {0, 0, 0, 0}
    };
    memcpy(buffer.payload, &message, sizeof(message));
    buffer.length    = sizeof(message);
    buffer.checksum  = checksum(0, 0, buffer.length, buffer.payload);
    process_data(&buffer, &data, &mess_received_count,
                 sizeof(message) + sizeof(buffer.length) + sizeof(buffer.checksum),
                 NULL, NULL, NULL);

    /* After one EMA step with total_overrun=4, total_underrun=0:
     *   ema_overrun  = 2.0 * (1 - α) + 4 * α  ≈ 2.0  (α is tiny)
     *   ema_underrun = 2.0 * (1 - α) + 0 * α  ≈ 2.0
     * Ratio ≈ 0.5, overrun_ratio ≈ 2.0
     * We only test direction: overrun_ratio > 0, overrun_vs_underrun in (0, 1). */
    assert_true(*data.metric_overrun_ratio > 0.0);
    assert_true(*data.metric_overrun_vs_underrun > 0.0);
    assert_true(*data.metric_overrun_vs_underrun < 1.0);

    /* With ema_overrun >> ema_underrun, ratio should approach 1. */
    data.ema_overrun  = 1000.0;
    data.ema_underrun = 0.001;
    memcpy(buffer.payload, &message, sizeof(message));
    buffer.checksum = checksum(0, 0, buffer.length, buffer.payload);
    process_data(&buffer, &data, &mess_received_count,
                 sizeof(message) + sizeof(buffer.length) + sizeof(buffer.checksum),
                 NULL, NULL, NULL);
    assert_true(*data.metric_overrun_vs_underrun > 0.99);

    /* With both EMA zero and zero counts, ratio should be 0.5. */
    data.ema_overrun  = 0.0;
    data.ema_underrun = 0.0;
    struct Reply_joint_metrics zero_msg = { .type = REPLY_JOINT_METRICS };
    memcpy(buffer.payload, &zero_msg, sizeof(zero_msg));
    buffer.length   = sizeof(zero_msg);
    buffer.checksum = checksum(0, 0, buffer.length, buffer.payload);
    process_data(&buffer, &data, &mess_received_count,
                 sizeof(zero_msg) + sizeof(buffer.length) + sizeof(buffer.checksum),
                 NULL, NULL, NULL);
    assert_float_equal(*data.metric_overrun_vs_underrun, 0.5, 1e-9);
}

static void test_unpack_spindle_speed(void **state) {
    (void) state;

    skeleton_t data = {0};
    memset(spindle_speed_out, 0, sizeof(spindle_speed_out));
    memset(spindle_speed_in,  0, sizeof(spindle_speed_in));
    memset(spindle_at_speed,  0, sizeof(spindle_at_speed));
    setup_data(&data);

    size_t rx_offset = 0;
    size_t received_count = 0;

    /* poles=4, speed=50.0 Hz → rpm = 50 * 120 / 4 = 1500 */
    struct NWBuffer buffer = {0};
    struct Reply_spindle_speed reply = {
        .type          = REPLY_SPINDLE_SPEED,
        .spindle_index = 0,
        .speed         = 50.0,
    };
    *data.spindle_speed_in[0] = 1500.0;

    memcpy(buffer.payload, &reply, sizeof(reply));
    buffer.length = sizeof(reply);  /* sizeof = 24, alligned32(24) = 24 */

    bool result = unpack_spindle_speed(&buffer, &rx_offset, &received_count, &data);

    assert_true(result);
    assert_int_equal(received_count, 1);
    assert_double_equal(*data.spindle_speed_out[0], 1500.0, 0.01);
    assert_true(*data.spindle_at_speed[0]);  /* |1500 - 1500| < 10 */
    assert_int_equal(rx_offset, 24);  /* sizeof(Reply_spindle_speed) = 24 = alligned32(24) */
}

static void test_unpack_spindle_not_at_speed(void **state) {
    (void) state;

    skeleton_t data = {0};
    memset(spindle_speed_out, 0, sizeof(spindle_speed_out));
    memset(spindle_speed_in,  0, sizeof(spindle_speed_in));
    memset(spindle_at_speed,  0, sizeof(spindle_at_speed));
    setup_data(&data);

    size_t rx_offset = 0;
    size_t received_count = 0;

    /* poles=4, speed=50.0 Hz → rpm = 1500; commanded = 1520 → |diff| = 20 > 10 */
    struct NWBuffer buffer = {0};
    struct Reply_spindle_speed reply = {
        .type          = REPLY_SPINDLE_SPEED,
        .spindle_index = 0,
        .speed         = 50.0,
    };
    *data.spindle_speed_in[0] = 1520.0;

    memcpy(buffer.payload, &reply, sizeof(reply));
    buffer.length = sizeof(reply);

    bool result = unpack_spindle_speed(&buffer, &rx_offset, &received_count, &data);

    assert_true(result);
    assert_int_equal(received_count, 1);
    assert_double_equal(*data.spindle_speed_out[0], 1500.0, 0.01);
    assert_false(*data.spindle_at_speed[0]);  /* |1500 - 1520| = 20 >= 10 */
}

static void test_unpack_spindle_config(void **state) {
    (void) state;

    size_t rx_offset = 0;
    size_t received_count = 0;
    struct Message_spindle_config last_spindle_config[MAX_SPINDLE] = {0};

    struct NWBuffer buffer = {0};
    struct Reply_spindle_config reply = {
        .type           = REPLY_SPINDLE_CONFIG,
        .spindle_index  = 0,
        .modbus_address = 2,
        .vfd_type       = 1,
        .bitrate        = 19200,
    };
    memcpy(buffer.payload, &reply, sizeof(reply));
    buffer.length = 8;  /* sizeof(reply) = 6, alligned32(6) = 8 */

    bool result = unpack_spindle_config(&buffer, &rx_offset, &received_count, last_spindle_config);

    assert_true(result);
    assert_int_equal(received_count, 1);
    assert_int_equal(rx_offset, 8);  /* alligned32(sizeof(reply)) = 8 */
    assert_int_equal(last_spindle_config[0].modbus_address, 2);
    assert_int_equal(last_spindle_config[0].vfd_type, 1);
    assert_int_equal(last_spindle_config[0].bitrate, 19200);
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_timing),
        cmocka_unit_test(test_joint_movement),
        cmocka_unit_test(test_joint_config),
        cmocka_unit_test(test_joint_metrics),
        cmocka_unit_test(test_joint_metrics_per_window),
        cmocka_unit_test(test_joint_metrics_ema_ratios),
        cmocka_unit_test(test_unpack_spindle_speed),
        cmocka_unit_test(test_unpack_spindle_not_at_speed),
        cmocka_unit_test(test_unpack_spindle_config)
    };

    return cmocka_run_group_tests(tests, NULL, NULL);
}

