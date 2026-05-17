#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <stdint.h>
#include <cmocka.h>

#include "../driver/rp2040_network.c"
#include "mocks/driver_mocks.h"
#include "../shared/messages.h"


/* Set up mock skeleton_t struct. */
hal_u32_t seq_in;
hal_u32_t seq_out;
hal_s32_t packet_interval;

hal_bit_t joint_enable_cmd[4];
hal_float_t joint_vel_limit[4];
hal_float_t joint_accel_limit[4];
hal_float_t joint_pos_fb[4];
hal_float_t joint_pos_cmd[4];
hal_float_t joint_scale[4] = {1000, 1000, 1000, 1000};
hal_float_t joint_vel_fb[4];
hal_s32_t joint_pos_error_fb[4];
hal_bit_t joint_enable_fb[4];
hal_float_t joint_vel_calculated[4];
hal_u32_t core1_period;
hal_u32_t core1_tick;
hal_u32_t core1_work_us;
hal_u32_t core0_work_us;
hal_float_t update_overrun;
hal_float_t update_underrun;

hal_float_t spindle_speed_fb[MAX_SPINDLE];
hal_float_t spindle_speed_cmd[MAX_SPINDLE];
hal_bit_t   spindle_at_speed[MAX_SPINDLE];

void setup_data(skeleton_t* data) {
  data->seq_in = &seq_in;
  data->seq_out = &seq_out;
  data->packet_interval = &packet_interval;

  for(size_t joint = 0; joint < MAX_JOINT; joint++) {
    data->joint_enable_cmd[joint] = &(joint_enable_cmd[joint]);
    data->joint_vel_limit[joint] = &(joint_vel_limit[joint]);
    data->joint_accel_limit[joint] = &(joint_accel_limit[joint]);
    data->joint_pos_fb[joint] = &(joint_pos_fb[joint]);
    data->joint_pos_cmd[joint] = &(joint_pos_cmd[joint]);
    data->joint_scale[joint] = &(joint_scale[joint]);
    data->joint_vel_fb[joint] = &(joint_vel_fb[joint]);
    data->joint_pos_error_fb[joint] = &(joint_pos_error_fb[joint]);
    data->joint_enable_fb[joint]      = &(joint_enable_fb[joint]);
    data->joint_vel_calculated[joint] = &(joint_vel_calculated[joint]);
  }
  data->update_overrun  = &update_overrun;
  data->update_underrun = &update_underrun;
  data->core1_period    = &core1_period;
  data->core1_tick      = &core1_tick;
  data->core1_work_us   = &core1_work_us;
  data->core0_work_us   = &core0_work_us;

  for (size_t s = 0; s < MAX_SPINDLE; s++) {
    data->spindle_speed_fb[s]  = &spindle_speed_fb[s];
    data->spindle_speed_cmd[s] = &spindle_speed_cmd[s];
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
    buffer.length = aligned32(sizeof(message));
    buffer.checksum = checksum(0, 0, buffer.length, buffer.payload);

    process_data(
            &buffer,
            &data,
            &mess_received_count,
            aligned32(sizeof(message)) + sizeof(buffer.length) + sizeof(buffer.checksum),
            NULL,
            NULL,
            NULL
            );


    assert_int_equal(*(data.seq_in), message.update_id);
    assert_int_equal(*(data.packet_interval), message.time_diff);
}

static void test_joint_movement(void **state) {
    (void) state; /* unused */

    struct NWBuffer buffer = {0};
    size_t mess_received_count = 0;

    skeleton_t data = {0};
    setup_data(&data);
   
    struct Reply_joint_movement message = {
        .type              = REPLY_JOINT_MOVEMENT,
        .count             = MAX_JOINT,
        .abs_pos_achieved  = {1234, 5678, 9, 10},
        .velocity_achieved = {7890, 1234, 5, 6},
        .enabled           = {1, 0, 1, 0},
        .velocity_cmd      = {100.5f, 200.5f, 300.5f, 400.5f},
        .update_period_us  = 1000,
        .core1_tick        = 9999,
    };

    memcpy(buffer.payload, &message, sizeof(message));
    buffer.length = aligned32(sizeof(message));
    buffer.checksum = checksum(0, 0, buffer.length, buffer.payload);

    process_data(
            &buffer,
            &data,
            &mess_received_count,
            aligned32(sizeof(message)) + sizeof(buffer.length) + sizeof(buffer.checksum),
            NULL,
            NULL,
            NULL
            );

    for(size_t joint = 0; joint < MAX_JOINT; joint++) {
        assert_double_equal(
                (*data.joint_pos_fb)[joint],
                (double)message.abs_pos_achieved[joint] / (*data.joint_scale)[joint],
                0.0001);
        assert_double_equal(
                (*data.joint_vel_fb)[joint],
                (double)message.velocity_achieved[joint] / 65536.0,
                0.0001);
        assert_int_equal(*data.joint_enable_fb[joint], message.enabled[joint]);
        assert_double_equal(*data.joint_vel_calculated[joint], message.velocity_cmd[joint], 0.001);
    }
    assert_int_equal(*data.core1_period, message.update_period_us);
    assert_int_equal(*data.core1_tick, message.core1_tick);
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
        .cmd_type = JOINT_CMD_VELOCITY,
        .max_velocity = 56.78,
        .max_accel = 90.12,
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
    assert_int_equal(last_joint_config.cmd_type, message.cmd_type);
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
        .overrun_occurred  = 1,
        .underrun_occurred = 1,
    };

    memcpy(buffer.payload, &message, sizeof(message));
    buffer.length = aligned32(sizeof(struct Reply_joint_metrics));
    buffer.checksum = checksum(0, 0, buffer.length, buffer.payload);

    process_data(
            &buffer,
            &data,
            &mess_received_count,
            buffer.length + sizeof(buffer.length) + sizeof(buffer.checksum),
            NULL,
            NULL,
            NULL
            );
}

/* EMA pins reflect combined overrun/underrun across all joints. */
static void test_joint_metrics_ema_ratios(void **state) {
    (void) state;

    struct NWBuffer buffer = {0};
    size_t mess_received_count = 0;

    update_overrun  = 0.0;
    update_underrun = 0.0;

    skeleton_t data = {0};
    setup_data(&data);

    /* Seed known EMA state: 2.0 overrun, 2.0 underrun → ratio should be 0.5 */
    data.ema_overrun  = 2.0;
    data.ema_underrun = 2.0;

    /* Send message with overrun_occurred=1, underrun_occurred=0. */
    struct Reply_joint_metrics message = {
        .type              = REPLY_JOINT_METRICS,
        .overrun_occurred  = 1,
        .underrun_occurred = 0,
    };
    memcpy(buffer.payload, &message, sizeof(message));
    buffer.length    = aligned32(sizeof(struct Reply_joint_metrics));
    buffer.checksum  = checksum(0, 0, buffer.length, buffer.payload);
    process_data(&buffer, &data, &mess_received_count,
                 buffer.length + sizeof(buffer.length) + sizeof(buffer.checksum),
                 NULL, NULL, NULL);

    /* After one EMA step with overrun=1, underrun=0:
     *   ema_overrun  = 2.0 * (1 - α) + 1 * α  (α = 1/1000, so ≈ 1.999)
     *   ema_underrun = 2.0 * (1 - α) + 0 * α  (≈ 1.998)
     * Both ratios reflect their respective EMA values. */
    assert_true(*data.update_overrun > 0.0);
    assert_true(*data.update_underrun > 0.0);

    /* With ema_underrun = 0 and zero underrun counts, underrun_ratio → 0. */
    data.ema_overrun  = 1000.0;
    data.ema_underrun = 0.0;
    message.overrun_occurred  = 1;
    message.underrun_occurred = 0;
    memcpy(buffer.payload, &message, sizeof(message));
    buffer.checksum = checksum(0, 0, buffer.length, buffer.payload);
    process_data(&buffer, &data, &mess_received_count,
                 buffer.length + sizeof(buffer.length) + sizeof(buffer.checksum),
                 NULL, NULL, NULL);
    assert_true(*data.update_underrun < 0.01);

    /* With both EMA zero and zero counts, both ratios should be 0. */
    data.ema_overrun  = 0.0;
    data.ema_underrun = 0.0;
    struct Reply_joint_metrics zero_msg = { .type = REPLY_JOINT_METRICS };
    memcpy(buffer.payload, &zero_msg, sizeof(zero_msg));
    buffer.length   = 4;
    buffer.checksum = checksum(0, 0, buffer.length, buffer.payload);
    process_data(&buffer, &data, &mess_received_count,
                 buffer.length + sizeof(buffer.length) + sizeof(buffer.checksum),
                 NULL, NULL, NULL);
    assert_float_equal(*data.update_underrun, 0.0, 1e-9);
}

static void test_unpack_spindle_speed(void **state) {
    (void) state;

    skeleton_t data = {0};
    memset(spindle_speed_fb,  0, sizeof(spindle_speed_fb));
    memset(spindle_speed_cmd, 0, sizeof(spindle_speed_cmd));
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
    *data.spindle_speed_cmd[0] = 1500.0;

    memcpy(buffer.payload, &reply, sizeof(reply));
    buffer.length = aligned32(sizeof(reply));

    bool result = unpack_spindle_speed(&buffer, &rx_offset, &received_count, &data);

    assert_true(result);
    assert_int_equal(received_count, 1);
    assert_double_equal(*data.spindle_speed_fb[0], 1500.0, 0.01);
    assert_true(*data.spindle_at_speed[0]);  /* |1500 - 1500| < 10 */
    assert_int_equal(rx_offset, aligned32(sizeof(reply)));
}

static void test_unpack_spindle_not_at_speed(void **state) {
    (void) state;

    skeleton_t data = {0};
    memset(spindle_speed_fb,  0, sizeof(spindle_speed_fb));
    memset(spindle_speed_cmd, 0, sizeof(spindle_speed_cmd));
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
    *data.spindle_speed_cmd[0] = 1520.0;

    memcpy(buffer.payload, &reply, sizeof(reply));
    buffer.length = aligned32(sizeof(reply));

    bool result = unpack_spindle_speed(&buffer, &rx_offset, &received_count, &data);

    assert_true(result);
    assert_int_equal(received_count, 1);
    assert_double_equal(*data.spindle_speed_fb[0], 1500.0, 0.01);
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
    buffer.length = 8;  /* sizeof(reply) = 6, aligned32(6) = 8 */

    bool result = unpack_spindle_config(&buffer, &rx_offset, &received_count, last_spindle_config);

    assert_true(result);
    assert_int_equal(received_count, 1);
    assert_int_equal(rx_offset, 8);  /* aligned32(sizeof(reply)) = 8 */
    assert_int_equal(last_spindle_config[0].modbus_address, 2);
    assert_int_equal(last_spindle_config[0].vfd_type, 1);
    assert_int_equal(last_spindle_config[0].bitrate, 19200);
}

static void reset_version_state(void) {
    version_checked = false;
    version_match   = false;
}

static void test_version__matching__ok(void **state) {
    (void)state;
    reset_version_state();

    struct NWBuffer buffer = {0};
    size_t rx_offset = 0;
    size_t received_count = 0;

    struct Reply_version reply = {
        .type           = REPLY_VERSION,
        .version_major  = PROTOCOL_VERSION_MAJOR,
        .version_minor  = PROTOCOL_VERSION_MINOR,
        .version_patch  = PROTOCOL_VERSION_PATCH,
        .version_branch = PROTOCOL_VERSION_BRANCH,
    };
    memcpy(buffer.payload, &reply, sizeof(reply));
    buffer.length = aligned32(sizeof(reply));

    bool result = unpack_version_reply(&buffer, &rx_offset, &received_count);

    assert_true(result);
    assert_true(version_checked);
    assert_true(get_version_match());
    assert_int_equal(received_count, 1);
    assert_int_equal(rx_offset, aligned32(sizeof(reply)));
}

static void test_version__patch_mismatch__not_ok(void **state) {
    (void)state;
    reset_version_state();

    struct NWBuffer buffer = {0};
    size_t rx_offset = 0;
    size_t received_count = 0;

    struct Reply_version reply = {
        .type           = REPLY_VERSION,
        .version_major  = PROTOCOL_VERSION_MAJOR,
        .version_minor  = PROTOCOL_VERSION_MINOR,
        .version_patch  = PROTOCOL_VERSION_PATCH + 1,
        .version_branch = PROTOCOL_VERSION_BRANCH,
    };
    memcpy(buffer.payload, &reply, sizeof(reply));
    buffer.length = aligned32(sizeof(reply));

    bool result = unpack_version_reply(&buffer, &rx_offset, &received_count);

    assert_true(result);
    assert_true(version_checked);
    assert_false(get_version_match());
    assert_int_equal(received_count, 1);
}

static void test_version__branch_mismatch__not_ok(void **state) {
    (void)state;
    reset_version_state();

    struct NWBuffer buffer = {0};
    size_t rx_offset = 0;
    size_t received_count = 0;

    struct Reply_version reply = {
        .type           = REPLY_VERSION,
        .version_major  = PROTOCOL_VERSION_MAJOR,
        .version_minor  = PROTOCOL_VERSION_MINOR,
        .version_patch  = PROTOCOL_VERSION_PATCH,
        .version_branch = PROTOCOL_VERSION_BRANCH + 1,
    };
    memcpy(buffer.payload, &reply, sizeof(reply));
    buffer.length = aligned32(sizeof(reply));

    bool result = unpack_version_reply(&buffer, &rx_offset, &received_count);

    assert_true(result);
    assert_true(version_checked);
    assert_false(get_version_match());
    assert_int_equal(received_count, 1);
}

static void test_version__already_checked__skips_second_check(void **state) {
    (void)state;
    reset_version_state();

    struct NWBuffer buffer = {0};
    size_t rx_offset = 0;
    size_t received_count = 0;

    /* First reply: matching. */
    struct Reply_version reply = {
        .type           = REPLY_VERSION,
        .version_major  = PROTOCOL_VERSION_MAJOR,
        .version_minor  = PROTOCOL_VERSION_MINOR,
        .version_patch  = PROTOCOL_VERSION_PATCH,
        .version_branch = PROTOCOL_VERSION_BRANCH,
    };
    memcpy(buffer.payload, &reply, sizeof(reply));
    buffer.length = aligned32(sizeof(reply));
    unpack_version_reply(&buffer, &rx_offset, &received_count);
    assert_true(get_version_match());

    /* Second reply: mismatching — should be ignored since already checked. */
    rx_offset = 0;
    reply.version_patch = PROTOCOL_VERSION_PATCH + 99;
    memcpy(buffer.payload, &reply, sizeof(reply));
    unpack_version_reply(&buffer, &rx_offset, &received_count);

    assert_true(get_version_match());  /* still true — second check was skipped */
    assert_int_equal(received_count, 2);
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_timing),
        cmocka_unit_test(test_joint_movement),
        cmocka_unit_test(test_joint_config),
        cmocka_unit_test(test_joint_metrics),
        cmocka_unit_test(test_joint_metrics_ema_ratios),
        cmocka_unit_test(test_unpack_spindle_speed),
        cmocka_unit_test(test_unpack_spindle_not_at_speed),
        cmocka_unit_test(test_unpack_spindle_config),
        cmocka_unit_test(test_version__matching__ok),
        cmocka_unit_test(test_version__patch_mismatch__not_ok),
        cmocka_unit_test(test_version__branch_mismatch__not_ok),
        cmocka_unit_test(test_version__already_checked__skips_second_check),
    };

    return cmocka_run_group_tests(tests, NULL, NULL);
}

