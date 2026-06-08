#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <stdint.h>
#include <string.h>
#include <cmocka.h>

#include "mocks/driver_mocks.h"
#include "../shared/messages.h"
#include "../shared/buffer.c"
#include "../shared/checksum.c"

/* ---- stub globals used by rp2040_eth_state.c ---- */

/* log_network_error() needs remote_addr (defined in rp2040_network.c in production). */
#include <netdb.h>
struct sockaddr_in remote_addr[MAX_DEVICES];

/* Controllable mock state for send_data / get_reply_non_block. */
static int    g_send_retval     = 0;
static int    g_send_call_count = 0;
static size_t g_reply_length    = 0;
static int    g_reply_call_count= 0;

/* ---- stub implementations of rp2040_network.c symbols ---- */
/* reset_nw_buf is provided by buffer.c (included above). */

size_t serialize_timing(struct NWBuffer *b, uint32_t id, uint32_t t) {
    (void)b; (void)id; (void)t; return 1;
}
bool get_version_checked(void) { return true; }
size_t serialize_version_request(struct NWBuffer *b) { (void)b; return 1; }
uint16_t serialize_gpio(struct NWBuffer *b, skeleton_t *d) { (void)b; (void)d; return 0; }
uint8_t get_detected_joint_count(void) { return 0; }
size_t serialize_joint_config(struct NWBuffer *b, uint8_t j, uint8_t e,
                               uint8_t s, uint8_t dr, float v, float a, uint8_t c) {
    (void)b; (void)j; (void)e; (void)s; (void)dr; (void)v; (void)a; (void)c; return 1;
}
size_t serialize_gpio_config(struct NWBuffer *b, uint8_t g, uint8_t t,
                              uint8_t i, uint8_t addr) {
    (void)b; (void)g; (void)t; (void)i; (void)addr; return 1;
}
bool serialise_spindle_config(struct NWBuffer *b, uint8_t s, uint8_t v,
                               uint8_t addr, uint16_t r) {
    (void)b; (void)s; (void)v; (void)addr; (void)r; return true;
}
size_t serialize_joint_pos(struct NWBuffer *b, skeleton_t *d) { (void)b; (void)d; return 1; }
bool serialise_spindle_speed_in(struct NWBuffer *b, skeleton_t *d) { (void)b; (void)d; return true; }

void process_data(struct NWBuffer *b, skeleton_t *d, size_t *c, size_t l,
                  struct Message_joint_config *jc, struct Message_gpio_config *gc,
                  struct Message_spindle_config *sc) {
    (void)b; (void)d; (void)c; (void)l; (void)jc; (void)gc; (void)sc;
}
void reset_version_check(void) {}

int send_data(int dev, struct NWBuffer *buf) {
    (void)dev; (void)buf;
    g_send_call_count++;
    return g_send_retval;
}
size_t get_reply_non_block(int dev, struct NWBuffer *buf) {
    (void)dev; (void)buf;
    g_reply_call_count++;
    return g_reply_length;
}

/* ---- code under test ---- */
#include "../driver/rp2040_eth_state.c"


/* ---- test skeleton_t setup ---- */

static hal_bit_t   v_eth_up;
static hal_bit_t   v_machine_on;
static hal_u32_t   v_rx_miss_count;
static hal_u32_t   v_seq_out;
static hal_u32_t   v_seq_in;
static hal_bit_t   v_config_complete;
static hal_bit_t   v_joint_enable_cmd[MAX_JOINT];
static hal_float_t v_joint_vel_fb[MAX_JOINT];
static hal_float_t v_joint_vel_limit[MAX_JOINT];
static hal_float_t v_joint_accel_limit[MAX_JOINT];
static hal_float_t v_joint_scale[MAX_JOINT];
static hal_float_t v_joint_pos_cmd[MAX_JOINT];
static hal_float_t v_joint_pos_fb[MAX_JOINT];
static hal_float_t v_joint_vel_cmd[MAX_JOINT];
static hal_float_t v_joint_ferror_suggest[MAX_JOINT];
static hal_float_t v_joint_vel_calculated[MAX_JOINT];
static hal_s32_t   v_joint_pos_error_fb[MAX_JOINT];
static hal_bit_t   v_joint_enable_fb[MAX_JOINT];

static skeleton_t make_data(void) {
    memset(&v_eth_up, 0, sizeof(v_eth_up));
    memset(&v_machine_on, 0, sizeof(v_machine_on));
    memset(&v_rx_miss_count, 0, sizeof(v_rx_miss_count));
    memset(&v_seq_out, 0, sizeof(v_seq_out));
    memset(&v_seq_in, 0, sizeof(v_seq_in));
    memset(&v_config_complete, 0, sizeof(v_config_complete));
    memset(v_joint_enable_cmd, 0, sizeof(v_joint_enable_cmd));
    memset(v_joint_vel_fb, 0, sizeof(v_joint_vel_fb));
    memset(v_joint_vel_limit, 0, sizeof(v_joint_vel_limit));
    memset(v_joint_accel_limit, 0, sizeof(v_joint_accel_limit));
    memset(v_joint_scale, 0, sizeof(v_joint_scale));
    memset(v_joint_pos_cmd, 0, sizeof(v_joint_pos_cmd));
    memset(v_joint_pos_fb, 0, sizeof(v_joint_pos_fb));
    memset(v_joint_vel_cmd, 0, sizeof(v_joint_vel_cmd));

    skeleton_t d = {0};
    d.eth_up          = &v_eth_up;
    d.machine_on      = &v_machine_on;
    d.rx_miss_count   = &v_rx_miss_count;
    d.seq_out         = &v_seq_out;
    d.seq_in          = &v_seq_in;
    d.config_complete = &v_config_complete;
    for(int i = 0; i < MAX_JOINT; i++) {
        d.joint_enable_cmd[i]     = &v_joint_enable_cmd[i];
        d.joint_vel_fb[i]         = &v_joint_vel_fb[i];
        d.joint_vel_limit[i]      = &v_joint_vel_limit[i];
        d.joint_accel_limit[i]    = &v_joint_accel_limit[i];
        d.joint_scale[i]          = &v_joint_scale[i];
        d.joint_pos_cmd[i]        = &v_joint_pos_cmd[i];
        d.joint_pos_fb[i]         = &v_joint_pos_fb[i];
        d.joint_vel_cmd[i]        = &v_joint_vel_cmd[i];
        d.joint_ferror_suggest[i] = &v_joint_ferror_suggest[i];
        d.joint_vel_calculated[i] = &v_joint_vel_calculated[i];
        d.joint_pos_error_fb[i]   = &v_joint_pos_error_fb[i];
        d.joint_enable_fb[i]      = &v_joint_enable_fb[i];
        d.joint_gpio_step[i] = -1;
        d.joint_gpio_dir[i]  = -1;
    }
    for(int g = 0; g < MAX_GPIO; g++) {
        d.gpio_type[g] = GPIO_TYPE_NOT_SET;
    }
    for(int s = 0; s < MAX_SPINDLE; s++) {
        d.spindle_vfd_type[s] = MODBUS_TYPE_NOT_SET;
    }
    return d;
}

static void reset_mocks(void) {
    g_send_retval      = 0;
    g_send_call_count  = 0;
    g_reply_length     = 0;
    g_reply_call_count = 0;
    eth_state_reset();
}


/* ---- test cases ---- */

/* eth_up goes false after MAX_SKIPPED_PACKETS consecutive rx misses (cable unplug). */
static void test_cable_unplug_sets_eth_down(void **state) {
    (void)state;
    reset_mocks();
    skeleton_t data = make_data();
    *data.eth_up = true;
    g_reply_length = 0;   /* no replies — simulates unplugged cable */

    for(int i = 0; i <= MAX_SKIPPED_PACKETS; i++) {
        eth_state_update(&data, 0, i, 0, 1);
    }
    assert_false(*data.eth_up);
}

/* eth_up goes false after MAX_SKIPPED_PACKETS iterations where send_data() fails.
 * Root cause of original bug: the old code returned early on send failure before
 * incrementing rx_miss_count, so eth_up never went false on interface-down. */
static void test_send_failure_sets_eth_down(void **state) {
    (void)state;
    reset_mocks();
    skeleton_t data = make_data();
    *data.eth_up = true;
    g_send_retval  = -1;   /* send always fails */
    g_reply_length = 0;    /* no replies */

    for(int i = 0; i <= MAX_SKIPPED_PACKETS; i++) {
        eth_state_update(&data, 0, i, 0, 1);
    }
    assert_false(*data.eth_up);
}

/* After send_data() fails, sends are skipped for 2000 periods but
 * get_reply_non_block still runs every period. */
static void test_cooloff_skips_send_but_not_receive(void **state) {
    (void)state;
    reset_mocks();
    skeleton_t data = make_data();
    *data.eth_up = false;  /* keep eth down so on_eth_down doesn't trigger */
    g_send_retval = -1;

    /* First call: send fires and fails → cooloff = 2000. */
    eth_state_update(&data, 0, 0, 0, 1);
    assert_int_equal(g_send_call_count, 1);

    /* Reset counters; run 2000 more cycles. */
    g_send_call_count  = 0;
    g_reply_call_count = 0;

    for(int i = 1; i <= 2000; i++) {
        eth_state_update(&data, 0, i, 0, 1);
    }
    assert_int_equal(g_send_call_count, 0);       /* no sends during cooloff */
    assert_int_equal(g_reply_call_count, 2000);   /* receive runs every period */
}

/* on_eth_up fires only when all joints report vel_fb == 0.0 AND
 * last_joint_config[joint].enable == false. */
static void test_recovery_waits_for_all_stopped(void **state) {
    (void)state;
    reset_mocks();
    skeleton_t data = make_data();
    *data.eth_up = false;
    g_reply_length = 1;   /* fake a received packet so the recovery path runs */

    /* While joint is still moving, recovery must not fire. */
    v_joint_vel_fb[0] = 1.0;
    eth_state_update(&data, 0, 0, 0, 1);
    assert_false(*data.eth_up);

    /* Once joint has stopped (and last_joint_config[0].enable is false after reset),
     * the next packet received triggers on_eth_up. */
    v_joint_vel_fb[0] = 0.0;
    eth_state_update(&data, 0, 1, 0, 1);
    assert_true(*data.eth_up);
    assert_true(*data.machine_on);
}

/* While eth_up is false, joint_enable_cmd is overridden to false every period
 * regardless of what LinuxCNC wrote to the HAL pin. */
static void test_force_disable_while_eth_down(void **state) {
    (void)state;
    reset_mocks();
    skeleton_t data = make_data();
    *data.eth_up = false;
    v_joint_enable_cmd[0] = true;   /* LinuxCNC tries to enable the joint */

    eth_state_update(&data, 0, 0, 0, 1);

    assert_false(*data.joint_enable_cmd[0]);
}


/* Recovery requires ALL joints stopped: one moving joint blocks machine-on.
 *
 * The existing test covers a single joint.  This test verifies that with two
 * joints, recovery stays blocked as long as any joint still reports vel_fb != 0.0,
 * and fires only when both are stopped. */
static void test_recovery_requires_all_joints_stopped_multi_joint(void **state) {
    (void)state;
    reset_mocks();
    skeleton_t data = make_data();
    *data.eth_up   = false;
    g_reply_length = 1;

    /* Joint 0 stopped, joint 1 still moving. */
    v_joint_vel_fb[0] = 0.0;
    v_joint_vel_fb[1] = 5.0;
    eth_state_update(&data, 0, 0, 0, 2);  /* num_joints=2 */
    assert_false(*data.eth_up);

    /* Both stopped: recovery fires. */
    v_joint_vel_fb[1] = 0.0;
    eth_state_update(&data, 0, 1, 0, 2);
    assert_true(*data.eth_up);
    assert_true(*data.machine_on);
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_cable_unplug_sets_eth_down),
        cmocka_unit_test(test_send_failure_sets_eth_down),
        cmocka_unit_test(test_cooloff_skips_send_but_not_receive),
        cmocka_unit_test(test_recovery_waits_for_all_stopped),
        cmocka_unit_test(test_recovery_requires_all_joints_stopped_multi_joint),
        cmocka_unit_test(test_force_disable_while_eth_down),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
