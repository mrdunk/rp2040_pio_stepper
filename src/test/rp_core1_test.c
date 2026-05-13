#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <stdint.h>
#include <cmocka.h>

#include "../rp2040/core1.h"
#include "../rp2040/config.h"

/* tick and last_packet_tick are defined in config.c (which is linked).
 * config.h provides the extern declarations — no re-definition needed here.
 * Tests can write to them directly to control Core1 behaviour. */

/* Intercept disable_joint() to count calls. */
static int disable_joint_call_count = 0;

void __wrap_disable_joint(const uint8_t joint, const uint8_t core) {
    (void)joint;
    (void)core;
    disable_joint_call_count++;
}

/* Intercept do_steps() to count calls. */
static int do_steps_call_count = 0;

uint8_t __wrap_do_steps(const uint8_t joint) {
    (void)joint;
    do_steps_call_count++;
    return 1;
}

/* Reset all state before each test.
 * tick and last_packet_tick are extern from config.h — write directly. */
static int test_setup(void **state) {
    (void)state;
    tick                        = 0;
    last_packet_tick            = 0;
    packet_generation           = 1;  /* ahead of last_packet_generation (0) */
    linuxcnc_restart_detected   = false;
    disable_joint_call_count    = 0;
    do_steps_call_count         = 0;
    core1_reset_for_test();
    return 0;
}

/* wait_for_packet: exits immediately when tick has advanced and generation has advanced. */
static void test_wait_for_packet_returns_when_generation_advances(void **state) {
    (void)state;
    tick              = 1;   /* tick (1) != last_tick (0) */
    last_packet_tick  = 1;   /* network healthy */
    /* packet_generation = 1 from test_setup; last_packet_generation = 0 from reset */
    wait_for_packet();
    /* Reaching this line proves the function returned without spinning forever. */
}

/* wait_for_packet: exits via network-loss fast-path when no packet for too long. */
static void test_wait_for_packet_returns_on_network_loss(void **state) {
    (void)state;
    tick              = MAX_MISSED_PACKET + 2;
    last_packet_tick  = 0;
    packet_generation = 0;  /* same as last_packet_generation — generation never advanced */
    wait_for_packet();
    /* Reaching this line proves the network-loss fallback fired. */
}

/* check_network_health: healthy when gap <= MAX_MISSED_PACKET. */
static void test_check_network_health_ok(void **state) {
    (void)state;
    tick             = 5;
    last_packet_tick = 5;
    assert_true(check_network_health());
}

/* check_network_health: healthy at the exact boundary. */
static void test_check_network_health_at_limit(void **state) {
    (void)state;
    tick             = MAX_MISSED_PACKET;
    last_packet_tick = 0;
    assert_true(check_network_health());
}

/* check_network_health: unhealthy one tick beyond the boundary. */
static void test_check_network_health_loss(void **state) {
    (void)state;
    tick             = MAX_MISSED_PACKET + 1;
    last_packet_tick = 0;
    assert_false(check_network_health());
}

/* handle_network_timeout: disables all MAX_JOINT joints on first call. */
static void test_handle_network_timeout_disables_all_joints(void **state) {
    (void)state;
    handle_network_timeout();
    assert_int_equal(MAX_JOINT, disable_joint_call_count);
}

/* handle_network_timeout: does NOT disable joints on second call (no_network debounce). */
static void test_handle_network_timeout_disables_once_per_outage(void **state) {
    (void)state;
    handle_network_timeout();
    handle_network_timeout();
    /* Should still be MAX_JOINT, not 2 * MAX_JOINT. */
    assert_int_equal(MAX_JOINT, disable_joint_call_count);
}

/* handle_network_recovery: clears the no_network flag so the next timeout
 * re-arms and disables joints again. */
static void test_handle_network_recovery_re_arms_timeout(void **state) {
    (void)state;
    handle_network_timeout();   /* sets no_network, disables MAX_JOINT joints */
    handle_network_recovery();  /* clears no_network */
    handle_network_timeout();   /* should disable joints again */
    assert_int_equal(MAX_JOINT * 2, disable_joint_call_count);
}

/* step_all_joints: calls do_steps() exactly MAX_JOINT times. */
static void test_step_all_joints_calls_do_steps_for_each_joint(void **state) {
    (void)state;
    step_all_joints();
    assert_int_equal(MAX_JOINT, do_steps_call_count);
}

/* core1_main loop: when linuxcnc_restart_detected is set, one iteration must
 * disable all joints, clear the flag, and still call step_all_joints so motors
 * can decelerate. */
static void test_core1_disables_joints_on_linuxcnc_restart(void **state) {
    (void)state;
    linuxcnc_restart_detected = true;
    tick             = 1;   /* tick (1) != last_tick (0) so wait_for_packet() returns */
    last_packet_tick = 1;   /* network appears healthy; restart flag should fire first */
    core1_run_once_for_test();
    assert_int_equal(MAX_JOINT, disable_joint_call_count);
    assert_int_equal(MAX_JOINT, do_steps_call_count);  /* step_all_joints always runs */
    assert_false(linuxcnc_restart_detected);
}

/* core1_main loop: when both linuxcnc_restart_detected and network health check
 * would fail, the restart path (else if) must take precedence. This ensures
 * handle_network_timeout() is called exactly once, not twice. */
static void test_core1_restart_while_network_unhealthy(void **state) {
    (void)state;
    linuxcnc_restart_detected = true;
    tick             = MAX_MISSED_PACKET + 2;  /* tick advanced so wait_for_packet() exits */
    last_packet_tick = 0;                      /* network genuinely unhealthy: gap > MAX_MISSED_PACKET */
    core1_run_once_for_test();
    /* The else if ensures only MAX_JOINT joints disabled, not 2 * MAX_JOINT */
    assert_int_equal(MAX_JOINT, disable_joint_call_count);
    assert_false(linuxcnc_restart_detected);
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test_setup(test_wait_for_packet_returns_when_generation_advances, test_setup),
        cmocka_unit_test_setup(test_wait_for_packet_returns_on_network_loss,          test_setup),
        cmocka_unit_test_setup(test_check_network_health_ok,                 test_setup),
        cmocka_unit_test_setup(test_check_network_health_at_limit,           test_setup),
        cmocka_unit_test_setup(test_check_network_health_loss,               test_setup),
        cmocka_unit_test_setup(test_handle_network_timeout_disables_all_joints,      test_setup),
        cmocka_unit_test_setup(test_handle_network_timeout_disables_once_per_outage, test_setup),
        cmocka_unit_test_setup(test_handle_network_recovery_re_arms_timeout,         test_setup),
        cmocka_unit_test_setup(test_step_all_joints_calls_do_steps_for_each_joint,   test_setup),
        cmocka_unit_test_setup(test_core1_disables_joints_on_linuxcnc_restart,       test_setup),
        cmocka_unit_test_setup(test_core1_restart_while_network_unhealthy,           test_setup),
    };

    return cmocka_run_group_tests(tests, NULL, NULL);
}
