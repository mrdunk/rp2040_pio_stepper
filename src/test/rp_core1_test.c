#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <stdint.h>
#include <cmocka.h>
#include <stdio.h>

#include "../rp2040/core1.h"
#include "../rp2040/pio.h"
#include "../rp2040/config.h"


extern volatile struct ConfigGlobal config;
extern uint32_t sm0[MAX_JOINT];
extern uint32_t sm1[MAX_JOINT];

uint8_t __real_do_steps(
        const uint8_t joint,
        const uint32_t update_period_us
    );

uint8_t __wrap_do_steps(
        const uint8_t joint,
        const uint32_t update_period_us
    ) {
    //printf("__wrap_do_steps(%u, %u)\n",
    //        joint, update_period_us);
    check_expected(joint);
    check_expected(update_period_us);

    return mock_type(uint8_t);
}

void __wrap_disable_joint(
        const uint8_t joint,
        const uint8_t core
    ) {
    //printf("__wrap_disable_joint(%u, %u)\n", joint, core);
    check_expected(joint);
    check_expected(core);
}

int __wrap_get_period() {
    //printf("__wrap_get_period()\n");
    return mock_type(int);
}

static size_t values_time_us_64[1000] = {1000,2000,3000,4000};
static size_t count_time_us_64 = 0;
size_t __wrap_time_us_64() {
    //printf("__wrap_time_us_64()\n");
    return values_time_us_64[count_time_us_64++];
}


uint8_t get_joint_config__enabled = 1;
double get_joint_config__velocity_requested = 0;
double get_joint_config__abs_pos_requested = 1000;
double get_joint_config__abs_pos_achieved = 1000;
double get_joint_config__max_velocity = 10;
double get_joint_config__max_accel = 1;
uint32_t __wrap_get_joint_config(
    const uint8_t joint,
    const uint8_t core,
    uint8_t* enabled,
    int8_t* io_pos_step,
    int8_t* io_pos_dir,
    double* velocity_requested,
    double* abs_pos_requested,
    int32_t* abs_pos_achieved,
    double* max_velocity,
    double* max_accel,
    int32_t* velocity_requested_tm1,
    int32_t* velocity_achieved,
    int32_t* step_len_ticks,
    int32_t* position_error)
{
    *enabled = get_joint_config__enabled;
    *velocity_requested = get_joint_config__velocity_requested;
    *abs_pos_requested = get_joint_config__abs_pos_requested;
    *abs_pos_achieved = get_joint_config__abs_pos_achieved;
    *max_velocity = get_joint_config__max_velocity;
    *max_accel = get_joint_config__max_accel;

    return mock_type(uint32_t);
}

void __wrap_init_pio(const uint32_t joint) {
    //printf("__wrap_init_pio(%u)\n", joint);
    check_expected(joint);
}

void __wrap_stop_joint(const uint32_t joint) {
    //printf("__wrap_stop_joint(%u)\n", joint);
    check_expected(joint);
}

uint32_t __wrap_pio_sm_get_rx_fifo_level(uint32_t pio, uint32_t sm) {
    check_expected(pio);
    check_expected(sm);
    return mock_type(uint32_t);
}


uint32_t __wrap_pio_sm_get_blocking(uint32_t pio, uint32_t sm) {
    check_expected(pio);
    check_expected(sm);
    return mock_type(uint32_t);
}

void __wrap_update_joint_config(
    const uint8_t joint,
    const uint8_t core,
    const uint8_t* enabled,
    const int8_t* io_pos_step,
    const int8_t* io_pos_dir,
    const double* velocity_requested,
    const double* abs_pos_requested,
    const int32_t* abs_pos_achieved,
    const double* max_velocity,
    const double* max_accel,
    const int32_t* velocity_requested_tm1,
    const int32_t* velocity_achieved,
    const int32_t* step_len_ticks,
    const int32_t* position_error
) {
    assert_int_equal(core, 1);
    check_expected(joint);
    check_expected_ptr(abs_pos_achieved);
}


/*** Tests ***/

static void test_packet_timeout(void **state) {
    (void) state; /* unused */

    // Overrides for time_us_64() calls.
    count_time_us_64 = 0;
    values_time_us_64[0] = 1000;
    values_time_us_64[1] = 2000;
    values_time_us_64[2] = 3000;
    values_time_us_64[3] = 4000;
    values_time_us_64[4] = 5000;
    values_time_us_64[5] = 6000;
    values_time_us_64[6] = 7000;
    values_time_us_64[7] = 8000;
    values_time_us_64[8] = 9000;
    values_time_us_64[9] = 10000;
    values_time_us_64[10] = 11000;

    will_return(__wrap_get_period, 1000);

    // Same tick value as last time.
    tick = 0;

    // Disables all joints.
    expect_value(__wrap_disable_joint, joint, 0);
    expect_value(__wrap_disable_joint, joint, 1);
    expect_value(__wrap_disable_joint, joint, 2);
    expect_value(__wrap_disable_joint, joint, 3);
    expect_value_count(__wrap_disable_joint, core, 1, 4);

    // Do steps.
    // Since joints are disabled, will be setting steps to 0.
    expect_value(__wrap_do_steps, joint, 0);
    expect_value(__wrap_do_steps, joint, 1);
    expect_value(__wrap_do_steps, joint, 2);
    expect_value(__wrap_do_steps, joint, 3);
    expect_value_count(
            __wrap_do_steps, update_period_us, 1000, 4);
    will_return_always(__wrap_do_steps, 1);

    // Call code being tested.
    update_all_joint();

    assert_in_range(count_time_us_64,
            MAX_MISSED_PACKET, MAX_MISSED_PACKET * 2);
}

static void test_packet_received(void **state) {
    (void) state; /* unused */

    will_return(__wrap_get_period, 1000);

    // Overrides for time_us_64() call.
    count_time_us_64 = 0;

    // tick value has changed since last time.
    tick = 1;

    // Do steps.
    expect_value(__wrap_do_steps, joint, 0);
    expect_value(__wrap_do_steps, joint, 1);
    expect_value(__wrap_do_steps, joint, 2);
    expect_value(__wrap_do_steps, joint, 3);
    expect_value_count(
            __wrap_do_steps, update_period_us, 1000, 4);
    will_return_always(__wrap_do_steps, 1);

    // Call code being tested.
    update_all_joint();

    assert_int_equal(count_time_us_64, 1);
}


static void test_do_steps__period_not_set(void **state) {
    (void) state; /* unused */

    uint32_t period = 0;
    expect_value(__wrap_init_pio, joint, 2);

    // Stops joint.
    expect_value(__wrap_stop_joint, joint, 2);

    uint16_t ret_val = __real_do_steps(2, period);
    assert_int_equal(ret_val, 0);
}


static void test_do_steps__data_not_updated(void **state) {
    (void) state; /* unused */

    uint32_t period = 1000;
    will_return(__wrap_get_joint_config, 0);

    expect_value(__wrap_init_pio, joint, 2);

    uint16_t ret_val = __real_do_steps(2, period);
    assert_int_equal(ret_val, 0);
}


static void test_do_steps__joint_not_enabled(void **state) {
    (void) state; /* unused */

    uint32_t period = 1000;
    will_return(__wrap_get_joint_config, 1);
    get_joint_config__enabled = 0;

    expect_value(__wrap_init_pio, joint, 2);

    // Stops joint.
    expect_value(__wrap_stop_joint, joint, 2);

    uint16_t ret_val = __real_do_steps(2, period);
    assert_int_equal(ret_val, 0);
}


int const check_uint_pointer(
        const size_t to_check,  const size_t provided) {
    printf("***\t%u\t%u\n", *(uint32_t*)to_check, provided);
    return *(uint32_t*)to_check == provided;
}

static void test_do_steps__no_rx_fifo_data(void **state) {
    (void) state; /* unused */

    uint32_t period = 1000;
    will_return(__wrap_get_joint_config, 1);
    get_joint_config__enabled = 1;
    get_joint_config__abs_pos_achieved = 8;

    sm1[2] = 2;

    expect_value(__wrap_init_pio, joint, 2);

    expect_value(__wrap_pio_sm_get_rx_fifo_level, pio, 0);
    expect_value(__wrap_pio_sm_get_rx_fifo_level, sm, 2);
    will_return(__wrap_pio_sm_get_rx_fifo_level, 0);

    // Correct value saved to config.
    // This is the value that was originally retrieved from config.
    expect_value(
            __wrap_update_joint_config,
            joint,
            2);
    expect_check(
            __wrap_update_joint_config,
            abs_pos_achieved,
            check_uint_pointer,
            get_joint_config__abs_pos_achieved);

    uint16_t ret_val = __real_do_steps(2, period);
    assert_int_equal(ret_val, 1);
}

static void test_do_steps__yes_rx_fifo_data(void **state) {
    (void) state; /* unused */

    uint32_t period = 1000;
    will_return(__wrap_get_joint_config, 1);
    get_joint_config__enabled = 1;

    sm1[2] = 2;

    expect_value(__wrap_init_pio, joint, 2);
    
    // Set 2 values in fifo.
    expect_value(__wrap_pio_sm_get_rx_fifo_level, pio, 0);
    expect_value(__wrap_pio_sm_get_rx_fifo_level, sm, 2);
    will_return(__wrap_pio_sm_get_rx_fifo_level, 2);

    // First fifo read.
    expect_value(__wrap_pio_sm_get_blocking, pio, 0);
    expect_value(__wrap_pio_sm_get_blocking, sm, 2);
    will_return(__wrap_pio_sm_get_blocking, 222);

    // Second fifo read.
    expect_value(__wrap_pio_sm_get_blocking, pio, 0);
    expect_value(__wrap_pio_sm_get_blocking, sm, 2);
    will_return(__wrap_pio_sm_get_blocking, 333);

    // Correct value saved to config.
    expect_value(
            __wrap_update_joint_config,
            joint,
            2);
    expect_check(
            __wrap_update_joint_config,
            abs_pos_achieved,
            check_uint_pointer,
            333);

    uint16_t ret_val = __real_do_steps(2, period);
    assert_int_equal(ret_val, 1);
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_packet_timeout),
        cmocka_unit_test(test_packet_received),
        cmocka_unit_test(test_do_steps__period_not_set),
        cmocka_unit_test(test_do_steps__data_not_updated),
        cmocka_unit_test(test_do_steps__joint_not_enabled),
        cmocka_unit_test(test_do_steps__no_rx_fifo_data),
        cmocka_unit_test(test_do_steps__yes_rx_fifo_data),
    };

    return cmocka_run_group_tests(tests, NULL, NULL);
}

