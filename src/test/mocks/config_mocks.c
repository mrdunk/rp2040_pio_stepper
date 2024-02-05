#include <stdint.h>
#include <stddef.h>

void update_joint_config(
    const uint8_t joint,
    const uint8_t core,
    const uint8_t* enabled,
    const int8_t* io_pos_step,
    const int8_t* io_pos_dir,
    const double* rel_pos_requested,
    const double* abs_pos_requested,
    const uint32_t* abs_pos_acheived,
    const double* max_velocity,
    const double* max_accel,
    const int32_t* velocity_requested,
    const int32_t* velocity_acheived,
    const int32_t* step_len_ticks
)
{
}

void update_packet_metrics(
    uint32_t update_id,
    uint32_t time,
    int32_t* id_diff,
    int32_t* time_diff
) {
}

size_t serialise_timing(
        uint8_t* tx_buf,
        size_t* tx_buf_len,
        int32_t update_id,
        int32_t time_diff
) {
    return 0;
}

size_t serialise_joint_movement(
    const uint32_t joint,
    uint8_t* tx_buf,
    size_t* tx_buf_len,
    uint8_t wait_for_data)
{
    return 0;
}

size_t serialise_joint_config(
    const uint32_t joint,
    uint8_t* tx_buf,
    size_t* tx_buf_len,
    uint8_t wait_for_data)
{
    return 0;
}

void update_period(uint32_t update_time_us) {
}

