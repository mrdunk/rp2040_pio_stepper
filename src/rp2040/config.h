#ifndef CONFIG__H
#define CONFIG__H

#include <stdlib.h>
#include <stdint.h>
#include <limits.h>
#include <stdbool.h>

#include "stepper_control.h"
#include "ring_buffer.h"

#define CORE0 0
#define CORE1 1

// Semaphore for synchronizing cores.
extern volatile uint32_t tick;

/* Configuration object for an axis.
 * This is the format for the global config that is shared between cores. */
struct ConfigAxis {
  uint8_t updated_from_c0;      // Data was updated on core 0.
  uint8_t updated_from_c1;      // Data was updated on core 1.
  int8_t enabled;
  int8_t io_pos_step;           // Physical step IO pin. 
  int8_t io_pos_dir;            // Physical direction IO pin.
  double rel_pos_requested;     // In steps. Default value is UINT_MAX / 2.
  double abs_pos_requested;     // In steps. Default value is UINT_MAX / 2.
  uint32_t abs_pos_acheived;    // In steps. Default value is UINT_MAX / 2.
  double max_velocity;
  double max_accel_ticks;       // ticks / update_time_ticks ^ 2
  int32_t velocity_requested;   // Calculated steps per update_time_us.
  int32_t velocity_acheived;    // Steps per update_time_us.
  int32_t pos_error;            // Difference between requested position and that reported by PIO.
  int32_t step_len_ticks;       // Length of steps requested from the PIO.
  float kp;                     // Proportional position tuning. <= 1.0
};

/* Configuration object for global settings.
 * This is the format for the global config that is shared between cores. */
struct ConfigGlobal {
  uint32_t last_update_id;    // Sequence number of last packet received.
  int32_t last_update_time;   // Sequence number of last packet received.
  uint32_t update_time_us;    // Driven by how often we get axis updates from controlling host.
  bool pio_io_configured;     // PIO IO pins set.

  struct ConfigAxis axis[MAX_AXIS];
};


void init_config();

/* Update the period of the main timing loop. 
 * This should closely match the rate at which we receive axis position data. */
void update_period(uint32_t update_time_us);

/* Get the period of the main timing loop. 
 * This should closely match the rate at which we receive axis position data. */
uint32_t get_period();

/* Set metrics for tracking successful update transmission and jitter. */
void update_packet_metrics(
    uint32_t update_id, uint32_t time, int32_t* id_dif, int32_t* time_dif);

uint8_t has_new_c0_data(const uint8_t axis);

void update_axis_config(
    const uint8_t axis,
    const uint8_t core,
    const uint8_t* enabled,
    const int8_t* io_pos_step,
    const int8_t* io_pos_dir,
    const double* rel_pos_requested,
    const double* abs_pos_requested,
    const uint32_t* abs_pos_acheived,
    const double* max_velocity,
    const double* max_accel_ticks,
    const int32_t* velocity_requested,
    const int32_t* velocity_acheived,
    const int32_t* pos_error,
    const int32_t* step_len_ticks,
    const float* kp
);

uint32_t get_axis_config(
    const uint8_t axis,
    const uint8_t core,
    uint8_t* enabled,
    int8_t* io_pos_step,
    int8_t* io_pos_dir,
    double* rel_pos_requested,
    double* abs_pos_requested,
    uint32_t* abs_pos_acheived,
    double* max_velocity,
    double* max_accel_ticks,
    int32_t* velocity_requested,
    int32_t* velocity_acheived,
    int32_t* pos_error,
    int32_t* step_len_ticks,
    float* kp
    );

/* Serialise metrics stored in global config in a format for sending over UDP. */
size_t serialise_metrics(uint8_t* tx_buf, size_t* tx_buf_len, int32_t update_id, int32_t time_diff);

/* Serialise data stored in global config in a format for sending over UDP. */
size_t serialise_axis_config(
    const uint32_t axis,
    uint8_t* tx_buf,
    size_t* tx_buf_len,
    uint8_t wait_for_data);

#endif  // CONFIG__H
