#ifndef CONFIG__H
#define CONFIG__H

#include <stdlib.h>
#include <stdint.h>
#include <limits.h>

#include "stepper_control.h"

#define CORE0 0
#define CORE1 1


/* Configuration object for an axis.
 * This is the format for the global config that is shared between cores. */
struct ConfigAxis {
  uint8_t updated_from_c0;      // Data was updated on core 0.
  uint8_t updated_from_c1;      // Data was updated on core 1.
  uint32_t abs_pos_requested;   // In steps. Default value is UINT_MAX / 2.
  uint32_t abs_pos_acheived;    // In steps. Default value is UINT_MAX / 2.
  uint32_t min_step_len_ticks;
  uint32_t max_accel_ticks;     // ticks / update_time_ticks ^ 2
  int32_t velocity_acheived;    // Steps per update_time_us.
  float kp;                     // PID tuning
  float ki;                     // PID tuning
  float kd;                     // PID tuning
};

/* Configuration object for global settings.
 * This is the format for the global config that is shared between cores. */
struct ConfigGlobal {
  uint32_t update_time_us;  // Driven by how often we get axis updates from controlling host.

  struct ConfigAxis axis[MAX_AXIS];
};


void init_config();

/* Update the period of the main timing loop. 
 * This should closely match the rate at which we receive axis position data. */
void update_period(uint32_t update_time_us);

/* Get the period of the main timing loop. 
 * This should closely match the rate at which we receive axis position data. */
uint32_t get_period();

uint8_t has_new_c0_data(const uint8_t axis);

void update_axis_config(
    const uint8_t axis,
    const uint8_t core,
    const uint32_t* abs_pos_requested,
    const uint32_t* abs_pos_acheived,
    const uint32_t* min_step_len_ticks,
    const uint32_t* max_accel_ticks,
    const int32_t* velocity_acheived,
    const float* kp,
    const float* ki,
    const float* kd
);

uint32_t get_axis_config(
    const uint8_t axis,
    const uint8_t core,
    uint32_t* abs_pos_requested,
    uint32_t* abs_pos_acheived,
    uint32_t* min_step_len_ticks,
    uint32_t* max_accel_ticks,
    int32_t* velocity_acheived,
    float* kp,
    float* ki,
    float* kd
    );

/* Serialise data stored in global config in a format for sending over UDP. */
size_t serialise_axis_config(
    const uint32_t axis,
    uint8_t* msg_machine,
    size_t* msg_machine_len,
    size_t msg_machine_len_max,
    uint8_t always);

/* A ring buffer that returns the average value of it's contents.
 * Used for calculating average period between incoming network updates. */
#define RING_BUF_AVE_LEN 1000
struct Ring_buf_ave {
  uint32_t buf[RING_BUF_AVE_LEN];
  size_t head;
  uint32_t total;
  size_t count;
};

size_t ring_buf_ave(struct Ring_buf_ave* data, uint32_t new_val);

#endif  // CONFIG__H