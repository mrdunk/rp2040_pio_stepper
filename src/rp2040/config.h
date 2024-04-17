#ifndef CONFIG__H
#define CONFIG__H

#include <stdlib.h>
#include <stdint.h>
#include <limits.h>
#include <stdbool.h>

#include "messages.h"
#include "modbus.h"
#include "buffer.h"
#include "stepper_control.h"
#include "ring_buffer.h"

#define CORE0 0
#define CORE1 1

#define MAX_MISSED_PACKET 10

// Semaphore for synchronizing cores.
extern volatile uint32_t tick;

/* Configuration object for an joint.
 * This is the format for the global config that is shared between cores. */
struct ConfigAxis {
  uint8_t updated_from_c0;        // Data was updated on core 0.
  uint8_t updated_from_c1;        // Data was updated on core 1.
  int8_t enabled;
  int8_t io_pos_step;             // Physical step IO pin.
  int8_t io_pos_dir;              // Physical direction IO pin.
  double velocity_requested;      // In steps. Default value is UINT_MAX / 2.
  double abs_pos_requested;       // In steps. Default value is UINT_MAX / 2.
  int32_t abs_pos_achieved;       // In steps. Default value is UINT_MAX / 2.
  double max_velocity;
  double max_accel;               // ticks / update_time_ticks ^ 2
  int32_t velocity_requested_tm1; // Velocity requested last cycle. Compare to velocity_achieved.
  int32_t velocity_achieved;      // Steps per update_time_us.
  int32_t step_len_ticks;         // Length of steps requested from the PIO.
  int32_t position_error;         // Difference between requested and achieved position.
};

/* Configuration object for a single GPIO. */
struct ConfigGPIO {
  uint8_t type;                 // See GPIO_TYPE_XXXX in messages.h.
  uint8_t index;                // IP pin number.
  uint8_t address;              // i2c address if applicable.
  bool value;                   // Last set value for output pins.
};

/* Configuration object for an i2c interface. */
struct ConfigI2c {
  int8_t io_scl;                // 
  int8_t io_sda;                // 
  uint8_t address;              // i2c address.
};

/* Configuration object for global settings.
 * This is the format for the global config that is shared between cores. */
struct ConfigGlobal {
  uint32_t last_update_id;    // Sequence number of last packet received.
  int32_t last_update_time;   // Sequence number of last packet received.
  uint32_t update_time_us;    // Driven by how often we get joint updates from controlling host.
  bool pio_io_configured;     // PIO IO pins set.

  struct ConfigAxis joint[MAX_JOINT];
  struct ConfigGPIO gpio[MAX_GPIO];
  struct ConfigI2c i2c[MAX_I2C_MCP];
  bool gpio_confirmation_pending[MAX_GPIO_BANK];
};


void init_config();

/* Update the period of the main timing loop.
 * This should closely match the rate at which we receive joint position data. */
void update_period(uint32_t update_time_us);

/* Get the period of the main timing loop.
 * This should closely match the rate at which we receive joint position data. */
uint32_t get_period();

/* Set metrics for tracking successful update transmission and jitter. */
void update_packet_metrics(
    struct Message_timing* message,
    int32_t* id_diff,
    int32_t* time_diff);

uint8_t has_new_c0_data(const uint8_t joint);

void update_joint_config(
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
);

uint32_t get_joint_config(
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
    int32_t* position_error
    );

void disable_joint(const uint8_t joint, const uint8_t core);

/* Serialise metrics stored in global config in a format for sending over UDP. */
bool serialise_timing(struct NWBuffer* tx_buf, int32_t update_id, int32_t time_diff);

/* Serialise data stored in global config in a format for sending over UDP. */
bool serialise_joint_movement(
    struct NWBuffer* tx_buf,
    uint8_t wait_for_data);

/* Serialise data stored in global config in a format for sending over UDP. */
bool serialise_joint_config(
    const uint32_t joint,
    struct NWBuffer* tx_buf);

/* Serialise data stored in global config in a format for sending over UDP. */
bool serialise_joint_metrics(struct NWBuffer* tx_buf);

bool serialise_spindle_speed_out(
    struct NWBuffer* tx_buf, float speed, struct vfd_stats *vfd_stats);

bool serialise_spindle_config(size_t spindle, struct NWBuffer* tx_buf);

#endif  // CONFIG__H
