

#include "rtapi.h"      /* RTAPI realtime OS API */
#include "rtapi_app.h"    /* RTAPI realtime module decls */

#include "hal.h"        /* HAL public API decls */

#include <limits.h>
#include <stddef.h>

#include "rp2040_defines.h"
#include "../shared/messages.h"

/* module information */
MODULE_AUTHOR("Duncan Law");
MODULE_DESCRIPTION("RP2040 based IO for LinuxCNC HAL");
MODULE_LICENSE("GPL");

static int num_joints = 0;
MODULE_PARM(num_joints, "i");
MODULE_PARM_DESC(num_joints, "Number of joints configured in LinuxCNC ([KINS]JOINTS)");


/***********************************************************************
 *                STRUCTURES AND GLOBAL VARIABLES                       *
 ************************************************************************/

#include "skeleton.h"


#include "rp2040_network.c"      /* This project's network code. */
#include "rp2040_eth_state.c"    /* HAL-free Ethernet state machine. */

/* pointer to array of skeleton_t structs in shared memory, 1 per port */
static skeleton_t *port_data_array;

/* other globals */
static int component_id;    /* component ID */

/***********************************************************************
 *                  LOCAL FUNCTION DECLARATIONS                         *
 ************************************************************************/
static void write_port(void *arg, long period);

/***********************************************************************
 *                       INIT AND EXIT CODE                             *
 ************************************************************************/

enum t_types {
  PIN = 0,
  U32 = 2,
  S32 = 3,
  FLOAT = 4
};

bool init_hal_pin(
    enum t_types types,
    const hal_pin_dir_t hal_pin_dir,
    void* data_p,
    const int component_id,
    const int device_num,
    const char* io_type,
    const int chan_num,
    const int chan_num_len,
    const char* specific_name
) {
    int retval;
    if(chan_num < 0) {
      switch(types) {
        case PIN:
          retval = hal_pin_bit_newf(hal_pin_dir, data_p, component_id, "rp2040_eth.%d.%s", device_num, io_type);
          break;
        case U32:
          retval = hal_pin_u32_newf(hal_pin_dir, data_p, component_id, "rp2040_eth.%d.%s", device_num, io_type);
          break;
        case S32:
          retval = hal_pin_s32_newf(hal_pin_dir, data_p, component_id, "rp2040_eth.%d.%s", device_num, io_type);
          break;
        case FLOAT:
          retval = hal_pin_float_newf(hal_pin_dir, data_p, component_id, "rp2040_eth.%d.%s", device_num, io_type);
          break;
      }
    } else {
      char format[64];
      int check = snprintf(format, 64, "rp2040_eth.%%d.%%s.%%0%dd.%%s", chan_num_len);
      if (check < 0 || check >= 64) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "ERROR: Invalid string length=%i\n",
            check);
        hal_exit(component_id);
        return false;
      }

      switch(types) {
        case PIN:
          retval = hal_pin_bit_newf(hal_pin_dir, data_p, component_id, format, device_num, io_type, chan_num, specific_name);
          break;
        case U32:
          retval = hal_pin_u32_newf(hal_pin_dir, data_p, component_id, format, device_num, io_type, chan_num, specific_name);
          break;
        case S32:
          retval = hal_pin_s32_newf(hal_pin_dir, data_p, component_id, format, device_num, io_type, chan_num, specific_name);
          break;
        case FLOAT:
          retval = hal_pin_float_newf(hal_pin_dir, data_p, component_id, format, device_num, io_type, chan_num, specific_name);
          break;
      }
    }
    if (retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,
                      "RP2040: ERROR: var export failed with err=%i\n",
                      retval);
      hal_exit(component_id);
      return false;
    }
    return true;
}

static bool init_hal_param(
    enum t_types types,
    void* data_p,
    const int component_id,
    const int device_num,
    const char* io_type,
    const int chan_num,
    const int chan_num_len,
    const char* specific_name
) {
    int retval = -1;
    char format[64];
    int check = snprintf(format, 64, "rp2040_eth.%%d.%%s.%%0%dd.%%s", chan_num_len);
    if (check < 0 || check >= 64) {
      rtapi_print_msg(RTAPI_MSG_ERR, "ERROR: Invalid string length=%i\n", check);
      hal_exit(component_id);
      return false;
    }
    switch(types) {
      case U32:
        retval = hal_param_u32_newf(HAL_RW, data_p, component_id, format,
                                    device_num, io_type, chan_num, specific_name);
        break;
      case S32:
        retval = hal_param_s32_newf(HAL_RW, data_p, component_id, format,
                                    device_num, io_type, chan_num, specific_name);
        break;
      case FLOAT:
        retval = hal_param_float_newf(HAL_RW, data_p, component_id, format,
                                      device_num, io_type, chan_num, specific_name);
        break;
      case PIN:
        rtapi_print_msg(RTAPI_MSG_ERR, "RP2040: ERROR: bit-type HAL params not supported\n");
        return false;
    }
    if (retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,
                      "RP2040: ERROR: param export failed with err=%i\n",
                      retval);
      hal_exit(component_id);
      return false;
    }
    return true;
}


#define ARRAY_SIZE(a) ((int)(sizeof(a) / sizeof((a)[0])))

/* Per-channel pointer fields in skeleton_t are plain arrays of pointers with no
 * padding between elements. stride = sizeof(ptr) steps correctly through them. */
typedef struct {
    enum t_types   type;
    hal_pin_dir_t  dir;
    size_t         offset;
    size_t         stride;
    const char*    io_type;
    int            chan_num;       /* -1 = scalar (no channel suffix); ignored for
                                     per-channel tables — loop variable used instead */
    int            chan_num_len;
    const char*    specific_name;
} PinDef;

/* HAL params: direct storage in skeleton_t (not pointer-to-pointer).
 * stride = sizeof(value_type) steps through plain value arrays. */
typedef struct {
    enum t_types   type;
    size_t         offset;
    size_t         stride;
    const char*    io_type;
    int            chan_num_len;
    const char*    specific_name;
} ParamDef;

static const PinDef gpio_pins[] = {
    { PIN, HAL_OUT, offsetof(skeleton_t, gpio_data_in),         sizeof(hal_bit_t*),   "gpio", 0, 2, "in"         }, // Input state read from hardware
    { PIN, HAL_OUT, offsetof(skeleton_t, gpio_data_in_not),     sizeof(hal_bit_t*),   "gpio", 0, 2, "in-not"     }, // Inverted input state
    { PIN, HAL_IN,  offsetof(skeleton_t, gpio_data_out),        sizeof(hal_bit_t*),   "gpio", 0, 2, "out"        }, // Output command
    { PIN, HAL_IN,  offsetof(skeleton_t, gpio_data_out_invert), sizeof(hal_bit_t*),   "gpio", 0, 2, "out-invert" }, // Invert output before writing to hardware
};

static const ParamDef gpio_params[] = {
    { U32, offsetof(skeleton_t, gpio_type),    sizeof(hal_u32_t), "gpio", 2, "type"    }, // GPIO device type — see rp2040_gpio_types.ini
    { U32, offsetof(skeleton_t, gpio_index),   sizeof(hal_u32_t), "gpio", 2, "index"   }, // Pin index within the GPIO device
    { U32, offsetof(skeleton_t, gpio_address), sizeof(hal_u32_t), "gpio", 2, "address" }, // I2C address of the GPIO device (MCP23017 only)
};

static const ParamDef joint_params[] = {
    { S32, offsetof(skeleton_t, joint_gpio_step), sizeof(hal_s32_t), "joint", 1, "gpio-step" }, // RP2040 GPIO pin number for the step signal
    { S32, offsetof(skeleton_t, joint_gpio_dir),  sizeof(hal_s32_t), "joint", 1, "gpio-dir"  }, // RP2040 GPIO pin number for the direction signal
    { U32, offsetof(skeleton_t, joint_cmd_type),  sizeof(hal_u32_t), "joint", 1, "cmd-type"  }, // Step command mode: 0=position (default), 1=velocity
};

static const PinDef joint_pins[] = {
    { PIN,   HAL_IN,  offsetof(skeleton_t, joint_enable_cmd),     sizeof(hal_bit_t*),   "joint", 0, 1, "enable-cmd"       }, // Enable joint (LinuxCNC command)
    { FLOAT, HAL_IN,  offsetof(skeleton_t, joint_vel_limit),      sizeof(hal_float_t*), "joint", 0, 1, "vel-limit"        }, // Maximum velocity (units/sec)
    { FLOAT, HAL_IN,  offsetof(skeleton_t, joint_accel_limit),    sizeof(hal_float_t*), "joint", 0, 1, "accel-limit"      }, // Maximum acceleration (units/sec²)
    { FLOAT, HAL_IN,  offsetof(skeleton_t, joint_scale),          sizeof(hal_float_t*), "joint", 0, 1, "scale"            }, // Steps per unit; applied to both position and velocity commands
    { FLOAT, HAL_IN,  offsetof(skeleton_t, joint_pos_cmd),        sizeof(hal_float_t*), "joint", 0, 1, "pos-cmd"          }, // Position command; consumed by firmware in position mode (cmd-type=0); also used to compute pos-error-fb
    { FLOAT, HAL_IN,  offsetof(skeleton_t, joint_vel_cmd),        sizeof(hal_float_t*), "joint", 0, 1, "vel-cmd"          }, // Velocity command from LinuxCNC
    { FLOAT, HAL_OUT, offsetof(skeleton_t, joint_pos_fb),         sizeof(hal_float_t*), "joint", 0, 1, "pos-fb"           }, // Position feedback (cumulative step count ÷ scale)
    { FLOAT, HAL_OUT, offsetof(skeleton_t, joint_vel_fb),         sizeof(hal_float_t*), "joint", 0, 1, "vel-fb"           }, // Velocity feedback (steps/period; Q16.16 from firmware, exact zero when stopped)
    { S32,   HAL_OUT, offsetof(skeleton_t, joint_pos_error_fb),   sizeof(hal_s32_t*),   "joint", 0, 1, "pos-error-fb"     }, // Difference between commanded and actual step count (raw steps, unscaled)
    { PIN,   HAL_OUT, offsetof(skeleton_t, joint_enable_fb),      sizeof(hal_bit_t*),   "joint", 0, 1, "enable-fb"        }, // RP2040's actual enabled state; may remain false after network recovery until protocol re-enables
    { FLOAT, HAL_OUT, offsetof(skeleton_t, joint_vel_calculated), sizeof(hal_float_t*), "joint", 0, 1, "vel-calculated"   }, // Velocity the RP2040 computed after applying vel-limit and accel-limit
    { FLOAT, HAL_OUT, offsetof(skeleton_t, joint_ferror_suggest), sizeof(hal_float_t*), "joint", 0, 1, "ferror-suggest"   }, // Expected following error at vel-limit given current round-trip latency (units); use as FERROR lower bound
};

static const PinDef spindle_pins[] = {
    { PIN,   HAL_IN,  offsetof(skeleton_t, spindle_fwd),       sizeof(hal_bit_t*),   "spindle", 0, 1, "fwd"       }, // Run spindle forward
    { PIN,   HAL_IN,  offsetof(skeleton_t, spindle_rev),       sizeof(hal_bit_t*),   "spindle", 0, 1, "rev"       }, // Run spindle reverse
    { FLOAT, HAL_IN,  offsetof(skeleton_t, spindle_speed_cmd), sizeof(hal_float_t*), "spindle", 0, 1, "speed-cmd" }, // Commanded speed
    { FLOAT, HAL_OUT, offsetof(skeleton_t, spindle_speed_fb),  sizeof(hal_float_t*), "spindle", 0, 1, "speed-fb"  }, // Actual speed feedback
    { PIN,   HAL_OUT, offsetof(skeleton_t, spindle_at_speed),  sizeof(hal_bit_t*),   "spindle", 0, 1, "at-speed"  }, // Spindle has reached commanded speed
};

static const PinDef scalar_pins[] = {
    { PIN,   HAL_OUT, offsetof(skeleton_t, machine_on),      0, "machine-on",      -1, 1, NULL }, // True when RP2040 Ethernet link is established and communicating
    { U32,   HAL_OUT, offsetof(skeleton_t, seq_out),         0, "seq-out",         -1, 0, NULL }, // Sequence number stamped on each packet sent to RP2040
    { U32,   HAL_OUT, offsetof(skeleton_t, core1_period),    0, "core1-period",    -1, 0, NULL }, // RP2040 core1 measured time between loop iterations (µs)
    { U32,   HAL_OUT, offsetof(skeleton_t, core1_tick),      0, "core1-tick",      -1, 0, NULL }, // RP2040 core1 loop iteration counter; frozen value indicates firmware hang
    { S32,   HAL_OUT, offsetof(skeleton_t, packet_interval), 0, "packet-interval", -1, 0, NULL }, // Time between consecutive packets from LinuxCNC timestamps (ns); nominally equals the servo period
    { U32,   HAL_OUT, offsetof(skeleton_t, seq_in),          0, "seq-in",          -1, 0, NULL }, // Sequence number echoed back by RP2040; seq-out − seq-in gives round-trip latency in cycles
    { U32,   HAL_OUT, offsetof(skeleton_t, rx_miss_count),   0, "rx-miss-count",   -1, 0, NULL }, // Consecutive cycles without a response; resets to 0 on success; triggers network-down at MAX_SKIPPED_PACKETS
    { PIN,   HAL_OUT, offsetof(skeleton_t, eth_up),          0, "eth-up",          -1, 0, NULL }, // Ethernet link state as seen by the driver
    { PIN,   HAL_OUT, offsetof(skeleton_t, config_complete), 0, "config-complete",  -1, 0, NULL }, // All joint/GPIO/spindle configs confirmed by firmware
    { FLOAT, HAL_OUT, offsetof(skeleton_t, update_overrun),  0, "update-overrun",  -1, 0, NULL }, // EMA of cycles where Core1 received more than one update from Core0 per period
    { FLOAT, HAL_OUT, offsetof(skeleton_t, update_underrun), 0, "update-underrun", -1, 0, NULL }, // EMA of cycles where Core1 found no new update from Core0
    { U32,   HAL_OUT, offsetof(skeleton_t, core1_work_us),   0, "core1-work-us",   -1, 0, NULL }, // µs Core1 spent working last period (excludes time waiting for tick)
    { U32,   HAL_OUT, offsetof(skeleton_t, core0_work_us),   0, "core0-work-us",   -1, 0, NULL }, // µs Core0 spent working last period (packet received → response sent, incl. modbus)
};

int rtapi_app_main(void)
{
  char name[HAL_NAME_LEN + 1];
  int retval;

  /* only one device at the moment */
  int device_num = 0;

  /* STEP 1: initialise the driver */
  component_id = hal_init("hal_rp2040_eth");
  if (component_id < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR,
        "RP2040: ERROR: hal_init() failed\n");
    return -1;
  }

  printf("RP2040: INFO: driver version %d.%d.%d branch 0x%08x\n",
      PROTOCOL_VERSION_MAJOR, PROTOCOL_VERSION_MINOR, PROTOCOL_VERSION_PATCH,
      PROTOCOL_VERSION_BRANCH);

  if (num_joints <= 0) {
    rtapi_print_msg(RTAPI_MSG_ERR,
        "RP2040: ERROR: num_joints not set — defaulting to %d. "
        "Add num_joints=[KINS]JOINTS to your loadrt line: "
        "loadrt hal_rp2040_eth num_joints=[KINS]JOINTS\n", MAX_JOINT);
    num_joints = MAX_JOINT;
  }

  /* STEP 2: allocate shared memory for skeleton data */
  port_data_array = hal_malloc(sizeof(skeleton_t));
  if (port_data_array == 0) {
    rtapi_print_msg(RTAPI_MSG_ERR,
        "RP2040: ERROR: hal_malloc() failed\n");
    goto port_error;
  }

  /* Set some default values. */
  for(int gpio_bank = 0; gpio_bank < MAX_GPIO_BANK; gpio_bank++) {
    port_data_array->gpio_data_received[gpio_bank] = 0;
    port_data_array->gpio_confirmation_pending[gpio_bank] = true;
  }


  /* Set up the HAL pins and params via descriptor tables. */
  for (int i = 0; i < MAX_GPIO; i++) {
    for (int j = 0; j < ARRAY_SIZE(gpio_pins); j++) {
      const PinDef* def = &gpio_pins[j];
      void* fp = (void*)((char*)port_data_array + def->offset + i * def->stride);
      if (!init_hal_pin(def->type, def->dir, fp, component_id, device_num,
                        def->io_type, i, def->chan_num_len, def->specific_name)) {
        goto port_error;
      }
    }
    for (int j = 0; j < ARRAY_SIZE(gpio_params); j++) {
      const ParamDef* def = &gpio_params[j];
      void* fp = (void*)((char*)port_data_array + def->offset + i * def->stride);
      if (!init_hal_param(def->type, fp, component_id, device_num,
                          def->io_type, i, def->chan_num_len, def->specific_name)) {
        goto port_error;
      }
    }
  }
  /* Default values written after all pins/params are registered. */
  for (int i = 0; i < MAX_GPIO; i++) {
    *port_data_array->gpio_data_in[i]         = true;
    *port_data_array->gpio_data_in_not[i]     = false;
    *port_data_array->gpio_data_out[i]        = false;
    *port_data_array->gpio_data_out_invert[i] = false;
    port_data_array->gpio_type[i]    = GPIO_TYPE_NOT_SET;
    port_data_array->gpio_index[i]   = 0;
    port_data_array->gpio_address[i] = 0;
  }

  for (int j = 0; j < ARRAY_SIZE(scalar_pins); j++) {
    const PinDef* def = &scalar_pins[j];
    void* fp = (void*)((char*)port_data_array + def->offset);
    if (!init_hal_pin(def->type, def->dir, fp, component_id, device_num,
                      def->io_type, def->chan_num, def->chan_num_len, def->specific_name)) {
      goto port_error;
    }
  }
  *port_data_array->machine_on = false;

  for (int i = 0; i < MAX_JOINT; i++) {
    for (int j = 0; j < ARRAY_SIZE(joint_pins); j++) {
      const PinDef* def = &joint_pins[j];
      void* fp = (void*)((char*)port_data_array + def->offset + i * def->stride);
      if (!init_hal_pin(def->type, def->dir, fp, component_id, device_num,
                        def->io_type, i, def->chan_num_len, def->specific_name)) {
        goto port_error;
      }
    }
    for (int j = 0; j < ARRAY_SIZE(joint_params); j++) {
      const ParamDef* def = &joint_params[j];
      void* fp = (void*)((char*)port_data_array + def->offset + i * def->stride);
      if (!init_hal_param(def->type, fp, component_id, device_num,
                          def->io_type, i, def->chan_num_len, def->specific_name)) {
        goto port_error;
      }
    }
  }
  for (int i = 0; i < MAX_JOINT; i++) {
    *port_data_array->joint_enable_cmd[i]  = false;
    port_data_array->joint_gpio_step[i]    = -1;
    port_data_array->joint_gpio_dir[i]     = -1;
    port_data_array->joint_cmd_type[i]     = JOINT_CMD_POSITION;
  }

  /* Export spindle pins. */
  for (int i = 0; i < MAX_SPINDLE; i++) {
    for (int j = 0; j < ARRAY_SIZE(spindle_pins); j++) {
      const PinDef* def = &spindle_pins[j];
      void* fp = (void*)((char*)port_data_array + def->offset + i * def->stride);
      if (!init_hal_pin(def->type, def->dir, fp, component_id, device_num,
                        def->io_type, i, def->chan_num_len, def->specific_name)) {
        goto port_error;
      }
    }

    retval = hal_param_u32_newf(HAL_RW, &(port_data_array->spindle_vfd_type[i]),
        component_id, "rp2040_eth.%d.spindle.%d.vfd-type", device_num, i);
    if (retval < 0) {
      goto port_error;
    }
    port_data_array->spindle_vfd_type[i] = MODBUS_TYPE_NOT_SET;

    retval = hal_param_u32_newf(HAL_RW, &(port_data_array->spindle_address[i]),
        component_id, "rp2040_eth.%d.spindle.%d.address", device_num, i);
    if (retval < 0) {
      goto port_error;
    }
    port_data_array->spindle_address[i] = 1;

    retval = hal_param_float_newf(HAL_RW, &(port_data_array->spindle_poles[i]),
        component_id, "rp2040_eth.%d.spindle.%d.poles", device_num, i);
    if (retval < 0) {
      goto port_error;
    }
    port_data_array->spindle_poles[i] = 2;

    retval = hal_param_u32_newf(HAL_RW, &(port_data_array->spindle_bitrate[i]),
        component_id, "rp2040_eth.%d.spindle.%d.bitrate", device_num, i);
    if (retval < 0) {
      goto port_error;
    }
    port_data_array->spindle_bitrate[i] = 9600;
  }

  /* STEP 4: export write function */
  rtapi_snprintf(name, sizeof(name), "rp2040_eth.%d.write", device_num);
  retval = hal_export_funct(name, write_port, &(port_data_array[device_num]), 1, 0,
      component_id);
  if (retval < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR,
        "RP2040: ERROR: port %d write funct export failed\n",
        device_num);
    goto port_error;
  }

  retval = init_eth(device_num);

  if (retval < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR,
        "RP2040: ERROR: Failed to find device %d on the network.\n",
        device_num);
    goto port_error;
  }

  rtapi_print_msg(RTAPI_MSG_INFO,
      "RP2040: installed driver.\n");
  hal_ready(component_id);
  return 0;

port_error:
    rtapi_print_msg(RTAPI_MSG_ERR,
        "RP2040: ERROR: port %d var export failed with err=%i\n",
        device_num, retval);
    hal_exit(component_id);
    return -1;
}

void rtapi_app_exit(void)
{
  hal_exit(component_id);
}

/**************************************************************
 * REALTIME PORT WRITE FUNCTION                                *
 **************************************************************/

static void write_port(void *arg, long period)
{
  static size_t count = 0;
  skeleton_t *data = arg;

  eth_state_update(data, 0, count, (uint32_t)rtapi_get_time(), num_joints);

  /* Sync position: if joint not enabled, track RP position so LinuxCNC
   * resumes from the right place after enabling. */
  for(uint32_t joint = 0; joint < MAX_JOINT; joint++) {
    if(! *data->joint_enable_cmd[joint]) {
      *data->joint_pos_cmd[joint] = *data->joint_pos_fb[joint];
    }
  }

  /* Compute per-joint suggested FERROR: vel-limit × round-trip-latency.
   * Velocity mode uses 2× because a full reversal swings f-error by
   * 2×latency×vel.  Zero when eth is down or packet-interval is not valid. */
  hal_s32_t latency_cycles = (hal_s32_t)*data->seq_out - (hal_s32_t)*data->seq_in;
  if (*data->eth_up && *data->packet_interval > 0 && latency_cycles > 0) {
    double latency_s = (double)latency_cycles * (double)*data->packet_interval * 1e-9;
    for (uint32_t joint = 0; joint < MAX_JOINT; joint++) {
      double vl = fabs((double)*data->joint_vel_limit[joint]);
      double multiplier = (data->joint_cmd_type[joint] == JOINT_CMD_VELOCITY) ? 2.0 : 1.0;
      *data->joint_ferror_suggest[joint] = (hal_float_t)(multiplier * vl * latency_s);
    }
  } else {
    for (uint32_t joint = 0; joint < MAX_JOINT; joint++) {
      *data->joint_ferror_suggest[joint] = 0.0f;
    }
  }

  count++;
}

