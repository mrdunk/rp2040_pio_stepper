#include <math.h>
#include <stdio.h>
#include <netdb.h>
#include <string.h>

#include "rp2040_defines.h"
#include "../shared/messages.h"
#include "../shared/buffer.c"
#include "../shared/checksum.c"
#include "../rp2040/modbus.h"

#ifdef BUILD_TESTS

#include "../test/mocks/driver_mocks.h"

#endif  // BUILD_TESTS

/* Declare and null-check an unpacked message. Must be used at the top of a
 * function body, not inside a bare if/else — it expands to two statements. */
#define UNPACK_MSG(T, var, buf, offset) \
    T* var = unpack_nw_buff((buf), *(offset), (offset), NULL, sizeof(T)); \
    if (!(var)) return false;

/* Network globals. */
static uint8_t detected_joint_count = 0;  /* set from first Reply_joint_movement.count */
static bool version_checked = false;     /* set when REPLY_VERSION received */
static bool version_match   = false;     /* set when version + branch both matched */
struct sockaddr_in remote_addr[MAX_DEVICES];
int sockfd[MAX_DEVICES] = {-1};


/* Initialise network for UDP. */
int init_eth(int device) {
  char *hostname = "192.168.12.2";
  int portno = 5002;

  /* socket: create the NW socket */
  sockfd[device] = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (sockfd[device] < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "ERROR opening socket\n");
    return -1;
  }

  /* Set zero timeout on socket. */
  struct timeval timeout;
  timeout.tv_sec  = 0;
  timeout.tv_usec = 0;
  int rc = setsockopt(sockfd[device], SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(struct timeval));
  if (rc < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "ERROR setting SOL_SOCKET, SO_RCVTIMEO\n");
    return -1;
  }

  /* SO_REUSEADDR should allow reuse of IP/port combo when quickly stopping and
   * restarting program. */
  int option = 1;
  rc = setsockopt(sockfd[device], SOL_SOCKET, SO_REUSEADDR, (void *)&option, sizeof(option));
  if (rc < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "ERROR setting SOL_SOCKET, SO_REUSEADDR\n");
    return -1;
  }

  /* Small receive buffer to prevent stale replies accumulating at startup.
   * Kernel doubles the value internally; 2048 bytes holds ~7 packets (260 bytes each).
   * Without this the default ~200KB buffer can queue hundreds of replies, causing
   * a large steady-state gap between seq-out and seq-in. */
  int rcvbuf = 2048;
  rc = setsockopt(sockfd[device], SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));
  if (rc < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "ERROR setting SOL_SOCKET, SO_RCVBUF\n");
    return -1;
  }

  /* Bind to local port so the kernel can route incoming packets to this socket
   * directly without a routing table lookup on each receive. */
  struct sockaddr_in local_addr = {0};
  local_addr.sin_family = AF_INET;
  local_addr.sin_addr.s_addr = INADDR_ANY;
  local_addr.sin_port = htons(portno);
  rc = bind(sockfd[device], (struct sockaddr*)&local_addr, sizeof(local_addr));
  if (rc < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "ERROR binding socket to local port\n");
    return -1;
  }

  /* gethostbyname: get the server's DNS entry */
  struct hostent *server = gethostbyname(hostname);
  if (server == NULL) {
    rtapi_print_msg(RTAPI_MSG_ERR, "ERROR, no such host as %s\n", hostname);
    return -1;
  }

  /* build the server's Internet address */
  memset(&remote_addr[device], 0, sizeof(remote_addr[device]));

  remote_addr[device].sin_family = AF_INET;  // IPv4
  memmove((char *)&remote_addr[device].sin_addr.s_addr, (char *)server->h_addr, server->h_length);
  remote_addr[device].sin_port = htons(portno);

  return sockfd[device];
}

/* Send data via UDP.*/
int send_data(int device, struct NWBuffer* buffer) {
  socklen_t addr_len = sizeof(remote_addr[device]);
  int n = sendto(
      sockfd[device],
      (void*)buffer,
      sizeof(buffer->length) + sizeof(buffer->checksum) + buffer->length,
      MSG_DONTROUTE,
      (struct sockaddr *)&remote_addr[device],
      addr_len);
  if (n < 0) {
    return errno;
  }
  return 0;
}

/* Get data via UDP. */
size_t get_reply_non_block(int device, struct NWBuffer* receive_buffer) {
  socklen_t addr_len = sizeof(remote_addr[device]);
  int flags = MSG_DONTWAIT | MSG_DONTROUTE;
  ssize_t receive_count = recvfrom(
      sockfd[device],
      (void*)receive_buffer,
      sizeof(struct NWBuffer),
      flags,
      (struct sockaddr *)&remote_addr[device],
      &addr_len);
  if (receive_count < 0) {
    return 0;
  }
  return receive_count;
}

size_t serialize_timing(
    struct NWBuffer* buffer,
    uint32_t update_id,
    uint32_t time
) {
  union MessageAny message;
  message.timing.type = MSG_TIMING;
  message.timing.update_id = update_id;
  message.timing.time = time;

  return pack_nw_buff(buffer, &message, sizeof(struct Message_timing));
}

size_t serialize_joint_pos(
    struct NWBuffer* buffer,
    skeleton_t* data
) {
  struct Message_set_joints_pos message = {0};
  message.type  = MSG_SET_JOINT_ABS_POS;
  message.count = detected_joint_count;  /* firmware reads only this many */

  for(size_t joint = 0; joint < MAX_JOINT; joint++) {
    double position = *data->joint_scale[joint] * *data->joint_pos_cmd[joint];
    double velocity = *data->joint_scale[joint] * *data->joint_vel_cmd[joint];
    message.position[joint] = position;
    message.velocity[joint] = velocity;
  }

  return pack_nw_buff(buffer, &message, sizeof(struct Message_set_joints_pos));
}

bool serialise_spindle_config(
    struct NWBuffer* tx_buf,
    uint8_t spindle,
    uint8_t vfd_type,
    uint8_t address,
    uint16_t bitrate
)
{
  struct Message_spindle_config message;
  message.type = MSG_SET_SPINDLE_CONFIG;
  message.spindle_index = spindle;
  message.vfd_type = vfd_type;
  message.modbus_address = address;
  message.bitrate = bitrate;

  return pack_nw_buff(tx_buf, &message, sizeof(message));
}

bool serialise_spindle_speed_in(
    struct NWBuffer* tx_buf,
    skeleton_t* data
)
{
  bool at_least_one_enabled = false;

  struct Message_spindle_speed message;
  message.type = MSG_SET_SPINDLE_SPEED;

  float speed;

  for(size_t spindle = 0; spindle < MAX_SPINDLE; spindle++) {
    if(data->spindle_vfd_type[spindle] == MODBUS_TYPE_NOT_SET) {
      continue;
    }
    at_least_one_enabled = true;

    speed =
      *data->spindle_speed_cmd[spindle] / (120.0 / data->spindle_poles[spindle]);
    message.speed[spindle] = speed;
  }

  if(at_least_one_enabled) {
    return pack_nw_buff(tx_buf, &message, sizeof(message));
  }
  return true;
}

size_t serialize_joint_enable(
    struct NWBuffer* buffer,
    uint8_t joint,
    uint8_t value
) {
  union MessageAny message;
  message.joint_enable.type = MSG_SET_JOINT_ENABLED;
  message.joint_enable.joint = joint;
  message.joint_enable.value = value;

  return pack_nw_buff(buffer, &message, sizeof(struct Message_joint_enable));
}

size_t serialize_joint_config(
    struct NWBuffer* buffer,
    uint8_t joint,
    uint8_t enable,
    uint8_t gpio_step,
    uint8_t gpio_dir,
    float max_velocity,
    float max_accel,
    uint8_t cmd_type
) {
  union MessageAny message;
  message.joint_config.type = MSG_SET_JOINT_CONFIG;
  message.joint_config.joint = joint;
  message.joint_config.enable = enable;
  message.joint_config.gpio_step = gpio_step;
  message.joint_config.gpio_dir = gpio_dir;
  message.joint_config.max_velocity = max_velocity;
  message.joint_config.max_accel = max_accel;
  message.joint_config.cmd_type = cmd_type;

  return pack_nw_buff(buffer, &message, sizeof(struct Message_joint_config));
}

size_t serialize_gpio_config(
    struct NWBuffer* buffer,
    uint8_t gpio,
    uint8_t gpio_type,
    uint8_t gpio_index,
    uint8_t gpio_address
) {
  union MessageAny message;
  message.gpio_config.type = MSG_SET_GPIO_CONFIG;
  message.gpio_config.gpio_type = gpio_type;
  message.gpio_config.gpio_count = gpio;
  message.gpio_config.index = gpio_index;
  message.gpio_config.address = gpio_address;

  return pack_nw_buff(buffer, &message, sizeof(struct Message_gpio_config));
}

uint16_t serialize_gpio(struct NWBuffer* buffer, skeleton_t* data) {
  uint16_t return_val = 0;
  bool confirmation_pending[MAX_GPIO_BANK];
  uint32_t to_send[MAX_GPIO_BANK];

  for(size_t bank = 0; bank < MAX_GPIO_BANK; bank++) {
    to_send[bank] = 0;
    confirmation_pending[bank] = false;
  }

  for(size_t gpio = 0; gpio < MAX_GPIO; gpio++) {
    size_t bank = gpio / 32;
    size_t gpio_per_bank = (gpio % 32);

    bool current_value = false;
    switch(data->gpio_type[gpio]) {
      case GPIO_TYPE_NATIVE_IN_DEBUG:
      case GPIO_TYPE_NATIVE_IN:
      case GPIO_TYPE_I2C_MCP_IN:
      case GPIO_TYPE_I2C_MCP_IN_PULLUP:
        current_value = *data->gpio_data_in[gpio];
        break;
      case GPIO_TYPE_NATIVE_OUT:
      case GPIO_TYPE_NATIVE_OUT_DEBUG:
      case GPIO_TYPE_I2C_MCP_OUT:
        current_value = *data->gpio_data_out_invert[gpio] ? ! *data->gpio_data_out[gpio] : *data->gpio_data_out[gpio];
        break;
      default:
        break;
    }

    // The value last received over the network.
    bool received_value = data->gpio_data_received[bank] & (0x1 << gpio_per_bank);

    if(current_value != received_value) {
      switch(data->gpio_type[gpio]) {
        case GPIO_TYPE_NATIVE_IN_DEBUG:
          printf("DBG: GPIO IN: %u  val: %u\n", gpio, current_value);
          // Note: no break here.
        case GPIO_TYPE_NATIVE_IN:
        case GPIO_TYPE_I2C_MCP_IN:
        case GPIO_TYPE_I2C_MCP_IN_PULLUP:
          // Network update to apply.
          *data->gpio_data_in[gpio] = received_value;
          *data->gpio_data_in_not[gpio] = !received_value;
          current_value = received_value;
          // Confirmation of HAL update to send on network.
          confirmation_pending[bank] = true;
          break;
        case GPIO_TYPE_NATIVE_OUT_DEBUG:
          printf("DBG: GPIO OUT: %u  val: %u\n", gpio, current_value);
          // Note: no break here.
        case GPIO_TYPE_NATIVE_OUT:
        case GPIO_TYPE_I2C_MCP_OUT:
          // HAL update to send on network.
          confirmation_pending[bank] = true;
          break;
        default:
          current_value = received_value;
          break;
      }
    }

    to_send[bank] |= ((uint32_t)current_value << gpio_per_bank);
  }

  for(int bank = 0; bank < MAX_GPIO_BANK; bank++) {
    if(confirmation_pending[bank] || data->gpio_confirmation_pending[bank]) {
      // Values differ from those received in the last NW update
      // or the last network update requested confirmation.
      struct Message_gpio message = {
        .type = MSG_SET_GPIO,
        .bank = bank,
        .values = to_send[bank],
        .confirmation_pending=confirmation_pending[bank]
      };
      return_val += pack_nw_buff(buffer, &message, sizeof(struct Message_gpio));
    }
  }

  return return_val;
}

/* Process received update documenting current the last packet received by the RP. */
bool unpack_timing(
    struct NWBuffer* rx_buf,
    size_t* rx_offset,
    size_t* received_count,
    skeleton_t* data
) {
  UNPACK_MSG(struct Reply_timing, reply, rx_buf, rx_offset);
  *data->seq_in = reply->update_id;
  *data->packet_interval = reply->time_diff;

  (*received_count)++;
  return true;
}

/* Process received update documenting current joint position and velocity. */
bool unpack_joint_movement(
    struct NWBuffer* rx_buf,
    size_t* rx_offset,
    size_t* received_count,
    skeleton_t* data
) {
  UNPACK_MSG(struct Reply_joint_movement, reply, rx_buf, rx_offset);

  if(detected_joint_count == 0) {
    detected_joint_count = reply->count;
    printf("INFO: firmware reports %u joints\n", detected_joint_count);
  } else if(detected_joint_count != reply->count) {
    printf("WARN: joint count changed %u -> %u; reflash firmware and reinstall driver\n",
        detected_joint_count, reply->count);
    detected_joint_count = reply->count;
  }

  size_t n = reply->count < MAX_JOINT ? reply->count : MAX_JOINT;
  for(size_t joint = 0; joint < n; joint++) {
    *data->joint_pos_fb[joint] =
      ((double)reply->abs_pos_achieved[joint]) / *data->joint_scale[joint];

    /* velocity_achieved is Q16.16 steps/period (exact internal value);
     * divide by 65536 to get steps/period as a float. */
    *data->joint_vel_fb[joint] =
      (double)reply->velocity_achieved[joint] / 65536.0;

    *data->joint_pos_error_fb[joint] = (int32_t)round(
        (*data->joint_pos_cmd[joint] - *data->joint_pos_fb[joint])
        * *data->joint_scale[joint]);

    *data->joint_enable_fb[joint]       = reply->enabled[joint];
    *data->joint_vel_calculated[joint]  = reply->velocity_cmd[joint];
  }

  *data->core1_period = reply->update_period_us;
  *data->core1_tick   = reply->core1_tick;

  (*received_count)++;
  return true;
}

uint8_t get_detected_joint_count(void) {
  return detected_joint_count;
}

size_t serialize_version_request(struct NWBuffer* buffer) {
  union MessageAny message;
  message.version_request.type = MSG_VERSION_REQUEST;
  return pack_nw_buff(buffer, &message, sizeof(struct Message_version_request));
}

bool get_version_checked(void) {
  return version_checked;
}

bool get_version_match(void) {
  return version_match;
}

void reset_version_check(void) {
  version_checked = false;
  version_match   = false;
}

bool unpack_version_reply(
    struct NWBuffer* rx_buf,
    size_t* rx_offset,
    size_t* received_count
) {
  UNPACK_MSG(struct Reply_version, reply, rx_buf, rx_offset);
  if (!version_checked) {
    bool ver_ok = (reply->version_major == PROTOCOL_VERSION_MAJOR &&
                   reply->version_minor == PROTOCOL_VERSION_MINOR &&
                   reply->version_patch == PROTOCOL_VERSION_PATCH);
    bool branch_ok = (reply->version_branch == PROTOCOL_VERSION_BRANCH);
    if (!ver_ok) {
      rtapi_print_msg(RTAPI_MSG_ERR,
          "RP2040: ERROR: version mismatch — firmware %d.%d.%d, driver %d.%d.%d. "
          "Reflash firmware and reinstall driver from the same source.\n",
          reply->version_major, reply->version_minor, reply->version_patch,
          PROTOCOL_VERSION_MAJOR, PROTOCOL_VERSION_MINOR, PROTOCOL_VERSION_PATCH);
    } else if (!branch_ok) {
      rtapi_print_msg(RTAPI_MSG_ERR,
          "RP2040: ERROR: branch mismatch — firmware 0x%08x, driver 0x%08x. "
          "Reflash firmware and reinstall driver from the same branch.\n",
          reply->version_branch, PROTOCOL_VERSION_BRANCH);
    } else {
      rtapi_print_msg(RTAPI_MSG_INFO,
          "RP2040: INFO: version OK (%d.%d.%d branch 0x%08x)\n",
          reply->version_major, reply->version_minor, reply->version_patch,
          reply->version_branch);
    }
    version_match   = ver_ok && branch_ok;
    version_checked = true;
  }
  (*received_count)++;
  return true;
}

/* Update last_joint_config with the values the RP confirmed — this stops
 * configure_joint() from retransmitting (diff disappears). If the reply never
 * arrives the diff persists and the config is resent next rotation. */
bool unpack_joint_config(
    struct NWBuffer* rx_buf,
    size_t* rx_offset,
    size_t* received_count,
    struct Message_joint_config* last_joint_config
) {
  UNPACK_MSG(struct Reply_joint_config, reply, rx_buf, rx_offset);
  size_t joint = reply->joint;

  printf("INFO: Received confirmation of config received by RP for joint: %u\n", joint);
  printf("      enable:       %u\n", reply->enable);
  printf("      gpio_step:    %i\n", reply->gpio_step);
  printf("      gpio_dir:     %i\n", reply->gpio_dir);
  printf("      max_velocity: %f\n", reply->max_velocity);
  printf("      max_accel:    %f\n", reply->max_accel);
  printf("      cmd_type:     %u\n", reply->cmd_type);

  last_joint_config[joint].enable = reply->enable;
  last_joint_config[joint].gpio_step = reply->gpio_step;
  last_joint_config[joint].gpio_dir = reply->gpio_dir;
  last_joint_config[joint].max_velocity = reply->max_velocity;
  last_joint_config[joint].max_accel = reply->max_accel;
  last_joint_config[joint].cmd_type = reply->cmd_type;

  (*received_count)++;
  return true;
}

/* Update last_gpio_config with the values the RP confirmed — this stops
 * configure_gpio() from retransmitting (diff disappears). If the reply never
 * arrives the diff persists and the config is resent next rotation. */
bool unpack_gpio_config(
    struct NWBuffer* rx_buf,
    size_t* rx_offset,
    size_t* received_count,
    struct Message_gpio_config* last_gpio_config
) {
  UNPACK_MSG(struct Reply_gpio_config, reply, rx_buf, rx_offset);
  size_t gpio = reply->gpio_count;

  printf("INFO: Received confirmation of config received by RP for gpio: %u\n", gpio);
  printf("      gpio_type:   %i\n", reply->gpio_type);
  printf("      index:       %i\n", reply->index);
  printf("      address:     %i\n", reply->address);

  last_gpio_config[gpio].gpio_type = reply->gpio_type;
  last_gpio_config[gpio].index = reply->index;
  last_gpio_config[gpio].address = reply->address;

  (*received_count)++;
  return true;
}

/* EMA time constant: τ = 1 s at the nominal 1 kHz servo rate. */
#define EMA_ALPHA (1.0 / 1000.0)

/* Process received update containing metrics data. */
bool unpack_joint_metrics(
    struct NWBuffer* rx_buf,
    size_t* rx_offset,
    size_t* received_count,
    skeleton_t* data
) {
  UNPACK_MSG(struct Reply_joint_metrics, reply, rx_buf, rx_offset);

  data->ema_overrun  = data->ema_overrun  * (1.0 - EMA_ALPHA) + reply->overrun_occurred  * EMA_ALPHA;
  data->ema_underrun = data->ema_underrun * (1.0 - EMA_ALPHA) + reply->underrun_occurred * EMA_ALPHA;

  *data->update_overrun  = (hal_float_t)data->ema_overrun;
  *data->update_underrun = (hal_float_t)data->ema_underrun;
  *data->core1_work_us   = reply->core1_work_us;
  *data->core0_work_us   = reply->core0_work_us;

  (*received_count)++;
  return true;
}

/* Process received update containing spindle data. */
bool unpack_spindle_speed(
    struct NWBuffer* rx_buf,
    size_t* rx_offset,
    size_t* received_count,
    skeleton_t* data
) {
  UNPACK_MSG(struct Reply_spindle_speed, reply, rx_buf, rx_offset);

  uint8_t spindle = reply->spindle_index;

  float rpm = reply->speed * 120.0 / data->spindle_poles[spindle];
  float expected_rpm = *data->spindle_speed_cmd[spindle];
  *data->spindle_speed_fb[spindle] = rpm;
  *data->spindle_at_speed[spindle] = fabs(rpm - expected_rpm) < 10.0;

  (*received_count)++;
  return true;
}

/* Update last_spindle_config with the values the RP confirmed — this stops
 * configure_spindle() from retransmitting (diff disappears). If the reply never
 * arrives the diff persists and the config is resent next rotation. */
bool unpack_spindle_config(
    struct NWBuffer* rx_buf,
    size_t* rx_offset,
    size_t* received_count,
    struct Message_spindle_config* last_spindle_config
) {
  UNPACK_MSG(struct Reply_spindle_config, reply, rx_buf, rx_offset);
  size_t spindle_index = reply->spindle_index;

  printf("INFO: Received confirmation of config received by RP for spindle: %u\n", spindle_index);
  printf("      modbus_address   %i\n", reply->modbus_address);
  printf("      vfd_type:        %i\n", reply->vfd_type);
  printf("      bitrate:         %i\n", reply->bitrate);

  last_spindle_config[spindle_index].modbus_address = reply->modbus_address;
  last_spindle_config[spindle_index].vfd_type = reply->vfd_type;
  last_spindle_config[spindle_index].bitrate = reply->bitrate;

  (*received_count)++;
  return true;
}

/* Process received update containing GPIO values. */
bool unpack_gpio(
    struct NWBuffer* rx_buf,
    size_t* rx_offset,
    size_t* received_count,
    skeleton_t* data
) {
  UNPACK_MSG(struct Reply_gpio, reply, rx_buf, rx_offset);
  size_t bank = reply->bank;
  uint32_t values = reply->values;

  data->gpio_confirmation_pending[bank] = reply->confirmation_pending;

  data->gpio_data_received[bank] = values;
  (*received_count)++;
  return true;
}

/* Extract structs from data received over network. */
void process_data(
    struct NWBuffer* rx_buf,
    skeleton_t* data,
    size_t* received_count,
    size_t expected_length,
    struct Message_joint_config* last_joint_config,
    struct Message_gpio_config* last_gpio_config,
    struct Message_spindle_config* last_spindle_config
) {
  size_t rx_offset = 0;

  if(rx_buf->length + sizeof(rx_buf->length) + sizeof(rx_buf->checksum) != expected_length) {
    printf("WARN: RX length not equal to expected. %u\n", *received_count);
    return;
  }
  if(rx_buf->length > NW_BUF_LEN) {
    printf("WARN: RX length greater than buffer size. %u\n", *received_count);
    return;
  }

  if(!checkNWBuff(rx_buf)) {
    printf("WARN: RX checksum fail.\n");
    return;
  }

  bool unpack_success = true;

  while(unpack_success) {
    struct Reply_header* header = unpack_nw_buff(
        rx_buf, rx_offset, NULL, NULL, sizeof(struct Reply_header));

    if(!header || ! header->type) {
      // Legitimate end of data.
      break;
    }
    switch(header->type) {
      case REPLY_VERSION:
        unpack_success = unpack_success && unpack_version_reply(
            rx_buf, &rx_offset, received_count);
        break;
      case REPLY_TIMING:
        ;
        unpack_success = unpack_success && unpack_timing(
            rx_buf, &rx_offset, received_count, data);
        break;
      case REPLY_JOINT_MOVEMENT:
        unpack_success = unpack_success && unpack_joint_movement(
            rx_buf, &rx_offset, received_count, data);
        break;
      case REPLY_JOINT_CONFIG:
        unpack_success = unpack_success && unpack_joint_config(
            rx_buf, &rx_offset, received_count, last_joint_config);
        break;
      case REPLY_GPIO_CONFIG:
        unpack_success = unpack_success && unpack_gpio_config(
            rx_buf, &rx_offset, received_count, last_gpio_config);
        break;
      case REPLY_JOINT_METRICS:
        unpack_success = unpack_success && unpack_joint_metrics(
            rx_buf, &rx_offset, received_count, data);
        break;
      case REPLY_SPINDLE_SPEED:
        unpack_success = unpack_success && unpack_spindle_speed(
            rx_buf, &rx_offset, received_count, data);
        break;
      case REPLY_SPINDLE_CONFIG:
        unpack_success = unpack_success && unpack_spindle_config(
            rx_buf, &rx_offset, received_count, last_spindle_config);
        break;
      case REPLY_GPIO:
        unpack_success = unpack_success && unpack_gpio(
            rx_buf, &rx_offset, received_count, data);
        break;
      default:
        printf("WARN: Invalid message type: %u\t%lu\n", header->type, *received_count);
        // Implies data corruption.
        unpack_success = false;
        break;
    }
  }

  if(rx_offset < rx_buf->length) {
    printf("WARN: Unconsumed RX buffer remainder: %u bytes.\t%u\n",
        rx_buf->length - rx_offset, *received_count);
    // Implies data corruption.
    // Received a message type in the header but not enough data in the buffer
    // to populate the struct.
  }
}

