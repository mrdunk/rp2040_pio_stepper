#ifndef DRIVER_MOCKS__H
#define DRIVER_MOCKS__H

#include <errno.h>
#include <stdbool.h>
#include <limits.h>
#include <stdint.h>
#include "../../driver/rp2040_defines.h"
#include "../../shared/messages.h"


#define RTAPI_MSG_ERR 1

typedef uint32_t hal_u32_t;
typedef int32_t hal_s32_t;
typedef double hal_float_t;
typedef bool hal_bit_t;

#include "../../driver/skeleton.h"


void rtapi_print_msg();


#endif  // DRIVER_MOCKS__H
