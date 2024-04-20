#ifndef _MOCK_WATCHDOG_H
#define _MOCK_WATCHDOG_H

#include <stdbool.h>

void watchdog_update();

void watchdog_enable (uint32_t delay_ms, bool pause_on_debug);


#endif  // _MOCK_WATCHDOG_H
