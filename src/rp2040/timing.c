#include <stdint.h>

#include "config.h"

#ifdef BUILD_TESTS

#include "../test/mocks/rp_mocks.h"

#else  // BUILD_TESTS

#include "pico/stdlib.h"
#include "pico/time.h"

#endif  // BUILD_TESTS

#include "timing.h"

/* EMA accumulator, stored ×64 for sub-µs accumulation.
 * Initialised to 1000 µs (LinuxCNC default servo period). */
static uint32_t ave_period_us_x64   = 1000u << 6;
static uint32_t last_timer_period_us = 0;
static uint64_t time_last            = 0;
static bool     time_initialized     = false;
static repeating_timer_t tick_timer;

/* Hardware timer ISR — increments Core1's tick semaphore. */
static bool tick_callback(repeating_timer_t *rt) {
    (void) rt;
    tick++;
    return true;  /* keep repeating */
}

/* Called once from core0_main() before the packet loop. */
void timing_init(void) {
    uint32_t ave_period_us = ave_period_us_x64 >> 6;
    add_repeating_timer_us(-(int32_t)ave_period_us, tick_callback, NULL, &tick_timer);
    last_timer_period_us = ave_period_us;
}

/* Called after every received packet.
 * Updates the EMA of inter-packet period; restarts the timer only when the
 * integer-µs average changes.  No busy-wait. */
void recover_clock(void) {
    uint64_t time_now = time_us_64();

    if(time_initialized) {
        uint32_t sample  = (uint32_t)(time_now - time_last);
        int32_t  id_diff = get_last_id_diff();
        if(id_diff >= 1) {
            /* Normalise by id_diff so that missed packets (id_diff > 1) do not
             * inflate the EMA.  When id_diff == 1 (normal) this is a no-op.
             * id_diff <= 0 means LinuxCNC restarted or first packet — skip the
             * EMA update entirely and keep the current timer period. */
            uint32_t normalized = sample / (uint32_t)id_diff;
            /* EMA: alpha = 1/64.  Accumulator stores ave × 64 so that sub-µs
             * drift accumulates before changing the integer output. */
            ave_period_us_x64 = ave_period_us_x64 - (ave_period_us_x64 >> 6) + normalized;
        }
        /* else: no reliable timing data — keep current EMA */
    }
    time_last        = time_now;
    time_initialized = true;

    uint32_t ave_period_us = ave_period_us_x64 >> 6;
    if(ave_period_us == 0) {
        ave_period_us = 1;  /* guard against zero on startup */
    }

    if(last_timer_period_us != ave_period_us) {
        cancel_repeating_timer(&tick_timer);
        add_repeating_timer_us(-(int32_t)ave_period_us, tick_callback, NULL, &tick_timer);
        update_period(ave_period_us);
        last_timer_period_us = ave_period_us;
    }
}

#ifdef BUILD_TESTS
void timing_reset_for_test(void) {
    ave_period_us_x64    = 1000u << 6;
    last_timer_period_us = 0;
    time_last            = 0;
    time_initialized     = false;
}
#endif
