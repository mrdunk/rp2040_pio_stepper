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
static uint32_t ave_period_us_x64 = 1000u << 6;
static uint32_t last_period_us    = 0;
static uint64_t time_last         = 0;
static bool     time_initialized  = false;
static alarm_id_t tick_alarm      = -1;

/* Hardware alarm ISR — increments Core1's tick semaphore.
 * Negative return tells the SDK to reschedule from the scheduled fire time
 * rather than from now, preventing drift accumulation on delayed wakeups. */
static int64_t tick_alarm_callback(alarm_id_t id, void *user_data) {
    (void) id; (void) user_data;
    tick++;
    return -(int64_t)(ave_period_us_x64 >> 6);
}

/* Called once from core0_main() before the packet loop. */
void timing_init(void) {
    uint32_t period_us = ave_period_us_x64 >> 6;
    last_period_us     = period_us;
    absolute_time_t fire_at = from_us_since_boot(time_us_64() + period_us / 4);
    tick_alarm = add_alarm_at(fire_at, tick_alarm_callback, NULL, true);
}

/* Called after every received packet.
 * Updates the EMA of inter-packet period; calls update_period when the
 * integer-µs average changes.  Phase-locks the tick alarm to the packet
 * arrival time: reschedules the alarm at time_now + period/4 on every call,
 * so the tick fires at a stable phase offset ahead of the next expected packet,
 * keeping overrun and underrun rates symmetric. */
void recover_clock(void) {
    uint64_t time_now = time_us_64();

    if (time_initialized) {
        uint32_t sample  = (uint32_t)(time_now - time_last);
        int32_t  id_diff = get_last_id_diff();
        if (id_diff >= 1) {
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

    uint32_t period_us = ave_period_us_x64 >> 6;
    if (period_us == 0) {
        period_us = 1;  /* guard against zero on startup */
    }

    if (last_period_us != period_us) {
        update_period(period_us);
        last_period_us = period_us;
    }

    /* Phase-lock: reschedule tick from this packet's arrival time + period/4.
     * Cancelling and rescheduling a one-shot alarm on every packet does not
     * starve Core1 (unlike restarting a repeating timer), because the fire
     * time is an absolute value rather than a countdown from now. */
    if (tick_alarm >= 0) {
        cancel_alarm(tick_alarm);
    }
    absolute_time_t fire_at = from_us_since_boot(time_now + period_us / 4);
    tick_alarm = add_alarm_at(fire_at, tick_alarm_callback, NULL, true);
}

#ifdef BUILD_TESTS
void timing_reset_for_test(void) {
    ave_period_us_x64 = 1000u << 6;
    last_period_us    = 0;
    time_last         = 0;
    time_initialized  = false;
    tick_alarm        = -1;
}
#endif
