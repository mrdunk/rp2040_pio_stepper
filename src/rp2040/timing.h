#ifndef TIMING__H
#define TIMING__H

/* Set up the hardware repeating timer that drives Core1's tick semaphore.
 * Must be called once from core0_main() before entering the packet loop. */
void timing_init(void);

/* Called after receiving a network packet.
 * Updates the EMA of inter-packet period; calls update_period when the
 * integer-µs average changes.  Phase-locks the tick alarm to the packet
 * arrival time + period/4 on every call to keep overrun/underrun symmetric. */
void recover_clock(void);

#ifdef BUILD_TESTS
/* Reset all static state — used by test setup fixtures only. */
void timing_reset_for_test(void);
#endif

#endif  // TIMING__H
