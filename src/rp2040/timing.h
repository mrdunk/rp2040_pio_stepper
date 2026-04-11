#ifndef TIMING__H
#define TIMING__H

/* Set up the hardware repeating timer that drives Core1's tick semaphore.
 * Must be called once from core0_main() before entering the packet loop. */
void timing_init(void);

/* Called after receiving a network packet.
 * Updates the EMA of inter-packet period; restarts the timer if the
 * integer-µs average changes. Does not busy-wait. */
void recover_clock(void);

#ifdef BUILD_TESTS
/* Reset all static state — used by test setup fixtures only. */
void timing_reset_for_test(void);
#endif

#endif  // TIMING__H
