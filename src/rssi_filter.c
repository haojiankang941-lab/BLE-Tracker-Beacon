/*
 * RSSI moving-average filter -- implementation.
 */

#include "rssi_filter.h"

/* ---- Internal state ----
 * window[] is a ring buffer: head points at the slot to be written next, which
 * is also where the oldest sample lives, so the next sample naturally overwrites
 * it.
 */
static int16_t window[RSSI_FILTER_WINDOW];
static uint8_t head;	/* next slot to write, 0 .. N-1 */
static uint8_t count;	/* number of valid samples, 0 .. N */
static int32_t sum;	/* running sum of every valid sample in the window */
static int8_t  avg;	/* most recently computed average */

/* ---- Diagnostic mirrors (deliberately not static) ----
 * On this board printk goes to UART0, which is not wired to the J-Link VCOM, so
 * printk output is invisible in practice. Publishing the key intermediate values
 * as globals with external linkage gives the debugger something to read out of
 * RAM. External linkage also guarantees the linker cannot discard them.
 */
int32_t g_rssi_raw;	/* most recent raw sample pushed in */
int32_t g_rssi_avg;	/* most recently computed average */
int32_t g_rssi_count;	/* current number of samples in the window */
int32_t g_rssi_head;	/* ring-buffer write pointer */


void rssi_filter_init(void)
{
	head = 0U;
	count = 0U;
	sum = 0;
	avg = 0;
	g_rssi_count = 0;
}


/* Integer division that rounds to nearest: sum / count.
 *
 * Why round instead of using C's plain integer division: C99 `/` truncates
 * toward zero, so -62.6 becomes -62. RSSI is always negative, so every average
 * would be biased upward -- i.e. toward "stronger signal", "closer". For an
 * application that decides whether someone has walked away, that is exactly the
 * wrong direction to be wrong in.
 *
 * The implementation adds half the divisor before truncating, taking the
 * absolute value first when negative so the result stays sign-symmetric
 * (round half away from zero). |sum| peaks around 640, so nothing overflows.
 */
static int8_t rssi_filter_round_div(int32_t s, uint8_t n)
{
	int32_t half = (int32_t)n / 2;

	if (s >= 0) {
		return (int8_t)((s + half) / (int32_t)n);
	}

	return (int8_t)(-(((-s) + half) / (int32_t)n));
}


int8_t rssi_filter_update(int8_t sample)
{
	if (count < RSSI_FILTER_WINDOW) {
		/* Window not full yet: just add. Nothing has to be subtracted,
		 * which lets a fresh connection produce a value immediately
		 * instead of waiting N seconds for the window to fill. */
		sum += sample;
		count++;
	} else {
		/* Window is full: subtract the sample about to be overwritten,
		 * then add the new one. One add and one subtract per sample keeps
		 * this O(1); recomputing the sum of N values would be O(N). */
		sum -= window[head];
		sum += sample;
	}

	window[head] = sample;
	head = (uint8_t)((head + 1U) % RSSI_FILTER_WINDOW);

	avg = rssi_filter_round_div(sum, count);

	g_rssi_raw = sample;
	g_rssi_avg = avg;
	g_rssi_count = count;
	g_rssi_head = head;

	return avg;
}
