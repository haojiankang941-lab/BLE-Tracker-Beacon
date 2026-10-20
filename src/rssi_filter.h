/*
 * RSSI moving-average filter.
 *
 * Responsibility boundary: this module only turns "a noisy stream of dBm values"
 * into "a smoothed stream of dBm values". It knows nothing about distance and
 * nothing about the buzzer, so it can be exercised without Bluetooth at all --
 * feed it a stream of numbers and inspect the output.
 */

#ifndef RSSI_FILTER_H_
#define RSSI_FILTER_H_

#include <stdint.h>

/** @brief Sliding-window length N (number of samples retained).
 *
 * Trade-off (counts below assume the poll period set by RSSI_POLL_INTERVAL_MS
 * in main.c, currently 100 ms):
 *   too short (e.g. 2 samples = 0.2 s)  -> barely smooths anything, a single
 *                          jittery reading passes straight through;
 *   too long (e.g. 50 samples = 5 s)    -> so sluggish that the buzzer reacts
 *                          long after the wearer has already walked away. For a
 *                          lost-item alarm, lag is more dangerous than jitter.
 * 5 samples at 100 ms is a 0.5 s observation window. Raising this to 50
 * restores the ~5 s window that was in use at the former 1 s poll rate.
 */
#define RSSI_FILTER_WINDOW 5

/** @brief Clear the filter back to the "no samples yet" state. */
void rssi_filter_init(void);

/** @brief Push one new sample and return the resulting average.
 *
 * @param[in] sample Raw RSSI for this poll (dBm, normally negative).
 *                   The caller must ensure it is a valid reading: the
 *                   controller's "not available" marker 0x7F
 *                   (BT_HCI_LE_RSSI_NOT_AVAILABLE) must never reach this
 *                   function. That check lives in read_conn_rssi() in main.c.
 *
 * @return Average RSSI over the current window (dBm). While fewer than N
 *         samples have been pushed, the average is taken over what is there,
 *         so a fresh connection yields a value immediately instead of stalling
 *         for N seconds.
 */
int8_t rssi_filter_update(int8_t sample);

#endif /* RSSI_FILTER_H_ */
