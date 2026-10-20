/*
 * Distance grading module.
 *
 * Responsibility boundary: this module does one thing -- "filtered dBm in, one
 * enum out". It does not know the buzzer exists, nor what an alarm rhythm looks
 * like. Turning a grade into a rhythm is main.c's job.
 */

#ifndef DISTANCE_MANAGER_H_
#define DISTANCE_MANAGER_H_

#include <stdint.h>

/** @brief Distance grade.
 *
 * The ORDER of the enumerators is significant: the hysteresis algorithm below
 * compares (uint8_t)level directly, so the values must increase strictly from
 * nearest to farthest. Inserting a new grade in the middle also requires
 * extending dist_threshold[] by one entry, or the number of thresholds and the
 * number of grades will no longer line up.
 */
enum distance_level {
	DIST_NEAR = 0,	/**< RSSI >= -50 dBm: right next to the phone */
	DIST_NORMAL,	/**< -65 .. -50 dBm: normal separation */
	DIST_FAR,	/**< -80 .. -65 dBm: getting away */
	DIST_LOST,	/**< RSSI < -80 dBm: presumed lost */
	DIST_UNKNOWN,	/**< No valid sample has ever arrived (not connected yet,
			 *   or just started). It is listed after DIST_LOST but does
			 *   NOT mean "farther than lost" -- it merely keeps the
			 *   values 0..3 cleanly mapped onto the four real grades.
			 */
};

/** @brief Boundaries between the first three grades (dBm). Entry 0 separates
 *         NEAR/NORMAL, entry 1 separates NORMAL/FAR, entry 2 separates
 *         FAR/LOST. They are the thresholds in the "moving away" direction.
 */
#define DIST_TH_NEAR_DBM (-50)
#define DIST_TH_FAR_DBM  (-65)
#define DIST_TH_LOST_DBM (-80)

/** @brief Hysteresis band (dB).
 *
 * Why it is required: RSSI dithers around a boundary (a +-5 dB swing is normal
 * in practice). With a single threshold, a device sitting near -65 dBm would
 * flip between NORMAL and FAR continuously, so the buzzer would alternate
 * between two rhythms and sound broken. The band gives the state memory: to
 * fall back from FAR to NORMAL the signal must be 3 dB stronger than what it
 * took to enter FAR.
 */
#define DIST_HYSTERESIS_DB 3

/** @brief Reset to the "no sample received yet" state. */
void distance_manager_reset(void);

/** @brief Feed a new filtered value, update the grade and return it. */
enum distance_level distance_manager_update(int8_t filtered_rssi);

/** @brief Human-readable grade name, for logging only. */
const char *distance_level_str(enum distance_level lv);

#endif /* DISTANCE_MANAGER_H_ */
