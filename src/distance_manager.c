/*
 * Distance grading module -- implementation.
 *
 * ==================== Why grade instead of compute distance ====================
 *
 * The textbook free-space path loss model is
 *     RSSI(d) = RSSI(d0) - 10*n*log10(d / d0)
 * which can in principle be inverted to solve for d. In practice nobody does
 * that, for four reasons:
 *
 * 1. RSSI does not measure distance, it measures received power. The same power
 *    reads several dB different after a change of obstacle, a turn of the body,
 *    or a hand over the antenna.
 *
 * 2. The path loss exponent n is environment-dependent: n~2 in free space, n~3
 *    in an office, n~4..6 through walls. The same device has a different n in
 *    the living room and in the stairwell, so a curve calibrated in one place is
 *    invalid in another.
 *
 * 3. RSSI quantisation is very coarse. BLE controllers report whole dBm, and
 *    the 10*log10 relationship means 10% distance accuracy would need RSSI
 *    accurate to 0.4 dB -- beyond what the radio can measure.
 *
 * 4. Worse, 10*log10(d) is logarithmic: the farther away, the less distance
 *    changes per dB. Near -50 dBm, 6 dB might mean the distance doubled; near
 *    -90 dBm, 6 dB might mean it multiplied several times. The computed distance
 *    is therefore least accurate precisely where it matters most.
 *
 * Conclusion: RSSI is good for "close / ok / far / gone" and bad for "3.7 m
 * away". This module therefore outputs a grade, not a distance.
 *
 * ==================== How the hysteresis works ====================
 *
 * All three boundaries share one algorithm instead of their own if/else:
 *
 *   For boundary i, if the current grade is already on its "far" side (that is,
 *   level > i), then crossing back to the "near" side requires beating the
 *   nominal threshold by HYST dB; otherwise the nominal threshold is used.
 *
 *   Then simply count how many boundaries the value falls outside of -- that
 *   count is the new grade.
 *
 * The benefit: the grade can still jump all the way in one step (NEAR straight
 * to LOST is not dragged out gradually), while each boundary independently
 * keeps its own backlash.
 */

#include "distance_manager.h"

static const int8_t dist_threshold[] = {
	DIST_TH_NEAR_DBM,
	DIST_TH_FAR_DBM,
	DIST_TH_LOST_DBM,
};

/* Three boundaries split the space into four grades, so the threshold count is
 * always DIST_LOST (== 3).
 */
#define DIST_THRESHOLD_COUNT 3

static enum distance_level cur_level = DIST_UNKNOWN;
static int8_t cur_rssi;

/* Diagnostic mirrors (deliberately not static -- see the same note in
 * rssi_filter.c): printk is invisible on this board, so read these out of RAM
 * to see what the module decided.
 */
int32_t g_dist_level = DIST_UNKNOWN;
int32_t g_dist_rssi;


void distance_manager_reset(void)
{
	cur_level = DIST_UNKNOWN;
	cur_rssi = 0;
	g_dist_level = DIST_UNKNOWN;
	g_dist_rssi = 0;
}


enum distance_level distance_manager_update(int8_t filtered_rssi)
{
	uint8_t below = 0U;

	for (uint8_t i = 0U; i < DIST_THRESHOLD_COUNT; i++) {
		int8_t th = dist_threshold[i];

		/* cur_level == DIST_UNKNOWN means there is no history yet, and
		 * the nominal threshold must be used. Its enumerator value is 4,
		 * larger than any i, so without this special case it would look
		 * like the value is already beyond every boundary and hysteresis
		 * would be applied to all of them at once. */
		if (cur_level != DIST_UNKNOWN && (uint8_t)cur_level > i) {
			th = (int8_t)(th + DIST_HYSTERESIS_DB);
		}

		if (filtered_rssi < th) {
			below++;
		}
	}

	/* below ranges over 0..3 and maps one-to-one onto NEAR / NORMAL / FAR /
	 * LOST. That mapping depends on the enumerator order in
	 * distance_manager.h. */
	cur_level = (enum distance_level)below;
	cur_rssi = filtered_rssi;

	g_dist_level = (int32_t)cur_level;
	g_dist_rssi = filtered_rssi;

	return cur_level;
}


const char *distance_level_str(enum distance_level lv)
{
	switch (lv) {
	case DIST_NEAR:
		return "NEAR";
	case DIST_NORMAL:
		return "NORMAL";
	case DIST_FAR:
		return "FAR";
	case DIST_LOST:
		return "LOST";
	case DIST_UNKNOWN:
	default:
		return "UNKNOWN";
	}
}
