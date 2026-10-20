/*
 * Buzzer control module (PWM output level + k_timer rhythm).
 *
 * Responsibility boundary: this module understands only its own
 * enum buzzer_pattern. It does not know what a "distance grade" is and does not
 * know what a "password" is. The distance side maps its grade to a pattern and
 * passes that in; the lock side passes a single boolean. As a result the module
 * can be tested on its own and is unaffected by distance-threshold tuning or by
 * changes in the unlock policy.
 */

#ifndef BUZZER_H_
#define BUZZER_H_

#include <stdbool.h>

/** @brief Alarm rhythm. */
enum buzzer_pattern {
	BUZZER_SILENT = 0,	/**< silent */
	BUZZER_SLOW,		/**< slow: an occasional short beep */
	BUZZER_FAST,		/**< fast: urgent on/off */
	BUZZER_CONTINUOUS,	/**< continuous tone */
};

/** @brief Initialise the PWM channel and drive the output to silence.
 *
 * @retval 0 on success, or a negative error code (PWM device not ready, ...).
 */
int buzzer_init(void);

/** @brief Master enable, driven by the unlock state and the Ring mute latch.
 *
 * While enabled == false everything is silent regardless of what the distance
 * side or the manual-ring side requested -- this is where "the password gates
 * the whole lost-item alarm" lands, and also where a Ring 0 write mutes the
 * automatic distance alarm. Passing the same value again is ignored, so callers
 * may simply call it once per loop iteration.
 */
void buzzer_set_enabled(bool enabled);

/** @brief Set the rhythm requested by the distance side.
 *
 * Does nothing when the request equals the current one -- otherwise a call every
 * second would keep interrupting a long-interval rhythm and "beep for 60 ms,
 * wait 3 s" would degrade to "beep once a second".
 */
void buzzer_set_pattern(enum buzzer_pattern pattern);

/** @brief Turn the manual ring request on or off. Also ignores a repeated value.
 *
 * Priority: BUZZER_CONTINUOUS / BUZZER_FAST (that is, LOST / FAR) outrank the
 * ring request. See buzzer_effective_pattern() in buzzer.c.
 */
void buzzer_set_ring(bool on);

#endif /* BUZZER_H_ */
