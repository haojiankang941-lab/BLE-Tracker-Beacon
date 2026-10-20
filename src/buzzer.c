/*
 * Buzzer control module -- implementation.
 *
 * ==================== Approach ====================
 *
 * The output level is produced by a PWM channel; the rhythm is produced by a
 * k_timer. Each does one job:
 *
 *   PWM     -> should this pin be "active" or "inactive" right now (i.e. sound
 *              or not sound)
 *   k_timer -> when to flip it
 *
 * Why not PWM alone: the nRF52 PWM peripheral tops out at a period of roughly
 * 262 ms (pwm_nrfx.c:101-146 -- prescaler limited to /128 and COUNTERTOP to
 * about 15 bits), which cannot express a second-scale rhythm such as "beep for
 * 60 ms, wait 3 s" in a single configuration.
 *
 * Why not GPIO alone: the requirement is to drive the buzzer through the Zephyr
 * PWM API, which also keeps the door open for a future *passive* buzzer -- that
 * would only need the duty cycle changed to a 50% square wave at 2..4 kHz, with
 * no other code touched.
 *
 * ==================== Why PWM can act as a plain switch ====================
 *
 * This is not a trick; it is part of the PWM subsystem's contract. pwm.h:596-599
 * states:
 *
 *     Passing 0 as @p pulse will cause the pin to be driven to a constant
 *     inactive level.
 *     Passing a non-zero @p pulse equal to @p period will cause the pin to be
 *     driven to a constant active level.
 *
 * The driver matches that (pwm_nrfx.c:199-207): a pulse of 0 or >= period
 * resolves to a constant compare value, and only the genuine intermediate
 * waveform path calls pwm_period_check_and_set(); afterwards :238-249 writes the
 * pin level through nrf_gpio_pin_write() as if it were an ordinary GPIO.
 *
 * That distinction matters here: the period check is never reached, so this
 * module is not bound by the 262 ms limit, and the busy-wait
 * `while (!nrfx_pwm_stopped_check(...))` at pwm_nrfx.c:280 is never reached
 * either (it only runs while pwm_needed != 0). This module can therefore call
 * pwm_set_dt() safely from a k_timer interrupt callback.
 *
 * ==================== About the buzzer being active ====================
 *
 * This board carries an *active* buzzer: it has its own internal oscillator and
 * sounds as soon as it sees a DC level. Neither the frequency nor the duty cycle
 * of a PWM wave changes its pitch or volume, so this module uses PWM only to
 * express "sound / silence" and the rhythm.
 *
 * ==================== Polarity ====================
 *
 * The buzzer on this board is *active high*. Schematic: the MCU pin drives the
 * base of NPN transistor Q1 (2N3904) through R11, with R12 10K pulling the base
 * to ground; emitter to ground, collector to the buzzer and then VCC3.3. That is
 * a classic low-side switch -- the base must be driven high for current to flow.
 *
 * The devicetree `pwms` entry carries no PWM_POLARITY_INVERTED and the pinctrl
 * group carries no nordic,invert, so:
 *   duty 0%   -> constant inactive level -> pin low  -> silent
 *   duty 100% -> constant active level   -> pin high -> sounding
 * which lines up one-to-one with buzzer_write(false) / (true) below.
 */

#include "buzzer.h"

#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/sys/util.h>

/* Take the pwms entry from zephyr,user in the devicetree
 * (boards/nrf52dk_nrf52832.overlay). zephyr,user exists to carry arbitrary
 * properties that have no binding of their own.
 *
 * The property name must not carry a bus prefix: unlike gpio, PWM does not
 * ignore a prefix during parsing. A name such as buzzer-pwms would be treated as
 * a different specifier space and dtc would demand a #buzzer-pwm-cells
 * declaration. See the overlay for the full explanation.
 */
static const struct pwm_dt_spec buzzer_pwm = PWM_DT_SPEC_GET(DT_PATH(zephyr_user));

/* Which rhythm a manual ring uses. FAST was chosen because locating a lost item
 * benefits from an immediately identifiable sound.
 */
#define BUZZER_RING_PATTERN BUZZER_FAST

/* Rhythm table: how long to sound (on_ms) and how long to stay quiet (off_ms).
 *
 * SILENT and CONTINUOUS do not use this table -- they have no "flip" concept and
 * are written directly as a level by buzzer_apply(). Their 0/0 rows exist only
 * to keep the array indexed by the enum; the timer callback still carries a
 * defensive check against them (see buzzer_timer_handler).
 *
 * Why the NORMAL grade is silent: the requirement names four grades
 * (near / normal / far / lost) but never says what "normal" should sound like.
 * Staying quiet is the more conservative default; this one line is all that
 * needs to change to get a very slow short beep instead.
 */
struct buzzer_step {
	uint16_t on_ms;
	uint16_t off_ms;
};

static const struct buzzer_step buzzer_steps[] = {
	[BUZZER_SILENT]     = { .on_ms = 0U,   .off_ms = 0U    },
	[BUZZER_SLOW]       = { .on_ms = 60U,  .off_ms = 3000U },
	[BUZZER_FAST]       = { .on_ms = 200U, .off_ms = 400U  },
	[BUZZER_CONTINUOUS] = { .on_ms = 0U,   .off_ms = 0U    },
};

/* ---- State ---- */
/* master switch: link authenticated and not muted by a Ring 0 write */
static bool buzzer_enabled;
static enum buzzer_pattern dist_pattern;	/* request from the distance side */
static bool ring_requested;			/* manual ring request */

/* The rhythm currently running. Kept separate from the "target rhythm" so the
 * code can tell whether anything needs changing: an unchanged rhythm must not
 * restart the timer, or a once-per-second call would keep cutting off a
 * long-interval rhythm.
 */
static enum buzzer_pattern running_pattern = BUZZER_SILENT;

/* Whether the current rhythm is in its "sound" or its "quiet" phase. */
static bool phase_on;

/* ---- Diagnostic mirrors (not static -- see the same note in rssi_filter.c) ----
 * printk is invisible on this board, so reading these out of RAM shows what the
 * buzzer module actually decided.
 */
int32_t g_buzzer_enabled;
int32_t g_buzzer_target;	/* rhythm chosen by buzzer_effective_pattern() */
int32_t g_buzzer_on;		/* whether the pin is currently "sounding" */
int32_t g_buzzer_pwm_ret;	/* return value of the last pwm_set_dt() */

static void buzzer_timer_handler(struct k_timer *timer);

K_TIMER_DEFINE(buzzer_timer, buzzer_timer_handler, NULL);

/* Drive the pin to "sound" or "silent".
 *
 * The period passed is buzzer_pwm.period (1 ms from the devicetree). The DC
 * branch does not validate the period, but passing the same value as period and
 * pulse lands exactly on the "constant active level" contract at pwm.h:598-599,
 * which makes the intent obvious at a glance.
 */
static int buzzer_write(bool on)
{
	int ret = pwm_set_dt(&buzzer_pwm, buzzer_pwm.period,
			     on ? buzzer_pwm.period : 0U);

	g_buzzer_on = on ? 1 : 0;
	g_buzzer_pwm_ret = ret;

	return ret;
}

/* Timer expiry -> flip the phase.
 *
 * This runs in *interrupt context* (k_timer expiry functions are called from the
 * system clock ISR), so only non-blocking work is allowed. The module header
 * above argues why pwm_set_dt() does not busy-wait in the DC case.
 *
 * Note that pm_device_runtime_get() must NOT be called here -- it takes a
 * semaphore and is not interrupt-safe. Waking the device happens once, in
 * buzzer_init().
 */
static void buzzer_timer_handler(struct k_timer *timer)
{
	ARG_UNUSED(timer);

	const struct buzzer_step *step = &buzzer_steps[running_pattern];

	/* Defensive: only SLOW / FAST ever start this timer. Should the state
	 * ever be corrupted into 0/0, K_MSEC(0) would make the timer expire
	 * again immediately inside the ISR, and the device would lock up.
	 * Silence is the safer failure mode. */
	if (step->on_ms == 0U || step->off_ms == 0U) {
		running_pattern = BUZZER_SILENT;
		(void)buzzer_write(false);
		return;
	}

	phase_on = !phase_on;

	if (phase_on) {
		(void)buzzer_write(true);
		k_timer_start(&buzzer_timer, K_MSEC(step->on_ms), K_NO_WAIT);
	} else {
		(void)buzzer_write(false);
		k_timer_start(&buzzer_timer, K_MSEC(step->off_ms), K_NO_WAIT);
	}
}

/* Merge the three inputs (master switch, distance rhythm, ring request) into one
 * rhythm by priority:
 *   1. master switch off -> silent (neither distance alarm nor ring sounds);
 *      that covers "not unlocked" and "muted by a Ring 0 write"
 *   2. distance FAR/LOST -> the distance alarm rhythm
 *   3. manual ring on   -> the ring rhythm
 *   4. otherwise        -> the distance-side table (NEAR slow beep / NORMAL
 *                          silent)
 *
 * FAR/LOST is tested explicitly rather than "distance rhythm is not silent",
 * because NEAR is audible too: if NEAR outranked the ring, the ring button would
 * stop working whenever the user stood next to the device, which makes no sense.
 */
static enum buzzer_pattern buzzer_effective_pattern(void)
{
	if (!buzzer_enabled) {
		return BUZZER_SILENT;
	}

	if (dist_pattern == BUZZER_FAST || dist_pattern == BUZZER_CONTINUOUS) {
		return dist_pattern;
	}

	if (ring_requested) {
		return BUZZER_RING_PATTERN;
	}

	return dist_pattern;
}

/* Commit the target rhythm to the hardware. Touches the timer only when the
 * rhythm actually changed.
 */
static void buzzer_apply(void)
{
	enum buzzer_pattern target = buzzer_effective_pattern();

	g_buzzer_target = (int32_t)target;

	if (target == running_pattern) {
		/* Unchanged -- let the current cycle finish rather than
		 * interrupting it. This is what makes "beep 60 ms, wait 3 s"
		 * actually sound like once every 3 s, given that the RSSI
		 * thread calls buzzer_set_pattern() every second. */
		return;
	}

	running_pattern = target;
	phase_on = false;

	switch (target) {
	case BUZZER_CONTINUOUS:
		k_timer_stop(&buzzer_timer);
		(void)buzzer_write(true);
		break;

	case BUZZER_SLOW:
	case BUZZER_FAST:
		/* Start with the "sound" phase: sounding first and waiting after
		 * reads as a more immediate alarm than the reverse. */
		k_timer_stop(&buzzer_timer);
		phase_on = true;
		(void)buzzer_write(true);
		k_timer_start(&buzzer_timer, K_MSEC(buzzer_steps[target].on_ms), K_NO_WAIT);
		break;

	case BUZZER_SILENT:
	default:
		k_timer_stop(&buzzer_timer);
		(void)buzzer_write(false);
		break;
	}
}


int buzzer_init(void)
{
	if (!pwm_is_ready_dt(&buzzer_pwm)) {
		return -ENODEV;
	}

	/* The PWM device MUST be resumed, otherwise the buzzer never sounds.
	 *
	 * Why: this project enables CONFIG_PM_DEVICE_RUNTIME=y (prj.conf), and
	 *   zephyr/soc/nordic/common/Kconfig.defconfig:10-11
	 *       config PM_DEVICE_RUNTIME_DEFAULT_ENABLE
	 *           default y if PM_DEVICE_RUNTIME
	 * makes runtime PM the default for *every* device. Consequently
	 *   zephyr/subsys/pm/device.c:415-422
	 * leaves the device in PM_DEVICE_STATE_SUSPENDED inside
	 * pm_device_driver_init() and returns without calling RESUME, while
	 * pwm_nrfx.c:390-392 already applied the pinctrl sleep state during init
	 * -- P0.12 is configured as an input and disconnected
	 * (pinctrl_nrf.c:580-583).
	 *
	 * Neither the PWM driver nor pwm.h ever gets this device on its own, so
	 * without the wake-up here pwm_set_dt() reports success while the pin is
	 * not driven at all.
	 *
	 * The device is deliberately never put back: the buzzer has to be ready
	 * on demand, so it stays ACTIVE at the cost of one channel held open (the
	 * nRF52832 PWM is a separate peripheral and does not consume MPSL's
	 * RTC/TIMER). The call returns 0 harmlessly when pm == NULL or when
	 * runtime PM is not enabled for the device.
	 */
	(void)pm_device_runtime_get(buzzer_pwm.dev);

	buzzer_enabled = false;
	dist_pattern = BUZZER_SILENT;
	ring_requested = false;
	running_pattern = BUZZER_SILENT;
	phase_on = false;

	g_buzzer_enabled = 0;
	g_buzzer_target = BUZZER_SILENT;

	/* Write the silent level explicitly rather than relying on the static
	 * idle level from pinctrl, so the power-on state is guaranteed by code
	 * and not by the devicetree. */
	return buzzer_write(false);
}


void buzzer_set_enabled(bool enabled)
{
	if (enabled == buzzer_enabled) {
		return;
	}

	buzzer_enabled = enabled;
	g_buzzer_enabled = enabled ? 1 : 0;

	buzzer_apply();
}


void buzzer_set_pattern(enum buzzer_pattern pattern)
{
	if (pattern == dist_pattern) {
		return;
	}

	dist_pattern = pattern;

	buzzer_apply();
}


void buzzer_set_ring(bool on)
{
	if (on == ring_requested) {
		return;
	}

	ring_requested = on;

	buzzer_apply();
}
