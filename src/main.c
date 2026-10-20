/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>  /* bt_hci_get_conn_handle / bt_hci_cmd_send_sync / BT_HCI_OP_READ_RSSI */
#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/sys/byteorder.h>  /* sys_cpu_to_le16 */
#include <dk_buttons_and_leds.h>
#include <nrfx_saadc.h>

#include "Beacon.h"
/* The three modules that make up the lost-item alarm.
 * Data flow: RSSI sampling -> rssi_filter (moving average) ->
 * distance_manager (grading) -> buzzer (rhythm). The buzzer's master switch is
 * driven by the unlock state tracked in this file. None of the three modules
 * knows about the others; main.c is the only place that wires them together.
 */
#include "buzzer.h"
#include "distance_manager.h"
#include "rssi_filter.h"

/* ==================== Crash forensics (diagnostic) ====================
 * With CONFIG_LOG=n the fault reporting inside z_arm_fault()/z_fatal_error() is
 * compiled out entirely, so a crash leaves no trace anywhere -- not on the UART
 * and not in any log buffer. Overriding k_sys_fatal_error_handler() (__weak in
 * kernel/fatal.c, called by z_fatal_error()) is therefore the only way to
 * capture anything: it copies the exception stack frame and the SCB fault
 * registers into a RAM structure that can be read out of band with a debugger.
 * The faulting PC can then be resolved to a source line with objdump/addr2line.
 */
#include <cmsis_core.h>
#include <zephyr/fatal.h>

struct fault_record {
	uint32_t magic;		/* 0xFA017ACE means the record is valid */
	uint32_t reason;	/* k_fatal_error_reason */
	uint32_t boot_step_when_faulted;
	uint32_t pc, lr, xpsr;
	uint32_t r0, r1, r2, r3, r12;
	uint32_t exc_return, fault_sp;
	uint32_t hfsr, cfsr, dfsr, mmfar, bfar, shcsr, icsr;
	uint32_t thread;
};

struct fault_record g_fault_rec;

static uint32_t boot_step;	/* number of boot milestones reached so far */

void k_sys_fatal_error_handler(unsigned int reason, const struct arch_esf *esf)
{
	struct fault_record *r = &g_fault_rec;

	r->magic = 0xFA017ACEu;
	r->reason = reason;
	r->boot_step_when_faulted = boot_step;

	/* Note: by the time this runs, CFSR has already been cleared by
	 * z_arm_fault() writing 1 to each sub-field (SCB->CFSR |= <subfield mask>
	 * at fault.c:319/477/541), so the CFSR read here is always 0 and tells
	 * us nothing. HFSR is not cleared and remains useful. */
	r->hfsr = SCB->HFSR;
	r->cfsr = SCB->CFSR;
	r->dfsr = SCB->DFSR;
	r->mmfar = SCB->MMFAR;
	r->bfar = SCB->BFAR;
	r->shcsr = SCB->SHCSR;
	r->icsr = SCB->ICSR;
	r->thread = (uint32_t)(uintptr_t)k_current_get();

	if (esf != NULL) {
		r->pc = esf->basic.pc;
		r->lr = esf->basic.lr;
		r->xpsr = esf->basic.xpsr;
		r->r0 = esf->basic.r0;
		r->r1 = esf->basic.r1;
		r->r2 = esf->basic.r2;
		r->r3 = esf->basic.r3;
		r->r12 = esf->basic.r12;
#if defined(CONFIG_EXTRA_EXCEPTION_INFO)
		r->exc_return = esf->extra_info.exc_return;
		r->fault_sp = esf->extra_info.msp;
#endif
	}

	/* Never return: returning would let z_fatal_error() proceed to
	 * k_thread_abort() and destroy the evidence. Lock interrupts and halt in
	 * place so the debugger can read the record. */
	(void)irq_lock();
	for (;;) {
		__asm__ volatile("nop");
	}
}


/* ==================== MPSL assertion forensics (diagnostic) ====================
 * m_assert_handler() in nrf/subsys/mpsl/init/mpsl_init.c has two forms:
 *   - CONFIG_MPSL_ASSERT_HANDLER=y -> calls the application's
 *     mpsl_assert_handle(file, line)
 *   - =n -> printk("MPSL ASSERT: %s, %d") with the file name truncated to 10
 *     characters, then k_oops()
 * The default path is useless on this board: printk goes to UART0, which is not
 * wired to the J-Link VCOM, so nobody ever sees the message.
 *
 * Enabling CONFIG_MPSL_ASSERT_HANDLER (see prj.conf) yields the full file path
 * and line number instead. MPSL is a prebuilt library
 * (CONFIG_MPSL_BUILD_TYPE_LIB=y), so file/line refer to Nordic's internal
 * sources which are not present locally -- but the name and line are enough to
 * pin down the assertion site.
 */
#include <mpsl/mpsl_assert.h>

char *g_assert_file;
uint32_t g_assert_line;
uint32_t g_assert_ipsr;
uint32_t g_assert_lr;
uint32_t g_assert_sp;
uint32_t g_assert_ret[8];
uint32_t g_assert_ret_cnt;

void mpsl_assert_handle(char *file, uint32_t line)
{
	uint32_t here = 0;
	uint32_t sp;

	g_assert_file = file;
	g_assert_line = line;
	g_assert_ipsr = __get_IPSR();
	g_assert_lr = (uint32_t)(uintptr_t)__builtin_return_address(0);

	/* Walk up the stack picking out words that look like a return address
	 * into flash (odd = Thumb), in order to locate the call site inside the
	 * prebuilt MPSL library that raised the assertion. */
	sp = (uint32_t)(uintptr_t)&here;
	g_assert_sp = sp;
	for (uint32_t i = 0; i < 64u; i++) {
		uint32_t a = sp + (i * 4u);
		uint32_t v;

		if (a + 4u > 0x20010000u) {
			break;
		}
		v = *(uint32_t *)(uintptr_t)a;
		if ((v & 1u) != 0u && v >= 0x100u && v < 0x80000u) {
			if (g_assert_ret_cnt < ARRAY_SIZE(g_assert_ret)) {
				g_assert_ret[g_assert_ret_cnt] = v;
				g_assert_ret_cnt++;
			}
		}
	}

	(void)irq_lock();
	for (;;) {
		__asm__ volatile("nop");
	}
}


/* BT_LE_ADV_PARAM() expands to a const struct array literal, so it must be
 * received through a const pointer; otherwise the const qualifier is dropped and
 * the compiler warns. */
static const struct bt_le_adv_param *adv_param = BT_LE_ADV_PARAM(
	(BT_LE_ADV_OPT_CONN |
	 BT_LE_ADV_OPT_USE_IDENTITY), 	/* connectable advertising, use the identity address */
	BT_GAP_ADV_FAST_INT_MIN_1,		/* fast advertising, minimum 30 ms */
	BT_GAP_ADV_FAST_INT_MAX_1,		/* fast advertising, maximum 60 ms */
	BT_ID_DEFAULT				/* advertise with the default identity */
	);

static struct bt_conn *my_conn = NULL;

static struct bt_gatt_exchange_params exchange_params;

static void exchange_func(struct bt_conn *conn, uint8_t att_err,
			  struct bt_gatt_exchange_params *params);

#define DEVICE_NAME "Tracker Beacon"
#define DEVICE_NAME_LEN (sizeof("Tracker Beacon") - 1)

#define RUN_STATUS_LED DK_LED1
#define CON_STATUS_LED DK_LED2
/* The buzzer must not be addressed through a DK_LEDn macro. DK_LED3 is merely
 * array index 2 of the dk_buttons_and_leds leds[] array, which indexes the
 * /leds child nodes in declaration order -- that is /leds/led_2, physically
 * P0.19 -- whereas the buzzer on this board is wired to P0.12. The two do not
 * line up, so the buzzer is driven by the buzzer module over PWM1 channel 0
 * rather than through a gpio_dt_spec declared here.
 */
/* Button 1 of the DK: an indication carrying the current button state is sent on
 * every change. */
#define FIND_BUTTON DK_BTN1_MSK

#define STACKSIZE 1024
#define PRIORITY 7

#define RUN_LED_BLINK_INTERVAL 1000
#define RUN_RING_BLINK_INTERVAL 100
#define NOTIFY_INTERVAL 60000
/* RSSI poll period, in milliseconds. */
#define RSSI_POLL_INTERVAL_MS 100

/* LED1 heartbeat: ring_thread ticks at a fixed RUN_RING_BLINK_INTERVAL (100 ms)
 * and toggles the LED every RUN_LED_HEARTBEAT_TICKS ticks -> 500 ms on,
 * 500 ms off, roughly 1 Hz. */
#define RUN_LED_HEARTBEAT_TICKS 5U

/* Fixed pairing passkey, returned by auth_app_passkey(). Must be in 0..999999.
 *
 * SECURITY: this is a public constant. It keeps an accidental third party from
 * connecting, but it does not stop anyone who has read this source; a shipped
 * product must derive a per-device passkey instead. See the Security section of
 * README.md.
 */
#define PAIRING_PASSKEY 123456U

static bool app_button_state;

/* Manual ring request, set from the ring characteristic's write callback and
 * polled by ring_thread. */
static bool ring_state;

/* Ring mute latch: writing 0 to characteristic ...1528 sets it, writing 1 clears
 * it. While set, ring_thread keeps the buzzer's master switch off, which
 * silences the automatic distance alarm as well as the manual ring. Default
 * false preserves the existing behaviour: pairing alone still arms the alarm.
 * RAM-only and cleared in on_disconnected(). */
static bool alarm_muted;

/* Most recent battery voltage in mV. Reserved for a future real battery circuit;
 * nothing reads it yet. */
static uint32_t battery_voltage;
static nrfx_saadc_channel_t channel = NRFX_SAADC_DEFAULT_CHANNEL_SE(NRF_SAADC_INPUT_AIN0, 0);

static bool bluetooth_connected = false;

/* "The passkey has been accepted" -- the unlock criterion, alongside the Ring 0
 * mute latch above.
 *
 * Why it is equivalent to a password lock: the passkey comes from
 * auth_app_passkey() (PAIRING_PASSKEY), which is the 6-digit code the phone's
 * pairing dialog asks for. The Ring characteristic carries
 * BT_GATT_PERM_WRITE_AUTHEN (the LED one deliberately does not -- see
 * app_led_cb()), and the stack guarantees such a write only reaches its callback
 * while the link is authenticated (>= BT_SECURITY_L3) -- gatt.c:3105-3109.
 * Therefore:
 *
 *     start/stop the ring -> the write must succeed -> the callback must run ->
 *     the link must be authenticated -> the passkey must have been entered
 *
 * Break any link in that chain and the buzzer cannot be touched, so this flag
 * serves directly as the lock; the application does not need a password
 * characteristic of its own.
 *
 * The buzzer's master switch is `link_authenticated && !alarm_muted`. The mute
 * latch can only be set or cleared through the same authenticated Ring write,
 * so adding it does not weaken the lock.
 *
 * It is set in on_security_changed() and cleared in on_disconnected(), and
 * nowhere else.
 */
static bool link_authenticated = false;

/* Normal-operation flag: set only once advertising has started successfully.
 * ring_thread uses it to decide whether to emit the LED1 heartbeat, which makes
 * "LED1 is blinking" equivalent to "advertising is up" and usable as a direct
 * diagnostic conclusion. When advertising fails to start, main() leaves LED1
 * solid and this flag false, so the heartbeat never overwrites that failure
 * indication.
 */
static bool app_running = false;

/* SAADC DMA target buffer. The conversion is triggered in blocking mode
 * (nrfx_saadc_mode_trigger() returns only once the sample is ready), so there is
 * no race and the buffer does not need to be volatile. */
static int16_t sample;

#define BATTERY_SAMPLE_INTERVAL_MS 2000

/* SAADC conversion constants (single-ended / internal reference / GAIN1_6;
 * see configure_saadc()):
 *   VREF       = 600 mV  (NRF_SAADC_REFERENCE_INTERNAL)
 *   gain       = 1/6     (NRF_SAADC_GAIN1_6: the input is attenuated by 1/6
 *                         before being compared against VREF)
 *   full scale = VREF / gain = 600 * 6 = 3600 mV
 *   per LSB    = 3600 / 4096 = 0.879 mV
 * Inverse: V_in[mV] = code * 3600 / 4096
 */
#define SAADC_VREF_MV         600U
#define SAADC_GAIN_FACTOR     6U                                    /* = 1 / (1/6) */
#define SAADC_RESOLUTION_BITS 12U
#define SAADC_FULL_SCALE_MV   (SAADC_VREF_MV * SAADC_GAIN_FACTOR)   /* 3600 mV */
/* Largest sample code at 12-bit resolution (2^12 - 1). Used to compress the raw
 * code linearly onto 0..100; see adc_sample_to_bas_level(). */
#define SAADC_FULL_SCALE_CODE 4095U

/* Convert a raw SAADC sample code into the battery percentage the standard
 * Battery Service expects.
 *
 * Why compress instead of forwarding the raw value: the standard Battery Level
 * characteristic (0x2A19) is pinned by the Bluetooth specification to a uint8
 * percentage in 0..100, and Zephyr's implementation rejects anything outside
 * that range outright --
 *     zephyr/subsys/bluetooth/services/bas/bas.c:117-119
 *         if (level > 100U) { return -EINVAL; }
 * so a 12-bit code (0..4095) cannot be passed through; it has to be compressed
 * linearly onto 0..100.
 *
 * This is a placeholder conversion. The board has no real battery or divider
 * circuit, so P0.02 only ever reads the SAADC's own input, and deliberately no
 * physical voltage-to-charge mapping is attempted here. Once a real cell and
 * divider are fitted, this should compute the level from battery_voltage (mV)
 * against a discharge curve instead.
 */
static uint8_t adc_sample_to_bas_level(int16_t raw)
{
	/* Sampling noise can push raw slightly negative, and raw is an int16_t
	 * while the largest valid code is 4095, so both ends are clamped. */
	if (raw <= 0) {
		return 0U;
	}
	if (raw >= (int16_t)SAADC_FULL_SCALE_CODE) {
		return 100U;
	}

	return (uint8_t)(((int32_t)raw * 100) / (int32_t)SAADC_FULL_SCALE_CODE);
}

static void battery_sample_timer_handler(struct k_timer *timer);

K_TIMER_DEFINE(battery_sample_timer, battery_sample_timer_handler, NULL);

static void battery_sample_timer_handler(struct k_timer *timer)
{
	ARG_UNUSED(timer);

	/* In simple mode with event_handler == NULL, nrfx takes the blocking
	 * path and returns only once the conversion is done, which makes it safe
	 * to call from this timer (ISR) context. nrfx v4 returns an int: 0 on
	 * success, a negative errno otherwise (no longer nrfx_err_t).
	 */
	int ret = nrfx_saadc_mode_trigger();
	if (ret != 0) {
		return;
	}

	/* V_in[mV] = code * 3600 / 4096. Multiply before dividing: the worst
	 * case is 4095 * 3600 = 14,742,000, which fits in an int32_t. */
	int32_t mv = ((int32_t)sample * (int32_t)SAADC_FULL_SCALE_MV)
		     >> SAADC_RESOLUTION_BITS;

	/* Sampling noise can push sample slightly negative; clamp to 0, or the
	 * assignment to a uint32_t would wrap around. */
	battery_voltage = (mv > 0) ? (uint32_t)mv : 0U;
}

/* Returns 0 on success, or a negative errno from the nrfx layer. Every failure
 * is reported to the caller so that a broken SAADC can be surfaced at boot
 * instead of silently leaving the battery level frozen.
 */
static int configure_saadc(void)
{
	/* Connect the ADC interrupt to the nrfx interrupt handler. */
	IRQ_CONNECT(DT_IRQN(DT_NODELABEL(adc)),
		    DT_IRQ(DT_NODELABEL(adc), priority),
		    nrfx_isr, nrfx_saadc_irq_handler, 0);

	/* nrfx v4 returns an int: 0 on success, a negative errno otherwise. */
	int err = nrfx_saadc_init(DT_IRQ(DT_NODELABEL(adc), priority));

	if (err != 0) {
		return err;
	}

	/* Configure the SAADC channel. */
	channel.channel_config.gain = NRF_SAADC_GAIN1_6;
	err = nrfx_saadc_channels_config(&channel, 1);
	if (err != 0) {
		return err;
	}

	/* Configure the nrfx SAADC driver in simple, blocking mode. */
	err = nrfx_saadc_simple_mode_set(BIT(0),
					 NRF_SAADC_RESOLUTION_12BIT,
					 NRF_SAADC_OVERSAMPLE_DISABLED,
					 NULL);
	if (err != 0) {
		return err;
	}

	/* Set the buffer the sample is stored in. */
	err = nrfx_saadc_buffer_set(&sample, 1);
	if (err != 0) {
		return err;
	}

	/* Start the periodic battery sampling timer. */
	k_timer_start(&battery_sample_timer, K_NO_WAIT, K_MSEC(BATTERY_SAMPLE_INTERVAL_MS));

	return 0;
}

static void on_le_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency,
				uint16_t timeout)
{
	/* None of these is used: they existed only to feed a debug print. */
	ARG_UNUSED(conn);
	ARG_UNUSED(interval);
	ARG_UNUSED(latency);
	ARG_UNUSED(timeout);
}

/* The nRF52832 supports the 1M PHY only -- no 2M and no Coded PHY -- so asking
 * for 1M/1M amounts to keeping the default. The Coded PHY is the long-range
 * option and would need BT_GAP_LE_PHY_CODED, but that PHY does not exist on this
 * part. */
static void update_phy(struct bt_conn *conn)
{
	const struct bt_conn_le_phy_param preferred_phy = {
		.options = BT_CONN_LE_PHY_OPT_NONE,
		.pref_rx_phy = BT_GAP_LE_PHY_1M,
		.pref_tx_phy = BT_GAP_LE_PHY_1M,
	};

	/* There is no other PHY to fall back to, so a failure here leaves
	 * nothing to recover; the result is deliberately ignored. */
	(void)bt_conn_le_phy_update(conn, &preferred_phy);
}

static void on_le_phy_updated(struct bt_conn *conn, struct bt_conn_le_phy_info *param)
{
	/* The result is of no interest on this part, since 1M is the only PHY it
	 * supports and the callback can therefore only confirm the default. */
	ARG_UNUSED(conn);
	ARG_UNUSED(param);
}
static void on_security_changed(struct bt_conn *conn, bt_security_t level, enum bt_security_err err)
{
	ARG_UNUSED(conn);

	/* The single unlock criterion for the whole project.
	 *
	 * The threshold must be BT_SECURITY_L3, exactly matching the level that
	 * bt_gatt_check_perm() tests BT_GATT_PERM_WRITE_AUTHEN against --
	 * gatt.c:3105-3109 in NCS v3.4.0:
	 *
	 *     if (mask & BT_GATT_PERM_AUTHEN_MASK) {
	 *             if (bt_conn_get_security(conn) < BT_SECURITY_L3) {
	 *                     return BT_ATT_ERR_AUTHENTICATION;
	 *             }
	 *     }
	 *
	 * L3 on both sides is what keeps "the characteristic accepts the write"
	 * and "the buzzer is armed" in step. With L2 here, a write would be
	 * accepted while the buzzer stayed mute; with L4 it would be the reverse.
	 *
	 * err == BT_SECURITY_ERR_SUCCESS is required as well: the stack calls
	 * this callback when pairing fails or is cancelled too, and those cases
	 * must not be mistaken for an unlock.
	 */
	link_authenticated = (err == BT_SECURITY_ERR_SUCCESS) && (level >= BT_SECURITY_L3);
}

static void update_data_length(struct bt_conn *conn)
{
	struct bt_conn_le_data_len_param my_data_len = {
		.tx_max_len = BT_GAP_DATA_LEN_MAX,
		.tx_max_time = BT_GAP_DATA_TIME_MAX,
	};

	/* Best effort: if this fails the link simply keeps whatever MTU was
	 * negotiated, so there is nothing to handle. */
	(void)bt_conn_le_data_len_update(conn, &my_data_len);
}

static void update_mtu(struct bt_conn *conn)
{
	exchange_params.func = exchange_func;

	/* Best effort, for the same reason as update_data_length(). */
	(void)bt_gatt_exchange_mtu(conn, &exchange_params);
}

static void on_le_data_len_updated(struct bt_conn *conn, struct bt_conn_le_data_len_info *info)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(info);
}

static void exchange_func(struct bt_conn *conn, uint8_t att_err,
			  struct bt_gatt_exchange_params *params)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(params);
	ARG_UNUSED(att_err);
}
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),

};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_LBS_VAL),
};

/* LED characteristic (...1525): drives DK_LED4 (P0.20) and nothing else.
 *
 * Writing 01/00 turns that LED on or off, and it stays that way until the next
 * write. It does not touch the buzzer: ring control is characteristic ...1528's
 * job alone, so switching a LED can never produce a sound, and asking for a ring
 * means writing ...1528 -- which is why that requires pairing first.
 *
 * This characteristic is deliberately unauthenticated (BT_GATT_PERM_WRITE), so
 * writing it does not trigger pairing either; only the Button read and the Ring
 * write do. Pairing is what arms the buzzer, so keeping it off this path is what
 * makes "switch the LED" independent of "enable the lost-item alarm".
 *
 * DK_LED4 is a plain on/off output and does not blink; the only LED that blinks
 * is LED1 (P0.17), the 1 Hz heartbeat emitted by ring_thread. P0.20 is otherwise
 * unused: the board's /pwmleds node is on pwm0, which is P0.17.
 */
static void app_led_cb(bool led_state)
{
	dk_set_led(DK_LED4, led_state ? 1U : 0U);
}

/* The only ring switch: writing 01/00 to characteristic ...1528.
 *
 * This callback runs only while the link is authenticated (>= L3). The
 * characteristic's permission is BT_GATT_PERM_WRITE_AUTHEN, so an
 * unauthenticated write is rejected by bt_gatt_check_perm() (gatt.c:3105-3109)
 * and never reaches here -- which is exactly how "enter the passkey before the
 * lost-item ring can be started or muted" holds, with no lock check needed in
 * this function.
 *
 * 1 = request the ring and clear the mute latch; 0 = stop the ring and set the
 * mute latch, which silences the automatic distance alarm too (see
 * ring_thread). Two flags are needed because before the first write to ...1528
 * neither flag is set: the distance alarm stays armed as soon as pairing
 * completes, exactly as it did before this feature existed.
 *
 * It only records the request. What actually makes a sound is the 100 ms poll of
 * buzzer_set_ring() / buzzer_set_enabled() in ring_thread.
 */
static void app_ring_cb(bool on)
{
	if (on) {
		ring_state = true;	/* request first ... */
		alarm_muted = false;	/* ... then unmute */
	} else {
		alarm_muted = true;	/* mute first ... */
		ring_state = false;	/* ... then drop the request */
	}
}

static bool app_button_cb(void)
{
	return app_button_state;
}
static struct my_lbs_cb app_callbacks = {
	.led_cb = app_led_cb,
	.ring_cb = app_ring_cb,
	.button_cb = app_button_cb,
};

/* Define the thread functions  */
static void send_data_thread(void)
{
	while (1) {
		/* Refresh the standard Battery Service level.
		 *
		 * bt_bas_set_battery_level() calls bt_gatt_notify() internally
		 * and normalises -ENOTCONN to 0 (bas.c:129), so calling it while
		 * disconnected is safe and needs no guard. More to the point, the
		 * standard Battery Level characteristic is readable, so a phone
		 * can read the current value without subscribing -- which means
		 * the value has to stay fresh at all times, not only while
		 * connected. */
		bt_bas_set_battery_level(adc_sample_to_bas_level(sample));

		k_sleep(K_MSEC(NOTIFY_INTERVAL));
	}
}

/* Ticks at a fixed RUN_RING_BLINK_INTERVAL (100 ms). One counter drives both:
 *   - LED1: toggled every RUN_LED_HEARTBEAT_TICKS ticks -> permanent heartbeat
 *   - the buzzer: hands "link authenticated and not muted" (the master switch)
 *     and "is a ring requested" over to the buzzer module
 * The fixed tick is what makes a periodic heartbeat possible at all.
 */
static void ring_thread(void)
{
	uint32_t tick = 0;
	bool run_led_on = false;

	while (1) {
		/* ---- LED1: running heartbeat ---- */
		if (app_running && ((tick % RUN_LED_HEARTBEAT_TICKS) == 0U)) {
			run_led_on = !run_led_on;
			dk_set_led(RUN_STATUS_LED, run_led_on ? 1U : 0U);
		}

		/* ---- buzzer ----
		 * The unlock state and the Ring 0 mute latch together feed the
		 * buzzer's master switch. Polling is used rather than a callback
		 * from Beacon.c so that Beacon.c never needs to know the buzzer
		 * exists and the two modules stay independent. With a 100 ms poll
		 * the buzzer is armed at most 100 ms after pairing completes (and
		 * muted at most 100 ms after a Ring 0 write). Both setters ignore
		 * an unchanged value, so calling them every tick costs nothing.
		 *
		 * The ring request is handed over first: on a Ring 1 write this
		 * makes the first tick after the write see both "ring requested"
		 * and "unmuted", so the manual beeping starts immediately instead
		 * of producing one tick of the distance rhythm. */
		buzzer_set_ring(ring_state);
		buzzer_set_enabled(link_authenticated && !alarm_muted);

		tick++;
		k_sleep(K_MSEC(RUN_RING_BLINK_INTERVAL));
	}
}

/* ==================== Reading RSSI and notifying it ====================
 * NCS v3.4.0 has no high-level RSSI API: neither bt_conn_get_info() nor
 * struct bt_conn_info / struct bt_conn_le_info carries an rssi field
 * (conn.h:1072-1116, :1202-1223), and there is no bt_conn_get_rssi(). The only
 * route is the HCI Read RSSI command (BT_HCI_OP_READ_RSSI, opcode 0x1405,
 * hci_types.h:1136-1144), which is also what Nordic's own
 * nrf/samples/bluetooth/rssi_power_control/central/src/main.c:133-187 does.
 *
 * On the controller side this opcode is only dispatched when
 * CONFIG_BT_CTLR_CONN_RSSI=y (see prj.conf).
 *
 * Why a dedicated thread: bt_hci_cmd_send_sync() blocks on a semaphore waiting
 * for Command Complete, so it may only be called from a sleepable thread. The
 * SDK reinforces this with a TODO at hci_core.c:450-456 warning against sending
 * synchronous commands from the system work queue.
 *
 * Line numbers are quoted against nRF Connect SDK v3.4.0 and drift between
 * releases.
 */

/* Read the current connection's RSSI once. On a 0 return *rssi is valid (dBm,
 * normally negative); otherwise the return value is a negative error code. */
static int read_conn_rssi(struct bt_conn *conn, int8_t *rssi)
{
	struct net_buf *buf;
	struct net_buf *rsp = NULL;
	struct bt_hci_cp_read_rssi *cp;
	struct bt_hci_rp_read_rssi *rp;
	uint16_t handle;
	int err;

	/* bt_hci_get_conn_handle() dereferences conn with no NULL check of its
	 * own, so passing NULL here would HardFault; the caller has to make sure
	 * conn is non-NULL. */
	err = bt_hci_get_conn_handle(conn, &handle);
	if (err) {
		return err;
	}

	buf = bt_hci_cmd_alloc(K_FOREVER);
	if (!buf) {
		return -ENOMEM;
	}

	cp = net_buf_add(buf, sizeof(*cp));
	cp->handle = sys_cpu_to_le16(handle);

	err = bt_hci_cmd_send_sync(BT_HCI_OP_READ_RSSI, buf, &rsp);
	if (err) {
		/* Failure path: buf has already been freed by the send layer
		 * (hci_core.c:446 or :490) and rsp is never filled in, so the
		 * caller has nothing to clean up either way. */
		return err;
	}

	/* On success rsp is always non-NULL (hci_core.c:508-509 hands ownership
	 * to the caller) and must be net_buf_unref()'d, or HCI event buffers
	 * leak continuously. */
	rp = (void *)rsp->data;
	if (rp->status != 0) {
		err = -EIO;
	} else if (rp->rssi == BT_HCI_LE_RSSI_NOT_AVAILABLE) {
		/* 0x7F: the controller cannot produce a valid RSSI just yet
		 * (common right after connecting). Skip this round rather than
		 * report it -- sent to a phone as-is it would read 127 dBm. */
		err = -ENODATA;
	} else {
		*rssi = rp->rssi;
	}

	net_buf_unref(rsp);
	return err;
}

/* The one place where "distance" and "the buzzer" meet, and they meet only here:
 * distance_manager does not know a buzzer exists and buzzer does not know
 * distance exists, so main.c does the translation. The cost is that changing the
 * alarm policy means editing main.c; the benefit is that the two modules compile,
 * test and get reused independently.
 *
 * Mapping NORMAL to silence is a choice, not a requirement: the specification
 * names four grades but never says what "normal" should sound like. Changing this
 * one line is all it takes to give it a very slow short beep instead. */
static enum buzzer_pattern pattern_for_level(enum distance_level lv)
{
	switch (lv) {
	case DIST_NEAR:
		return BUZZER_SLOW;
	case DIST_NORMAL:
		return BUZZER_SILENT;
	case DIST_FAR:
		return BUZZER_FAST;
	case DIST_LOST:
		return BUZZER_CONTINUOUS;
	case DIST_UNKNOWN:
	default:
		return BUZZER_SILENT;
	}
}

static void rssi_thread(void)
{
	int8_t raw = 0;

	while (1) {
		/* bluetooth_connected is tested before my_conn because
		 * on_disconnected() clears the flag first and only then unrefs and
		 * NULLs the pointer; testing in the same order keeps the race
		 * window as small as possible. Even on a real race it is safe: the
		 * callback reaches sys_reboot() while the host still holds a
		 * bt_conn reference, so the object cannot be reclaimed before the
		 * reset and bt_hci_get_conn_handle() merely returns -ENOTCONN
		 * (conn->state != CONNECTED), which is harmless here. */
		if (bluetooth_connected && my_conn != NULL) {
			if (read_conn_rssi(my_conn, &raw) == 0) {
				/* Data flow: raw RSSI -> moving average -> distance
				 * grade -> alarm rhythm. The order must not be
				 * swapped: filter first, grade second, or the jitter
				 * in the raw value feeds straight into the grading and
				 * the buzzer flips back and forth on the thresholds. */
				int8_t filtered = rssi_filter_update(raw);
				enum distance_level lv = distance_manager_update(filtered);

				/* Runtime trace; visible on the UART console. */
				printk("RSSI raw:%d avg:%d dist:%s\n", raw, filtered,
				       distance_level_str(lv));

				/* The phone always gets the *raw* value, so that the
				 * jitter it sees stays available as information;
				 * filtering serves this device's alarm decision only. */
				(void)my_lbs_send_rssi_notify(raw);

				buzzer_set_pattern(pattern_for_level(lv));
			}
		}

		k_sleep(K_MSEC(RSSI_POLL_INTERVAL_MS));
	}
}

/* Send an indication on a button press. */
static void button_changed(uint32_t button_state, uint32_t has_changed)
{
	if (has_changed & FIND_BUTTON) {
		uint32_t user_button_state = button_state & FIND_BUTTON;
		my_lbs_send_button_state_indicate(user_button_state);
		app_button_state = user_button_state ? true : false;
	}
}
static void on_connected(struct bt_conn *conn, uint8_t err)
{
	struct bt_conn_info info;
	int ret;

	if (err) {
		printk("Connection failed (err %u)\n", err);
		return;
	}

	printk("Connected\n");
	bluetooth_connected = true;
	my_conn = bt_conn_ref(conn);

	/* The connection parameters are only of interest when debugging; the
	 * call is still made so that a failure is reported rather than silently
	 * swallowed. */
	ret = bt_conn_get_info(conn, &info);
	if (ret) {
		/* Not fatal: every call below works from my_conn, not from info,
		 * so returning here would only leave the connection LED dark and
		 * skip the PHY / data-length / MTU updates for no reason. */
		printk("bt_conn_get_info() returned %d\n", ret);
	}

	/* Update the PHY. */
	update_phy(my_conn);
	/* Update the data length and MTU. */
	update_data_length(my_conn);
	update_mtu(my_conn);

	/* Connection LED: lit for 2 s, then dark again. */
	dk_set_led_on(CON_STATUS_LED);
	k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL * 2));
	dk_set_led_off(CON_STATUS_LED);
}

static void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
	ARG_UNUSED(conn);

	printk("Disconnected (reason %u)\n", reason);
	bluetooth_connected = false;

	/* Lock on disconnect. Clearing link_authenticated and the Ring mute latch
	 * is the whole of it: the next connection has to pair and authenticate
	 * again, at which point on_security_changed() sets the flag once more, and
	 * the distance alarm starts from its default (enabled) state.
	 *
	 * The filter and the distance grade are reset for the same reason -- the
	 * next connection starts from scratch. Keeping the previous round's
	 * average would grade the fresh connection from stale data. */
	link_authenticated = false;
	alarm_muted = false;
	ring_state = false;
	buzzer_set_enabled(false);
	rssi_filter_init();
	distance_manager_reset();
	buzzer_set_pattern(BUZZER_SILENT);

	/* Release the reference taken in on_connected(). The device reboots a few
	 * lines further down, so the leak would not accumulate, but unref'ing
	 * here keeps the refcounting symmetric. */
	if (my_conn) {
		bt_conn_unref(my_conn);
		my_conn = NULL;
	}

	/* Disconnection LED: lit for 2 s, then reboot. */
	dk_set_led_on(RUN_STATUS_LED);
	k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL * 2));
	dk_set_led_off(RUN_STATUS_LED);

	/* Disconnecting reboots the whole device. sys_reboot() is the portable
	 * form of a raw NVIC_SystemReset(); Nordic's sys_arch_reboot() ends in
	 * exactly that call, so the behaviour is unchanged. */
	sys_reboot(SYS_REBOOT_COLD);
}

static struct bt_conn_cb connection_callbacks = {
	.connected = on_connected,
	.disconnected = on_disconnected,
	.le_param_updated = on_le_param_updated,
	.le_phy_updated = on_le_phy_updated,
	.le_data_len_updated = on_le_data_len_updated,
	.security_changed = on_security_changed,
};

/* Nothing to show: this device has no display, so the passkey travels the other
 * way (see auth_app_passkey()). Kept so that the callback table matches the
 * pairing method the phone sees. Use bt_addr_le_to_str() plus printk here to log
 * the peer address when debugging. */
static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(passkey);
}

/* Supplies the fixed pairing passkey, replacing the removed bt_passkey_set().
 * The value must be within 0..999999, or the stack answers
 * BT_SMP_ERR_UNSPECIFIED and pairing fails; returning BT_PASSKEY_RAND instead
 * asks the stack to generate a random one. */
static uint32_t auth_app_passkey(struct bt_conn *conn)
{
	ARG_UNUSED(conn);
	return PAIRING_PASSKEY;
}

static void auth_cancel(struct bt_conn *conn)
{
	ARG_UNUSED(conn);
}

static struct bt_conn_auth_cb conn_auth_callbacks = {
	.passkey_display = auth_passkey_display,
	.pairing_confirm = NULL,
	.cancel = auth_cancel,
	/* Requires CONFIG_BT_APP_PASSKEY=y; without it this member does not exist
	 * and the file does not compile. */
	.app_passkey = auth_app_passkey,
};

static int init_button(void)
{
	return dk_buttons_init(button_changed);
}

/* ============== Boot self-test indication (locating a hang without a console) ====
 *
 * A device that hangs during init gives no useful information on its own: the
 * printk()s in the failure branches are worthless when nobody is reading the
 * UART. This is the LED-only alternative.
 *
 * LED2 (CON_STATUS_LED) gives one short flash (250 ms on / 350 ms off) as main()
 * is about to attempt each boot step, so counting the flashes after power-up
 * says directly which step was reached:
 *
 *     1  SAADC configuration      (configure_saadc)
 *     2  button / LED library     (init_button)
 *     3  Bluetooth stack          (bt_enable)
 *     4  connection callbacks     (bt_conn_cb_register)
 *     5  LBS service              (my_lbs_init)
 *     6  pairing callbacks        (bt_conn_auth_cb_register)
 *     7  advertising              (bt_le_adv_start)
 *
 * On success all seven flash, then LED1 emits the startup blink and settles into
 * the 1 Hz heartbeat = healthy.
 *
 * dk_leds_init() and buzzer_init() are deliberately not numbered: they run before
 * any LED indication is trustworthy, and a failure in either returns from main()
 * with no indication at all.
 *
 * Two kinds of "stuck at step N" are distinguished by LED1:
 *   - main() failed at step N -> LED1 loops [N slow flashes, then a 1.5 s pause]
 *   - the system hung inside step N (a dead wait, say) -> LED1 stays dark and
 *     LED2 stops counting
 * Neither tells you the errno without the UART, but both pin down the layer.
 *
 * boot_step is declared at the top of the file because the fault recorder needs
 * it; it always equals the step currently being attempted, which is also the
 * number of times LED2 has flashed.
 */

static void boot_progress(void)
{
	boot_step++;
	dk_set_led_on(CON_STATUS_LED);
	k_sleep(K_MSEC(250));
	dk_set_led_off(CON_STATUS_LED);
	k_sleep(K_MSEC(350));
}

/* Boot failure indication: LED1 flashes slowly N times, pauses, then repeats.
 * N is boot_step -- the step that failed, which is also exactly the number of
 * times LED2 flashed, so the two LEDs report the same number and there is only
 * one numbering scheme to remember. app_running is still false when this runs,
 * so ring_thread never writes LED1 and cannot fight the heartbeat for it. */
static void boot_fail_beacon(void) __attribute__((noreturn));
static void boot_fail_beacon(void)
{
	while (1) {
		for (uint32_t i = 0; i < boot_step; i++) {
			dk_set_led_on(RUN_STATUS_LED);
			k_sleep(K_MSEC(400));
			dk_set_led_off(RUN_STATUS_LED);
			k_sleep(K_MSEC(400));
		}
		k_sleep(K_MSEC(1500));
	}
}

int main(void)
{
	int ret;

	/* Diagnostic banner: the build timestamp of this firmware. If it never
	 * appears on the console, the chip is not running the image just built
	 * (stale firmware, or flashing the wrong target), which rules out a
	 * whole class of misdiagnosis at once. */
	printk("\n=== Tracker Beacon boot (build %s %s) ===\n", __DATE__, __TIME__);
	printk("Device name: \"%s\"\n", DEVICE_NAME);

	/* LEDs first: until the LED pins are configured as outputs, every LED
	 * indication below is meaningless because dk_set_led() silently fails. */
	ret = dk_leds_init();
	if (ret) {
		printk("LEDs init failed (err %d)\n", ret);
		return -1;	/* no LEDs, so no LED-based indication is possible */
	}

	/* The buzzer is owned by the buzzer module (PWM1 channel 0 / P0.12).
	 * buzzer_init() wakes the PWM device and drives the output to silence
	 * explicitly; buzzer.c explains at length why the wake-up is mandatory
	 * (CONFIG_PM_DEVICE_RUNTIME leaves the device suspended at boot). Nothing
	 * sounds at power-up: the buzzer only responds to the unlock state and
	 * the distance grade, both of which are still inactive here. */
	ret = buzzer_init();
	if (ret) {
		printk("Buzzer init failed (err %d)\n", ret);
		return -1;	/* without the buzzer there is no alarm to speak of */
	}

	/* Put the filter and the distance grade into a defined initial state.
	 * After rssi_filter_init() the filter holds no samples, and after
	 * distance_manager_reset() the grade is DIST_UNKNOWN. The first sample
	 * then grades immediately against the nominal thresholds with no
	 * hysteresis, so the alarm works as soon as a connection exists instead
	 * of waiting for the window to fill. */
	rssi_filter_init();
	distance_manager_reset();

	/* ---- Boot steps. Each one is announced by boot_progress() before it is
	 * attempted, so the number of LED2 flashes equals the step number and a
	 * failure is reported with the same number on LED1. Any failure is
	 * terminal: main() stops here rather than running with a half-configured
	 * device. */
	boot_progress();	/* step 1: SAADC */
	ret = configure_saadc();
	if (ret) {
		printk("SAADC init failed (err %d)\n", ret);
		boot_fail_beacon();
	}

	boot_progress();	/* step 2: buttons */
	ret = init_button();
	if (ret) {
		printk("Button init failed (err %d)\n", ret);
		boot_fail_beacon();
	}

	boot_progress();	/* step 3: Bluetooth stack */
	ret = bt_enable(NULL);
	if (ret) {
		printk("Bluetooth init failed (err %d)\n", ret);
		boot_fail_beacon();
	}

	boot_progress();	/* step 4: connection callbacks */
	ret = bt_conn_cb_register(&connection_callbacks);
	if (ret) {
		printk("Failed to register connection callbacks (err %d)\n", ret);
		boot_fail_beacon();
	}

	boot_progress();	/* step 5: LBS service */
	ret = my_lbs_init(&app_callbacks);
	if (ret) {
		printk("Failed to init LBS (err:%d)\n", ret);
		boot_fail_beacon();
	}
	printk("Bluetooth initialized\n");

	boot_progress();	/* step 6: pairing callbacks */
	ret = bt_conn_auth_cb_register(&conn_auth_callbacks);
	if (ret) {
		printk("Failed to register authorization callbacks (err %d)\n", ret);
		boot_fail_beacon();
	}

	boot_progress();	/* step 7: advertising */
	ret = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (ret) {
		printk("Advertising failed to start (err %d)\n", ret);
		boot_fail_beacon();
	}

	/* Startup blink on LED1: 11 toggles at 100 ms, about 1.1 s. Deliberately
	 * in front of app_running = true, so this owns LED1 exclusively and the
	 * heartbeat cannot interleave with it. */
	for (int i = 0; i <= 10; i++) {
		dk_set_led(RUN_STATUS_LED, i % 2);
		k_sleep(K_MSEC(RUN_RING_BLINK_INTERVAL));
	}
	printk("Advertising successfully started as \"%s\"\n", DEVICE_NAME);

	/* Advertising is up: hand LED1 over to ring_thread for the permanent
	 * heartbeat. "LED1 is blinking" therefore means "advertising started". */
	app_running = true;

	return 0;
}

K_THREAD_DEFINE(send_data_thread_id, STACKSIZE, send_data_thread, NULL, NULL, NULL, PRIORITY, 0, 0);
K_THREAD_DEFINE(ring_thread_id, STACKSIZE, ring_thread, NULL, NULL, NULL, PRIORITY, 0, 0);
K_THREAD_DEFINE(rssi_thread_id, STACKSIZE, rssi_thread, NULL, NULL, NULL, PRIORITY, 0, 0);
