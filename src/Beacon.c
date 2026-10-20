/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include "Beacon.h"

static bool indicate_enabled;
static bool button_state;
static struct my_lbs_cb lbs_cb;

/* ==================== The "password" behind the lost-item ring ====================
 *
 * The passkey is NOT stored by this file: it is the *link-layer pairing
 * passkey*, supplied by auth_app_passkey() in main.c. Of the writable
 * characteristics, only the Ring one carries BT_GATT_PERM_WRITE_AUTHEN. The LED
 * one carries plain BT_GATT_PERM_WRITE on purpose: switching the LED must not
 * trigger pairing, because pairing is what arms the buzzer.
 *
 * For the Ring characteristic, gatt.c:3105-3109 (NCS v3.4.0) says of the
 * authentication permission mask:
 *
 *     if (mask & BT_GATT_PERM_AUTHEN_MASK) {
 *             if (bt_conn_get_security(conn) < BT_SECURITY_L3) {
 *                     return BT_ATT_ERR_AUTHENTICATION;
 *             }
 *     }
 *
 * In other words write_ring() is invoked only while the link is authenticated
 * (>= L3). If the peer has not paired yet, the stack answers with an
 * authentication error, the phone prompts for the passkey, and once pairing
 * completes att_req_retry() (att.c:3306-3315) resends the same request -- which
 * is why the user sees "enter the passkey and the write just succeeds" without
 * tapping twice.
 *
 * Note that pairing is per *link*, not per characteristic: once the peer pairs
 * for the Ring characteristic, every AUTHEN attribute on that link is usable.
 * The LED does not use a second password; it is simply outside the gate.
 *
 * So write_ring() does not need a lock check of its own. It also does not
 * interpret the value beyond 0/1: main.c decides that 1 rings and re-arms the
 * buzzer while 0 stops the ring and mutes all alarms.
 *
 * Concrete link-layer line numbers drift between SDK releases; they are quoted
 * here against nRF Connect SDK v3.4.0. */

static struct bt_gatt_indicate_params ind_params;

static void mylbsbc_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	indicate_enabled = (value == BT_GATT_CCC_INDICATE);
}

/* Called once a remote device has acknowledged the indication at its host layer. */

static void indicate_cb(struct bt_conn *conn, struct bt_gatt_indicate_params *params, uint8_t err)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(params);
	ARG_UNUSED(err);
}

static ssize_t write_led(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
			 uint16_t len, uint16_t offset, uint8_t flags)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(attr);
	ARG_UNUSED(flags);

	if (len != 1U) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	if (offset != 0) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	if (lbs_cb.led_cb) {
		uint8_t val = *((uint8_t *)buf);

		if (val == 0x00 || val == 0x01) {
			lbs_cb.led_cb(val ? true : false);
		} else {
			return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
		}
	}

	return len;
}

/* Write the ring switch (one byte, 0 or 1).
 *
 * 1 = start the lost-item ring and re-arm the buzzer; 0 = stop the ring and
 * mute every alarm, including the automatic RSSI distance beeping, until a 1
 * is written again. The application-side latch lives in main.c, so this layer
 * stays free of buzzer knowledge.
 *
 * There is deliberately no "is it unlocked" test here. The characteristic's
 * permission is BT_GATT_PERM_WRITE_AUTHEN, and gatt.c:3105-3109 guarantees such
 * a write callback runs only when bt_conn_get_security(conn) >=
 * BT_SECURITY_L3, i.e. the peer has paired and authenticated. Reaching this
 * function therefore already means the passkey was accepted; testing it again
 * would be redundant. See the comment block at the top of this file. */
static ssize_t write_ring(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
			  uint16_t len, uint16_t offset, uint8_t flags)
{
	uint8_t val;

	ARG_UNUSED(conn);
	ARG_UNUSED(attr);
	ARG_UNUSED(flags);

	if (offset != 0U) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	if (len != 1U) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	val = *(const uint8_t *)buf;
	if (val > 1U) {
		return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}

	if (lbs_cb.ring_cb) {
		lbs_cb.ring_cb(val ? true : false);
	}

	return (ssize_t)len;
}

static ssize_t read_button(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
			   uint16_t len, uint16_t offset)
{
	/* attr->user_data points at button_state, the pointer handed to
	 * BT_GATT_CHARACTERISTIC(..., &button_state). It is taken as a bool* and
	 * not a char* so that sizeof(*value) matches the type: both are 1 today,
	 * so the number of bytes on the wire is unchanged, but this removes the
	 * trap where a change of type for button_state would silently truncate.
	 */
	const bool *value = attr->user_data;

	if (lbs_cb.button_cb) {
		button_state = lbs_cb.button_cb();
		return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(*value));
	}

	return 0;
}

/* LED Button Service Declaration */
BT_GATT_SERVICE_DEFINE(
	my_lbs_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_LBS),
	BT_GATT_CHARACTERISTIC(BT_UUID_LBS_BUTTON, BT_GATT_CHRC_READ | BT_GATT_CHRC_INDICATE,
			       BT_GATT_PERM_READ_AUTHEN, read_button, NULL, &button_state),
	BT_GATT_CCC(mylbsbc_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

	/* LED characteristic. Deliberately BT_GATT_PERM_WRITE rather than
	 * ..._AUTHEN: writing it must not trigger pairing, because the pairing
	 * passkey is the buzzer's unlock (…1528). With AUTHEN here, a user who
	 * only wanted to switch the LED would pair as a side effect, and
	 * on_security_changed() would arm the distance alarm at the same moment.
	 */
	BT_GATT_CHARACTERISTIC(BT_UUID_LBS_LED, BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, NULL,
			       write_led, NULL),

	/* RSSI characteristic: notify only, value is a signed 8-bit dBm reading,
	 * so the value attribute is PERM_NONE -- a phone subscribes by writing
	 * the CCC rather than reading the attribute directly.
	 *
	 * New characteristics are always appended at the END of the service so
	 * that the indices of the attributes above never move. That is what keeps
	 * the hard-coded attrs[2] in my_lbs_send_button_state_indicate() (the
	 * Button characteristic value) valid; the RSSI block occupies [6]
	 * declaration, [7] value, [8] CCC, [9] CUD.
	 *
	 * The CUD descriptor gives the phone a readable label: BLE
	 * characteristics have no "name" field, so a client can only derive a
	 * display name by looking the UUID up in its built-in table, and 0x1526
	 * is a private number that no such table contains.
	 */
	BT_GATT_CHARACTERISTIC(BT_UUID_LBS_RSSI, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, NULL,
			       NULL),
	/* Passing NULL as the CCC's cfg_changed is legal: gatt.c:2076 and :3325
	 * both check for NULL before calling. This service does not need to track
	 * its own subscription state -- whether a value should be sent is decided
	 * by the host from the CCC (gatt.c:2401-2410).
	 */
	BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CUD("RSSI (dBm)", BT_GATT_PERM_READ),

	/* Ring control characteristic, appended at the end for the same
	 * index-stability reason as above: nothing in [0]..[9] moves, so attrs[2]
	 * stays valid. This characteristic occupies [10] declaration,
	 * [11] value, [12] CUD.
	 *
	 * The permission is BT_GATT_PERM_WRITE_AUTHEN, which is how "pair with
	 * the device and enter the passkey before you can start the lost-item
	 * ring" is implemented: the passkey lives at the link layer, supplied by
	 * auth_app_passkey() in main.c. Writing 0 needs the same authentication
	 * and is the only way to mute the distance alarm, so the gate also covers
	 * "silence everything".
	 *
	 * It carries a CUD because 0x1528 is a private UUID that no phone's
	 * characteristic lookup table contains -- without the descriptor it would
	 * show up as "Unknown Characteristic".
	 */
	BT_GATT_CHARACTERISTIC(BT_UUID_LBS_RING, BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE_AUTHEN, NULL,
			       write_ring, NULL),
	BT_GATT_CUD("Ring Control (1 = ring, 0 = mute all) - pair first", BT_GATT_PERM_READ),
);
/* A function to register application callbacks for the LED and Button characteristics  */
int my_lbs_init(struct my_lbs_cb *callbacks)
{
	if (callbacks) {
		lbs_cb.led_cb = callbacks->led_cb;
		lbs_cb.ring_cb = callbacks->ring_cb;
		lbs_cb.button_cb = callbacks->button_cb;
	}

	return 0;
}

int my_lbs_send_button_state_indicate(bool button_state)
{
	if (!indicate_enabled) {
		return -EACCES;
	}
	ind_params.attr = &my_lbs_svc.attrs[2];
	ind_params.func = indicate_cb; // A remote device has ACKed at its host layer (ATT ACK)
	ind_params.destroy = NULL;
	ind_params.data = &button_state;
	ind_params.len = sizeof(button_state);
	return bt_gatt_indicate(NULL, &ind_params);
}

int my_lbs_send_rssi_notify(int8_t rssi)
{
	/* bt_gatt_notify_uuid() looks the characteristic up by UUID instead of
	 * hard-coding attrs[n], so characteristics can be added to or removed
	 * from this service without getting an index wrong. attrs[0] is the
	 * service declaration, whose position is fixed, and serves as the search
	 * origin (gatt.h:1517-1518). A NULL first argument means "notify every
	 * peer that has enabled the CCC".
	 */
	int rc = bt_gatt_notify_uuid(NULL, BT_UUID_LBS_RSSI, &my_lbs_svc.attrs[0], &rssi,
				     sizeof(rssi));

	/* "Nobody subscribed" (with CONFIG_BT_GATT_ENFORCE_SUBSCRIPTION=y the host
	 * reports -EINVAL, see gatt.c:2401-2410) and "nobody connected"
	 * (-ENOTCONN, gatt.c:2812) are both normal states for a periodic sender,
	 * so reporting 0 rather than an error is deliberate -- the same
	 * normalisation bas.c:129 applies to -ENOTCONN. */
	if (rc == -EINVAL || rc == -ENOTCONN) {
		return 0;
	}

	return rc;
}
