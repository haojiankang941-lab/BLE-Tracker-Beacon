/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef BT_LBS_H_
#define BT_LBS_H_

/**@file
 * @defgroup bt_lbs LED Button Service API
 * @{
 * @brief API for the LED Button Service (LBS).
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/types.h>

/** @brief LBS Service UUID. */
#define BT_UUID_LBS_VAL BT_UUID_128_ENCODE(0x00001523, 0x1212, 0xefde, 0x1523, 0x785feabcd123)

/** @brief Button Characteristic UUID. */
#define BT_UUID_LBS_BUTTON_VAL                                                                     \
	BT_UUID_128_ENCODE(0x00001524, 0x1212, 0xefde, 0x1523, 0x785feabcd123)

/** @brief LED Characteristic UUID. */
#define BT_UUID_LBS_LED_VAL BT_UUID_128_ENCODE(0x00001525, 0x1212, 0xefde, 0x1523, 0x785feabcd123)

#define BT_UUID_LBS BT_UUID_DECLARE_128(BT_UUID_LBS_VAL)
#define BT_UUID_LBS_BUTTON BT_UUID_DECLARE_128(BT_UUID_LBS_BUTTON_VAL)
#define BT_UUID_LBS_LED BT_UUID_DECLARE_128(BT_UUID_LBS_LED_VAL)

/** @brief RSSI Characteristic UUID.
 *  Follows the service's existing base UUID with the first field taken one step
 *  further, from 0x1525 to 0x1526. This is a private UUID (not a Bluetooth SIG
 *  assigned number), so a phone has no name to look it up under and will most
 *  likely show it as "Unknown Characteristic"; the CUD descriptor attached to
 *  the characteristic is what gives it a readable label. */
#define BT_UUID_LBS_RSSI_VAL                                                                       \
	BT_UUID_128_ENCODE(0x00001526, 0x1212, 0xefde, 0x1523, 0x785feabcd123)

#define BT_UUID_LBS_RSSI BT_UUID_DECLARE_128(BT_UUID_LBS_RSSI_VAL)
/** @brief Ring control Characteristic UUID (a single byte is written, 0 or 1).
 *
 *  The permission is BT_GATT_PERM_WRITE_AUTHEN, so a phone must finish pairing
 *  (entering the passkey) before it can write here. Once pairing completes,
 *  att_req_retry() (att.c:3306-3315 in NCS v3.4.0) resends the pending request
 *  automatically, so the user does not have to tap a second time. */
#define BT_UUID_LBS_RING_VAL                                                                       \
	BT_UUID_128_ENCODE(0x00001528, 0x1212, 0xefde, 0x1523, 0x785feabcd123)

#define BT_UUID_LBS_RING BT_UUID_DECLARE_128(BT_UUID_LBS_RING_VAL)

/** @brief Callback type for when an LED state change is received. */
typedef void (*led_cb_t)(const bool led_state);

/** @brief Callback type for when a ring (buzzer) state change is received.
 *
 *  Kept separate from led_cb because switching a LED and sounding the buzzer are
 *  two distinct pieces of hardware: the LED goes through dk_set_led on DK_LED4
 *  (P0.20), the buzzer through PWM1 channel 0 (P0.12). Characteristic ...1525
 *  drives only the LED and ...1528 only the buzzer; neither touches the other,
 *  so "turn the LED on" can never make a sound.
 *
 *  ring_state = true starts the manual ring; false stops it and also mutes the
 *  automatic distance alarm until the next true. */
typedef void (*ring_cb_t)(const bool ring_state);

/** @brief Callback type for when the button state is pulled. */
typedef bool (*button_cb_t)(void);

/** @brief Callback struct used by the LBS Service. */
struct my_lbs_cb {
	/** LED state change callback. */
	led_cb_t led_cb;
	/** Ring state change callback. */
	ring_cb_t ring_cb;
	/** Button read callback. */
	button_cb_t button_cb;
};

/** @brief Initialize the LBS Service.
 *
 * This function registers application callback functions with the My LBS
 * Service
 *
 * @param[in] callbacks Struct containing pointers to callback functions
 *			used by the service. This pointer can be NULL
 *			if no callback functions are defined.
 *
 *
 * @retval 0 If the operation was successful.
 *           Otherwise, a (negative) error code is returned.
 */
int my_lbs_init(struct my_lbs_cb *callbacks);

/** @brief Send the button state as indication.
 *
 * This function sends a binary state, typically the state of a
 * button, to all connected peers.
 *
 * @param[in] button_state The state of the button.
 *
 * @retval 0 If the operation was successful.
 *           Otherwise, a (negative) error code is returned.
 */
int my_lbs_send_button_state_indicate(bool button_state);

/** @brief Send the connection RSSI over the RSSI characteristic.
 *
 * Sends the dBm value as a signed 8-bit number to every peer that has enabled
 * the characteristic's CCC.
 *
 * @param[in] rssi RSSI value (dBm, normally negative).
 *
 * @retval 0 on success. Also 0 when no peer is subscribed or none is connected
 *           -- that is a no-op, not a failure.
 *         Any other negative value is a real error code.
 */
int my_lbs_send_rssi_notify(int8_t rssi);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* BT_LBS_H_ */
