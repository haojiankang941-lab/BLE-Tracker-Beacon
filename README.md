# BLE Tracker Beacon

An anti-lost beacon for the **nRF52832** (nRF52 DK), built on Zephyr / nRF Connect
SDK. It advertises as a Bluetooth LE peripheral, reports the connection RSSI to the
phone, grades how far away the phone is, and sounds a buzzer when the item is
getting away. A standard Battery Service reports the battery level.

The lost-item ring is gated on pairing: writing the Ring characteristic requires
an authenticated link, so entering the pairing passkey is what "unlocks" the
buzzer. Writing `1` rings and re-arms the alarm; writing `0` stops the ring and
mutes everything — including the automatic distance alarm — until the next `1`.
The LED characteristic is deliberately unauthenticated — switching it never
triggers pairing and can never arm the buzzer.

- **Board target:** `nrf52dk/nrf52832`
- **Toolchain:** nRF Connect SDK v3.4.0 (Zephyr 4.4.0)
- **Device name:** `Tracker Beacon`

## Contents

- [Hardware](#hardware)
- [LED indicator table](#led-indicator-table)
- [Buzzer patterns](#buzzer-patterns)
- [Distance grading](#distance-grading)
- [BLE services](#ble-services)
- [Threads](#threads)
- [Build and flash](#build-and-flash)
- [Security notice](#security-notice)
- [License and attribution](#license-and-attribution)

## Hardware

| Function | Pin | Notes |
|---|---|---|
| LED1 / run status | P0.17 | `DK_LED1`, active low. Blinks, see the table below. |
| LED2 / connection status | P0.18 | `DK_LED2`, active low. |
| LED3 | P0.19 | `DK_LED3`, active low. Unused, never driven. |
| LED4 | P0.20 | `DK_LED4`, active low. Remote-controlled over BLE, latched. |
| Buzzer | P0.12 | Driven by **PWM1 channel 0**, active high through a low-side NPN (Q1, 2N3904). |
| Button 1 | P0.13 | `DK_BTN1`. Sends an indication with the button state on every change. |
| SAADC input | P0.02 | `AIN0`. Battery sampling placeholder; see the note in [Distance grading](#distance-grading). |
| Console | P0.06 / P0.08 | UART0 via the on-board VCOM. |

There is no real battery circuit on this board, so P0.02 only ever reads the SAADC's
own input. `adc_sample_to_bas_level()` compresses the raw 12-bit code linearly onto
the 0..100 range that the standard Battery Level characteristic permits
(`bas.c:117-119` rejects anything above 100). Fitting a real cell and divider means
replacing that function with one that maps millivolts against a discharge curve.

## LED indicator table

There are two independent indication paths. **LED1** reports operating state,
**LED2** reports boot progress, and the two report the same number when boot fails,
so a failure can be identified from either one.

### LED1 — P0.17, run status

| Pattern | Meaning |
|---|---|
| 11 toggles at 100 ms (~1.1 s), once | Firmware reached the end of boot and advertising has just started |
| 500 ms on / 500 ms off, forever (~1 Hz) | **Healthy.** Boot complete, advertising; this is the running heartbeat |
| Solid for 2 s, then the device reboots | The phone disconnected — the link was lost |
| N slow flashes (400 ms on / 400 ms off), then a 1.5 s pause, repeating | **Boot step N failed.** The device is stopped here |
| Dark, with LED2 also dark | `dk_leds_init()` or `buzzer_init()` failed; `main()` returned before any indication |

### LED2 — P0.18, boot progress

| Pattern | Meaning |
|---|---|
| One 250 ms flash as each boot step begins | Boot progress counter — count the flashes to see how far startup got |
| Solid for 2 s | A phone has just connected |
| Dark | Idle |

### LED4 — P0.20, remote control

Latched on or off by writing `01` / `00` to characteristic `0x1525`. It stays in that
state until the next write. It does **not** blink, and it is not related to the
buzzer.

### Reading a boot failure

A healthy boot flashes LED2 **7** times (plus the LED1 startup blink). If the device
stops early, LED2 shows how many steps were started and LED1 flashes the same number
in a loop:

| Flashes | Step that failed |
|---|---|
| 1 | SAADC configuration (`configure_saadc`) |
| 2 | Button / LED library (`init_button`) |
| 3 | Bluetooth stack (`bt_enable`) |
| 4 | Connection callbacks (`bt_conn_cb_register`) |
| 5 | LBS service (`my_lbs_init`) |
| 6 | Pairing callbacks (`bt_conn_auth_cb_register`) |
| 7 | Advertising (`bt_le_adv_start`) |

Two different faults produce the same LED2 count, and LED1 tells them apart: if
`main()` *failed* at step N, LED1 loops its N-flash pattern; if the system *hung*
inside step N (a dead wait, for example), LED1 stays dark and LED2 simply stops
counting. Neither case reveals the error code without a console — connect the VCOM
and read the `printk()` output for that.

## Buzzer patterns

The buzzer is an **active** type with its own oscillator, so PWM is used only to
express "sound / silence" and the rhythm; frequency and duty cycle do not change its
pitch. It is wired through a low-side NPN, so the pin is **active high**.

| Pattern | Sound | Silence | Used for |
|---|---|---|---|
| `BUZZER_SILENT` | — | — | Not unlocked, muted by Ring `0`, or grade NORMAL |
| `BUZZER_SLOW` | 60 ms | 3000 ms | Grade NEAR |
| `BUZZER_FAST` | 200 ms | 400 ms | Grade FAR |
| `BUZZER_CONTINUOUS` | constant | — | Grade LOST |

Which pattern actually sounds is decided by priority, highest first:

1. Not authenticated, or muted by a Ring `0` write → silence, regardless of
   anything else. This is the lock.
2. Grade FAR or LOST → the distance alarm rhythm.
3. Lost-item ring requested → `BUZZER_FAST`.
4. Otherwise → the distance-side table above.

FAR/LOST is tested explicitly rather than "the distance rhythm is not silent", so
that the manual ring keeps working while the grade is NEAR.

**Muting:** writing `0` to the Ring characteristic sets a RAM latch, so nothing
sounds — not the manual ring, not FAR/LOST, not the NEAR slow beep — until the
next `1` is written (which also starts the ring). Before the first Ring write the
latch is clear, so pairing alone still arms the distance alarm. The latch is not
persisted: disconnecting reboots the device and the alarm returns to its default
enabled state. RSSI measurement, filtering, grading and notification keep running
while muted; only the sound output stops.

> **Note:** NEAR currently maps to `BUZZER_SLOW`, i.e. the device beeps while you are
> standing next to it and goes quiet at normal distance. That mapping is this
> project's own choice and may well be the wrong way round for an anti-lost device —
> `pattern_for_level()` in [src/main.c](src/main.c) is a single switch statement if
> you want to change it.

## Distance grading

RSSI cannot be converted into a distance in metres — antenna orientation, body
blocking, multipath and transmit-power differences all swamp the path-loss model.
The device therefore grades the *link quality* into four bands rather than pretending
to know a distance:

| Grade | Filtered RSSI | Buzzer |
|---|---|---|
| `DIST_NEAR` | ≥ −50 dBm | slow beep |
| `DIST_NORMAL` | −65 .. −50 dBm | silent |
| `DIST_FAR` | −80 .. −65 dBm | fast beep |
| `DIST_LOST` | < −80 dBm | continuous |
| `DIST_UNKNOWN` | no valid sample yet | silent |

- **Filtering:** a moving average over a window of **N = 5** samples at a **100 ms**
  poll interval, so the value reflects roughly the last 0.5 seconds. While fewer than
  5 samples have arrived the average covers what is there, so a fresh connection gets
  a grade immediately instead of stalling. Raise `RSSI_FILTER_WINDOW` to 50 to restore
  the former ~5 s observation window.
- **Hysteresis:** **3 dB**. RSSI dithers by ±5 dB in normal use, so a single
  threshold would flip the grade back and forth around a boundary and make the buzzer
  sound broken. Leaving a band requires the signal to be 3 dB past the threshold that
  was needed to enter it.
- **Polling:** one `HCI_Read_RSSI` every 100 ms. The raw value is what gets sent to the
  phone; the filter serves this device's alarm decision only.
- **Reset:** disconnecting clears the filter and the grade, so a reconnection is not
  graded from the previous session's data.
- **Muting:** grading continues while the Ring characteristic holds `0`; the
  resulting rhythm is suppressed at the buzzer's master switch. See
  [Buzzer patterns](#buzzer-patterns).

## BLE services

### LED Button Service (LBS) — custom, 16-bit alias `0x1523`

Base UUID `0000xxxx-1212-efde-1523-785feabcd123`. The UUIDs are private (not
Bluetooth SIG assigned), so phones have no name to look up; the CUD descriptors below
are what make them readable in a client.

| UUID | Characteristic | Properties | Permission | Descriptor |
|---|---|---|---|---|
| `0x1524` | Button | Read, Indicate | Read: authenticated | — |
| `0x1525` | LED | Write | none (no pairing needed) | — |
| `0x1526` | RSSI | Notify | none (subscribe via CCC) | `RSSI (dBm)` |
| `0x1528` | Ring | Write | **Write: authenticated** | `Ring Control (1 = ring, 0 = mute all) - pair first` |

- **Button (`0x1524`)** — readable, and the device sends an indication whenever the
  DK button is pressed or released.
- **LED (`0x1525`)** — write `01` to switch **LED4 (P0.20)** on and `00` to switch it
  off. The state latches. This characteristic is unauthenticated and does not touch
  the buzzer, so switching the LED neither triggers pairing nor produces a sound.
- **RSSI (`0x1526`)** — notify only. Subscribe to the CCC and the device pushes the
  current raw RSSI every 100 ms as a signed 8-bit dBm value.
- **Ring (`0x1528`)** — write `01` to start the lost-item ring (and re-arm the
  distance alarm) and `00` to stop it and mute every alarm until the next `01`.
  This is the only thing that can sound the buzzer on demand.

### Battery Service — standard, `0x180F`

Provided by Zephyr's built-in implementation. The Battery Level characteristic
(`0x2A19`) is readable, so a phone can read it without subscribing, and the value is
refreshed every 60 seconds. On this board it is a placeholder — see
[Hardware](#hardware).

### Pairing

The device uses a fixed **application passkey** (`CONFIG_BT_APP_PASSKEY`), not the
display/confirm method. Connect from a client such as nRF Connect and the pairing
dialog asks for the 6-digit code.

Writing `0x1528` before pairing returns an authentication error, which makes the
phone raise its passkey prompt; once pairing completes the stack resends the
pending write automatically (`att_req_retry()`, `att.c:3306-3315`), so the user
does not have to send it a second time. Reading `0x1524` (Button) is also
authenticated and triggers the same pairing. Writing `0x1525` (LED) needs no
pairing at all.

Pairing is a property of the link, not of a characteristic: once the phone has
paired — whether to read the Button or to write the Ring — every authenticated
attribute on that link becomes available. The LED is outside the gate by design,
not protected by a second password.

### Advertising

Connectable, using the identity address, at the fast interval of **30–60 ms**
(`BT_GAP_ADV_FAST_INT_MIN_1` / `BT_GAP_ADV_FAST_INT_MAX_1`). The advertising packet
carries the device name and the flags; the 128-bit LBS service UUID is in the scan
response.

## Threads

Three threads, each with a 1024-byte stack and priority 7:

| Thread | Period | Work |
|---|---|---|
| `send_data_thread` | 60 s | Refreshes the standard Battery Service level. |
| `ring_thread` | 100 ms | Drives the LED1 heartbeat and hands the unlock-plus-mute decision and the ring request to the buzzer module. |
| `rssi_thread` | 100 ms | Reads the connection RSSI over HCI, filters it, grades it and sets the buzzer rhythm. |

The RSSI read needs its own thread because `bt_hci_cmd_send_sync()` blocks waiting for
Command Complete and therefore may only be called from a sleepable thread; the SDK
warns against issuing synchronous HCI commands from the system work queue
(`hci_core.c:450-456`).

Inside the RSSI thread the data flow is strictly *filter → grade → rhythm*. Grading
the raw value instead would feed the jitter straight into the thresholds.

## Source layout

| File | Responsibility |
|---|---|
| [src/main.c](src/main.c) | Wiring: boot sequence, connection and security callbacks, RSSI polling, and the translation from a distance grade to a buzzer rhythm. |
| [src/Beacon.c](src/Beacon.c) / [.h](src/Beacon.h) | The GATT service definition and its callbacks. Knows nothing about distance or the buzzer. |
| [src/rssi_filter.c](src/rssi_filter.c) / [.h](src/rssi_filter.h) | Moving-average filter. No knowledge of Bluetooth. |
| [src/distance_manager.c](src/distance_manager.c) / [.h](src/distance_manager.h) | Filtered dBm in, grade out. No knowledge of the buzzer. |
| [src/buzzer.c](src/buzzer.c) / [.h](src/buzzer.h) | PWM output level plus a timer-driven rhythm. No knowledge of distance or of the passkey. |
| [boards/nrf52dk_nrf52832.overlay](boards/nrf52dk_nrf52832.overlay) | HFXO cold-start time and the PWM1/buzzer pin configuration. |
| [boards/nrf52dk_nrf52832.conf](boards/nrf52dk_nrf52832.conf) | Board-level Kconfig. |
| [prj.conf](prj.conf) | Application Kconfig, annotated with the reason for each symbol. |

The three alarm modules do not know about each other; `main.c` is the only place that
connects them. That is why the alarm policy (which grade sounds how) lives there, in
one switch statement.

## Build and flash

Requires a working nRF Connect SDK v3.4.0 toolchain.

```bash
west build -b nrf52dk/nrf52832
west flash
```

To watch the boot log, open the VCOM port (UART0, 115200 8N1) before resetting the
board — the Serial Terminal built into nRF Connect for Desktop works, as does any
other terminal program.

Note that the boot banner prints the build timestamp, so seeing the log at all also
confirms that the chip is running the image you just built.

## Security notice

The pairing passkey is the compile-time constant **`123456`** (`PAIRING_PASSKEY` in
[src/main.c](src/main.c)).

This keeps an accidental bystander from connecting, but it is a **public value** that
anyone who has read this source can type in. It must not be treated as a secret. A
shipped product needs a per-device passkey derived from something unique to that unit
— the Nordic hardware ID, for example — and should consider bonding and LE Secure
Connections key storage as well.

Apart from link-layer pairing there is no other authentication, and the device
accepts a new connection from anyone who knows that passkey. The LED
characteristic (`0x1525`) is not authenticated at all: any connected peer can
switch P0.20 with a single unauthenticated write. That is intentional — it keeps
LED control from triggering pairing and hence from arming the buzzer — but it
does mean the LED is not an access-controlled resource.

## License and attribution

This project is derived from Nordic Semiconductor's **LED Button Service (LBS)**
sample and is distributed under Nordic's licence,
`SPDX-License-Identifier: LicenseRef-Nordic-5-Clause`, which is preserved in every
source file header. See [NOTICE](NOTICE) for the provenance and the list of
modifications.

`LicenseRef-Nordic-5-Clause` is a Nordic licence reference, **not** an OSI-approved
open-source licence. Review Nordic's licence terms before redistributing or using
this commercially.
