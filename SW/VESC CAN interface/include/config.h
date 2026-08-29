#pragma once
//
// config.h — all the tunable/user-specific settings for the pot -> VESC
// CAN controller live here so main.cpp stays readable.
//

#include <Arduino.h>
#include "driver/twai.h"

// ---------------------------------------------------------------------------
// CAN (TJA1051 transceiver)
// ---------------------------------------------------------------------------
#define PIN_CAN_TX               GPIO_NUM_17
#define PIN_CAN_RX                GPIO_NUM_18

// Must match VESC Tool -> App Settings -> General -> CAN Baud Rate.
// 500 kbit/s is the VESC default; change this define (and the matching
// TWAI_TIMING_CONFIG_xxx() call in main.cpp) if yours is set differently.
#define CAN_BAUDRATE_BPS          500000

// VESC Tool -> App Settings -> General -> Controller ID.
// Confirmed value for this VESC (read from its own STATUS broadcasts /
// VESC Tool's App Settings -> General -> Controller ID). If you ever swap
// to a different VESC, check this against that VESC's own Controller ID
// before relying on it — a mismatch here is why the motor won't rotate.
#define VESC_CONTROLLER_ID        97

// ---------------------------------------------------------------------------
// ADS1115 ADC (linear potentiometer)
// ---------------------------------------------------------------------------
#define PIN_I2C_SDA                7
#define PIN_I2C_SCL                6
#define ADS1115_I2C_ADDR           0x48   // default address, ADDR pin -> GND

// The working hardware is connected to ADS1115 input A0. Adafruit's API is
// zero-indexed, so channel 0 means the pin silkscreened A0.
#define POT_ADC_CHANNEL            0

// Gain / full-scale input range for the ADS1115. Pick the entry that covers
// your potentiometer supply voltage:
//   GAIN_TWOTHIRDS -> +/-6.144V  (use if the pot is powered from 5V)
//   GAIN_ONE       -> +/-4.096V  (use if the pot is powered from 3.3V)
// Default assumes the pot is powered from the ESP32's 3.3V rail.
#define POT_ADC_GAIN               GAIN_ONE

// ---------------------------------------------------------------------------
// Potentiometer calibration / mapping
// ---------------------------------------------------------------------------
// Calibration runs every boot, guided by the buzzer (see below): buzz ->
// move pot to one end -> buzz -> move pot to the other end -> longer buzz
// = done. Both endpoints (and their midpoint, used as the zero/center) are
// derived from what's actually measured, so nothing needs to be at a
// specific position when the board powers up.
#define CALIB_MOVE_WINDOW_MS        3000  // time given to move to each end
#define CALIB_SETTLE_SAMPLE_MS      500   // extra time spent averaging once there
#define CALIB_SAMPLE_INTERVAL_MS    5
#define CALIB_MIN_VALID_SPAN_RAW    500   // sanity floor for |max-min|, see main.cpp

// Safe placeholder used internally while calibration is invalid. Motor CAN
// output is gated off in that state; this value is never used to drive it.
#define POT_ADC_MAX_OFFSET         13000

// Percentage of the calibrated half-range, centered on zero, that is treated as
// "no command" (RPM = 0). Absorbs pot noise/mechanical slop around center.
#define POT_DEADBAND_PERCENT       5

// Potentiometer low-pass filter: arithmetic mean of the most recent N ADC
// samples. A larger window rejects more noise but adds more response lag.
// The first real ADC reading seeds the whole window, so the filter never
// ramps up from zero and cannot create a false command during startup.
#define POT_MOVING_AVERAGE_SAMPLES_DEFAULT  8
#define POT_MOVING_AVERAGE_SAMPLES_MAX     128

// ---------------------------------------------------------------------------
// Calibration buzzer
// ---------------------------------------------------------------------------
// Passive piezo buzzer — needs a driven PWM tone (not just a DC level) to
// make sound, so buzz() (main.cpp) drives it with the core's tone()/
// noTone() (LEDC-based PWM square wave) rather than digitalWrite().
// Note: IO39 is also the JTAG MTCK pin, but works fine as a plain GPIO/PWM
// output as long as you're not using JTAG debugging.
#define PIN_BUZZER                 39
#define BUZZER_FREQ_HZ             2500  // audible tone frequency, Hz — tune to your buzzer's resonant peak for max volume
#define BUZZ_SHORT_MS              150   // "move to this end now" cue
#define BUZZ_COMPLETE_MS           600   // "calibration complete" cue

// ---------------------------------------------------------------------------
// VESC command output: RPM (speed) or Current (torque) control
// ---------------------------------------------------------------------------
// Which VESC CAN command the pot maps to. Switchable live from the web
// UI's System tab (takes effect immediately, no restart needed — unlike
// controller ID/CAN baud).
//   VESC_CONTROL_MODE_RPM     - CAN_PACKET_SET_RPM: commands electrical
//                                RPM directly. The VESC's own internal
//                                control loop then does whatever it takes
//                                (current-wise) to hit that RPM. This is
//                                what the project has used until now.
//   VESC_CONTROL_MODE_CURRENT - CAN_PACKET_SET_CURRENT: commands motor
//                                current (torque) directly; actual speed
//                                becomes whatever the load/slope lets that
//                                torque produce. Softer, more "hand
//                                throttle"-like feel; doesn't fight a
//                                person's own walking speed the way a
//                                fixed-RPM command can.
// The closed-loop speed PID (see SPEED_PID_* below) only makes sense in
// RPM mode — it's a no-op while VESC_CONTROL_MODE_CURRENT is selected,
// regardless of its enabled setting.
#define VESC_CONTROL_MODE_RPM      0
#define VESC_CONTROL_MODE_CURRENT  1
#define VESC_CONTROL_MODE_DEFAULT  VESC_CONTROL_MODE_RPM

// CAN_PACKET_SET_RPM sets *electrical* RPM (ERPM = mechanical RPM * motor
// pole pairs), not mechanical RPM. Tune this to the max ERPM you want at
// full pot deflection, respecting your VESC's Motor Configuration ERPM/
// current limits.
//   mechanical RPM = ERPM / pole_pairs
//   wheel speed (m/s) = mechanical RPM / 60 * wheel_circumference_m
#define VESC_MAX_ERPM              3000

// ---------------------------------------------------------------------------
// Speed limit unit: ERPM (raw, as above) or km/h (converted to ERPM at
// runtime using motor pole pairs / gear ratio / wheel diameter, same math as
// the speed telemetry below)
// ---------------------------------------------------------------------------
// ERPM is awkward to reason about directly ("what does 3000 ERPM feel
// like?") and depends on pole pairs/gearing/wheel size that only matter
// electrically, not to the rider. km/h is what actually matters when
// setting a top speed, so that's the default — but ERPM is kept as an
// option since it's still the more direct/predictable knob if you'd rather
// not rely on pole pairs/wheel/gear being set correctly.
//   SPEED_LIMIT_UNIT_ERPM - sysMaxErpm/sysMaxErpmBack (VESC_MAX_ERPM etc.)
//                            are used directly, unchanged from before this
//                            existed.
//   SPEED_LIMIT_UNIT_KMH  - maxSpeedKmh/maxSpeedKmhBackward are converted to
//                            an equivalent ERPM each loop (via motorPolePairs/
//                            gearRatio/wheelDiameterMm) and that's what's
//                            actually used as the saturation/clamp limit.
//                            Get pole pairs/wheel/gear right first (see
//                            "Speed / distance conversion" on the System
//                            tab) or this will be wrong in the same way the
//                            speed telemetry would be.
#define SPEED_LIMIT_UNIT_ERPM      0
#define SPEED_LIMIT_UNIT_KMH       1
#define SPEED_LIMIT_UNIT_DEFAULT   SPEED_LIMIT_UNIT_KMH

// Starting points only — pick your real max speed here once pole pairs/
// wheel/gear are set correctly.
#define VESC_MAX_SPEED_KMH_DEFAULT           6.0f
#define VESC_MAX_SPEED_KMH_BACKWARD_DEFAULT  3.0f

// Motor current commanded at full pot deflection in CURRENT control mode,
// amps. Keep it under your VESC's Motor Configuration current limit —
// that limit is enforced by the VESC regardless of what's requested here.
#define VESC_MAX_CURRENT_A         15.0f

// ---------------------------------------------------------------------------
// Forward/backward asymmetric limits + reverse disable
// ---------------------------------------------------------------------------
// VESC_MAX_ERPM / VESC_MAX_CURRENT_A above are the FORWARD-direction limits.
// These are the separate backward-direction limits — set lower than forward
// for a slower/gentler reverse, or leave equal to forward for symmetric
// behavior (the default, i.e. no change from before this existed).
#define VESC_MAX_ERPM_BACKWARD_DEFAULT        VESC_MAX_ERPM
#define VESC_MAX_CURRENT_BACKWARD_A_DEFAULT   VESC_MAX_CURRENT_A
// When false, the backward direction is fully disabled: computeErpm()/
// computeCurrentA() return exactly 0 for any pot position past center in
// the reverse direction (not just a smaller limit) — the motor is
// commanded zero torque that way, full stop. A closed-loop PID trim or a
// braking command can never override this either (see main.cpp) — it's
// enforced as the final clamp on whatever gets sent over CAN.
#define VESC_REVERSE_ENABLED_DEFAULT          true
// Flips the electrical command/feedback sign at the VESC boundary without
// changing the user's logical forward/backward controls.
#define VESC_INVERT_MOTOR_DIRECTION_DEFAULT   false

// Current used by CAN_PACKET_SET_CURRENT_HANDBRAKE while Dashboard Park is
// active. Start conservatively: holding current creates continuous motor and
// VESC heat. The user can tune this live in System -> Downhill braking / Park.
#define PARK_HANDBRAKE_CURRENT_A_DEFAULT       5.0f

// How often a CAN command is sent. VESC's own command timeout (App
// Settings -> General -> Timeout) will stop the motor if messages stop
// arriving faster than that, so this MUST be sent well within that
// timeout. 20 Hz is a safe default; do not go below ~5 Hz.
#define CAN_SEND_INTERVAL_MS       50

// ---------------------------------------------------------------------------
// Speed / distance (from the VESC's own CAN status broadcast, i.e. actual
// measured motor speed — NOT the commanded ERPM above)
// ---------------------------------------------------------------------------
// Requires VESC Tool -> App Settings -> General -> CAN Status Message ->
// "Status Message 1" to be enabled (it broadcasts ERPM/current/duty on the
// CAN bus periodically). If the Serial Monitor keeps showing the VESC
// reading as "stale", check that setting and its rate first.

// Confirmed pole-pair count for this motor (from its datasheet / VESC
// Tool's FOC wizard). If you ever swap motors, update this before trusting
// the speed/distance numbers or the km/h speed limit.
#define MOTOR_POLE_PAIRS           14

// PLACEHOLDER — MUST SET to your actual (driven) wheel diameter in mm.
#define WHEEL_DIAMETER_MM          200.0f

// Motor shaft revolutions per wheel revolution. 1.0 = direct drive / hub
// motor. Set to your reduction ratio (e.g. 5.0 for a 5:1 belt/gear drive)
// if this isn't a direct-drive hub motor.
#define GEAR_RATIO                 1.0f

// If no CAN status frame has been received from the VESC in this long, the
// measured ERPM/speed is treated as stale (0 speed reported, trip distance
// stops accumulating) rather than trusting an old reading.
#define VESC_STATUS_TIMEOUT_MS     500

// ---------------------------------------------------------------------------
// Battery voltage sense (resistor divider into an ADC pin)
// ---------------------------------------------------------------------------
// Selectable dashboard/control telemetry source. VESC Status Message 5 is the
// first-boot default; the ESP32 divider remains available as an alternative.
#define BATTERY_VOLTAGE_SOURCE_ESP32   0
#define BATTERY_VOLTAGE_SOURCE_VESC    1
#define BATTERY_VOLTAGE_SOURCE_DEFAULT BATTERY_VOLTAGE_SOURCE_VESC

// Extra VESC status messages are commonly sent much more slowly than Status 1.
// Keep them live across low broadcast rates; the UI separately reports whether
// a message has never been received or has become stale.
#define VESC_EXTRA_STATUS_TIMEOUT_MS  10000

// Divider: BATT+ ---[510k]---+---[15k]--- GND, ESP32 ADC reads the
// midpoint. Vbatt = Vadc * (R_UPPER + R_LOWER) / R_LOWER.
#define PIN_VBAT_ADC               4
#define VBAT_DIVIDER_R_UPPER_OHM   510000.0f
#define VBAT_DIVIDER_R_LOWER_OHM   15000.0f

// Empirical correction factor — leave at 1.0 initially, then measure the
// pack with a multimeter and set this to (real_voltage / reported_voltage)
// to null out resistor tolerance error.
#define VBAT_CALIBRATION_SCALE     1.0f

// Exponential moving-average filter coefficient for the battery reading.
#define VBAT_EMA_ALPHA             0.1f

// ---------------------------------------------------------------------------
// Status LEDs
// ---------------------------------------------------------------------------
// IO40 - CAN status: blinks while sending/receiving CAN traffic, OFF when
// idle, solid ON while there's a CAN error (bus-off/recovering, or the
// last transmit failed).
#define PIN_LED_CAN                40
// IO41 - other (non-CAN) errors: blinks while such an error is active
// (currently: ADS1115 not found at boot -> blinks forever/halts; pot
// calibration span too small -> a brief warning blink, then continues).
#define PIN_LED_ERROR              41

#define CAN_LED_BLINK_MS           100  // toggle period while CAN is active
#define CAN_ACTIVITY_TIMEOUT_MS    300  // no tx/rx within this long -> LED goes idle/off
#define ERROR_LED_BLINK_MS         300  // blink period for the other-errors LED

// ---------------------------------------------------------------------------
// Debug
// ---------------------------------------------------------------------------
#define SERIAL_BAUDRATE            115200
#define DEBUG_PRINT_INTERVAL_MS    250

// Pick ONE debug view for the Serial Monitor:
//   DEBUG_MODE_CAN    - print every CAN frame sent/received (id, dlc,
//                        bytes, ok/fail) plus periodic bus status counters
//                        (state, tx/rx error counters, etc). Use this to
//                        troubleshoot the CAN bus itself.
//   DEBUG_MODE_VALUES - print the application summary line (pot position,
//                        speed, distance, battery voltage). Normal
//                        operating view, no raw CAN traffic printed.
#define DEBUG_MODE_CAN     1
#define DEBUG_MODE_VALUES  0

// <-- set this to DEBUG_MODE_CAN or DEBUG_MODE_VALUES.
// CAN traffic has been confirmed decoding correctly (status frames parse
// as expected), so this is now DEBUG_MODE_VALUES — also decluttering the
// Serial Monitor for the WebSocket-telemetry diagnostics in web_server.cpp
// (DEBUG_MODE_CAN's line-per-frame output was heavy enough that the
// throttled "[ws] broadcastTelemetry:" lines were easy to miss/scroll
// past). Switch back to DEBUG_MODE_CAN if you need to inspect raw CAN
// traffic again later.
#define DEBUG_MODE          DEBUG_MODE_VALUES

// ---------------------------------------------------------------------------
// Potentiometer calibration persistence (NVS / flash)
// ---------------------------------------------------------------------------
// Calibration (min/max raw ADC counts) is saved to flash (via the
// Preferences/NVS library) after a successful buzzer calibration, and
// loaded back on every subsequent boot — so you're not asked to
// recalibrate every restart. To force a fresh calibration anyway (e.g.
// pot was moved/replaced), send 'c' over Serial within
// CALIB_RECAL_PROMPT_MS of boot, or use the web UI's Pot Calibration tab
// (works any time, not just at boot).
#define CALIB_NVS_NAMESPACE        "potcal"
#define CALIB_RECAL_PROMPT_MS      3000

// ---------------------------------------------------------------------------
// ICM20948 9-axis IMU (compass + fall detection) — FSPI (SPI2)
// ---------------------------------------------------------------------------
// Wiring: CS=IO8, SCK=IO12, MOSI=IO11, MISO=IO13, INT=IO42 (data-ready
// interrupt; used only to time reads efficiently, all fall-detection logic
// is still evaluated in the main loop, not the ISR). Note IO42 is also the
// JTAG MTMS pin — fine as a plain GPIO as long as you're not using JTAG
// debugging, same situation as the buzzer/LED pins above.
#define PIN_IMU_CS                 8
#define PIN_IMU_SCK                12
#define PIN_IMU_MOSI               11
#define PIN_IMU_MISO               13
#define PIN_IMU_INT                42
#define IMU_SPI_CLOCK_HZ           4000000  // 4MHz — well within the ICM20948's SPI limit

// How often the IMU is sampled/fused. 100Hz is plenty for both compass and
// fall detection while leaving headroom for CAN/pot/web work each loop.
#define IMU_SAMPLE_INTERVAL_MS     10

// Complementary filter blend for pitch/roll: weight given to the
// gyro-integrated angle vs. the accelerometer's own (noisy but driftless)
// angle each sample. Closer to 1.0 = smoother but slower to correct drift.
#define IMU_COMPLEMENTARY_ALPHA    0.98f

// ---------------------------------------------------------------------------
// Fall detection
// ---------------------------------------------------------------------------
// "Tilt angle" = angle between the IMU's current measured gravity vector
// and the gravity vector captured during the upright/zero calibration —
// this works regardless of how the IMU is actually mounted (doesn't need
// to be dead level), see imu.cpp.
//
// Two independent triggers, either one latches a fall (OR'd together):
//   1) sustained tilt beyond FALL_ANGLE_THRESHOLD_DEG (a topple/roll-over)
//   2) a sudden acceleration spike beyond IMPACT_ACCEL_THRESHOLD_G (a drop
//      or hard knock that might not tip the device far, e.g. falling down
//      stairs while still land on a small footprint)
#define FALL_ANGLE_THRESHOLD_DEG      55.0f
// How long the tilt must stay past the threshold before it counts as a
// fall (debounces bumps/curbs from a real tip-over).
#define FALL_CONFIRM_MS               150
// Hysteresis margin subtracted from FALL_ANGLE_THRESHOLD_DEG for recovery,
// so it doesn't chatter in/out right at the trigger angle.
#define FALL_RECOVER_MARGIN_DEG       15.0f
// How long the device must stay continuously upright (below the recovery
// angle) before motor output auto-resumes.
#define FALL_RECOVER_STABLE_MS        2000

#define IMPACT_DETECT_ENABLED_DEFAULT true
#define IMPACT_ACCEL_THRESHOLD_G       2.5f

// How many fall/impact events (timestamp + trigger + angle) are kept for
// the web UI's event log.
#define FALL_EVENT_LOG_SIZE            20

// ---------------------------------------------------------------------------
// Fall-proximity warning buzzer
// ---------------------------------------------------------------------------
// Reuses the calibration buzzer (PIN_BUZZER) to give an audible early
// warning as tilt approaches FALL_ANGLE_THRESHOLD_DEG — silent below the
// "start" percentage of the threshold, then beeping faster and faster (beep
// period shrinks linearly) as tilt closes in on the threshold. Doesn't
// conflict with the calibration buzzer: that one only ever runs blocking,
// at boot, before the main loop (and this warning beeper) starts.
#define FALL_WARNING_BUZZER_ENABLED_DEFAULT   true
// Tilt, as a percentage of FALL_ANGLE_THRESHOLD_DEG, at which beeping
// starts (0 = beeps from dead-level upright, which would be constant noise;
// 100 = never beeps before the fall itself triggers). 50 means silence up
// to half the fall angle, then increasingly urgent beeping the rest of the
// way there.
#define FALL_WARNING_START_PERCENT_DEFAULT    50.0f
// Beep half-period (ms) at the two ends of the warning range: slow right at
// the start percentage, fast right at the threshold itself.
#define FALL_WARNING_MAX_HALF_PERIOD_MS       450
#define FALL_WARNING_MIN_HALF_PERIOD_MS       45

// ---------------------------------------------------------------------------
// Safety-interruption resume gating + "starting" warning buzzer
// ---------------------------------------------------------------------------
// FALL_RECOVER_STABLE_MS above only says the IMU no longer considers the
// device fallen (tilt's been back under the recovery band long enough).
// That alone isn't safe to resume on: the pot may have gotten knocked to a
// full-deflection position during the fall, and driving off at whatever
// that happens to command the instant recovery completes would be
// startling at best. So main.cpp adds a second, mandatory gate on top of
// Imu::isFallen() going false: the pot must also be back in its centered
// dead zone before the motor is allowed to respond to it again. Once both
// are true, this buzzer plays a short beep sequence as a "starting now"
// warning; only once it finishes does the motor actually resume. If the pot
// leaves the dead zone again mid-sequence, the beep sequence aborts and it
// goes back to waiting for the pot to re-center. This gating is always
// active and the audible restart warning is mandatory. The same center-and-
// warning gate is also used after web potentiometer calibration and motor-
// direction changes.
#define RESUME_WARNING_BEEP_COUNT      3     // beeps before the motor is allowed to resume
#define RESUME_WARNING_BEEP_ON_MS      120
#define RESUME_WARNING_BEEP_OFF_MS     120
#define RESUME_WARNING_ENABLED_DEFAULT true // retained for settings compatibility; warning is mandatory

// ---------------------------------------------------------------------------
// Magnetometer (compass) calibration
// ---------------------------------------------------------------------------
// Hard-iron offsets (µT) captured by the web UI's "rotate device" wizard;
// soft-iron (ellipse) correction is intentionally not implemented — hard-
// iron alone is usually good enough for a heading readout at this
// precision, and adding it would need a full 3x3 fit for little benefit.
#define MAG_CAL_DEFAULT_OFFSET_X       0.0f
#define MAG_CAL_DEFAULT_OFFSET_Y       0.0f
#define MAG_CAL_DEFAULT_OFFSET_Z       0.0f
// Magnetic declination at your location (degrees, + = East), so heading
// reads true north instead of magnetic north. Look yours up at
// ngdc.noaa.gov/geomag/calculators/magcalc if you care about true vs.
// magnetic north; 0 leaves it as magnetic heading.
#define MAG_DECLINATION_DEG            0.0f

// ---------------------------------------------------------------------------
// Closed-loop speed PID (trims the pot-feedforward ERPM using the VESC's
// own measured ERPM feedback over CAN). Defaults to OFF: the existing
// open-loop pot->ERPM mapping already works and is what's been tested —
// enable this from the web UI once you want the ESP32 to actively correct
// for slope/load instead of just commanding a fixed ERPM for a given pot
// position.
// ---------------------------------------------------------------------------
#define SPEED_PID_ENABLED_DEFAULT      false
#define SPEED_PID_KP_DEFAULT           0.6f
#define SPEED_PID_KI_DEFAULT           0.3f
#define SPEED_PID_KD_DEFAULT           0.02f
// Clamp on the PID's *trim*, in ERPM, added on top of the feedforward
// value — keeps a runaway PID from commanding something wildly different
// from what the pot asked for.
#define SPEED_PID_MAX_TRIM_ERPM        1500
#define SPEED_PID_UPDATE_INTERVAL_MS   50

// ---------------------------------------------------------------------------
// Downhill braking (regenerative/mechanical braking via the VESC's own
// CAN_PACKET_SET_CURRENT_BRAKE command)
// ---------------------------------------------------------------------------
// Detects "the wheel is spinning faster than the pot is currently asking
// for" (gravity/momentum overpowering the commanded speed/torque — e.g.
// rolling downhill) and, when enabled, sends a braking-current command
// instead of the normal RPM/current command for that cycle. This is
// independent of (and not blocked by) VESC_REVERSE_ENABLED_DEFAULT above —
// braking always opposes whatever direction the wheel is actually moving,
// it never drives it, so disabling reverse doesn't need to disable this.
// Off by default: the existing behavior (freewheel/coast when the pot asks
// for less than the wheel is already doing) is what's been tested.
#define DOWNHILL_BRAKING_ENABLED_DEFAULT       false
// Braking current magnitude (amps) sent via SET_CURRENT_BRAKE once engaged.
// Keep well under your VESC's Motor Configuration current limit.
#define DOWNHILL_BRAKE_CURRENT_A_DEFAULT       5.0f
// How far (in ERPM) actual measured speed must exceed the pot's currently-
// intended ERPM before braking engages — avoids chattering right at the
// setpoint from ordinary measurement noise.
#define DOWNHILL_BRAKE_ENGAGE_ERPM_MARGIN_DEFAULT  300

// ---------------------------------------------------------------------------
// Web interface (WiFi Access Point + ESPAsyncWebServer + WebSocket)
// ---------------------------------------------------------------------------
// ESP32 creates its own hotspot — connect a phone/laptop directly to it,
// no router/internet needed. Change the password before real use (WPA2
// requires >= 8 characters); both are also editable from the web UI itself
// (System tab) and persisted to NVS, these are only the first-boot
// defaults.
#define WIFI_AP_SSID_DEFAULT           "VESC-Controller"
#define WIFI_AP_PASSWORD_DEFAULT       "vesc-trailer"
#define WIFI_AP_CHANNEL                 6
#define WIFI_AP_MAX_CLIENTS              4
// The mDNS hostname is generated from the configured AP SSID. This fallback
// is used only if the SSID contains no DNS-safe letters or digits.
#define MDNS_HOSTNAME_FALLBACK          "vesc-controller"

#define WEB_TELEMETRY_INTERVAL_MS       100   // ~10Hz live dashboard updates
#define CAN_LOG_RING_SIZE               64    // frames kept for the CAN Monitor tab

#define SETTINGS_NVS_NAMESPACE          "cfg"
