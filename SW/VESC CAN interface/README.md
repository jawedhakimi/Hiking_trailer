# VESC CAN Controller — pot follow + LCD + IMU + web dashboard (ESP32-S3 N16R8)

Single ESP32-S3 module that reads a centered linear potentiometer through
an ADS1115 ADC and drives a VESC motor controller directly over CAN
(through a TJA1051 transceiver), fuses a 9-axis ICM20948 IMU for compass
heading and fall detection (motor cuts out on a fall, auto-resumes once
upright), shows live telemetry on the attached 1.69-inch touch LCD, and
hosts its own WiFi Access Point with a web dashboard for
live telemetry, a CAN bus monitor, and calibrating/tuning everything
without reflashing. No ESP-NOW / second ESP32 — this is a single-node
design.

## Hardware / wiring

| Signal | ESP32-S3 pin | Notes |
|---|---|---|
| CAN TX -> TJA1051 TXD | IO17 | |
| CAN RX <- TJA1051 RXD | IO18 | |
| I2C SDA -> ADS1115 | IO7 | primary I2C bus |
| I2C SCL -> ADS1115 | IO6 | primary I2C bus |
| Potentiometer wiper -> ADS1115 | A0 (first single-ended channel) | Confirmed working connection; `POT_ADC_CHANNEL = 0` uses Adafruit's zero-indexed API |
| Battery voltage divider midpoint | IO4 | 510k (pack+ side) / 15k (GND side) divider |
| Calibration buzzer | IO39 | passive piezo buzzer, driven with a PWM tone |
| CAN status LED | IO40 | blinks on CAN tx/rx activity, off when idle, solid ON on CAN error |
| Other-error LED | IO41 | blinks on non-CAN errors |
| ICM20948 CS | IO8 | shared FSPI bus, independent chip select |
| ICM20948 SCK | IO12 | FSPI |
| ICM20948 MOSI | IO11 | FSPI, shared with LCD DIN |
| ICM20948 MISO | IO13 | FSPI |
| ICM20948 INT | IO42 | reserved for future interrupt-driven reads; currently just an input, not wired into the ISR yet |
| LCD DIN / CLK | IO11 / IO12 | shared FSPI MOSI/SCK with IMU |
| LCD CS / DC | IO10 / IO15 | independent chip select |
| LCD reset / backlight | IO45 / IO46 | |
| Touch SDA / SCL | IO47 / IO48 | secondary I2C bus (`Wire1`) |
| Touch reset / interrupt | IO14 / IO21 | CST816S |

The LCD Home page shows measured VESC speed, selected battery voltage,
trip distance, compass heading, and shaft-power state. The Info page adds
motor current, duty cycle, FET/motor temperatures, CAN freshness, and the
safety state. Navigation is touch-enabled, and so are several controls —
these are live, not read-only:
- Home page **Enable switch** — toggles shaft power the same way the web
  UI's Enable does, including the same rejection (pot calibration must be
  complete/not mid-calibration) if it can't be turned on.
- Settings page **Fall Angle slider** — changes
  `g_settings.fallAngleThresholdDeg` live and saves it to flash ~750ms
  after the last drag.
- Settings page **Calibrate / PotMin / PotMax buttons** — drive the same
  pot-endpoint calibration state machine as the web UI's calibration
  wizard.
- Settings page **Set Upright Zero button** — runs the IMU's upright-zero
  calibration, same as the web UI's IMU tab.

All of these go through the same `ControlLock` guarding and settings/
calibration code paths the web UI uses, so the two stay consistent with
each other.

TJA1051 needs its own 3.3V/5V + GND per its datasheet, `CANH`/`CANL` go to
the VESC's CAN bus (with a 120Ω termination resistor at each physical end
of the bus — the VESC usually has one built in/switchable; add one at the
ESP32 end if this is the far end of the bus).

Potentiometer: pin 1 -> 3.3V, pin 3 -> GND, pin 2 (wiper/signal) -> ADS1115
channel A0. This keeps the wiper's voltage swing within the ADS1115's
±4.096V (`GAIN_ONE`) input range by default; change `POT_ADC_GAIN` in
`include/config.h` if you power the pot from 5V instead.

ICM20948: note IO42 is also the JTAG MTMS pin (same situation as the
buzzer on IO39/LEDs on IO40-41) — fine as a plain GPIO as long as you're
not doing JTAG debugging.

## Project layout

```
platformio.ini        - board/target config, library deps, LittleFS filesystem setting
include/config.h       - pins + first-boot defaults for every tunable (see settings.h for the runtime layer)
data/                  - web dashboard (index.html, style.css, app.js) — flashed separately, see below
src/main.cpp           - setup/loop: pot read -> ERPM, IMU update + fall cutoff, CAN send, web telemetry handoff
src/display_app.*       - ST7789/CST816S + LVGL bridge and live LCD telemetry updates
lib/ui/                 - generated SquareLine Studio 280x240 UI
src/settings.h/.cpp     - runtime-tunable settings (deadband, PID gains, fall thresholds, WiFi creds, ...), NVS-backed
src/potcal.h/.cpp       - potentiometer endpoint calibration (buzzer-guided at boot, or live from the web UI), NVS-backed
src/vesc_can.h/.cpp     - VESC CAN packet builder/parser (SET_RPM/CURRENT/DUTY, Status 1/2/4/5) + CAN Monitor ring buffer
src/imu.h/.cpp          - ICM20948 driver: tilt-compensated compass heading, mounting-independent fall/impact detection
src/speed_pid.h/.cpp    - small generic PID controller (used as an optional trim on the pot's feedforward ERPM)
src/odometry.h/.cpp     - trip distance from Status 5 tachometer, with Status 1 ERPM fallback
src/web_server.h/.cpp   - WiFi AP, LittleFS static hosting, REST API, WebSocket telemetry/CAN-log broadcast
```

## Before you build

1. **Install PlatformIO** (VS Code extension or `pip install platformio`).
2. Open this folder as a PlatformIO project.
3. `platformio.ini` targets the generic `esp32-s3-devkitc-1` board with
   flash/PSRAM overrides for N16R8 (16MB flash / 8MB octal PSRAM). If
   `pio run` fails to find that board on your installed platform version,
   try swapping to `board = esp32-s3-devkitc-1-n16r8` and remove the
   `board_build.*` override lines — see the comment in `platformio.ini`.
4. This firmware has two independent flash images — **do both** the first
   time:
   ```
   pio run --target upload      # the firmware itself (src/)
   pio run --target uploadfs    # the web dashboard (data/) onto LittleFS
   ```
   After that, `uploadfs` only needs re-running when you change files
   under `data/`; `upload` (or plain `pio run` + `upload`) is enough for
   any C++ change.
5. This was verified with a real `pio run` (firmware) and
   `pio run --target buildfs` (filesystem image) in a clean environment —
   both compiled without errors or warnings before this was handed back to
   you, so a fresh `pio run` should Just Work modulo your local toolchain.

## VESC-side setup (do this first, in VESC Tool)

1. **App Settings -> General -> Controller ID**: note the value — you can
   set it either in `include/config.h` (`VESC_CONTROLLER_ID`, first-boot
   default) or later from the web UI's System tab (persisted, takes effect
   after a restart).
2. **App Settings -> General -> CAN Baud Rate**: confirm/set to match
   `CAN_BAUDRATE_BPS` (500 kbit/s by default) — also editable from the web
   UI's System tab now (restart required).
3. **App Settings -> General -> Timeout**: this is your motor's fail-safe —
   if CAN commands stop arriving for longer than this, the VESC stops the
   motor on its own, independent of anything this firmware does. Keep it
   short (e.g. 500ms-1s).
4. **App Settings -> General -> CAN Status Message**: enable at least
   **Status Message 1** (speed/current/duty) — needed for the dashboard's
   speed readout and the optional PID. Enable **Status Message 4**
   (temperatures + input current) and **Status Message 5** (tachometer +
   input voltage) too if you want those on the dashboard and real
   tachometer-based trip distance instead of no distance at all. The CAN
   Monitor decodes all six standard broadcasts: drive values, Ah, Wh,
   temperatures/input current/PID position, tachometer/input voltage, and
   ADC/PPM inputs.
5. **Motor Configuration -> Limits**: set sane ERPM and current limits —
   enforced by the VESC regardless of what this firmware (or a bad pot
   reading/wiring fault) requests. This is your real safety backstop.
6. Confirm CAN termination: 120Ω across CANH/CANL at each physical end of
   the bus.

## Connecting to the web dashboard

The ESP32 creates its own WiFi network (Access Point) — no router or
internet needed:

1. After flashing both images and powering up, connect your phone/laptop
   to WiFi network **`VESC-Controller`** (password `vesc-trailer` by
   default — **change this from the System tab** once you're in, it's not
   a real secret otherwise).
2. Open `http://192.168.4.1/` or the lowercase SSID followed by `.local`
   in a browser. For example, the default **`VESC-Controller`** SSID is
   available at `http://vesc-controller.local/` after the controller starts.
3. You should land on the Dashboard tab with live telemetry updating
   ~10x/second over a WebSocket. If it says "connecting…" and never goes
   green, double check you're on the right WiFi network and that
   `uploadfs` was actually run (the REST API still works without it, but
   there's no page to load).

Tabs: **Dashboard** (speed/power/orientation/CAN bus at a glance, boot-resetting
motor-power permission, Park handbrake switch, fall status banner), **CAN Monitor** (live frame log plus decoded Status 1–6
values), **Pot Calibration** (live raw reading + capture-endpoint wizard),
**Speed PID** (enable/tune the closed-loop trim, live target-vs-actual
chart), **IMU / Fall / Compass** (live orientation, upright-zero and
magnetometer calibration, fall threshold tuning, event log), **System**
(VESC/CAN settings, speed/distance conversion, selectable ESP32/VESC battery voltage, WiFi
credentials, restart / factory reset).

## Potentiometer calibration

Two ways to (re)calibrate, both save to the same flash-backed min/max —
use whichever's convenient:

**Buzzer-guided (at boot, no WiFi needed)** — unchanged from before: runs
automatically the first time there's no saved calibration, or any time you
send `c` + Enter over Serial within `CALIB_RECAL_PROMPT_MS` (3s) of boot.
Short buzz -> move to one end -> short buzz -> move to the other end ->
long buzz = saved.

**Web wizard (Pot Calibration tab, any time)** — click **Start calibration**
first. The controller immediately sends zero and locks motor commands. Move
the pot to one end and click **Capture MIN**, then move it to the other end
and click **Capture MAX**. The new pair is staged and only replaces the saved
calibration when its span is valid. Motor output remains locked afterward
until the handle returns to the newly calibrated center.

Either way, if the measured span comes back too small (`<
CALIB_MIN_VALID_SPAN_RAW`, default 500 counts) it's rejected — the buzzer
flow leaves motor CAN output disabled and blinks the IO41 warning LED. The
dashboard remains available so valid MIN and MAX endpoints can be captured;
motor commands begin only after the calibration becomes valid.

If you just want to sanity-check the raw ADC signal before wiring anything
else up, set `#define CALIBRATION_PRINT_RAW 1` at the top of
`src/main.cpp` — it streams `raw / filtered / offsetFromCenter` to Serial
and never drives the motor.

## IMU: compass + fall detection

The ICM20948 does two independent jobs, both explained in more depth in
the comments at the top of `src/imu.cpp`:

**Fall detection** (safety-critical) measures the angle between the
*current* gravity vector and whatever gravity vector was captured during
calibration — so it works regardless of exactly how the board is mounted.
Calibrate it from the **IMU tab -> "Set upright zero"** with the device
sitting the way it normally does when everything's fine. Two independent
triggers, either one latches a fall:
- sustained tilt past **Fall angle threshold** (default 55°) for at least
  **Confirm time** (default 150ms) — a topple/roll-over;
- a sudden acceleration spike past **Impact threshold** (default 2.5g) —
  catches a hard knock/drop that might not tip the device far enough to
  trip the angle check. Toggle-able independently.

On a fall: CAN motor commands immediately switch to `0` ERPM (this
firmware does **not** attempt to cut system power — see "What 'shutdown'
means here" below). Recovery has two stages, both automatic — no button
press needed for either, though there's also a manual "force-clear" button
for testing:

1. Once tilt drops back below `(threshold - recovery margin)` and *stays*
   there continuously for the **Auto-recover stable time** (default 2s),
   the IMU itself no longer considers the device fallen.
2. That alone doesn't restart the motor — the pot may have gotten knocked
   to a full-deflection position during the fall, and driving off at
   whatever that happens to command the instant the timer above elapses
   would be startling at best. So the motor stays at zero until the pot is
   back in its centered dead zone, then the buzzer beeps a few times as a
   "starting now" warning (toggle-able — **IMU tab**), and only once that
   finishes does normal control actually resume. Moving the pot back out of
   the dead zone mid-warning aborts it and goes back to waiting for it to
   re-center.

All of this — thresholds, margin, timings, live tilt readout, and a
timestamped event log — is on the IMU tab.

**Tune these on the actual hardware**: the angle threshold and confirm
time genuinely depend on your mounting and how bumpy the ride is — too
sensitive and curbs/bumps look like falls, too loose and a real fall takes
too long to catch. Start conservative and watch the live tilt number on
the dashboard while you shake/bump/tilt the device by hand before trusting
it in the field.

**Compass heading** uses standard tilt-compensated pitch/roll math, which
assumes the IMU's Z axis is roughly vertical (board mounted roughly
horizontal) — this is a *separate* assumption from the fall-detection
angle above, which doesn't care about mounting orientation at all.
Calibrate the magnetometer from the IMU tab: **Start**, slowly tumble the
whole device through all axes for 15-20s (figure-8 motions work well),
**Stop & Save** — this computes and stores a hard-iron offset per axis.
Set **Magnetic declination** for your location if you want true-north
instead of magnetic-north headings (look yours up at
ngdc.noaa.gov/geomag/calculators/magcalc).

If the IMU isn't detected at boot (wiring issue), the firmware logs a
warning and **keeps running without fall protection** rather than halting
the whole controller — the dashboard shows a persistent red banner
("fall protection is NOT active") so this is impossible to miss, but pot
-> CAN control still works. Check FSPI wiring (CS=8 SCK=12 MOSI=11
MISO=13) if you see this.

### What "shut-down" means here

Per your answer during setup: a fall makes this firmware **stop commanding
the motor** (repeated `SET_RPM 0` over CAN) — it does **not** try to cut
power to the ESP32 or the rest of the system. That needs hardware this
design doesn't currently have (a self-latching power relay/MOSFET with an
enable GPIO) — if you want true power-off-on-fall later, that's a hardware
addition (a relay in the battery/12V feed, gated by a spare GPIO) plus a
few lines in `imu.cpp`'s fall-latch code to drive it; happy to help design
that when/if you add the relay.

## Speed PID (optional closed-loop trim)

Off by default — the original open-loop pot -> ERPM mapping already drives
the motor and is what's been tested. When enabled (Speed PID tab), the
ESP32 reads the VESC's own measured ERPM back over CAN (Status Message 1)
and runs a PID loop whose output is *added* to the pot's feedforward ERPM,
clamped to `±Max trim`, so a bad PID tune can't override the pot's
sense of direction/magnitude wildly, just trim it. Useful if you notice
the trailer's actual speed sagging under load/slope compared to what a
given pot position used to give you on flat ground.

Tuning tips: start with `Ki=0` and only `Kp`, increase until it tracks
reasonably without oscillating, then add a little `Ki` to kill steady-state
error, `Kd` last (and sparingly — CAN feedback is relatively low-rate and
noisy-derivative gains amplify that). The live target-vs-actual chart on
the same tab is there specifically for this. The PID resets its internal
state whenever VESC status feedback goes stale (bus glitch, VESC rebooted,
etc.) so it doesn't "wake up" with a stale integral term.

## Trip distance

Prefers the **VESC's own tachometer** (CAN Status Message 5), scaled by pole
pairs / gear ratio / wheel diameter. When Status 5 is disabled or becomes
stale, it automatically integrates Status 1 ERPM instead so the trip counter
continues working. The dashboard shows whether the current source is the
tachometer or an ERPM estimate. Reset it with the Dashboard tab's "Reset
trip" button.

## Dashboard Park

The slide switch beside the speed gauge sends the VESC's dedicated
`CAN_PACKET_SET_CURRENT_HANDBRAKE` command repeatedly while Park is active,
holding the wheel at its current position. Park stays active if the browser
disconnects, but intentionally starts off after a controller reboot. Releasing
Park uses the same center-handle and warning-beep gate as other safety
interruptions before drive commands resume. Set the holding current in System;
use the lowest reliable value because sustained holding current heats the motor
and VESC. Falls and incomplete/active potentiometer calibration override Park.

## Motor-power enable

The header's **Power Enabled/Disabled** switch is a runtime permission and always starts
Disabled after an ESP32 boot or restart. While it is Disabled, the normal control loop
sends no periodic motor commands, so moving the potentiometer cannot start or
hold the motor. Switching it OFF sends one immediate zero-current command,
clears Park, and then stops transmission. Switching it ON still requires the
handle to be centered and the restart warning beeps to finish before drive
commands resume. The adjacent **Restart Device** button reboots the ESP32 and
therefore returns motor power to Disabled.

## Debug mode (`DEBUG_MODE` in `config.h`)

Unchanged: one `#define` picks what the Serial Monitor shows.

```c
#define DEBUG_MODE   DEBUG_MODE_CAN     // or DEBUG_MODE_VALUES
```

- **`DEBUG_MODE_CAN`** — prints every CAN frame sent and received plus a
  periodic bus-status line with the TWAI driver's error counters. Useful
  for CAN-level troubleshooting; **the web UI's CAN Monitor tab now covers
  most of this without needing a Serial connection**, so this mode is
  mainly for bench debugging before WiFi/the dashboard are even relevant.
- **`DEBUG_MODE_VALUES`** — an application summary line (position, speed,
  trip distance, battery, tilt, heading, fall state) instead.

## Status LEDs (IO40 CAN, IO41 other errors)

Unchanged from before — see the comments in `config.h`
(`PIN_LED_CAN`/`PIN_LED_ERROR`) and `updateCanStatusLed()` /
`haltWithCanError()` / `haltWithOtherError()` in `main.cpp`. Briefly:
IO40 blinks on CAN activity, solid ON on a CAN error/init failure. IO41
blinks-forever on a fatal non-CAN error (ADS1115 missing), or a short
6-blink warning burst on a non-fatal one (pot calibration span too small).

## Tuning — now mostly live/persisted, not just compile-time

Everything that used to be a `#define` you'd edit and reflash for routine
tuning (deadband, moving-average sample count, max ERPM, CAN send interval, controller
ID, CAN baud, pole pairs, wheel diameter, gear ratio, battery calibration
scale, PID gains, fall thresholds, WiFi credentials) now lives in
`settings.h`/`settings.cpp`, is editable from the relevant web UI tab, and
is saved to flash (NVS) immediately on Save — it survives reboots. The
`#define`s in `config.h` are now just the **first-boot defaults** (and the
pins, which are still compile-time since they're physical wiring). Two
fields (VESC controller ID, CAN baud rate) need a restart to take effect
since they require reinitializing the CAN driver — the web UI tells you
when that's the case and offers a one-click restart.

## Safety notes

This drives a motorized trailer that follows a person — please treat every
fail-safe below as a backstop, not a substitute for testing with the
trailer up on blocks (wheels off the ground) first:

- The firmware halts (and never drives the motor) if the ADS1115 isn't
  found on I2C, or if the CAN driver fails to install/start. The IMU
  missing is **not** fatal (see "IMU" above) — but fall protection is off
  until it's fixed, which the dashboard makes loud and obvious.
- Once the user explicitly enables motor power, a command (including explicit
  `0` ERPM inside the deadband or after a fall) is sent at a fixed rate, so the
  VESC's own Timeout fail-safe engages quickly if this ESP32 crashes or loses
  power. Before Enable, no periodic motor commands are sent.
- Fall detection overrides everything else (including the speed PID) —
  the ERPM command is forced to `0` whenever `Imu::isFallen()` is true,
  computed *after* the pot/PID logic, right before the value is sent. This
  override actually stays in force a bit longer than `Imu::isFallen()`
  itself: `updateSafetyResumeGating()` in `main.cpp` also requires the pot to be
  back in its centered dead zone and the mandatory resume warning beep to
  finish before letting the motor respond again — see "IMU: compass +
  fall detection" above. Pot calibration and motor-direction changes use the
  same center-and-beep gate before control resumes.
- Set the VESC's Motor Configuration current/ERPM limits conservatively —
  they're enforced independently of anything this firmware sends.
- Add a physical, always-accessible power cutoff / e-stop to the VESC's
  main power, independent of this controller.
- The WiFi AP has no bearing on safety-critical behavior — the control
  loop runs the same whether or not anyone's connected to the dashboard;
  the web layer only reports state and lets you change settings, it does
  not sit in the motor command path.

## Open items / assumptions to double-check

- **Potentiometer input**: the working hardware uses ADS1115 A0
  (`POT_ADC_CHANNEL = 0` in Adafruit's zero-indexed API).
- **Pot supply voltage / ADS1115 gain**: defaults to `GAIN_ONE` (±4.096V),
  assuming 3.3V pot supply. Change `POT_ADC_GAIN` in `config.h` for 5V.
- **VESC CAN Controller ID / baud rate**: confirm in VESC Tool, set from
  the System tab if not the current defaults (97 / 500 kbit/s).
- Control mode is **RPM (ERPM)**, sent via `CAN_PACKET_SET_RPM`, optionally
  trimmed by the closed-loop PID above. `vesc_can.h/.cpp` also has
  `sendSetCurrent()`/`sendSetDuty()` if you want to switch modes later.
- **Motor pole pairs, wheel diameter, gear ratio**: set real values on the
  System tab ("Speed / distance conversion" card) — they affect displayed
  speed, the km/h speed limit conversion below, the optional PID's feedback
  scaling, and trip distance.
- **Speed limit unit**: the System tab's "Control mode" card lets the max
  forward/backward speed (RPM mode) be set in **km/h** (default) or raw
  **ERPM** — pick whichever's more useful; the field for the unit *not*
  selected shows a grayed-out, live-computed equivalent using the pole
  pairs/wheel/gear values above, so you can sanity-check either number. The
  km/h option only converts to the right ERPM if pole pairs/wheel/gear are
  actually set correctly — same caveat as the speed telemetry.
- **Battery divider calibration scale**: the System tab has a "measure &
  compute" helper — read the pack with a multimeter, type it in, done.
  It only applies to the **ESP32 analog divider** source. Selecting
  **VESC CAN Status 5** uses the VESC-reported input voltage directly and
  requires Status Message 5 to remain enabled.
- **Motor direction**: if the physical motor runs backward for a Forward
  command, enable **Invert motor direction** in the System tab. Commands and
  VESC speed/current feedback are sign-corrected together. Changing it while
  the handle is displaced holds motor output at zero until the handle returns
  to center.
- **IMU mounting**: run the "Set upright zero" and magnetometer
  calibration wizards once the IMU is physically mounted where it'll stay
  — both are orientation-dependent.
- **Fall thresholds are un-tuned defaults** — validate by hand (tilt/bump
  the device while watching the live dashboard) before relying on them.
- **WiFi AP password**: change `vesc-trailer` from the System tab before
  relying on this for anything you'd mind a stranger connecting to.
- Not implemented (ideas for later, if useful): true power-off-on-fall via
  a relay (see "What shutdown means here"), onboard DMP/quaternion fusion
  instead of the current complementary filter (the ICM20948 supports this
  in hardware, would give smoother/more robust orientation at the cost of
  more complex firmware), a captive-portal redirect on AP connect, data
  logging to flash for post-ride analysis/CSV export, and an mDNS-based
  "join my WiFi as well as hosting an AP" (station+AP) mode if you ever
  want the dashboard reachable from a router network too.
