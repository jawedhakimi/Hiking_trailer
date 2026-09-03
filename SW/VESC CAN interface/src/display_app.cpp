#include "display_app.h"

#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include <SPI.h>
#include <Wire.h>
#include <WiFi.h>
#include <lvgl.h>
#include <math.h>
#include <stdio.h>
#include <ui.h>

#include "config.h"
#include "control_lock.h"
#include "imu.h"
#include "potcal.h"
#include "settings.h"
#include "web_server.h"

namespace DisplayApp {
namespace {

Arduino_DataBus *displayBus = new Arduino_HWSPI(
    PIN_LCD_DC, PIN_LCD_CS, PIN_LCD_SCK, PIN_LCD_MOSI,
    PIN_IMU_MISO, &SPI, true);
Arduino_GFX *display = new Arduino_ST7789(
    displayBus, PIN_LCD_RST, LCD_ROTATION, true,
    LCD_NATIVE_WIDTH, LCD_NATIVE_HEIGHT, 0, 20, 0, 0);

constexpr uint8_t TOUCH_I2C_ADDRESS = 0x15;
constexpr uint32_t TOUCH_I2C_CLOCK_HZ = 100000;
// The CST816S only pulses its INT line around actual touch events (not
// continuously at a fixed report rate), and our main loop() interleaves a
// lot of other work (CAN, pot ADC, WiFi/AsyncWebServer, IMU) between calls
// to DisplayApp::update(). If a single interrupt-driven sample were only
// ever reported as "pressed" for the one LVGL indev poll immediately after
// it (the previous behavior), any jitter in how often that poll actually
// runs could make an ongoing touch flicker released-then-pressed-again from
// LVGL's point of view — read as "sometimes registers, sometimes doesn't."
// Holding the last known contact as pressed for a short grace window after
// the most recent interrupt (instead of collapsing to released on the very
// next poll) smooths over that jitter; a genuine finger-up is still caught
// within one grace window of the last real contact.
constexpr uint32_t TOUCH_HOLD_GRACE_MS = 80;
volatile bool touchEventPending = false;

lv_disp_draw_buf_t drawBuffer;
lv_color_t *buffer1 = nullptr;
lv_color_t *buffer2 = nullptr;
bool ready = false;
lv_point_t lastTouchPoint = {0, 0};
uint32_t lastTouchContactMs = 0;
bool haveTouchContact = false;
uint32_t lastTelemetryMs = 0;
bool updatingControls = false;

void IRAM_ATTR onTouchInterrupt() {
    touchEventPending = true;
}

bool writeTouchRegister(uint8_t reg, uint8_t value) {
    Wire1.beginTransmission(TOUCH_I2C_ADDRESS);
    Wire1.write(reg);
    Wire1.write(value);
    return Wire1.endTransmission() == 0;
}

bool readTouchRegisters(uint8_t reg, uint8_t *data, size_t length) {
    Wire1.beginTransmission(TOUCH_I2C_ADDRESS);
    Wire1.write(reg);
    if (Wire1.endTransmission(false) != 0) return false;
    if (Wire1.requestFrom(TOUCH_I2C_ADDRESS, length, true) != length) return false;
    for (size_t i = 0; i < length; ++i) data[i] = Wire1.read();
    return true;
}

uint8_t probeTouchAddress() {
    Wire1.beginTransmission(TOUCH_I2C_ADDRESS);
    return Wire1.endTransmission();
}

void scanTouchBus() {
    bool foundAny = false;
    Serial.printf("Touch: scanning Wire1 SDA=%d SCL=%d...\n", PIN_TOUCH_SDA, PIN_TOUCH_SCL);
    for (uint8_t address = 0x08; address <= 0x77; ++address) {
        Wire1.beginTransmission(address);
        if (Wire1.endTransmission() == 0) {
            Serial.printf("Touch: I2C device found at 0x%02X\n", address);
            foundAny = true;
        }
    }
    if (!foundAny) Serial.println("Touch: no I2C devices found on Wire1");
}

bool beginTouch() {
    if (!Wire1.begin(PIN_TOUCH_SDA, PIN_TOUCH_SCL, TOUCH_I2C_CLOCK_HZ)) {
        Serial.println("Touch: Wire1 initialization failed");
        return false;
    }
    Wire1.setTimeOut(50);

    // Pulled up in software as a hardening measure: the CST816S INT line is
    // open-drain/active-low on many modules, and a floating INPUT can add
    // extra edge noise on top of the timing jitter this file already works
    // around in readTouch(). If the board already has a hardware pull-up
    // this is a harmless no-op.
    pinMode(PIN_TOUCH_INT, INPUT_PULLUP);
    pinMode(PIN_TOUCH_RST, OUTPUT);
    digitalWrite(PIN_TOUCH_RST, HIGH);
    delay(100);
    digitalWrite(PIN_TOUCH_RST, LOW);
    delay(10);
    digitalWrite(PIN_TOUCH_RST, HIGH);
    delay(100);

    uint8_t error = 4;
    for (uint8_t attempt = 1; attempt <= 10; ++attempt) {
        error = probeTouchAddress();
        if (error == 0) break;
        delay(25);
    }
    if (error != 0) {
        Serial.printf("Touch: no ACK from CST816S at 0x%02X (I2C error %u)\n",
                      TOUCH_I2C_ADDRESS, error);
        scanTouchBus();
        return false;
    }

    uint8_t version = 0;
    uint8_t versionInfo[3] = {0, 0, 0};
    const bool haveVersion = readTouchRegisters(0x15, &version, 1);
    const bool haveVersionInfo = readTouchRegisters(0xA7, versionInfo, sizeof(versionInfo));
    if (haveVersion && haveVersionInfo) {
        Serial.printf("Touch: CST816S detected at 0x%02X, version=%u info=%02X-%02X-%02X\n",
                      TOUCH_I2C_ADDRESS, version,
                      versionInfo[0], versionInfo[1], versionInfo[2]);
    } else {
        Serial.printf("Touch: CST816S acknowledged at 0x%02X (version read unavailable)\n",
                      TOUCH_I2C_ADDRESS);
    }

    if (!writeTouchRegister(0xFE, 0xFE)) {
        Serial.println("Touch: warning: could not disable automatic sleep");
    }
    touchEventPending = false;
    attachInterrupt(digitalPinToInterrupt(PIN_TOUCH_INT), onTouchInterrupt, RISING);
    Serial.printf("Touch: IRQ enabled on GPIO %d (RISING)\n", PIN_TOUCH_INT);
    return true;
}

bool readTouchEvent(int &x, int &y, uint8_t &event, uint8_t &points) {
    if (!touchEventPending) return false;
    touchEventPending = false;

    Wire1.beginTransmission(TOUCH_I2C_ADDRESS);
    Wire1.write(0x01);
    // Match the vendor CST816S driver: this controller expects a STOP before
    // the following requestFrom() when reading the live touch registers.
    if (Wire1.endTransmission(true) != 0) return false;
    if (Wire1.requestFrom(TOUCH_I2C_ADDRESS, static_cast<uint8_t>(6), true) != 6) return false;

    const uint8_t gesture = Wire1.read();
    points = Wire1.read() & 0x0F;
    const uint8_t xHighAndEvent = Wire1.read();
    const uint8_t xLow = Wire1.read();
    const uint8_t yHigh = Wire1.read();
    const uint8_t yLow = Wire1.read();
    (void)gesture;
    event = xHighAndEvent >> 6;
    x = ((xHighAndEvent & 0x0F) << 8) | xLow;
    y = ((yHigh & 0x0F) << 8) | yLow;
    // The CST816S reports raw=(0,0) as a sentinel — seen repeatedly on real
    // hardware (Serial log) interspersed with otherwise-plausible touch
    // points, almost certainly on the trailing IRQ of a finger lift, before
    // it has fresh coordinate data to report. There is nothing to tap at
    // that literal corner in this UI, so any occurrence is spurious, not a
    // real touch — treat it as "no touch" rather than passing it to LVGL as
    // a valid press at screen (0, LCD_NATIVE_WIDTH - 1). Left unfiltered,
    // this can inject a bogus press at a location the user never touched,
    // which is exactly the kind of thing that could make some taps (nav
    // buttons especially) register unreliably depending on timing.
    if (x == 0 && y == 0) return false;
    return true;
}

void flushDisplay(lv_disp_drv_t *driver, const lv_area_t *area, lv_color_t *colors) {
    const uint32_t width = area->x2 - area->x1 + 1;
    const uint32_t height = area->y2 - area->y1 + 1;
#if LV_COLOR_16_SWAP != 0
    display->draw16bitBeRGBBitmap(area->x1, area->y1,
                                  reinterpret_cast<uint16_t *>(&colors->full), width, height);
#else
    display->draw16bitRGBBitmap(area->x1, area->y1,
                                reinterpret_cast<uint16_t *>(&colors->full), width, height);
#endif
    lv_disp_flush_ready(driver);
}

lv_point_t rotateTouchPoint(int rawX, int rawY) {
    lv_point_t point;
    switch (LCD_ROTATION & 3) {
        case 1:
            point.x = rawY;
            point.y = LCD_NATIVE_WIDTH - 1 - rawX;
            break;
        case 2:
            point.x = LCD_NATIVE_WIDTH - 1 - rawX;
            point.y = LCD_NATIVE_HEIGHT - 1 - rawY;
            break;
        case 3:
            point.x = LCD_NATIVE_HEIGHT - 1 - rawY;
            point.y = rawX;
            break;
        default:
            point.x = rawX;
            point.y = rawY;
            break;
    }
    point.x = constrain(point.x, 0, display->width() - 1);
    point.y = constrain(point.y, 0, display->height() - 1);
    return point;
}

void readTouch(lv_indev_drv_t *, lv_indev_data_t *data) {
    int x = 0;
    int y = 0;
    uint8_t event = 0;
    uint8_t points = 0;
    if (readTouchEvent(x, y, event, points)) {
        const bool validContact = x >= 0 && x < LCD_NATIVE_WIDTH &&
                                  y >= 0 && y < LCD_NATIVE_HEIGHT;
        if (validContact) {
            // CST816S raises INT for touch updates. As in the vendor driver,
            // the interrupt itself is the contact indication; the points byte
            // is not reliable on every firmware revision.
            lastTouchPoint = rotateTouchPoint(x, y);
            // Diagnostic: log raw vs. rotated coordinates so a mismapped
            // rotation (nav bar taps landing outside the actual button, for
            // example) can be pinned down from the numbers instead of
            // guessing. Rate-limited so a drag on the slider/switch doesn't
            // flood the log — one line per ~150ms is plenty to correlate a
            // tap against where a widget actually sits on screen (280x240
            // landscape; the bottom nav bar occupies roughly the last 50px
            // of the 240px height).
            static uint32_t lastLogMs = 0;
            uint32_t nowMs = millis();
            if (nowMs - lastLogMs >= 150) {
                lastLogMs = nowMs;
                Serial.printf("Touch: raw=(%d,%d) -> screen=(%d,%d)\n", x, y,
                              lastTouchPoint.x, lastTouchPoint.y);
            }
            lastTouchContactMs = nowMs;
            haveTouchContact = true;
            data->point = lastTouchPoint;
            data->state = LV_INDEV_STATE_PR;
            return;
        }
    }

    // No fresh sample this poll. Rather than collapsing straight to
    // "released" (which is what let single-cycle jitter in loop() timing
    // make an ongoing touch flicker press/release from LVGL's point of
    // view), keep reporting the last known contact as pressed until
    // TOUCH_HOLD_GRACE_MS has elapsed since the last real interrupt-driven
    // sample. A genuine finger-up is still caught within one grace window.
    if (haveTouchContact && (millis() - lastTouchContactMs) <= TOUCH_HOLD_GRACE_MS) {
        data->point = lastTouchPoint;
        data->state = LV_INDEV_STATE_PR;
        return;
    }

    haveTouchContact = false;
    data->point = lastTouchPoint;
    data->state = LV_INDEV_STATE_REL;
}

void setLabel(lv_obj_t *label, const char *text) {
    if (label != nullptr && strcmp(lv_label_get_text(label), text) != 0) {
        lv_label_set_text(label, text);
    }
}

void onEnableChanged(lv_event_t *event) {
    if (updatingControls || lv_event_get_code(event) != LV_EVENT_VALUE_CHANGED) return;
    lv_obj_t *sw = lv_event_get_target(event);
    const bool requested = lv_obj_has_state(sw, LV_STATE_CHECKED);
    if (!WebServerApp::setShaftPowerEnabled(requested)) {
        updatingControls = true;
        lv_obj_clear_state(sw, LV_STATE_CHECKED);
        updatingControls = false;
    }
}

void refreshTelemetry(const Telemetry &t) {
    char text[384];
    static bool haveBatteryValue = false;
    static float lastBatteryVoltage = 0.0f;
    static bool haveDistanceValue = false;
    static float lastDistanceM = 0.0f;

    if (t.batteryFresh && isfinite(t.batteryVoltage)) {
        lastBatteryVoltage = t.batteryVoltage;
        haveBatteryValue = true;
    }
    if (t.odometryAvailable && isfinite(t.tripDistanceM)) {
        lastDistanceM = t.tripDistanceM;
        haveDistanceValue = true;
    }

    const float shownSpeed = fabsf(t.speedKmh);
    const int arcSpeed = constrain(static_cast<int>(lroundf(shownSpeed)), 0, 20);
    if (ui_ArcSpeed != nullptr) lv_arc_set_value(ui_ArcSpeed, arcSpeed);
    if (t.vescStatusFresh) {
        snprintf(text, sizeof(text), "%.1f km/h", shownSpeed);
    } else {
        snprintf(text, sizeof(text), "-- km/h");
    }
    setLabel(ui_Speed, text);

    if (haveBatteryValue) {
        snprintf(text, sizeof(text), "%.1fV", lastBatteryVoltage);
    } else {
        snprintf(text, sizeof(text), "--V");
    }
    setLabel(ui_Batterlevel, text);

    if (!haveDistanceValue) {
        snprintf(text, sizeof(text), "--");
    } else if (lastDistanceM < 1000.0f) {
        snprintf(text, sizeof(text), "%.0f m", lastDistanceM);
    } else {
        snprintf(text, sizeof(text), "%.1f km", lastDistanceM / 1000.0f);
    }
    setLabel(ui_Distance, text);

    if (t.imuPresent) {
        snprintf(text, sizeof(text), "%.0f Deg", t.headingDeg);
    } else {
        snprintf(text, sizeof(text), "-- Deg");
    }
    setLabel(ui_CompasAngle, text);

    if (ui_SwitchEnable != nullptr) {
        updatingControls = true;
        if (t.shaftPowerEnabled) lv_obj_add_state(ui_SwitchEnable, LV_STATE_CHECKED);
        else lv_obj_clear_state(ui_SwitchEnable, LV_STATE_CHECKED);
        updatingControls = false;
    }

    // Pot-position indicator: SquareLine wires ui_event_SliderPotPosition to
    // update ui_PotPosition's text on LV_EVENT_VALUE_CHANGED, but that event
    // only fires for user interaction, not for a programmatic
    // lv_slider_set_value() like this one — so the label is refreshed
    // explicitly here via the same helper the generated callback uses, to
    // stay in sync with the slider's live value.
    if (ui_SliderPotPosition != nullptr) {
        const int potPct = constrain(static_cast<int>(lroundf(t.potPositionPercent)), -100, 100);
        lv_slider_set_value(ui_SliderPotPosition, potPct, LV_ANIM_OFF);
        _ui_slider_set_text_value(ui_PotPosition, ui_SliderPotPosition, "Pot-Position: ", "");
    }

    // Network info block (SSID/Pass/mDNS host) now lives directly on Home
    // instead of the removed Info screen. Values are static for the life of
    // the AP, but softAPIP()/mdnsHostname() aren't necessarily valid yet the
    // moment DisplayApp::begin() runs (WebServerApp::begin() starts after
    // it) — recomputing each refresh is cheap and self-corrects once WiFi
    // comes up, and setLabel() already no-ops once the text stops changing.
    if (ui_SSIDInfo != nullptr) {
        snprintf(text, sizeof(text), "SSID: %s", g_settings.wifiSsid);
        setLabel(ui_SSIDInfo, text);
    }
    if (ui_PassInfo != nullptr) {
        snprintf(text, sizeof(text), "Pass: %s", g_settings.wifiPassword);
        setLabel(ui_PassInfo, text);
    }
    if (ui_NetworkInfo != nullptr) {
        const String ip = WiFi.softAPIP().toString();
        const char *hostname = WebServerApp::mdnsHostname();
        if (hostname == nullptr || hostname[0] == '\0') hostname = MDNS_HOSTNAME_FALLBACK;
        snprintf(text, sizeof(text), "Visit: http://%s/ or http://%s.local/", ip.c_str(), hostname);
        setLabel(ui_NetworkInfo, text);
    }
}

} // namespace

bool begin() {
    if (ready) return true;

    if (!display->begin()) {
        Serial.println("LCD: ST7789 initialization failed");
        return false;
    }
    pinMode(PIN_LCD_BACKLIGHT, OUTPUT);
    digitalWrite(PIN_LCD_BACKLIGHT, HIGH);
    display->fillScreen(RGB565_BLACK);

    lv_init();
    const size_t pixels = static_cast<size_t>(display->width()) * LCD_LVGL_BUFFER_LINES;
    buffer1 = static_cast<lv_color_t *>(heap_caps_malloc(pixels * sizeof(lv_color_t), MALLOC_CAP_DMA));
    buffer2 = static_cast<lv_color_t *>(heap_caps_malloc(pixels * sizeof(lv_color_t), MALLOC_CAP_DMA));
    if (buffer1 == nullptr || buffer2 == nullptr) {
        if (buffer1 != nullptr) heap_caps_free(buffer1);
        if (buffer2 != nullptr) heap_caps_free(buffer2);
        buffer1 = buffer2 = nullptr;
        Serial.println("LCD: LVGL draw-buffer allocation failed");
        return false;
    }

    lv_disp_draw_buf_init(&drawBuffer, buffer1, buffer2, pixels);
    static lv_disp_drv_t displayDriver;
    lv_disp_drv_init(&displayDriver);
    displayDriver.hor_res = display->width();
    displayDriver.ver_res = display->height();
    displayDriver.flush_cb = flushDisplay;
    displayDriver.draw_buf = &drawBuffer;
    lv_disp_drv_register(&displayDriver);

    if (!beginTouch()) {
        Serial.println("LCD: CST816S touch initialization failed");
    }
    static lv_indev_drv_t inputDriver;
    lv_indev_drv_init(&inputDriver);
    inputDriver.type = LV_INDEV_TYPE_POINTER;
    inputDriver.read_cb = readTouch;
    lv_indev_drv_register(&inputDriver);

    ui_init();
    if (ui_SwitchEnable != nullptr) {
        lv_obj_add_event_cb(ui_SwitchEnable, onEnableChanged, LV_EVENT_VALUE_CHANGED, nullptr);
    }

    ready = true;
    Serial.printf("LCD: ready (%dx%d landscape)\n", display->width(), display->height());
    return true;
}

void update(const Telemetry &telemetry) {
    if (!ready) return;

    const uint32_t now = millis();
    if (now - lastTelemetryMs >= LCD_TELEMETRY_INTERVAL_MS) {
        lastTelemetryMs = now;
        refreshTelemetry(telemetry);
    }
    lv_timer_handler();
}

bool isReady() { return ready; }

} // namespace DisplayApp
