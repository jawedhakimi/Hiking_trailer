#include "web_server.h"

#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <LittleFS.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncJson.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include "driver/twai.h"

#include "config.h"
#include "settings.h"
#include "potcal.h"
#include "imu.h"
#include "vesc_can.h"
#include "odometry.h"
#include "control_lock.h"

namespace WebServerApp {

namespace {

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

TelemetrySnapshot g_telemetry;
uint32_t g_lastTelemetryBroadcastMs = 0;
uint32_t g_lastCanSeqSent = 0;
bool g_parkEnabled = false;
bool g_shaftPowerEnabled = false; // deliberately not persisted
String g_mdnsHostname;

bool g_restartRequested = false;
uint32_t g_restartAtMs = 0;

void scheduleRestart(uint32_t delayMs = 400) {
    g_restartRequested = true;
    g_restartAtMs = millis() + delayMs;
}

String mdnsHostnameFromSsid(const char *ssid) {
    String hostname;
    hostname.reserve(63);
    bool previousWasHyphen = false;

    for (size_t i = 0; ssid != nullptr && ssid[i] != '\0' && hostname.length() < 63; ++i) {
        char c = ssid[i];
        bool isUpper = c >= 'A' && c <= 'Z';
        bool isLower = c >= 'a' && c <= 'z';
        bool isDigit = c >= '0' && c <= '9';

        if (isUpper || isLower || isDigit) {
            hostname += isUpper ? static_cast<char>(c + ('a' - 'A')) : c;
            previousWasHyphen = false;
        } else if (!hostname.isEmpty() && !previousWasHyphen) {
            hostname += '-';
            previousWasHyphen = true;
        }
    }

    while (hostname.endsWith("-")) {
        hostname.remove(hostname.length() - 1);
    }
    if (hostname.isEmpty()) {
        hostname = MDNS_HOSTNAME_FALLBACK;
    }
    return hostname;
}

const char *canStateToStr(twai_state_t state) {
    switch (state) {
        case TWAI_STATE_STOPPED: return "STOPPED";
        case TWAI_STATE_RUNNING: return "RUNNING";
        case TWAI_STATE_BUS_OFF: return "BUS_OFF";
        case TWAI_STATE_RECOVERING: return "RECOVERING";
        default: return "?";
    }
}

void sendJson(AsyncWebServerRequest *request, JsonDocument &doc, int code = 200) {
    AsyncResponseStream *response = request->beginResponseStream("application/json");
    response->setCode(code);
    serializeJson(doc, *response);
    request->send(response);
}

// ------------------------------------------------------------------------
// WebSocket
// ------------------------------------------------------------------------

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
               void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_CONNECT) {
        Serial.printf("WS client #%u connected from %s\n", client->id(),
                       client->remoteIP().toString().c_str());
    } else if (type == WS_EVT_DISCONNECT) {
        Serial.printf("WS client #%u disconnected\n", client->id());
    }
    // No inbound commands over WS in this version — all actions go through
    // the REST API below, WS is telemetry-out only.
}

void broadcastTelemetry() {
    ControlLock::Guard guard;
    // --- Temporary diagnostics (throttled to ~1/2s) ------------------------
    // Prints whether we think a client is attached and, if a broadcast is
    // attempted, how many bytes were serialized and what textAll() reported.
    // Safe to remove once live telemetry is confirmed working.
    static uint32_t lastLogMs = 0;
    bool shouldLog = (millis() - lastLogMs >= 2000);

    size_t clientCount = ws.count();
    if (clientCount == 0) {
        if (shouldLog) {
            lastLogMs = millis();
            Serial.println("[ws] broadcastTelemetry: ws.count()==0, nothing to send");
        }
        return;
    }

    JsonDocument doc;
    doc["type"] = "telemetry";
    doc["potPct"] = g_telemetry.potPositionPercent;
    doc["potRaw"] = g_telemetry.potRaw;
    doc["potRawInstant"] = g_telemetry.potRawInstant;
    doc["controlMode"] = g_telemetry.controlMode;
    doc["targetErpm"] = g_telemetry.targetErpm;
    doc["targetCurrentA"] = g_telemetry.targetCurrentA;
    doc["actualErpm"] = g_telemetry.actualErpm;
    doc["vescFresh"] = g_telemetry.vescStatusFresh;
    doc["speedMps"] = g_telemetry.speedMps;
    doc["speedKmh"] = g_telemetry.speedMps * 3.6f;
    doc["tripM"] = Odometry::tripDistanceM();
    doc["odometryAvailable"] = g_telemetry.odometryAvailable;
    doc["odometryUsingTachometer"] = g_telemetry.odometryUsingTachometer;
    doc["vBatt"] = g_telemetry.vBatt;
    doc["vBattEsp32"] = g_telemetry.vBattEsp32;
    doc["vBattVesc"] = g_telemetry.vBattVesc;
    doc["batteryVoltageSource"] = g_telemetry.batteryVoltageSource;
    doc["batteryVoltageFresh"] = g_telemetry.batteryVoltageFresh;
    doc["motorCurrentA"] = g_telemetry.motorCurrentA;
    doc["duty"] = g_telemetry.dutyCycle;
    doc["fetTempC"] = g_telemetry.fetTempC;
    doc["motorTempC"] = g_telemetry.motorTempC;
    doc["inputCurrentA"] = g_telemetry.inputCurrentA;
    doc["ahConsumed"] = g_telemetry.ampHoursConsumed;
    doc["ahCharged"] = g_telemetry.ampHoursCharged;
    doc["whConsumed"] = g_telemetry.wattHoursConsumed;
    doc["whCharged"] = g_telemetry.wattHoursCharged;
    doc["pidPositionDeg"] = g_telemetry.pidPositionDeg;
    doc["tachometerRaw"] = g_telemetry.tachometerRaw;
    doc["adc1V"] = g_telemetry.adc1V;
    doc["adc2V"] = g_telemetry.adc2V;
    doc["adc3V"] = g_telemetry.adc3V;
    doc["ppm"] = g_telemetry.ppm;
    doc["status1Fresh"] = g_telemetry.vescStatusFresh;
    doc["status2Fresh"] = g_telemetry.status2Fresh;
    doc["status3Fresh"] = g_telemetry.status3Fresh;
    doc["status4Fresh"] = g_telemetry.status4Fresh;
    doc["status5Fresh"] = g_telemetry.status5Fresh;
    doc["status6Fresh"] = g_telemetry.status6Fresh;
    doc["status1Ever"] = g_telemetry.status1Ever;
    doc["status2Ever"] = g_telemetry.status2Ever;
    doc["status3Ever"] = g_telemetry.status3Ever;
    doc["status4Ever"] = g_telemetry.status4Ever;
    doc["status5Ever"] = g_telemetry.status5Ever;
    doc["status6Ever"] = g_telemetry.status6Ever;
    doc["status1AgeMs"] = g_telemetry.status1AgeMs;
    doc["status2AgeMs"] = g_telemetry.status2AgeMs;
    doc["status3AgeMs"] = g_telemetry.status3AgeMs;
    doc["status4AgeMs"] = g_telemetry.status4AgeMs;
    doc["status5AgeMs"] = g_telemetry.status5AgeMs;
    doc["status6AgeMs"] = g_telemetry.status6AgeMs;

    doc["pidEnabled"] = g_settings.speedPidEnabled;
    doc["pidActive"] = g_telemetry.pidActive;
    doc["pidTrimErpm"] = g_telemetry.pidTrimErpm;

    doc["brakingActive"] = g_telemetry.brakingActive;
    doc["parkEnabled"] = g_telemetry.parkEnabled;
    doc["parkHolding"] = g_telemetry.parkHolding;
    doc["shaftPowerEnabled"] = g_telemetry.shaftPowerEnabled;

    doc["postFallLockout"] = g_telemetry.postFallLockout;
    doc["resumeWarnActive"] = g_telemetry.resumeWarnActive;
    doc["calibrationReady"] = g_telemetry.calibrationReady;
    doc["calibrationActive"] = g_telemetry.calibrationActive;
    doc["calibrationWaitingForCenter"] = g_telemetry.calibrationWaitingForCenter;
    doc["directionChangeLockout"] = g_telemetry.directionChangeLockout;

    doc["imuPresent"] = Imu::isPresent();
    doc["pitch"] = Imu::pitchDeg();
    doc["roll"] = Imu::rollDeg();
    doc["tilt"] = Imu::tiltDeg();
    doc["heading"] = Imu::headingDeg();
    doc["accelG"] = Imu::accelMagnitudeG();
    doc["fallen"] = Imu::isFallen();
    doc["fallEventCount"] = Imu::fallEventCount();

    if (Imu::magCalibrationActive()) {
        float minX, maxX, minY, maxY, minZ, maxZ;
        Imu::getMagCalPreview(minX, maxX, minY, maxY, minZ, maxZ);
        doc["magCalActive"] = true;
        JsonObject mc = doc["magCal"].to<JsonObject>();
        mc["minX"] = minX; mc["maxX"] = maxX;
        mc["minY"] = minY; mc["maxY"] = maxY;
        mc["minZ"] = minZ; mc["maxZ"] = maxZ;
    } else {
        doc["magCalActive"] = false;
    }

    doc["canState"] = g_telemetry.canState;
    doc["canTxErr"] = g_telemetry.canTxErrorCount;
    doc["canRxErr"] = g_telemetry.canRxErrorCount;

    doc["uptimeS"] = millis() / 1000;
    doc["freeHeap"] = ESP.getFreeHeap();

    String out;
    size_t written = serializeJson(doc, out);
    AsyncWebSocket::SendStatus sendStatus = ws.textAll(out);

    if (shouldLog) {
        lastLogMs = millis();
        const char *statusStr = sendStatus == AsyncWebSocket::SendStatus::ENQUEUED             ? "ENQUEUED"
                                 : sendStatus == AsyncWebSocket::SendStatus::PARTIALLY_ENQUEUED ? "PARTIALLY_ENQUEUED"
                                                                                                 : "DISCARDED";
        Serial.printf("[ws] broadcastTelemetry: clients=%u jsonBytes=%u outLen=%u status=%s heap=%u\n",
                      (unsigned)clientCount, (unsigned)written, (unsigned)out.length(), statusStr,
                      (unsigned)ESP.getFreeHeap());
    }
}

void broadcastCanLog() {
    ControlLock::Guard guard;
    if (ws.count() == 0) return;

    VescCan::CanLogEntry entries[16];
    size_t n = VescCan::getLogEntriesAfter(g_lastCanSeqSent, entries, 16);
    if (n == 0) return;
    g_lastCanSeqSent = entries[n - 1].seq;

    JsonDocument doc;
    doc["type"] = "can";
    JsonArray arr = doc["frames"].to<JsonArray>();
    for (size_t i = 0; i < n; i++) {
        JsonObject f = arr.add<JsonObject>();
        f["seq"] = entries[i].seq;
        f["t"] = entries[i].millisAt;
        f["id"] = entries[i].identifier;
        f["dlc"] = entries[i].dlc;
        f["tx"] = entries[i].tx;
        f["ok"] = entries[i].ok;
        char hex[24] = {0};
        int pos = 0;
        for (uint8_t b = 0; b < entries[i].dlc && b < 8; b++) {
            pos += snprintf(hex + pos, sizeof(hex) - pos, "%02X ", entries[i].data[b]);
        }
        f["data"] = hex;
    }

    String out;
    serializeJson(doc, out);
    ws.textAll(out);
}

// ------------------------------------------------------------------------
// REST API
// ------------------------------------------------------------------------

void setupApi() {
    server.on("/api/settings", HTTP_GET, [](AsyncWebServerRequest *request) {
        ControlLock::Guard guard;
        JsonDocument doc;
        g_settings.toJson(doc.to<JsonObject>());
        sendJson(request, doc);
    });

    auto *settingsHandler = new AsyncCallbackJsonWebHandler(
        "/api/settings", [](AsyncWebServerRequest *request, JsonVariant &json) {
            ControlLock::Guard guard;
            JsonObject obj = json.as<JsonObject>();
            bool passwordRejected = false;
            bool restartNeeded = g_settings.fromJson(obj, &passwordRejected);
            g_settings.save();
            JsonDocument doc;
            doc["ok"] = true;
            doc["restartRequired"] = restartNeeded;
            doc["passwordRejected"] = passwordRejected;
            sendJson(request, doc);
        });
    server.addHandler(settingsHandler);

    // --- Potentiometer calibration -----------------------------------------
    server.on("/api/pot/calibration", HTTP_GET, [](AsyncWebServerRequest *request) {
        ControlLock::Guard guard;
        JsonDocument doc;
        doc["min"] = PotCal::minRaw();
        doc["max"] = PotCal::maxRaw();
        doc["center"] = PotCal::centerRaw();
        doc["maxOffset"] = PotCal::maxOffsetRaw();
        doc["valid"] = PotCal::isValid();
        doc["hasMin"] = PotCal::hasCapturedMin();
        doc["hasMax"] = PotCal::hasCapturedMax();
        doc["sessionActive"] = PotCal::isWebCalibrationActive();
        doc["waitingForCenter"] = PotCal::isWaitingForCenter();
        sendJson(request, doc);
    });

    server.on("/api/pot/start", HTTP_POST, [](AsyncWebServerRequest *request) {
        ControlLock::Guard guard;
        PotCal::startWebCalibration();
        // Stop immediately, then main.cpp suppresses all further motor
        // command frames until calibration is complete and the pot centers.
        bool stopSent = g_settings.controlMode == VESC_CONTROL_MODE_CURRENT
            ? VescCan::sendSetCurrent(g_settings.vescControllerId, 0.0f)
            : VescCan::sendSetRpm(g_settings.vescControllerId, 0);
        JsonDocument doc;
        doc["ok"] = true;
        doc["stopSent"] = stopSent;
        sendJson(request, doc);
    });

    server.on("/api/pot/cancel", HTTP_POST, [](AsyncWebServerRequest *request) {
        ControlLock::Guard guard;
        PotCal::cancelWebCalibration();
        JsonDocument doc;
        doc["ok"] = true;
        doc["waitingForCenter"] = PotCal::isWaitingForCenter();
        sendJson(request, doc);
    });

    server.on("/api/pot/capture", HTTP_POST, [](AsyncWebServerRequest *request) {
        ControlLock::Guard guard;
        if (!PotCal::isWebCalibrationActive() || PotCal::isWaitingForCenter()) {
            request->send(409, "application/json", "{\"ok\":false,\"error\":\"start a calibration session first\"}");
            return;
        }
        if (!request->hasParam("which")) {
            request->send(400, "application/json", "{\"ok\":false,\"error\":\"missing 'which' param (min|max)\"}");
            return;
        }
        bool isMax = request->getParam("which")->value() == "max";
        int32_t raw = PotCal::readAveraged(300); // ~300ms averaged sample
        bool ok = PotCal::captureEndpoint(raw, isMax);
        JsonDocument doc;
        doc["ok"] = ok;
        doc["raw"] = raw;
        doc["min"] = PotCal::minRaw();
        doc["max"] = PotCal::maxRaw();
        doc["center"] = PotCal::centerRaw();
        doc["valid"] = PotCal::isValid();
        doc["hasMin"] = PotCal::hasCapturedMin();
        doc["hasMax"] = PotCal::hasCapturedMax();
        doc["sessionActive"] = PotCal::isWebCalibrationActive();
        doc["waitingForCenter"] = PotCal::isWaitingForCenter();
        // "Span too small" is an expected, actionable outcome the user
        // just needs to retry (move the pot further before capturing) —
        // not a server error. Always answer 200 and let the frontend
        // branch on the `ok` field; app.js's generic api() helper throws
        // on ANY non-2xx status (see its HTTP-level catch), which used to
        // turn this into a raw "Failed: HTTP422" instead of the intended
        // "Span too small..." guidance from potCaptureMsg().
        sendJson(request, doc, 200);
    });

    auto *potSetHandler = new AsyncCallbackJsonWebHandler(
        "/api/pot/set", [](AsyncWebServerRequest *request, JsonVariant &json) {
            ControlLock::Guard guard;
            JsonObject obj = json.as<JsonObject>();
            bool ok = false;
            if (obj["min"].is<int32_t>() && obj["max"].is<int32_t>()) {
                ok = PotCal::setEndpoints(obj["min"], obj["max"]);
            }
            JsonDocument doc;
            doc["ok"] = ok;
            doc["min"] = PotCal::minRaw();
            doc["max"] = PotCal::maxRaw();
            // Same reasoning as /api/pot/capture above: an invalid span is
            // a normal validation outcome, not an HTTP-level error.
            sendJson(request, doc, 200);
        });
    server.addHandler(potSetHandler);

    // --- IMU / fall / compass calibration -----------------------------------
    server.on("/api/imu/zero", HTTP_POST, [](AsyncWebServerRequest *request) {
        ControlLock::Guard guard;
        Imu::calibrateUprightZero();
        JsonDocument doc;
        doc["ok"] = true;
        sendJson(request, doc);
    });

    server.on("/api/imu/mag/start", HTTP_POST, [](AsyncWebServerRequest *request) {
        ControlLock::Guard guard;
        Imu::startMagCalibration();
        JsonDocument doc;
        doc["ok"] = true;
        sendJson(request, doc);
    });

    server.on("/api/imu/mag/stop", HTTP_POST, [](AsyncWebServerRequest *request) {
        ControlLock::Guard guard;
        bool save = true;
        if (request->hasParam("save")) save = request->getParam("save")->value() != "false";
        Imu::stopMagCalibration(save);
        JsonDocument doc;
        doc["ok"] = true;
        sendJson(request, doc);
    });

    server.on("/api/fall/events", HTTP_GET, [](AsyncWebServerRequest *request) {
        ControlLock::Guard guard;
        Imu::FallEvent events[FALL_EVENT_LOG_SIZE];
        size_t n = Imu::getFallEventLog(events, FALL_EVENT_LOG_SIZE);
        JsonDocument doc;
        JsonArray arr = doc["events"].to<JsonArray>();
        for (size_t i = 0; i < n; i++) {
            JsonObject e = arr.add<JsonObject>();
            e["t"] = events[i].millisAt;
            e["angle"] = events[i].angleDeg;
            e["trigger"] = events[i].trigger == Imu::FallTrigger::IMPACT ? "impact" : "tilt";
        }
        doc["totalCount"] = Imu::fallEventCount();
        sendJson(request, doc);
    });

    server.on("/api/fall/clear", HTTP_POST, [](AsyncWebServerRequest *request) {
        ControlLock::Guard guard;
        Imu::clearFallLatch();
        JsonDocument doc;
        doc["ok"] = true;
        sendJson(request, doc);
    });

    // --- CAN log (initial paint; live updates come over the WebSocket) -----
    server.on("/api/can/log", HTTP_GET, [](AsyncWebServerRequest *request) {
        ControlLock::Guard guard;
        VescCan::CanLogEntry entries[CAN_LOG_RING_SIZE];
        size_t n = VescCan::getLogEntriesAfter(0, entries, CAN_LOG_RING_SIZE);
        JsonDocument doc;
        JsonArray arr = doc["frames"].to<JsonArray>();
        for (size_t i = 0; i < n; i++) {
            JsonObject f = arr.add<JsonObject>();
            f["seq"] = entries[i].seq;
            f["t"] = entries[i].millisAt;
            f["id"] = entries[i].identifier;
            f["dlc"] = entries[i].dlc;
            f["tx"] = entries[i].tx;
            f["ok"] = entries[i].ok;
            char hex[24] = {0};
            int pos = 0;
            for (uint8_t b = 0; b < entries[i].dlc && b < 8; b++) {
                pos += snprintf(hex + pos, sizeof(hex) - pos, "%02X ", entries[i].data[b]);
            }
            f["data"] = hex;
        }
        sendJson(request, doc);
    });

    // --- Trip / system -------------------------------------------------------
    server.on("/api/trip/reset", HTTP_POST, [](AsyncWebServerRequest *request) {
        ControlLock::Guard guard;
        Odometry::resetTrip();
        JsonDocument doc;
        doc["ok"] = true;
        sendJson(request, doc);
    });

    server.on("/api/park", HTTP_POST, [](AsyncWebServerRequest *request) {
        ControlLock::Guard guard;
        if (!request->hasParam("enabled")) {
            JsonDocument doc;
            doc["error"] = "Missing enabled=true|false";
            sendJson(request, doc, 400);
            return;
        }
        String value = request->getParam("enabled")->value();
        if (value != "true" && value != "false" && value != "1" && value != "0") {
            JsonDocument doc;
            doc["error"] = "enabled must be true or false";
            sendJson(request, doc, 400);
            return;
        }
        bool requested = value == "true" || value == "1";
        if (requested && !g_shaftPowerEnabled) {
            JsonDocument doc;
            doc["error"] = "Enable motor power before engaging Park";
            sendJson(request, doc, 409);
            return;
        }
        g_parkEnabled = requested;
        JsonDocument doc;
        doc["ok"] = true;
        doc["parkEnabled"] = g_parkEnabled;
        sendJson(request, doc);
    });

    server.on("/api/shaft-power", HTTP_POST, [](AsyncWebServerRequest *request) {
        ControlLock::Guard guard;
        if (!request->hasParam("enabled")) {
            JsonDocument doc;
            doc["error"] = "Missing enabled=true|false";
            sendJson(request, doc, 400);
            return;
        }
        String value = request->getParam("enabled")->value();
        if (value != "true" && value != "false" && value != "1" && value != "0") {
            JsonDocument doc;
            doc["error"] = "enabled must be true or false";
            sendJson(request, doc, 400);
            return;
        }

        bool requested = value == "true" || value == "1";
        if (requested && (!PotCal::isValid() || PotCal::isWebCalibrationActive())) {
            JsonDocument doc;
            doc["error"] = "Complete potentiometer calibration before enabling motor power";
            sendJson(request, doc, 409);
            return;
        }
        if (!requested) {
            // Release torque immediately, then main.cpp stops periodic motor
            // commands altogether. Also clear Park so a later Enable cannot
            // unexpectedly reapply the handbrake.
            if (PotCal::isValid() && !PotCal::isWebCalibrationActive()) {
                VescCan::sendSetCurrent(g_settings.vescControllerId, 0.0f);
            }
            g_parkEnabled = false;
        }
        g_shaftPowerEnabled = requested;

        JsonDocument doc;
        doc["ok"] = true;
        doc["shaftPowerEnabled"] = g_shaftPowerEnabled;
        doc["parkEnabled"] = g_parkEnabled;
        sendJson(request, doc);
    });

    server.on("/api/system/info", HTTP_GET, [](AsyncWebServerRequest *request) {
        JsonDocument doc;
        doc["uptimeS"] = millis() / 1000;
        doc["freeHeap"] = ESP.getFreeHeap();
        doc["buildDate"] = __DATE__ " " __TIME__;
        doc["mac"] = WiFi.softAPmacAddress();
        doc["ip"] = WiFi.softAPIP().toString();
        doc["mdnsHost"] = g_mdnsHostname + ".local";
        sendJson(request, doc);
    });

    server.on("/api/system/restart", HTTP_POST, [](AsyncWebServerRequest *request) {
        JsonDocument doc;
        doc["ok"] = true;
        sendJson(request, doc);
        scheduleRestart();
    });

    server.on("/api/system/factory-reset", HTTP_POST, [](AsyncWebServerRequest *request) {
        ControlLock::Guard guard;
        Preferences p;
        p.begin(SETTINGS_NVS_NAMESPACE, false); p.clear(); p.end();
        p.begin(CALIB_NVS_NAMESPACE, false); p.clear(); p.end();
        p.begin("imucal", false); p.clear(); p.end();
        JsonDocument doc;
        doc["ok"] = true;
        sendJson(request, doc);
        scheduleRestart();
    });

    server.onNotFound([](AsyncWebServerRequest *request) {
        if (request->method() == HTTP_OPTIONS) {
            request->send(200);
            return;
        }
        request->send(404, "application/json", "{\"ok\":false,\"error\":\"not found\"}");
    });
}

} // namespace

bool begin() {
    if (!LittleFS.begin(true)) {
        Serial.println("LittleFS mount failed — web UI files won't be served. "
                        "Did you run 'pio run --target uploadfs'?");
        // Not fatal: the REST API and WebSocket still work without static
        // files, useful for debugging with curl even if the dashboard
        // itself can't load.
    }

    WiFi.mode(WIFI_AP);
    WiFi.softAP(g_settings.wifiSsid, g_settings.wifiPassword, WIFI_AP_CHANNEL, 0, WIFI_AP_MAX_CLIENTS);
    Serial.printf("WiFi AP '%s' up, IP=%s\n", g_settings.wifiSsid, WiFi.softAPIP().toString().c_str());

    g_mdnsHostname = mdnsHostnameFromSsid(g_settings.wifiSsid);
    if (MDNS.begin(g_mdnsHostname.c_str())) {
        MDNS.addService("http", "tcp", 80);
        Serial.printf("mDNS: http://%s.local/\n", g_mdnsHostname.c_str());
    } else {
        Serial.printf("mDNS failed for hostname '%s'\n", g_mdnsHostname.c_str());
    }

    ws.onEvent(onWsEvent);
    server.addHandler(&ws);

    setupApi();

    server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html").setCacheControl("max-age=600");

    server.begin();
    return true;
}

void loop() {
    ws.cleanupClients();

    uint32_t now = millis();
    if (now - g_lastTelemetryBroadcastMs >= WEB_TELEMETRY_INTERVAL_MS) {
        g_lastTelemetryBroadcastMs = now;
        broadcastTelemetry();
        broadcastCanLog();
    }

    if (g_restartRequested && (int32_t)(now - g_restartAtMs) >= 0) {
        Serial.println("Restarting (requested via web UI)...");
        delay(50);
        ESP.restart();
    }
}

void setTelemetry(const TelemetrySnapshot &snap) {
    g_telemetry = snap;
    twai_status_info_t status;
    if (twai_get_status_info(&status) == ESP_OK) {
        g_telemetry.canState = canStateToStr(status.state);
        g_telemetry.canTxErrorCount = status.tx_error_counter;
        g_telemetry.canRxErrorCount = status.rx_error_counter;
    }
}

bool isParkEnabled() { return g_parkEnabled; }
bool isShaftPowerEnabled() { return g_shaftPowerEnabled; }

} // namespace WebServerApp
