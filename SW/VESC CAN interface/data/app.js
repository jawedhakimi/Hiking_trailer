// VESC Controller web dashboard — vanilla JS, no build step (served
// straight off LittleFS). Talks to the REST API for settings/actions and
// the /ws WebSocket for live telemetry + CAN frames.

// ---------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------
const $ = (id) => document.getElementById(id);

function toast(msg, isErr) {
    const t = $('toast');
    t.textContent = msg;
    t.className = 'toast show' + (isErr ? ' err' : '');
    clearTimeout(toast._to);
    toast._to = setTimeout(() => { t.className = 'toast'; }, 2600);
}

async function api(path, opts) {
    const res = await fetch(path, opts);
    let body = null;
    try { body = await res.json(); } catch (e) { /* no body */ }
    if (!res.ok) {
        throw new Error((body && body.error) ? body.error : ('HTTP ' + res.status));
    }
    return body;
}
async function apiGet(path) { return api(path); }
async function apiPost(path, jsonBody) {
    if (jsonBody !== undefined) {
        return api(path, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(jsonBody) });
    }
    return api(path, { method: 'POST' });
}

function fmt(v, d) { return (typeof v === 'number') ? v.toFixed(d === undefined ? 1 : d) : '--'; }

function vescStatusText(t, number, decodedValue) {
    if (t[`status${number}Fresh`]) return decodedValue;
    if (t[`status${number}Ever`]) {
        const ageSeconds = Math.max(0, Number(t[`status${number}AgeMs`]) || 0) / 1000;
        return `${decodedValue} · stale (${ageSeconds.toFixed(1)} s ago)`;
    }
    return `Not received — enable STATUS ${number} broadcast in VESC Tool`;
}

// ---------------------------------------------------------------------
// Tabs
// ---------------------------------------------------------------------
document.querySelectorAll('#tabNav button').forEach((btn) => {
    btn.addEventListener('click', () => {
        document.querySelectorAll('#tabNav button').forEach((b) => b.classList.remove('active'));
        document.querySelectorAll('.tab').forEach((t) => t.classList.remove('active'));
        btn.classList.add('active');
        $('tab-' + btn.dataset.tab).classList.add('active');
        if (btn.dataset.tab === 'system') loadSystemInfo();
        if (btn.dataset.tab === 'imu') loadFallEvents();
        if (btn.dataset.tab === 'can' && canRows.length === 0) loadCanHistory();
    });
});

// ---------------------------------------------------------------------
// WebSocket
// ---------------------------------------------------------------------
let ws;
let wsReconnectMs = 1000;
let lastTelemetryAt = 0; // 0 = never received one
let lastEsp32BatteryVoltage = NaN;
let parkRequestPending = false;
let shaftPowerRequestPending = false;

function wsConnect() {
    const proto = location.protocol === 'https:' ? 'wss://' : 'ws://';
    ws = new WebSocket(proto + location.host + '/ws');
    ws.onopen = () => {
        $('connDot').className = 'ok';
        $('connText').textContent = 'connected (waiting for data…)';
        wsReconnectMs = 1000;
    };
    ws.onclose = () => {
        $('connDot').className = '';
        $('connText').textContent = 'reconnecting…';
        $('parkSwitch').disabled = true;
        $('shaftPowerSwitch').disabled = true;
        setTimeout(wsConnect, wsReconnectMs);
        wsReconnectMs = Math.min(wsReconnectMs * 1.5, 10000);
    };
    ws.onerror = () => { ws.close(); };
    ws.onmessage = (ev) => {
        let msg;
        try { msg = JSON.parse(ev.data); } catch (e) {
            console.error('WS message was not valid JSON:', ev.data);
            return;
        }
        if (msg.type === 'telemetry') {
            lastTelemetryAt = Date.now();
            $('connText').textContent = 'live';
            onTelemetry(msg);
        } else if (msg.type === 'can') onCanFrames(msg.frames);
    };
}
wsConnect();

// Watchdog: catches the case where the WebSocket transport connects fine
// (onopen fires) but no telemetry ever actually arrives — a different
// failure mode than "never connects at all", and one that would otherwise
// leave the dashboard silently frozen with no indication anything's wrong.
setInterval(() => {
    const connected = ws && ws.readyState === WebSocket.OPEN;
    const staleMs = lastTelemetryAt ? Date.now() - lastTelemetryAt : Infinity;
    const banner = $('fallBanner');
    if (!connected) {
        $('shaftPowerSwitch').disabled = true;
        $('parkSwitch').disabled = true;
        $('connText').textContent = 'reconnecting…';
        banner.className = 'banner warn';
        banner.textContent = 'Not connected to the controller — reconnecting…';
    } else if (staleMs > 3000) {
        $('shaftPowerSwitch').disabled = true;
        $('parkSwitch').disabled = true;
        $('connText').textContent = 'connected — NO telemetry arriving';
        banner.className = 'banner warn';
        banner.textContent = 'WebSocket connected, but no telemetry has arrived — see console (F12) / Serial Monitor.';
    }
    // If connected and fresh, onTelemetry()'s own banner logic (fallen/OK) is authoritative — leave it alone here.
}, 1000);

// ---------------------------------------------------------------------
// Telemetry -> Dashboard / Pot / PID / IMU live fields
// ---------------------------------------------------------------------
function onTelemetry(t) {
    // Dashboard
    $('dSpeed').innerHTML = fmt(t.speedKmh, 1) + '<span class="unit">km/h</span>';
    $('dPot').innerHTML = fmt(t.potPct, 0) + '<span class="unit">%</span>';
    const speedPct = Math.max(0, Math.min(100, Math.abs(t.speedKmh || 0) / Math.max(dashboardMaxSpeedKmh, 0.1) * 100));
    $('speedGaugeArc').style.strokeDasharray = speedPct + ' 100';
    $('speedGaugeArc').style.stroke = speedPct > 95 ? 'var(--warn)' : 'var(--accent2)';
    const potPct = Math.max(-100, Math.min(100, t.potPct || 0));
    $('dPotKnob').style.left = (50 + potPct / 2) + '%';
    $('dPotTrackFill').style.left = (potPct < 0 ? 50 + potPct / 2 : 50) + '%';
    $('dPotTrackFill').style.width = Math.abs(potPct / 2) + '%';
    if (t.controlMode === 1) {
        $('dTargetLabel').textContent = 'Target current';
        $('dTargetVal').innerHTML = fmt(t.targetCurrentA, 1) + '<span class="unit">A</span>';
        $('dControlMode').textContent = 'Current';
    } else {
        $('dTargetLabel').textContent = 'Target ERPM';
        $('dTargetVal').textContent = Math.round(t.targetErpm);
        $('dControlMode').textContent = 'Speed';
    }
    $('dActualErpm').textContent = t.vescFresh ? Math.round(t.actualErpm) : '--';
    $('dTrip').innerHTML = fmt(t.tripM, 1) + '<span class="unit">m</span>';
    $('dTripSource').textContent = !t.odometryAvailable
        ? 'waiting for VESC'
        : (t.odometryUsingTachometer ? 'tachometer' : 'ERPM estimate');
    lastEsp32BatteryVoltage = t.vBattEsp32;
    $('dVbatSource').textContent = t.batteryVoltageSource === 1 ? 'VESC' : 'ESP32';
    $('dVbat').innerHTML = t.batteryVoltageFresh
        ? fmt(t.vBatt, 2) + '<span class="unit">V</span>'
        : '--<span class="unit">V</span>';
    $('dCurrent').innerHTML = fmt(t.motorCurrentA, 1) + '<span class="unit">A</span>';
    $('dCurrentIn').innerHTML = fmt(t.inputCurrentA, 1) + '<span class="unit">A</span>';
    $('dDuty').innerHTML = fmt(t.duty * 100, 0) + '<span class="unit">%</span>';
    $('dAh').innerHTML = fmt(t.ahConsumed, 2) + '<span class="unit">Ah</span>';
    $('dTemp').innerHTML = fmt(t.fetTempC, 0) + ' / ' + fmt(t.motorTempC, 0) + '<span class="unit">°C</span>';
    $('dHeading').textContent = t.imuPresent ? Math.round(t.heading) : '--';
    $('dTilt').innerHTML = fmt(t.tilt, 1) + '<span class="unit">°</span>';
    $('dPitch').innerHTML = fmt(t.pitch, 1) + '<span class="unit">°</span>';
    $('dRoll').innerHTML = fmt(t.roll, 1) + '<span class="unit">°</span>';
    $('dAccel').innerHTML = fmt(t.accelG, 2) + '<span class="unit">g</span>';
    $('dCanState').textContent = t.canState;
    $('dCanState').className = 'pill ' + (t.canState === 'RUNNING' ? 'ok' : 'bad');
    $('dCanErr').textContent = 'tx ' + t.canTxErr + ' / rx ' + t.canRxErr;
    $('dBraking').className = 'pill warn';
    $('dBraking').style.display = t.brakingActive ? 'inline-block' : 'none';
    $('shaftPowerSwitch').checked = !!t.shaftPowerEnabled;
    $('shaftPowerSwitch').disabled = shaftPowerRequestPending || !t.calibrationReady || t.calibrationActive;
    $('shaftEnableControl').classList.toggle('active', !!t.shaftPowerEnabled);
    $('shaftPowerLabel').textContent = t.shaftPowerEnabled ? 'Power Enabled' : 'Power Disabled';
    $('parkSwitch').checked = !!t.parkEnabled;
    $('parkSwitch').disabled = parkRequestPending || !t.shaftPowerEnabled;
    document.querySelector('.park-control').classList.toggle('active', !!t.parkEnabled);
    $('parkStateLabel').textContent = t.parkEnabled ? 'Parked' : 'Drive';
    $('parkStateHint').textContent = t.parkEnabled
        ? (t.parkHolding ? 'Wheel position is held' : 'Park requested — safety override active')
        : 'Wheel free to drive';
    updatePotCalibrationControls(t.calibrationActive, t.calibrationWaitingForCenter);

    $('vs1').textContent = vescStatusText(t, 1,
        `${Math.round(t.actualErpm)} ERPM · ${fmt(t.motorCurrentA,1)} A motor · ${fmt(t.duty * 100,1)}% duty`);
    $('vs2').textContent = vescStatusText(t, 2,
        `${fmt(t.ahConsumed,3)} Ah used · ${fmt(t.ahCharged,3)} Ah charged`);
    $('vs3').textContent = vescStatusText(t, 3,
        `${fmt(t.whConsumed,2)} Wh used · ${fmt(t.whCharged,2)} Wh charged`);
    $('vs4').textContent = vescStatusText(t, 4,
        `${fmt(t.fetTempC,1)}°C FET · ${fmt(t.motorTempC,1)}°C motor · ${fmt(t.inputCurrentA,1)} A in · ${fmt(t.pidPositionDeg,1)}° PID`);
    $('vs5').textContent = vescStatusText(t, 5,
        `${fmt(t.vBattVesc,1)} V in · tacho ${t.tachometerRaw}`);
    $('vs6').textContent = vescStatusText(t, 6,
        `ADC ${fmt(t.adc1V,3)} / ${fmt(t.adc2V,3)} / ${fmt(t.adc3V,3)} V · PPM ${fmt(t.ppm * 100,1)}%`);

    drawCompass(t.heading, t.imuPresent);
    drawOrientationCube(t.pitch, t.roll, t.heading, t.imuPresent);
    updateTiltGauge(t.tilt, currentFallAngleThreshold);

    const banner = $('fallBanner');
    if (t.calibrationActive) {
        banner.className = 'banner warn';
        banner.textContent = t.calibrationWaitingForCenter
            ? 'Pot calibration saved — motor locked until the handle is centered.'
            : 'Pot calibration active — motor output is locked.';
    } else if (!t.calibrationReady) {
        banner.className = 'banner warn';
        banner.textContent = 'Pot calibration incomplete — motor CAN output is disabled. Capture MIN and MAX in Pot Calibration.';
    } else if (t.directionChangeLockout) {
        banner.className = 'banner warn';
        banner.textContent = 'Motor direction changed — center the handle before driving resumes.';
    } else if (t.fallen) {
        banner.className = 'banner bad';
        banner.textContent = '⚠ FALL DETECTED — motor output cut. Recovers automatically once upright.';
    } else if (!t.shaftPowerEnabled) {
        banner.className = 'banner warn';
        banner.textContent = 'Power disabled — no periodic motor commands are being sent. Enable it above when ready.';
    } else if (t.parkEnabled) {
        banner.className = 'banner warn';
        banner.textContent = t.parkHolding
            ? 'PARK active — VESC handbrake is holding the wheel position.'
            : 'PARK selected — handbrake is waiting for the active safety condition to clear.';
    } else if (t.postFallLockout) {
        banner.className = 'banner warn';
        banner.textContent = t.resumeWarnActive
            ? '🔊 Safety interruption cleared — starting warning in progress…'
            : 'Safety interruption cleared — center the handle to resume driving.';
    } else if (!t.imuPresent) {
        banner.className = 'banner bad';
        banner.textContent = 'IMU not detected — fall protection is NOT active.';
    } else {
        banner.className = 'banner ok';
        banner.textContent = 'System OK';
    }

    // Pot tab live numbers
    $('potRawInstantLive').textContent = t.potRawInstant;
    $('potRawLive').textContent = t.potRaw;
    $('potPctLive').innerHTML = fmt(t.potPct, 0) + '<span class="unit">%</span>';

    // IMU tab live numbers
    $('imuTilt').innerHTML = fmt(t.tilt, 1) + '<span class="unit">°</span>';
    $('imuPitch').innerHTML = fmt(t.pitch, 1) + '<span class="unit">°</span>';
    $('imuRoll').innerHTML = fmt(t.roll, 1) + '<span class="unit">°</span>';
    $('imuHeading').innerHTML = (t.imuPresent ? Math.round(t.heading) : '--') + '<span class="unit">°</span>';
    $('imuAccel').innerHTML = fmt(t.accelG, 2) + '<span class="unit">g</span>';

    if (t.magCalActive && t.magCal) {
        const mc = t.magCal;
        $('magCalPreview').textContent =
            `Capturing… X[${fmt(mc.minX,0)}, ${fmt(mc.maxX,0)}] Y[${fmt(mc.minY,0)}, ${fmt(mc.maxY,0)}] Z[${fmt(mc.minZ,0)}, ${fmt(mc.maxZ,0)}] µT — keep rotating`;
    } else if (!magCalJustStopped) {
        $('magCalPreview').textContent = 'Not calibrating.';
    }

    // PID sparkline
    pidPushSample(t.targetErpm, t.actualErpm);
}

let currentFallAngleThreshold = 55;
let dashboardMaxSpeedKmh = 6;
let magCalJustStopped = false;

function updateTiltGauge(tilt, threshold) {
    const pct = Math.max(0, Math.min(100, (tilt / Math.max(threshold, 1)) * 100));
    const el = $('tiltGaugeFill');
    el.style.width = pct + '%';
    el.className = 'gauge-fill' + (pct > 90 ? ' bad' : pct > 60 ? ' warn' : '');
    $('tiltThresholdLabel').textContent = 'limit ' + fmt(threshold, 0) + '°';
}

// ---------------------------------------------------------------------
// Compass rose (inline SVG, drawn once then just rotated)
// ---------------------------------------------------------------------
let compassBuilt = false;
function drawCompass(heading, present) {
    const svg = $('compassSvg');
    if (!compassBuilt) {
        svg.innerHTML = `
            <circle cx="75" cy="75" r="68" fill="none" stroke="var(--border)" stroke-width="2"/>
            <circle cx="75" cy="75" r="54" fill="none" stroke="var(--border)" stroke-width="1" stroke-dasharray="2 7"/>
            <text x="75" y="18" fill="var(--text-dim)" font-size="13" text-anchor="middle">N</text>
            <text x="75" y="140" fill="var(--text-dim)" font-size="13" text-anchor="middle">S</text>
            <text x="14" y="80" fill="var(--text-dim)" font-size="13" text-anchor="middle">W</text>
            <text x="136" y="80" fill="var(--text-dim)" font-size="13" text-anchor="middle">E</text>
            <g id="needle">
              <polygon points="75,18 66,80 75,68 84,80" fill="var(--bad)"/>
              <polygon points="75,132 66,80 75,92 84,80" fill="var(--text-dim)"/>
            </g>`;
        compassBuilt = true;
    }
    const needle = svg.querySelector('#needle');
    needle.style.opacity = present ? '1' : '.25';
    needle.setAttribute('transform', `rotate(${present ? heading : 0} 75 75)`);
}

function drawOrientationCube(pitch, roll, heading, present) {
    const cube = $('orientationCube');
    const p = Number.isFinite(pitch) ? pitch : 0;
    const r = Number.isFinite(roll) ? roll : 0;
    const h = Number.isFinite(heading) ? heading : 0;
    cube.style.opacity = present ? '1' : '.25';
    cube.style.transform = `rotateY(${-h}deg) rotateX(${p}deg) rotateZ(${-r}deg)`;
    $('cubeOrientationLabel').textContent = present
        ? `Pitch ${p.toFixed(1)}° · Roll ${r.toFixed(1)}°`
        : 'IMU unavailable';
    cube.parentElement.parentElement.setAttribute('aria-label', present
        ? `Device orientation: pitch ${p.toFixed(1)} degrees, roll ${r.toFixed(1)} degrees, heading ${h.toFixed(0)} degrees`
        : 'Device orientation unavailable');
}

// ---------------------------------------------------------------------
// PID sparkline
// ---------------------------------------------------------------------
const pidHistory = { target: [], actual: [] };
const PID_HISTORY_LEN = 150;
const PID_SAMPLE_INTERVAL_SECONDS = 0.1;
function pidPushSample(target, actual) {
    pidHistory.target.push(target);
    pidHistory.actual.push(actual);
    if (pidHistory.target.length > PID_HISTORY_LEN) { pidHistory.target.shift(); pidHistory.actual.shift(); }
    drawPidSpark();
}
function drawPidSpark() {
    const canvas = $('pidSpark');
    if (!canvas.width || canvas.width !== canvas.clientWidth || canvas.height !== canvas.clientHeight) {
        canvas.width = canvas.clientWidth; canvas.height = canvas.clientHeight;
    }
    const ctx = canvas.getContext('2d');
    const w = canvas.width, h = canvas.height;
    ctx.clearRect(0, 0, w, h);
    const all = pidHistory.target.concat(pidHistory.actual);
    let lo = all.length ? Math.min(...all) : -1000;
    let hi = all.length ? Math.max(...all) : 1000;
    if (hi - lo < 10) { hi += 5; lo -= 5; }
    const rangePad = (hi - lo) * 0.08;
    lo -= rangePad; hi += rangePad;

    const margin = { left: 56, right: 12, top: 17, bottom: 28 };
    const plotW = Math.max(1, w - margin.left - margin.right);
    const plotH = Math.max(1, h - margin.top - margin.bottom);
    const xForI = (i) => margin.left + (i / (PID_HISTORY_LEN - 1)) * plotW;
    const yForV = (v) => margin.top + (1 - (v - lo) / (hi - lo)) * plotH;
    const styles = getComputedStyle(document.documentElement);
    const gridColor = styles.getPropertyValue('--border').trim() || '#334155';
    const labelColor = styles.getPropertyValue('--text-dim').trim() || '#94a3b8';

    ctx.font = '11px system-ui, sans-serif';
    ctx.fillStyle = labelColor;
    ctx.strokeStyle = gridColor;
    ctx.lineWidth = 1;
    ctx.textBaseline = 'middle';
    for (let tick = 0; tick <= 4; tick++) {
        const fraction = tick / 4;
        const y = margin.top + fraction * plotH;
        const value = hi - fraction * (hi - lo);
        ctx.beginPath(); ctx.moveTo(margin.left, y); ctx.lineTo(w - margin.right, y); ctx.stroke();
        ctx.textAlign = 'right';
        ctx.fillText(Math.round(value).toLocaleString(), margin.left - 7, y);
    }
    const spanSeconds = (PID_HISTORY_LEN - 1) * PID_SAMPLE_INTERVAL_SECONDS;
    ctx.textBaseline = 'top';
    for (let tick = 0; tick <= 3; tick++) {
        const fraction = tick / 3;
        const x = margin.left + fraction * plotW;
        const secondsAgo = (1 - fraction) * spanSeconds;
        ctx.beginPath(); ctx.moveTo(x, margin.top); ctx.lineTo(x, margin.top + plotH); ctx.stroke();
        ctx.textAlign = tick === 0 ? 'left' : tick === 3 ? 'right' : 'center';
        ctx.fillText(tick === 3 ? '0 s' : `-${Math.round(secondsAgo)} s`, x, margin.top + plotH + 6);
    }
    ctx.textAlign = 'left';
    ctx.textBaseline = 'top';
    ctx.fillText('ERPM', 5, 2);

    function plot(arr, color) {
        if (!arr.length) return;
        ctx.beginPath();
        ctx.strokeStyle = color;
        ctx.lineWidth = 2;
        const offset = PID_HISTORY_LEN - arr.length;
        arr.forEach((v, i) => {
            const x = xForI(i + offset);
            const y = yForV(v);
            if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
        });
        ctx.stroke();
    }
    plot(pidHistory.target, '#4c8dff');
    plot(pidHistory.actual, '#35c46a');
}
window.addEventListener('resize', drawPidSpark);

// ---------------------------------------------------------------------
// CAN monitor
// ---------------------------------------------------------------------
// One static row per distinct CAN identifier, like a live spreadsheet —
// not a scrolling per-frame log. A row is created the first time an ID is
// seen and then just has its cells updated in place on every subsequent
// frame of that ID, keeping the table's size bounded by "how many distinct
// IDs are on the bus" instead of growing forever.
let canPaused = false;
const canRowsById = new Map(); // numeric CAN identifier -> { tr, count }
let canFrameCounter = 0;
let canRateWindow = [];

const CAN_PACKET_NAMES = {
    0: 'SET_DUTY', 1: 'SET_CURRENT', 2: 'SET_CURRENT_BRAKE', 3: 'SET_RPM',
    4: 'SET_POS', 10: 'SET_CURRENT_REL', 11: 'SET_CURRENT_BRAKE_REL',
    12: 'SET_CURRENT_HANDBRAKE', 13: 'SET_CURRENT_HANDBRAKE_REL',
    9: 'STATUS', 14: 'STATUS_2', 15: 'STATUS_3', 16: 'STATUS_4', 27: 'STATUS_5', 58: 'STATUS_6',
};

function canBytes(hex) {
    return String(hex || '').trim().split(/\s+/).filter(Boolean).map((v) => parseInt(v, 16));
}
function canI16(b, i) {
    const v = ((b[i] || 0) << 8) | (b[i + 1] || 0);
    return v & 0x8000 ? v - 0x10000 : v;
}
function canI32(b, i) {
    return ((b[i] || 0) << 24) | ((b[i + 1] || 0) << 16) |
           ((b[i + 2] || 0) << 8) | (b[i + 3] || 0);
}
function decodeVescFrame(packetId, hex) {
    const b = canBytes(hex);
    if (b.length < 4) return '—';
    switch (packetId) {
        case 0: return `Duty ${(canI32(b,0) / 1000).toFixed(2)}%`;
        case 1: return `Motor current ${(canI32(b,0) / 1000).toFixed(3)} A`;
        case 2: return `Brake current ${(canI32(b,0) / 1000).toFixed(3)} A`;
        case 3: return `Target ${canI32(b,0)} ERPM`;
        case 4: return `Position ${(canI32(b,0) / 1000000).toFixed(3)}°`;
        case 9: return b.length >= 8 ? `${canI32(b,0)} ERPM · ${(canI16(b,4)/10).toFixed(1)} A · ${(canI16(b,6)/10).toFixed(1)}% duty` : 'Invalid STATUS length';
        case 10: return `Relative current ${(canI32(b,0) / 1000).toFixed(2)}%`;
        case 11: return `Relative brake ${(canI32(b,0) / 1000).toFixed(2)}%`;
        case 12: return `Handbrake ${(canI32(b,0) / 1000).toFixed(3)} A`;
        case 13: return `Relative handbrake ${(canI32(b,0) / 1000).toFixed(2)}%`;
        case 14: return b.length >= 8 ? `${(canI32(b,0)/10000).toFixed(4)} Ah used · ${(canI32(b,4)/10000).toFixed(4)} Ah charged` : 'Invalid STATUS 2 length';
        case 15: return b.length >= 8 ? `${(canI32(b,0)/10000).toFixed(4)} Wh used · ${(canI32(b,4)/10000).toFixed(4)} Wh charged` : 'Invalid STATUS 3 length';
        case 16: return b.length >= 8 ? `${(canI16(b,0)/10).toFixed(1)}°C FET · ${(canI16(b,2)/10).toFixed(1)}°C motor · ${(canI16(b,4)/10).toFixed(1)} A in · ${(canI16(b,6)/50).toFixed(1)}° PID` : 'Invalid STATUS 4 length';
        case 27: return b.length >= 6 ? `Tacho ${canI32(b,0)} · ${(canI16(b,4)/10).toFixed(1)} V in` : 'Invalid STATUS 5 length';
        case 58: return b.length >= 8 ? `ADC ${(canI16(b,0)/1000).toFixed(3)} / ${(canI16(b,2)/1000).toFixed(3)} / ${(canI16(b,4)/1000).toFixed(3)} V · PPM ${(canI16(b,6)/10).toFixed(1)}%` : 'Invalid STATUS 6 length';
        default: return '—';
    }
}

$('btnCanPause').addEventListener('click', () => {
    canPaused = !canPaused;
    $('btnCanPause').textContent = canPaused ? 'Resume' : 'Pause';
});
$('btnCanClear').addEventListener('click', () => {
    canRowsById.clear();
    $('canRows').innerHTML = '';
});

async function loadCanHistory() {
    try {
        const data = await apiGet('/api/can/log');
        onCanFrames(data.frames, true);
    } catch (e) { /* ignore */ }
}

// Creates (if needed) the static row for this CAN id, keeping the table
// sorted by id ascending so rows have a stable, predictable position.
function canRowFor(id) {
    let entry = canRowsById.get(id);
    if (entry) return entry;

    const tr = document.createElement('tr');
    tr.dataset.id = String(id);
    tr.innerHTML = '<td></td><td></td><td></td><td></td><td></td><td></td><td>0</td><td></td>';
    entry = { tr, count: 0 };
    canRowsById.set(id, entry);

    const tbody = $('canRows');
    const nextSibling = Array.from(tbody.children).find((r) => Number(r.dataset.id) > id);
    if (nextSibling) tbody.insertBefore(tr, nextSibling); else tbody.appendChild(tr);
    return entry;
}

function onCanFrames(frames, isHistory) {
    if (!frames || !frames.length) return;
    if (canPaused && !isHistory) return;

    frames.forEach((f) => {
        canFrameCounter++;
        canRateWindow.push(Date.now());
        const packetId = (f.id >> 8) & 0xFF;
        const ctrlId = f.id & 0xFF;
        const name = CAN_PACKET_NAMES[packetId] || ('id ' + packetId);

        const entry = canRowFor(f.id);
        entry.count++;
        entry.tr.className = (f.tx ? 'tx' : 'rx') + (f.ok === false ? ' fail' : '');
        const c = entry.tr.children;
        c[0].textContent = f.tx ? 'TX' : 'RX';
        c[1].textContent = '0x' + f.id.toString(16).toUpperCase() + ' (ctrl ' + ctrlId + ')';
        c[2].textContent = name;
        c[3].textContent = decodeVescFrame(packetId, f.data);
        c[4].textContent = f.dlc;
        c[5].textContent = f.data;
        c[6].textContent = entry.count;
        c[7].textContent = f.t;
    });
}

setInterval(() => {
    const now = Date.now();
    canRateWindow = canRateWindow.filter((t) => now - t < 2000);
    $('canRate').textContent = Math.round(canRateWindow.length / 2) + ' fps';
}, 1000);

// ---------------------------------------------------------------------
// Settings load/save
// ---------------------------------------------------------------------
async function loadSettings() {
    const s = await apiGet('/api/settings');
    currentFallAngleThreshold = s.fallAngleThresholdDeg;

    $('potDeadband').value = s.deadbandPercent;
    $('potFilterSamples').value = s.potFilterSamples;
    if (s.speedLimitUnit === 0) {
        const circumference = (s.wheelDiameterMm / 1000) * Math.PI;
        const divisor = Math.max((s.motorPolePairs || 1) * (s.gearRatio || 1), 0.0001);
        const erpmLimit = Math.max(s.maxErpm || 0, s.maxErpmBackward || 0);
        dashboardMaxSpeedKmh = Math.max((erpmLimit / divisor / 60) * circumference * 3.6, 0.1);
    } else {
        dashboardMaxSpeedKmh = Math.max(s.maxSpeedKmh || 0, s.maxSpeedKmhBackward || 0, 0.1);
    }
    $('speedGaugeMax').textContent = fmt(dashboardMaxSpeedKmh, 1) + ' km/h';

    $('pidEnabled').checked = s.speedPidEnabled;
    $('pidKp').value = s.pidKp;
    $('pidKi').value = s.pidKi;
    $('pidKd').value = s.pidKd;
    $('pidMaxTrim').value = s.pidMaxTrimErpm;

    $('fallAngle').value = s.fallAngleThresholdDeg;
    $('fallMargin').value = s.fallRecoverMarginDeg;
    $('fallConfirm').value = s.fallConfirmMs;
    $('fallRecover').value = s.fallRecoverStableMs;
    $('impactEnabled').checked = s.impactDetectEnabled;
    $('impactThreshold').value = s.impactAccelThresholdG;
    $('fallWarnEnabled').checked = s.fallWarningBuzzerEnabled;
    $('fallWarnStartPct').value = s.fallWarningStartPercent;
    $('magDecl').value = s.magDeclinationDeg;

    $('sysCtrlId').value = s.vescControllerId;
    $('sysCanBaud').value = String(s.canBaudrateBps);
    $('sysControlMode').value = String(s.controlMode);
    $('sysInvertMotor').checked = s.invertMotorDirection;
    $('sysSpeedLimitUnit').value = String(s.speedLimitUnit);
    $('sysMaxErpm').value = s.maxErpm;
    $('sysMaxSpeedKmh').value = s.maxSpeedKmh;
    $('sysMaxCurrent').value = s.maxCurrentA;
    $('sysReverseEnabled').checked = s.reverseEnabled;
    $('sysMaxErpmBack').value = s.maxErpmBackward;
    $('sysMaxSpeedKmhBack').value = s.maxSpeedKmhBackward;
    $('sysMaxCurrentBack').value = s.maxCurrentBackwardA;
    $('sysBrakingEnabled').checked = s.downhillBrakingEnabled;
    $('sysBrakeCurrent').value = s.brakeCurrentA;
    $('sysBrakeMargin').value = s.brakeEngageErpmMargin;
    $('sysParkCurrent').value = s.parkCurrentA;
    $('sysSendMs').value = s.canSendIntervalMs;
    $('sysPoles').value = s.motorPolePairs;
    $('sysWheel').value = s.wheelDiameterMm;
    $('sysGear').value = s.gearRatio;
    $('sysVbatSource').value = String(s.batteryVoltageSource);
    $('sysVbatScale').value = s.vbatCalibrationScale;
    $('sysSsid').value = s.wifiSsid;
    $('sysPassCurrent').value = '';
    $('sysPass').value = '';
    $('sysPass').placeholder = '(unchanged)';

    updateSpeedLimitUnitUI();
    updateVbatSourceUI();
}

function updateVbatSourceUI() {
    const useVesc = parseInt($('sysVbatSource').value, 10) === 1;
    $('vbatDividerCalibration').style.display = useVesc ? 'none' : 'block';
    $('vbatSourceHint').textContent = useVesc
        ? 'Requires VESC CAN Status Message 5. No ESP32 fallback is used if that message becomes stale.'
        : 'The ESP32 resistor divider is sampled locally; its calibration scale is applied before display.';
}
$('sysVbatSource').addEventListener('change', updateVbatSourceUI);

// ---------------------------------------------------------------------
// Speed limit unit (ERPM vs km/h): client-side mirror of main.cpp's
// erpmFromKmh() / the mechRpm speed-telemetry math, purely so the
// non-editable field can show a live "equivalent" preview without a round
// trip to the controller. The firmware does the authoritative conversion
// itself (see resolveSpeedLimitsErpm() in main.cpp) — this is just a UI aid.
function wheelCircumferenceMClient() {
    const wheelMm = parseFloat($('sysWheel').value) || 0;
    return (wheelMm / 1000) * Math.PI;
}
function erpmFromKmhClient(kmh) {
    const circ = wheelCircumferenceMClient();
    if (!(circ > 0.0001) || !(kmh > 0)) return 0;
    const mps = kmh / 3.6;
    const mechRpm = (mps * 60) / circ;
    const poles = parseFloat($('sysPoles').value) || 1;
    const gear = parseFloat($('sysGear').value) || 1;
    return Math.round(mechRpm * poles * gear);
}
function kmhFromErpmClient(erpm) {
    const circ = wheelCircumferenceMClient();
    const poles = parseFloat($('sysPoles').value) || 1;
    const gear = (parseFloat($('sysGear').value) || 1) || 1;
    if (!(circ > 0.0001) || !(poles > 0) || !(gear > 0.0001)) return 0;
    const mechRpm = erpm / poles / gear;
    const mps = (mechRpm / 60) * circ;
    return mps * 3.6;
}
function updateSpeedLimitUnitUI() {
    const kmhMode = parseInt($('sysSpeedLimitUnit').value, 10) !== 0;
    $('sysMaxSpeedKmh').readOnly = !kmhMode;
    $('sysMaxSpeedKmhBack').readOnly = !kmhMode;
    $('sysMaxErpm').readOnly = kmhMode;
    $('sysMaxErpmBack').readOnly = kmhMode;
    if (kmhMode) {
        $('sysMaxErpm').value = erpmFromKmhClient(parseFloat($('sysMaxSpeedKmh').value));
        $('sysMaxErpmBack').value = erpmFromKmhClient(parseFloat($('sysMaxSpeedKmhBack').value));
    } else {
        $('sysMaxSpeedKmh').value = kmhFromErpmClient(parseFloat($('sysMaxErpm').value)).toFixed(1);
        $('sysMaxSpeedKmhBack').value = kmhFromErpmClient(parseFloat($('sysMaxErpmBack').value)).toFixed(1);
    }
}
$('sysSpeedLimitUnit').addEventListener('change', updateSpeedLimitUnitUI);
['sysMaxSpeedKmh', 'sysMaxSpeedKmhBack', 'sysMaxErpm', 'sysMaxErpmBack', 'sysPoles', 'sysWheel', 'sysGear']
    .forEach((id) => $(id).addEventListener('input', updateSpeedLimitUnitUI));

async function saveSettings(patch, label) {
    try {
        const res = await apiPost('/api/settings', patch);
        if (res.passwordRejected) {
            toast('Current WiFi password was incorrect — password NOT changed (other settings were still saved)', true);
            return;
        }
        toast((label || 'Settings') + ' saved' + (res.restartRequired ? ' — restart required' : ''));
        if (res.restartRequired) offerRestart();
    } catch (e) {
        toast('Save failed: ' + e.message, true);
    }
}

function offerRestart() {
    if (confirm('This change needs a restart to take effect. Restart now?')) {
        apiPost('/api/system/restart').then(() => {
            toast('Restarting…');
        }).catch(() => {});
    }
}

$('btnSavePot').addEventListener('click', () => saveSettings({
    deadbandPercent: parseFloat($('potDeadband').value),
    potFilterSamples: parseInt($('potFilterSamples').value, 10),
}, 'Pot mapping'));

$('btnSavePid').addEventListener('click', () => saveSettings({
    speedPidEnabled: $('pidEnabled').checked,
    pidKp: parseFloat($('pidKp').value),
    pidKi: parseFloat($('pidKi').value),
    pidKd: parseFloat($('pidKd').value),
    pidMaxTrimErpm: parseInt($('pidMaxTrim').value, 10),
}, 'Speed PID'));

$('btnSaveFall').addEventListener('click', () => {
    currentFallAngleThreshold = parseFloat($('fallAngle').value);
    saveSettings({
        fallAngleThresholdDeg: currentFallAngleThreshold,
        fallRecoverMarginDeg: parseFloat($('fallMargin').value),
        fallConfirmMs: parseInt($('fallConfirm').value, 10),
        fallRecoverStableMs: parseInt($('fallRecover').value, 10),
        impactDetectEnabled: $('impactEnabled').checked,
        impactAccelThresholdG: parseFloat($('impactThreshold').value),
        fallWarningBuzzerEnabled: $('fallWarnEnabled').checked,
        fallWarningStartPercent: parseFloat($('fallWarnStartPct').value),
    }, 'Fall detection');
});

$('btnSaveDecl').addEventListener('click', () => saveSettings({
    magDeclinationDeg: parseFloat($('magDecl').value),
}, 'Declination'));

$('btnSaveSystem').addEventListener('click', () => {
    const patch = {
        vescControllerId: parseInt($('sysCtrlId').value, 10),
        canBaudrateBps: parseInt($('sysCanBaud').value, 10),
        controlMode: parseInt($('sysControlMode').value, 10),
        invertMotorDirection: $('sysInvertMotor').checked,
        speedLimitUnit: parseInt($('sysSpeedLimitUnit').value, 10),
        maxErpm: parseInt($('sysMaxErpm').value, 10),
        maxSpeedKmh: parseFloat($('sysMaxSpeedKmh').value),
        maxCurrentA: parseFloat($('sysMaxCurrent').value),
        reverseEnabled: $('sysReverseEnabled').checked,
        maxErpmBackward: parseInt($('sysMaxErpmBack').value, 10),
        maxSpeedKmhBackward: parseFloat($('sysMaxSpeedKmhBack').value),
        maxCurrentBackwardA: parseFloat($('sysMaxCurrentBack').value),
        downhillBrakingEnabled: $('sysBrakingEnabled').checked,
        brakeCurrentA: parseFloat($('sysBrakeCurrent').value),
        brakeEngageErpmMargin: parseInt($('sysBrakeMargin').value, 10),
        parkCurrentA: parseFloat($('sysParkCurrent').value),
        canSendIntervalMs: parseInt($('sysSendMs').value, 10),
        motorPolePairs: parseInt($('sysPoles').value, 10),
        wheelDiameterMm: parseFloat($('sysWheel').value),
        gearRatio: parseFloat($('sysGear').value),
        batteryVoltageSource: parseInt($('sysVbatSource').value, 10),
        vbatCalibrationScale: parseFloat($('sysVbatScale').value),
        wifiSsid: $('sysSsid').value,
    };
    const newPass = $('sysPass').value;
    if (newPass.length > 0) {
        const currentPass = $('sysPassCurrent').value;
        if (currentPass.length === 0) {
            toast('Enter the current WiFi password to set a new one', true);
            return;
        }
        patch.wifiPassword = newPass;
        patch.wifiPasswordCurrent = currentPass;
    }
    saveSettings(patch, 'System settings');
});

$('btnComputeVbat').addEventListener('click', () => {
    // NOTE: was `if (!measured || !reported)` — falsy-zero bug: a battery
    // genuinely reading 0.00V (e.g. no telemetry has arrived yet, so the
    // dashboard's default "0.0V" is still showing) made `!reported` true
    // and produced this same "enter a multimeter reading" toast even when
    // the multimeter field was filled in correctly. Checking for NaN (a
    // truly empty/non-numeric field) instead of falsiness fixes that, and
    // splitting the two checks gives an accurate message for each case.
    const measured = parseFloat($('sysVbatMeasured').value);
    if (Number.isNaN(measured) || measured <= 0) {
        toast('Enter a multimeter reading first', true);
        return;
    }
    const reported = lastEsp32BatteryVoltage;
    if (Number.isNaN(reported) || reported <= 0) {
        toast('No live battery reading yet — check the controller is connected and reporting telemetry', true);
        return;
    }
    const currentScale = parseFloat($('sysVbatScale').value) || 1.0;
    const reportedUnscaled = reported / currentScale;
    const newScale = measured / reportedUnscaled;
    $('sysVbatScale').value = newScale.toFixed(4);
    toast('Scale computed: ' + newScale.toFixed(4) + ' — click "Save all system settings" to apply');
});

// ---------------------------------------------------------------------
// Pot calibration actions
// ---------------------------------------------------------------------
async function refreshPotCal() {
    const c = await apiGet('/api/pot/calibration');
    $('potCalMin').textContent = c.min;
    $('potCalMax').textContent = c.max;
    $('potCalCenter').textContent = c.center;
    updatePotCalibrationControls(c.sessionActive, c.waitingForCenter);
}
function updatePotCalibrationControls(active, waitingForCenter) {
    const capturing = active && !waitingForCenter;
    $('btnStartPotCal').disabled = active;
    $('btnCancelPotCal').disabled = !capturing;
    $('btnCapMin').disabled = !capturing;
    $('btnCapMax').disabled = !capturing;
    $('potCalSessionMsg').textContent = waitingForCenter
        ? 'Calibration finished — return the handle to center to unlock the motor.'
        : active
            ? 'Calibration active — motor locked. Capture MIN and MAX.'
            : 'Calibration inactive — motor control uses the saved endpoints.';
}
function potCaptureMsg(r) {
    if (r.ok) return 'Calibration saved. Return the handle to center to unlock the motor.';
    if (!r.valid && (!r.hasMin || !r.hasMax)) {
        return 'Endpoint recorded. Now capture the ' + (r.hasMin ? 'MAX' : 'MIN') +
            ' endpoint; motor CAN output remains disabled until both are valid.';
    }
    return 'Span too small: this reading=' + r.raw + ', stored min=' + r.min + ' max=' + r.max +
        ' — move further before capturing, or check the live Raw ADC number ' +
        'actually changes as you move the pot.';
}
$('btnStartPotCal').addEventListener('click', async () => {
    try {
        const r = await apiPost('/api/pot/start');
        toast(r.stopSent ? 'Calibration started — motor locked' : 'Calibration started — stop frame failed; VESC timeout will stop the motor', !r.stopSent);
        refreshPotCal();
    } catch (e) { toast('Failed: ' + e.message, true); }
});
$('btnCancelPotCal').addEventListener('click', async () => {
    try {
        await apiPost('/api/pot/cancel');
        toast('Calibration cancelled — center the handle to unlock the motor');
        refreshPotCal();
    } catch (e) { toast('Failed: ' + e.message, true); }
});
$('btnCapMin').addEventListener('click', async () => {
    try { const r = await apiPost('/api/pot/capture?which=min'); toast(potCaptureMsg(r), !r.ok && r.hasMin && r.hasMax); refreshPotCal(); }
    catch (e) { toast('Failed: ' + e.message, true); }
});
$('btnCapMax').addEventListener('click', async () => {
    try { const r = await apiPost('/api/pot/capture?which=max'); toast(potCaptureMsg(r), !r.ok && r.hasMin && r.hasMax); refreshPotCal(); }
    catch (e) { toast('Failed: ' + e.message, true); }
});

// ---------------------------------------------------------------------
// IMU actions
// ---------------------------------------------------------------------
$('btnZero').addEventListener('click', async () => {
    try { await apiPost('/api/imu/zero'); toast('Upright zero calibrated'); }
    catch (e) { toast('Failed: ' + e.message, true); }
});
$('btnClearFall').addEventListener('click', async () => {
    try { await apiPost('/api/fall/clear'); toast('Fall latch cleared'); }
    catch (e) { toast('Failed: ' + e.message, true); }
});
$('btnMagStart').addEventListener('click', async () => {
    try { await apiPost('/api/imu/mag/start'); magCalJustStopped = false; toast('Rotate the device through all axes now…'); }
    catch (e) { toast('Failed: ' + e.message, true); }
});
$('btnMagStop').addEventListener('click', async () => {
    try { await apiPost('/api/imu/mag/stop?save=true'); magCalJustStopped = true; $('magCalPreview').textContent = 'Saved.'; toast('Compass calibration saved'); }
    catch (e) { toast('Failed: ' + e.message, true); }
});
$('btnMagCancel').addEventListener('click', async () => {
    try { await apiPost('/api/imu/mag/stop?save=false'); magCalJustStopped = true; $('magCalPreview').textContent = 'Cancelled.'; toast('Cancelled'); }
    catch (e) { toast('Failed: ' + e.message, true); }
});

async function loadFallEvents() {
    try {
        const data = await apiGet('/api/fall/events');
        const el = $('eventLog');
        if (!data.events || data.events.length === 0) {
            el.innerHTML = '<div class="hint">No events yet.</div>';
            return;
        }
        el.innerHTML = data.events.slice().reverse().map((e) =>
            `<div>[${(e.t/1000).toFixed(1)}s uptime] ${e.trigger.toUpperCase()} — angle ${e.angle.toFixed(1)}°</div>`
        ).join('') + `<div class="hint" style="margin-top:6px">${data.totalCount} total (log keeps the most recent ${data.events.length > 20 ? data.events.length : 20})</div>`;
    } catch (e) { /* ignore */ }
}
$('btnRefreshEvents').addEventListener('click', loadFallEvents);

// ---------------------------------------------------------------------
// Trip / system actions
// ---------------------------------------------------------------------
$('btnResetTrip').addEventListener('click', async () => {
    try { await apiPost('/api/trip/reset'); toast('Trip distance reset'); }
    catch (e) { toast('Failed: ' + e.message, true); }
});

$('parkSwitch').addEventListener('change', async (event) => {
    const requested = event.target.checked;
    parkRequestPending = true;
    event.target.disabled = true;
    try {
        const result = await apiPost('/api/park?enabled=' + (requested ? 'true' : 'false'));
        event.target.checked = !!result.parkEnabled;
        toast(result.parkEnabled ? 'Park engaged — wheel position held' : 'Park released — center handle for restart warning');
    } catch (e) {
        event.target.checked = !requested;
        toast('Park command failed: ' + e.message, true);
    } finally {
        parkRequestPending = false;
        event.target.disabled = !(ws && ws.readyState === WebSocket.OPEN);
    }
});

$('shaftPowerSwitch').addEventListener('change', async (event) => {
    const requested = event.target.checked;
    shaftPowerRequestPending = true;
    event.target.disabled = true;
    try {
        const result = await apiPost('/api/shaft-power?enabled=' + (requested ? 'true' : 'false'));
        event.target.checked = !!result.shaftPowerEnabled;
        if (!result.shaftPowerEnabled) $('parkSwitch').checked = false;
        toast(result.shaftPowerEnabled
            ? 'Power enabled — center handle for restart warning'
            : 'Power disabled — motor commands stopped');
    } catch (e) {
        event.target.checked = !requested;
        toast('Power command failed: ' + e.message, true);
    } finally {
        shaftPowerRequestPending = false;
        event.target.disabled = !(ws && ws.readyState === WebSocket.OPEN);
    }
});

$('btnHeaderRestart').addEventListener('click', async () => {
    if (!confirm('Restart the ESP32 now? Motor power will reset to Disabled.')) return;
    try { await apiPost('/api/system/restart'); toast('Restarting…'); }
    catch (e) { toast('Restart failed: ' + e.message, true); }
});

async function loadSystemInfo() {
    try {
        const info = await apiGet('/api/system/info');
        $('sysUptime').textContent = Math.floor(info.uptimeS / 3600) + 'h ' + Math.floor((info.uptimeS % 3600) / 60) + 'm';
        $('sysHeap').textContent = Math.round(info.freeHeap / 1024) + ' KB';
        $('sysBuildInfo').textContent = 'Firmware built ' + info.buildDate + ' — ' + info.mdnsHost + ' · ' + info.ip;
    } catch (e) { /* ignore */ }
}

$('btnRestart').addEventListener('click', async () => {
    if (!confirm('Restart the controller now? The motor will briefly stop responding to CAN commands.')) return;
    try { await apiPost('/api/system/restart'); toast('Restarting…'); }
    catch (e) { toast('Failed: ' + e.message, true); }
});
$('btnFactoryReset').addEventListener('click', async () => {
    if (!confirm('This erases ALL saved settings and calibration (pot, IMU, PID, WiFi) and restarts. Continue?')) return;
    try { await apiPost('/api/system/factory-reset'); toast('Erasing and restarting…'); }
    catch (e) { toast('Failed: ' + e.message, true); }
});

// ---------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------
loadSettings().catch((e) => toast('Could not load settings: ' + e.message, true));
refreshPotCal().catch(() => {});
loadSystemInfo().catch(() => {});
