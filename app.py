#!/usr/bin/env python3
"""
app.py

Standalone single-axis APU control - homing (continuous search) +
fixed +180/-180 moves + manual jog.
Run with:  python app.py

Serves the web UI at http://localhost:5000

Gauge is a full 360 deg circle labeled -180..+180, with HOME at the
top (12 o'clock). Positive angles sweep clockwise, negative sweep
counter-clockwise -- both meet at the bottom (180 / -180 is the same
physical point on this direct-drive shaft). The needle and the
"READINGS" box are both driven by the controller's live, encoder-based
angle (see single_axis_controller.py's "LIVE ANGLE FEEDBACK") -- not a
timed projection -- so they track the real shaft position as it moves.

Top of the page has:
  - a COM-port selector
  - acceleration / deceleration fields (0 = sudden start/stop)
  - manual speed + forward/backward jog buttons
  - a small READINGS box with the continuous live angle
"""

import sys
import io
import math
import argparse

if sys.platform == "win32":
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")
    sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding="utf-8", errors="replace")

from flask import Flask, render_template_string, jsonify, request

from single_axis_controller import (
    SingleAxisController,
    TARGET_ANGLE,
    DEFAULT_MANUAL_SPEED_RPM,
    list_serial_ports,
)

# =====================================================
# GAUGE SVG (static markup computed once at import time; only the
# needle rotates, driven by JS from /get_status)
# =====================================================

GAUGE_SIZE = 260
GAUGE_CENTER = GAUGE_SIZE / 2
GAUGE_R = GAUGE_SIZE / 2 - 22


def _polar(cx, cy, r, angle_deg):
    """0 deg = straight up, increasing CLOCKWISE (matches CSS rotate())."""
    rad = math.radians(angle_deg)
    return cx + r * math.sin(rad), cy - r * math.cos(rad)


def build_gauge_svg(major_step=30, color="#39ff14", dim_color="#1c5a1c"):
    cx = cy = GAUGE_CENTER
    r = GAUGE_R
    parts = [f'<circle cx="{cx}" cy="{cy}" r="{r:.1f}" fill="none" stroke="{dim_color}" stroke-width="2"/>']

    deg = -180
    while deg <= 180:
        x1, y1 = _polar(cx, cy, r, deg)
        x2, y2 = _polar(cx, cy, r - 10, deg)
        parts.append(f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" '
                     f'stroke="{color}" stroke-width="2"/>')
        lx, ly = _polar(cx, cy, r - 26, deg)
        parts.append(f'<text x="{lx:.1f}" y="{ly:.1f}" fill="{color}" font-size="10" '
                     f'font-family="Consolas,monospace" text-anchor="middle" '
                     f'dominant-baseline="middle">{deg}</text>')
        deg += major_step

    hx, hy = _polar(cx, cy, r + 14, 0)
    parts.append(f'<text x="{hx:.1f}" y="{hy:.1f}" fill="#ffffff" font-size="11" '
                 f'font-family="Consolas,monospace" text-anchor="middle" font-weight="bold">HOME</text>')
    return "".join(parts)


GAUGE_SVG = build_gauge_svg()

# =====================================================
# HTML
# =====================================================

MAIN_HTML = """
<!doctype html>
<html>
<head>
<meta charset="utf-8">
<title>Single Axis APU Control</title>
<style>
    * { box-sizing: border-box; }
    body { font-family: 'Segoe UI', Arial; background: linear-gradient(135deg, #0f172a, #020617);
           color: white; margin: 0; padding: 0; }

    .app-shell { max-width: 1120px; margin: 0 auto; padding: 16px 20px 28px; }

    /* ---------- sticky top bar: title, connection, STOP ---------- */
    .top-bar { position: sticky; top: 0; z-index: 500; display: flex; align-items: center;
               justify-content: space-between; gap: 16px;
               background: rgba(15, 23, 42, 0.92); backdrop-filter: blur(10px);
               padding: 14px 22px; border-radius: 14px; margin-bottom: 18px;
               box-shadow: 0 8px 25px rgba(0,0,0,0.5); border: 1px solid rgba(255,255,255,0.06); }
    .top-bar h1 { font-size: 19px; margin: 0; font-weight: 600; letter-spacing: 0.3px; }
    .top-bar-left { display: flex; align-items: center; gap: 14px; }
    .conn-indicator { display: flex; align-items: center; gap: 6px; font-size: 12px;
                       color: #94a3b8; letter-spacing: 0.5px; }
    .conn-dot { display: inline-block; width: 9px; height: 9px; border-radius: 50%; }
    .conn-yes { background: #22c55e; } .conn-no { background: #ef4444; }
    .stop-btn-header { background: #ef4444; padding: 13px 30px; font-size: 15px; font-weight: 700;
                        border-radius: 10px; border: none; color: white; cursor: pointer;
                        letter-spacing: 0.5px; box-shadow: 0 4px 14px rgba(239,68,68,0.4); }
    .stop-btn-header:hover { background: #dc2626; }

    /* ---------- dashboard grid: controls left, visuals right ---------- */
    .dashboard-grid { display: grid; grid-template-columns: 330px 1fr; gap: 18px; align-items: start; }
    @media (max-width: 820px) { .dashboard-grid { grid-template-columns: 1fr; } }

    .col-stack { display: flex; flex-direction: column; gap: 14px; }

    .card { background: rgba(30, 41, 59, 0.6); backdrop-filter: blur(10px); padding: 18px 20px;
            border-radius: 14px; box-shadow: 0px 6px 18px rgba(0,0,0,0.4);
            border: 1px solid rgba(255,255,255,0.05); text-align: left; }
    .card-title { font-size: 11px; color: #64748b; letter-spacing: 1.5px; font-weight: 600;
                  margin-bottom: 10px; text-transform: uppercase; }

    .config-row { display: flex; align-items: center; justify-content: space-between;
                  margin: 8px 0; gap: 10px; }
    .config-row label { font-size: 12.5px; color: #94a3b8; letter-spacing: 0.3px; }
    select, input[type=number] { background: #0f172a; color: #39ff14; border: 1px solid #334155;
             border-radius: 8px; padding: 8px 10px; font-size: 13px; font-family: Consolas, monospace; }
    input[type=number] { width: 85px; }
    select { flex: 1; min-width: 0; }

    button { padding: 10px 16px; border-radius: 9px; border: none; font-size: 13.5px;
             cursor: pointer; transition: 0.15s; color: white; }
    button:disabled { opacity: 0.35; cursor: not-allowed; }
    .apply-btn { background: #6366f1; }
    .refresh-btn { background: #334155; padding: 8px 12px; }
    .manual-btn { background: #06b6d4; flex: 1; padding: 12px 10px; font-size: 13.5px; font-weight: 600; }
    .home-btn { background: #3b82f6; flex: 1; padding: 13px 10px; font-weight: 600; }
    .plus-btn { background: #22c55e; flex: 1; padding: 13px 10px; font-weight: 600; }
    .minus-btn { background: #f59e0b; flex: 1; padding: 13px 10px; font-weight: 600; }

    /* ---------- right column: visuals card ---------- */
    .visual-card { display: flex; flex-direction: column; gap: 16px; }
    .visual-row { display: flex; gap: 20px; align-items: center; flex-wrap: wrap; justify-content: center; }

    .gauge-card { background: #000; border-radius: 14px; padding: 14px; flex-shrink: 0; }
    .gauge-needle { transform-origin: 130px 130px; transition: transform 0.08s linear; }
    .gauge-readout { font-size: 20px; font-weight: bold; margin-top: 6px; color: #39ff14; text-align: center; }

    .readings-panel { flex: 1; min-width: 180px; background: rgba(15, 23, 42, 0.55);
                       border-radius: 12px; padding: 18px; text-align: center; }
    .readings-label { font-size: 11px; color: #64748b; letter-spacing: 2px; margin-bottom: 6px;
                       font-weight: 600; }
    .readings-value { font-size: 38px; font-weight: bold; color: #39ff14; font-family: Consolas, monospace;
                       line-height: 1; }
    .readings-source { font-size: 11px; margin-top: 6px; letter-spacing: 1px; }
    .source-live { color: #22c55e; } .source-estimate { color: #f59e0b; }
    .state-readout { font-size: 14px; color: #cbd5e1; margin-top: 14px; letter-spacing: 1px;
                      font-weight: 600; }
    .brake-row { font-size: 11.5px; color: #94a3b8; margin-top: 8px; letter-spacing: 0.5px; }
    .brake-engaged { color: #ef4444; font-weight: 700; } .brake-released { color: #22c55e; font-weight: 700; }

    .action-row { display: flex; gap: 10px; }

    #toast-container { position: fixed; top: 90px; right: 20px; z-index: 2000; }
    .toast { background: #22c55e; color: white; padding: 14px 24px; border-radius: 10px; margin-bottom: 10px;
             box-shadow: 0 10px 15px -3px rgba(0, 0, 0, 0.4); border-left: 5px solid #14532d;
             animation: slideIn 0.4s ease-out, fadeOut 0.5s 2.5s forwards; font-size: 13.5px; }
    @keyframes slideIn { from { transform: translateX(100%); opacity: 0; } to { transform: translateX(0); opacity: 1; } }
    @keyframes fadeOut { from { opacity: 1; } to { opacity: 0; } }
</style>
</head>
<body>

<div id="toast-container"></div>

<div class="app-shell">

    <!-- ===================== STICKY TOP BAR ===================== -->
    <div class="top-bar">
        <div class="top-bar-left">
            <h1>&#9881; Single Axis APU Control</h1>
            <div class="conn-indicator"><span id="conn_dot" class="conn-dot conn-no"></span>CONNECTION</div>
        </div>
        <button class="stop-btn-header" onclick="doAction('/stop')">&#9940; STOP (Brake)</button>
    </div>

    <div class="dashboard-grid">

        <!-- ===================== LEFT: CONTROLS ===================== -->
        <div class="col-stack">

            <div class="card">
                <div class="card-title">COM Port</div>
                <div class="config-row">
                    <select id="port_select"></select>
                    <button class="refresh-btn" onclick="refreshPorts()">&#8635;</button>
                    <button class="apply-btn" onclick="connectPort()">Connect</button>
                </div>
            </div>

            <div class="card">
                <div class="card-title">Acceleration / Deceleration</div>
                <div class="config-row">
                    <label>Acceleration (0 = sudden start)</label>
                    <input type="number" id="accel_input" min="0" step="1" value="0">
                </div>
                <div class="config-row">
                    <label>Deceleration (0 = sudden stop)</label>
                    <input type="number" id="decel_input" min="0" step="1" value="0">
                </div>
                <div class="config-row" style="justify-content: flex-end;">
                    <button class="apply-btn" onclick="applyAccelDecel()">Apply</button>
                </div>
            </div>

            <div class="card">
                <div class="card-title">Manual Jog</div>
                <div class="config-row">
                    <label>Speed (RPM)</label>
                    <input type="number" id="manual_speed_input" min="0.1" step="1" value=\"""" + str(DEFAULT_MANUAL_SPEED_RPM) + """\">
                </div>
                <div class="config-row" style="gap: 10px;">
                    <button id="manual_back_btn" class="manual-btn" onclick="doManual('backward')">&#9664; REVERSE</button>
                    <button id="manual_fwd_btn" class="manual-btn" onclick="doManual('forward')">FORWARD &#9654;</button>
                </div>
            </div>

        </div>

        <!-- ===================== RIGHT: GAUGE + READINGS + ACTIONS ===================== -->
        <div class="card visual-card">

            <div class="visual-row">
                <div class="gauge-card">
                    <svg width="220" height="220" viewBox="0 0 260 260">
                        """ + GAUGE_SVG + """
                        <line id="needle" class="gauge-needle" x1="130" y1="130" x2="130" y2="40"
                              stroke="#39ff14" stroke-width="4" stroke-linecap="round"/>
                        <circle cx="130" cy="130" r="6" fill="#39ff14"/>
                    </svg>
                    <div class="gauge-readout"><span id="readout">0.0&deg;</span></div>
                </div>

                <div class="readings-panel">
                    <div class="readings-label">READINGS</div>
                    <div class="readings-value"><span id="live_readout">0.00</span>&deg;</div>
                    <div class="readings-source" id="readings_source">estimate</div>
                    <div class="state-readout" id="state_readout">NOT HOMED</div>
                    <div class="brake-row">BRAKE: <span id="brake_status" class="brake-engaged">ENGAGED</span></div>
                </div>
            </div>

            <div class="action-row">
                <button id="home_btn" class="home-btn" onclick="doAction('/home')">&#127968; HOME</button>
                <button id="minus_btn" class="minus-btn" onclick="doAction('/minus180')" disabled>&#8592; -180&deg;</button>
                <button id="plus_btn" class="plus-btn" onclick="doAction('/plus180')" disabled>+180&deg; &#8594;</button>
            </div>

        </div>

    </div>
</div>

<script>
    async function doAction(url) {
        try { await fetch(url, { method: 'POST' }); } catch (e) {}
    }

    async function doManual(direction) {
        const rpm = parseFloat(document.getElementById('manual_speed_input').value) || 1;
        try {
            await fetch('/manual/' + direction, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ rpm: rpm })
            });
        } catch (e) {}
    }

    async function refreshPorts() {
        try {
            const res = await fetch('/list_ports');
            const data = await res.json();
            const sel = document.getElementById('port_select');
            const current = sel.value;
            sel.innerHTML = '';
            data.ports.forEach(p => {
                const opt = document.createElement('option');
                opt.value = p.device;
                opt.innerText = p.device + (p.description ? ' - ' + p.description : '');
                sel.appendChild(opt);
            });
            if (data.current_port) {
                sel.value = data.current_port;
            } else if (current) {
                sel.value = current;
            }
        } catch (e) {}
    }

    async function connectPort() {
        const port = document.getElementById('port_select').value;
        if (!port) return;
        try {
            await fetch('/connect', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ port: port })
            });
        } catch (e) {}
    }

    async function applyAccelDecel() {
        const accel = parseFloat(document.getElementById('accel_input').value) || 0;
        const decel = parseFloat(document.getElementById('decel_input').value) || 0;
        try {
            await fetch('/set_accel_decel', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ accel: accel, decel: decel })
            });
        } catch (e) {}
    }

    setInterval(async () => {
        try {
            const res = await fetch('/get_notifications');
            const data = await res.json();
            if (data.message) {
                const container = document.getElementById('toast-container');
                const toast = document.createElement('div');
                toast.className = 'toast';
                toast.innerHTML = `<strong>${data.message}</strong>`;
                container.appendChild(toast);
                setTimeout(() => toast.remove(), 3200);
            }
        } catch (e) {}
    }, 800);

    setInterval(async () => {
        try {
            const res = await fetch('/get_status');
            const data = await res.json();

            const needle = document.getElementById('needle');
            const readout = document.getElementById('readout');
            const liveReadout = document.getElementById('live_readout');
            const readingsSource = document.getElementById('readings_source');
            const stateEl = document.getElementById('state_readout');
            const homeBtn = document.getElementById('home_btn');
            const plusBtn = document.getElementById('plus_btn');
            const minusBtn = document.getElementById('minus_btn');
            const manualFwdBtn = document.getElementById('manual_fwd_btn');
            const manualBackBtn = document.getElementById('manual_back_btn');
            const connDot = document.getElementById('conn_dot');
            const brakeStatus = document.getElementById('brake_status');

            // Needle + both readouts are driven by the SAME live value
            // from the controller, so they always agree with each other
            // and with the real shaft.
            if (needle) needle.style.transform = `rotate(${data.position}deg)`;
            if (readout) readout.innerText = data.position.toFixed(1) + '\u00b0';
            if (liveReadout) liveReadout.innerText = data.position.toFixed(2);
            if (readingsSource) {
                readingsSource.innerText = data.position_is_live ? 'live (encoder)' : 'estimate';
                readingsSource.className = 'readings-source ' + (data.position_is_live ? 'source-live' : 'source-estimate');
            }
            if (stateEl) stateEl.innerText = data.is_homed ? data.state : (data.state === 'MANUAL' ? 'MANUAL' : 'NOT HOMED');
            if (connDot) connDot.className = 'conn-dot ' + (data.connected ? 'conn-yes' : 'conn-no');
            if (brakeStatus) {
                brakeStatus.innerText = data.brake_engaged ? 'ENGAGED' : 'RELEASED';
                brakeStatus.className = data.brake_engaged ? 'brake-engaged' : 'brake-released';
            }

            if (homeBtn) homeBtn.disabled = data.busy || !data.connected;
            if (plusBtn) plusBtn.disabled = !data.is_homed || data.busy || data.state === 'PLUS180';
            if (minusBtn) minusBtn.disabled = !data.is_homed || data.busy || data.state === 'MINUS180';
            if (manualFwdBtn) manualFwdBtn.disabled = data.busy || !data.connected;
            if (manualBackBtn) manualBackBtn.disabled = data.busy || !data.connected;
        } catch (e) {}
    }, 250);

    refreshPorts();
</script>
</body>
</html>
"""
def build_app(controller: SingleAxisController):
    app = Flask(__name__)
    c = controller

    @app.route("/")
    def index():
        return render_template_string(MAIN_HTML)

    @app.route("/get_status")
    def get_status():
        return jsonify(c.get_status())

    @app.route("/get_notifications")
    def get_notifications():
        msg = c.latest_notification
        c.latest_notification = None
        return jsonify({"message": msg})

    @app.route("/list_ports")
    def list_ports_route():
        ports = [{"device": dev, "description": desc} for dev, desc in list_serial_ports()]
        return jsonify({"ports": ports, "current_port": c.port})

    @app.route("/connect", methods=["POST"])
    def connect_route():
        data = request.get_json(force=True, silent=True) or {}
        port = data.get("port")
        if not port:
            return jsonify({"ok": False, "error": "no port given"}), 400
        ok = c.reconnect(port)
        return jsonify({"ok": ok, "port": c.port, "connected": c.available})

    @app.route("/set_accel_decel", methods=["POST"])
    def set_accel_decel_route():
        data = request.get_json(force=True, silent=True) or {}
        accel = data.get("accel", 0)
        decel = data.get("decel", 0)
        c.set_accel_decel(accel, decel)
        return jsonify({"ok": True, "accel": c.jog_accel, "decel": c.jog_decel})

    @app.route("/home", methods=["POST"])
    def home_action():
        c.start_homing()
        return jsonify({"ok": True})

    @app.route("/plus180", methods=["POST"])
    def plus180():
        c.go_to_target(TARGET_ANGLE)
        return jsonify({"ok": True})

    @app.route("/minus180", methods=["POST"])
    def minus180():
        c.go_to_target(-TARGET_ANGLE)
        return jsonify({"ok": True})

    @app.route("/manual/forward", methods=["POST"])
    def manual_forward():
        data = request.get_json(force=True, silent=True) or {}
        rpm = data.get("rpm", c.manual_speed_rpm)
        c.manual_jog("forward", rpm)
        return jsonify({"ok": True})

    @app.route("/manual/backward", methods=["POST"])
    def manual_backward():
        data = request.get_json(force=True, silent=True) or {}
        rpm = data.get("rpm", c.manual_speed_rpm)
        c.manual_jog("backward", rpm)
        return jsonify({"ok": True})

    @app.route("/stop", methods=["POST"])
    def stop_action():
        c.stop_all()
        return jsonify({"ok": True})

    return app


def main():
    parser = argparse.ArgumentParser(description="Single-axis APU control (homing + +/-180 + manual jog)")
    parser.add_argument("--port", default=None,
                         help="Optional: connect to this serial port immediately on startup. "
                              "Normally left unset -- pick the port from the web UI instead.")
    parser.add_argument("--baudrate", type=int, default=9600)
    parser.add_argument("--webport", type=int, default=5000, help="Web UI port")
    args = parser.parse_args()

    controller = SingleAxisController(port=args.port, baudrate=args.baudrate)
    app = build_app(controller)

    print(f"[OK] Starting web UI at http://localhost:{args.webport}")
    app.run(host="0.0.0.0", port=args.webport)


if __name__ == "__main__":
    main()