# Single-Axis APU Control

Web-based control panel for a single azimuth drive (MOONS MBDV-520-AC, Modbus RTU).
Features: homing, fixed +180 / -180 moves, manual jog, live angle gauge and readout.

## Hardware required
- MOONS MBDV-520-AC drive connected over a serial (COM) port
- Home sensor on input X1

## Install
## Run
Then open http://localhost:5000 in a browser.

Options: `--port` (COM port), `--baudrate` (default 9600), `--webport` (default 5000).

## SAFETY WARNING
This software moves a physical motor. Keep the area clear before sending any
command. The red **STOP** button at the top of the page halts all motion immediately.
Do not expose this app to the public internet: it has no login.

## Files
- `app.py` - Flask web UI
- `single_axis_controller.py` - motor control logic
- `test_limit.py` - limit-switch diagnostic tool
- `configs.json` - saved motion configurations
