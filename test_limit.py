#!/usr/bin/env python3
"""
test_limit.py -- standalone limit-switch diagnostic.

Does ONE thing: connects to a drive and prints its raw Digital Input
Status register (40006) twice a second, forever. No Flask, no
joystick, no motor commands -- just the read, so you can isolate
whether the problem is the Modbus read itself or something in the
rest of the app.

USAGE:
    python test_limit.py --port COM4
    python test_limit.py --port COM5

WHAT TO DO:
    1. Run it against COM4 (azimuth). Leave the switches untouched
       for a few seconds -- note the resting value (e.g. 0x0000 or
       0x0003 or whatever it settles on).
    2. Physically trigger ONLY X1 on that drive, hold it, watch the
       value change, then release.
    3. Physically trigger ONLY X2 the same way.
    4. Repeat against COM5 (elevation).
    5. Send me the printed lines from all of that -- that tells us
       exactly which bit is X1, which is X2, and whether triggering
       makes the bit go 0->1 or 1->0.
"""

import argparse
import inspect
import time

from pymodbus.client import ModbusSerialClient

DIGITAL_INPUT_STATUS_REGISTER = 40006


def detect_id_kwarg():
    try:
        sig = inspect.signature(ModbusSerialClient.read_holding_registers)
        for name in ("device_id", "slave", "unit"):
            if name in sig.parameters:
                return name
    except Exception:
        pass
    return "slave"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True, help="e.g. COM4 or COM5")
    parser.add_argument("--baudrate", type=int, default=9600)
    args = parser.parse_args()

    id_kwarg = detect_id_kwarg()
    print(f"pymodbus id keyword detected as: {id_kwarg}")

    client = ModbusSerialClient(port=args.port, baudrate=args.baudrate, timeout=1)
    if not client.connect():
        print(f"FAILED to connect on {args.port}")
        return
    print(f"Connected on {args.port}. Reading register {DIGITAL_INPUT_STATUS_REGISTER} twice a second. Ctrl+C to stop.\n")

    offset = DIGITAL_INPUT_STATUS_REGISTER - 40001
    last_value = None

    while True:
        try:
            rr = client.read_holding_registers(offset, count=1, **{id_kwarg: 1})
            if rr is None or rr.isError():
                print(f"READ ERROR: {rr}")
            else:
                value = rr.registers[0]
                marker = "  <-- CHANGED" if value != last_value else ""
                bits = format(value, "016b")
                print(f"IS = 0x{value:04X}  binary={bits}  "
                      f"bit0(X1)={value & 1}  bit1(X2)={(value >> 1) & 1}{marker}")
                last_value = value
        except Exception as e:
            print(f"EXCEPTION: {e}")
        time.sleep(0.5)


if __name__ == "__main__":
    main()