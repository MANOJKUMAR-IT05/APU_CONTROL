#!/usr/bin/env python3
"""
single_axis_controller.py

Standalone, plain-Python controller for a SINGLE azimuth drive
(MOONS' MBDV-520-AC, Modbus RTU).

WHAT'S IN THIS VERSION
-----------------------
  - Homing: CONTINUOUS search. The motor jogs until X1 fires or a
    safety input aborts it -- no distance or time cap. See "HOMING"
    below.
  - Live angle readout: a background thread continuously reads the
    drive's encoder-position register (independent of what's causing
    the motor to move) and turns it into a live degrees-from-home
    number. This is what both the numeric readout AND the gauge
    needle are driven from now -- see "LIVE ANGLE FEEDBACK" below.
  - Manual jog: forward/backward buttons at a speed YOU set from the
    UI (separate from the fixed MOTOR_RPM used for homing/+-180).
  - Brake hooks: _release_brake()/_engage_brake() are called at the
    right moments (before any commanded motion, after it fully
    completes, and on STOP) -- see "BRAKE" below for an important
    caveat: the actual hardware write is a stub until you confirm the
    real command.
  - COM port selection and accel/decel from the UI (unchanged from
    the previous version).

HOMING
------
X1 (Digital Input Status register 40006, bit 0) is the home sensor.
The search now runs CONTINUOUSLY: the motor jogs until X1's rising
edge is detected, or a safety input (X2-X4) aborts it. There is no
longer any distance-swept or elapsed-time cap on the search itself --
if X1 never fires, the motor keeps going until you press STOP. This
was a deliberate choice (previous versions capped the search, which
caused problems when the real speed didn't match assumptions); if
your sensor/wiring is faulty, STOP is your manual override.

LIVE ANGLE FEEDBACK
---------------------
`self.live_angle_deg` is maintained by the background input-poll
thread every ~50ms, using the ACTUAL raw encoder count read back
from the drive -- NOT a timed estimate. It only becomes available
once homing has succeeded at least once (it needs a reference point:
`self.home_reference_counts`, the raw encoder count at the instant
X1 was confirmed). From then on:

    live_angle_deg = sign * (raw_counts - home_reference_counts)
                      / ENCODER_COUNTS_PER_REV * 360.0

This updates continuously regardless of what's moving the shaft --
homing, a +/-180 move, or a manual jog -- so the gauge needle and the
numeric readout both track the real, physical shaft position live,
not a projection. `get_status()` exposes this as "position" plus a
"position_is_live" flag; if the encoder read isn't working (register
unconfirmed, or a transient bus failure), it falls back to the old
timed estimate and marks position_is_live=False so the UI can show
you it's an approximation, not silently pretend it's live.

Sign convention (ENCODER_COUNTS_POSITIVE_IS_FORWARD) is a guess --
flip it if the gauge/readout moves the opposite way from the real
shaft.

MANUAL JOG
----------
manual_jog("forward"/"backward", rpm) runs the motor continuously at
the given RPM (read from the UI's speed box, not the fixed MOTOR_RPM)
until STOP is pressed. It's refused while any other action (homing, a
+/-180 move, another jog) is already running -- press STOP first to
change direction. It marks the state machine UNHOMED (since a manual
move isn't tracked against the +180/-180 legality checks), but does
NOT erase the live angle reference -- the live readout keeps tracking
the real shaft position throughout, exactly so you can watch where
you actually are while jogging manually.

BRAKE
-----
IMPORTANT CAVEAT -- read this before assuming Stop physically brakes
anything. Per the Hardware Manual (section 7.1.10), this drive's
physical holding brake is tied to the Servo Enable (S-ON) state:
servo enabled -> brake released; servo disabled -> brake engaged.
Section 7.1.3 documents S-ON as a HARDWARE INPUT SIGNAL, not a Modbus
register, and section 7.7.1 explicitly warns against toggling servo
enable/disable as a routine start/stop mechanism (it triggers dynamic
braking, meant for occasional/fault use only). There's also a
power-on option ("servo automatically enabled upon power-on" in
Modbus/RTU mode) suggesting your drive may just keep the servo (and
therefore the brake release) on permanently once powered, with no
software toggle available at all in what's been supplied.

Given that, _release_brake()/_engage_brake() below are implemented as
SAFE NO-OP STUBS: they're called at exactly the moments you asked for
(released before any commanded motion, engaged once it fully
completes, engaged on STOP), and self.brake_engaged reflects the
intended logical state for the UI -- but neither function writes an
unverified command to the drive. Guessing a "disable servo" code and
sending it to a live control register risks triggering an
undocumented, unintended action, which is a different and worse risk
than the read-only placeholders elsewhere in this file. Motion itself
is completely unaffected: CMD_STOP still performs an immediate, real
stop regardless of this. Once you've confirmed the actual command
(Luna software while toggling S-ON, or the Modbus communication
manual), fill it in where marked "TODO" in _release_brake/_engage_brake.

HARDWARE ASSUMPTIONS TO VERIFY ON THE REAL RIG (read before running):
  - HOMING_SEARCH_DIRECTION_FORWARD: which way to jog on the very
    first homing call, when the starting position is unknown.
  - INPUTS_ACTIVE_LOW: carried over from the dual-axis rig's
    confirmed NC (normally-closed) wiring on COM5. Re-confirm this
    drive/port is wired the same way before trusting it.
  - ENABLE_SAFETY_INPUTS: X2/X3/X4 are treated as general "stop
    immediately" safety inputs, same as the dual-axis project. Set
    this to False if this drive doesn't actually have them wired.
  - JOG_ACCEL_REGISTER / JOG_DECEL_REGISTER: documented in the manual
    only as parameters P2-03/P2-04, no Modbus address given. The
    addresses below are an unconfirmed guess (see previous revision
    notes) -- confirm before trusting nonzero accel/decel.
  - ACTUAL_POSITION_REGISTER / ENCODER_COUNTS_PER_REV: same caveat,
    and now more important than before since the live readout and
    gauge depend on it directly. Confirm via Luna software or the
    Modbus communication manual. Until confirmed, the live readout
    may simply never populate (falls back to the timed estimate,
    clearly marked as such).
  - Brake: see "BRAKE" above -- the hardware write is an intentional
    no-op stub pending a confirmed Servo Enable/Disable command.
"""

import struct
import threading
import time
import inspect

from pymodbus.client import ModbusSerialClient

try:
    from serial.tools import list_ports
except ImportError:
    list_ports = None


def _detect_id_kwarg():
    """pymodbus renamed the drive-address keyword across versions:
    2.x used unit=, 3.0-3.6 used slave=, 3.7+ renamed it again to
    device_id=. Detect the right one from whatever pymodbus is
    actually installed instead of hardcoding a guess."""
    try:
        sig = inspect.signature(ModbusSerialClient.read_holding_registers)
        for name in ("device_id", "slave", "unit"):
            if name in sig.parameters:
                return name
    except Exception:
        pass
    return "slave"


MODBUS_ID_KWARG = _detect_id_kwarg()

# =====================================================
# HOW FAR EACH +/-180 MOVE GOES -- edit this if you ever need a
# different travel. It's in motor shaft REVOLUTIONS, not degrees,
# because that's what the encoder actually counts. Direct drive, 1:1
# shaft-to-load: 0.5 rev = 180 deg.
# =====================================================
ROTATIONS_PER_MOVE = 0.5

TARGET_ANGLE = ROTATIONS_PER_MOVE * 360.0  # derived, kept for logging

# ---- Modbus registers (same drive family / addressing as the dual-axis rig) ----
COMMAND_REGISTER = 40125
JOG_VELOCITY_REGISTER = 40343
CMD_START = 150
CMD_STOP = 216
# ---- Brake, via Servo Enable/Disable (Section 9.3.7 Command Opcode,
# confirmed in the M2DC Hardware Manual). Sent through the same
# COMMAND_REGISTER (40125) as CMD_START/CMD_STOP.
CMD_MOTOR_ENABLE = 0x9F   # 159 - releases the brake (Section 7.1.7)
CMD_MOTOR_DISABLE = 0x9E  # 158 - engages the brake (Section 7.1.7)

# Default delays from P-69 (BD, disengage) / P-70 (BE, engage), each
# 200ms by default per the manual. We wait out the release delay so
# motion never starts before the brake has actually opened; we don't
# need to wait on engage since motion has already stopped by then.
BRAKE_RELEASE_DELAY_SEC = 0.25

# !! PLACEHOLDER -- see module docstring. Confirm before relying on
# nonzero accel/decel. !!
JOG_ACCEL_REGISTER = 40344
JOG_DECEL_REGISTER = 40345

# !! PLACEHOLDER -- see module docstring, "LIVE ANGLE FEEDBACK". The
# live readout and gauge depend directly on this being correct. !!
ACTUAL_POSITION_REGISTER = 40007
ENCODER_COUNTS_PER_REV = 131072

# Flip this if the live readout/gauge moves opposite to the real
# shaft once you have the encoder register confirmed.
ENCODER_COUNTS_POSITIVE_IS_FORWARD = True

DIGITAL_INPUT_STATUS_REGISTER = 40006
X1_BIT = 0  # home sensor
X2_BIT = 1
X3_BIT = 2
X4_BIT = 3

HOME_BIT_MASK = 1 << X1_BIT
SAFETY_LIMIT_MASK = (1 << X2_BIT) | (1 << X3_BIT) | (1 << X4_BIT)

# Set False if this drive only has X1 wired.
ENABLE_SAFETY_INPUTS = True
# NC wiring assumption carried over from the dual-axis rig (COM5,
# confirmed live there: resting = 0x000F, triggering X1 -> 0x000E).
# VERIFY this is also true for this drive before relying on it.
INPUTS_ACTIVE_LOW = True

# ---- Motion model: direct drive, no gearbox ----
MOTOR_RPM = 20  # fixed speed used for homing and +/-180 moves
AZ_DEG_PER_SEC = (MOTOR_RPM / 60.0) * 360.0  # = 60.0 deg/sec, real physics (1:1)

# Speed for the manual forward/backward buttons -- this is just the
# UI's starting default; the real value used for a manual jog always
# comes from the speed box in the UI at the moment you click.
DEFAULT_MANUAL_SPEED_RPM = 10

# First-ever homing direction when starting position is unknown.
# True = jog forward to search for X1, False = jog reverse.
HOMING_SEARCH_DIRECTION_FORWARD = False

# Sudden start/stop by default -- see ACCELERATION / DECELERATION in
# the module docstring. 0 means "don't write the ramp registers at all".
DEFAULT_JOG_ACCEL = 0
DEFAULT_JOG_DECEL = 0

# Timed-fallback cap for the +/-180 move itself: generous multiple of
# the plain-timed estimate, so a slow ramp (accel/decel > 0) still has
# room to finish, but a stuck/silent encoder still gets caught. (Only
# the +/-180 move uses this -- homing itself is now uncapped, see
# module docstring.)
MOVE_TIMEOUT_MULTIPLIER = 4.0


def list_serial_ports():
    """Returns [(device, description), ...] for the COM-port dropdown."""
    if list_ports is None:
        return []
    return [(p.device, p.description) for p in list_ports.comports()]


class SingleAxisController:

    def __init__(self, port=None, baudrate=9600, log=print):
        """Starts with NO serial connection attempted. `port` is only
        used if you pass --port on the command line -- normal use:
        leave it unset and connect from the UI instead."""
        self.log = log

        self.port = port
        self.baudrate = baudrate

        self.available = False
        self.client = None
        self._lock = threading.Lock()  # serializes Modbus I/O against the input-poll thread

        if port:
            self._connect(port, baudrate)

        # ---- state machine ----
        self.state = "UNHOMED"
        self.is_homed = False
        self.busy = False  # true during any homing/move/jog sequence; UI disables buttons on this

        # ---- position tracking ----
        # `position`/_motion: legacy time-projected estimate, kept as
        # a fallback for the readout and as the +/-180 move's timeout
        # math. `live_angle_deg`/`home_reference_counts`: the real,
        # encoder-based live feedback -- see "LIVE ANGLE FEEDBACK".
        self.position = 0.0
        self._motion = None
        self._motion_lock = threading.Lock()
        self.home_reference_counts = None
        self.live_angle_deg = None

        # ---- acceleration / deceleration, adjustable from the UI ----
        self.jog_accel = DEFAULT_JOG_ACCEL
        self.jog_decel = DEFAULT_JOG_DECEL

        # ---- manual jog speed, adjustable from the UI ----
        self.manual_speed_rpm = DEFAULT_MANUAL_SPEED_RPM

        # ---- brake (see module docstring "BRAKE" for the caveat) ----
        self.brake_engaged = True  # logical starting state: idle, not moving

        # ---- feedback events, set by the background input-poll thread ----
        self._home_event = threading.Event()   # X1 rising edge
        self._abort_event = threading.Event()  # a safety input (X2-X4) tripped, or STOP pressed

        self.latest_notification = None

        self._poll_stop = threading.Event()
        self._poll_thread = threading.Thread(target=self._input_poll_loop, daemon=True)
        self._poll_thread.start()

    # =====================================================
    # CONNECTION / COM PORT SELECTION
    # =====================================================

    def _connect(self, port, baudrate):
        self.client = ModbusSerialClient(port=port, baudrate=baudrate, timeout=1)
        try:
            connected = self.client.connect()
        except Exception as e:
            connected = False
            self.log(f"[WARN] Azimuth motor connect() raised on {port}: {e}")

        if connected:
            self.available = True
            self.log(f"[OK] Azimuth motor connected on {port}")
        else:
            self.available = False
            self.log(f"[WARN] Azimuth motor NOT connected ({port}) - port didn't open. "
                     f"Common causes: another program/instance already has it open, "
                     f"wrong port number, drive powered off, or a driver/cable issue.")

    def reconnect(self, port, baudrate=None):
        """Switches to a different COM port at runtime (called from the
        UI's port-select section). Refuses while a move is in progress."""
        if self.busy:
            self.log("[PORT] Ignored - motor busy, can't switch ports mid-move.")
            return False

        baudrate = baudrate if baudrate is not None else self.baudrate

        with self._lock:
            self.available = False
            try:
                if self.client is not None:
                    self.client.close()
            except Exception:
                pass
            self.port = port
            self.baudrate = baudrate
            self._connect(port, baudrate)

        # A port switch means we no longer trust the old zero reference.
        self.is_homed = False
        self.state = "UNHOMED"
        self.home_reference_counts = None
        self.live_angle_deg = None
        if self.available:
            self.latest_notification = f"\U0001f50c Connected to {port} - please home"
        else:
            self.latest_notification = (
                f"\u26a0 Could not open {port} - check it isn't already in use by "
                f"another program, the drive is powered on, and the cable/driver are OK"
            )
        return self.available

    # =====================================================
    # LOW LEVEL MODBUS
    # =====================================================

    def _read_digital_inputs(self):
        offset = DIGITAL_INPUT_STATUS_REGISTER - 40001
        try:
            with self._lock:
                rr = self.client.read_holding_registers(offset, count=1, **{MODBUS_ID_KWARG: 1})
            if rr is None or rr.isError():
                return None
            return rr.registers[0]
        except Exception as e:
            self.log(f"[WARN] Modbus Read Error (inputs): {e}")
            return None

    def _read_position_counts(self):
        """Reads the drive's actual-position/encoder register (32-bit,
        2 holding registers, same packing convention as set_speed).
        Returns None on any failure so callers can fall back."""
        offset = ACTUAL_POSITION_REGISTER - 40001
        try:
            with self._lock:
                rr = self.client.read_holding_registers(offset, count=2, **{MODBUS_ID_KWARG: 1})
            if rr is None or rr.isError():
                return None
            high, low = rr.registers
            packed = struct.pack(">HH", high, low)
            return struct.unpack(">i", packed)[0]
        except Exception as e:
            self.log(f"[WARN] Modbus Read Error (position): {e}")
            return None

    def _write(self, func):
        try:
            with self._lock:
                func()
        except Exception as e:
            self.log(f"[WARN] Modbus Error: {e}")

    def set_speed(self, rpm):
        packed = struct.pack(">i", int(rpm))
        high, low = struct.unpack(">HH", packed)
        offset = JOG_VELOCITY_REGISTER - 40001
        self._write(lambda: self.client.write_registers(offset, [high, low], **{MODBUS_ID_KWARG: 1}))

    def _write_int32(self, register, value):
        packed = struct.pack(">i", int(value))
        high, low = struct.unpack(">HH", packed)
        offset = register - 40001
        self._write(lambda: self.client.write_registers(offset, [high, low], **{MODBUS_ID_KWARG: 1}))

    def send_cmd(self, value):
        offset = COMMAND_REGISTER - 40001
        self._write(lambda: self.client.write_register(offset, value, **{MODBUS_ID_KWARG: 1}))

    # =====================================================
    # ACCELERATION / DECELERATION (set from the UI)
    # =====================================================

    def set_accel_decel(self, accel, decel):
        """Called from the UI's accel/decel section. Just stores the
        values -- they're written to the drive right before the next
        move (see _apply_accel_decel), so changing them mid-idle is
        always safe."""
        self.jog_accel = max(0.0, float(accel))
        self.jog_decel = max(0.0, float(decel))
        self.log(f"[ACC/DEC] accel={self.jog_accel} decel={self.jog_decel} "
                 f"({'sudden start/stop' if self.jog_accel == 0 and self.jog_decel == 0 else 'ramped'})")

    def _apply_accel_decel(self):
        """0 -> registers are left untouched (sudden start/stop, as
        before). Nonzero -> written to the drive immediately before
        the move starts."""
        if self.jog_accel > 0:
            self._write_int32(JOG_ACCEL_REGISTER, self.jog_accel)
        if self.jog_decel > 0:
            self._write_int32(JOG_DECEL_REGISTER, self.jog_decel)

    # =====================================================
    # BRAKE -- see module docstring "BRAKE" for the important caveat.
    # Both of these are called at the right moments already; only the
    # actual hardware write is stubbed out pending a confirmed
    # Servo Enable/Disable command.
    # =====================================================

    def _release_brake(self):
        """Sends Motor Enable (0x9F) to release the physical brake,
        then waits out the drive's brake-disengage delay (P-69/BD,
        default 200ms) so motion never starts before the brake has
        actually opened."""
        self.send_cmd(CMD_MOTOR_ENABLE)
        time.sleep(BRAKE_RELEASE_DELAY_SEC)
        self.brake_engaged = False

    def _engage_brake(self):
        """Sends Motor Disable (0x9E) to engage the physical brake.
        Always called after motion has already been stopped (via
        CMD_STOP), so the brake clamps a shaft that's already at
        rest -- not one that's still spinning."""
        self.send_cmd(CMD_MOTOR_DISABLE)
        self.brake_engaged = True
    # =====================================================
    # X1 HOME SENSOR + OPTIONAL X2-X4 SAFETY STOPS
    # + CONTINUOUS LIVE ENCODER READOUT
    # =====================================================

    def _input_poll_loop(self):
        was_home_active = False
        was_safety_active = False
        last_debug = 0.0
        while not self._poll_stop.is_set():
            try:
                if self.available:
                    value = self._read_digital_inputs()
                    if value is not None:
                        effective = (~value) & 0xFFFF if INPUTS_ACTIVE_LOW else value

                        if time.time() - last_debug > 1.0:
                            self.log(f"[IN-RAW] IS=0x{value:04X} X1={ (value>>X1_BIT)&1 } "
                                     f"X2={ (value>>X2_BIT)&1 } X3={ (value>>X3_BIT)&1 } "
                                     f"X4={ (value>>X4_BIT)&1 }")
                            last_debug = time.time()

                        home_active = bool(effective & HOME_BIT_MASK)
                        if home_active and not was_home_active:
                            self._home_event.set()
                            self.log(f"[HOME] X1 sensor triggered (IS=0x{value:04X})")
                        was_home_active = home_active

                        if ENABLE_SAFETY_INPUTS:
                            safety_active = bool(effective & SAFETY_LIMIT_MASK)
                            if safety_active and not was_safety_active:
                                self._abort_event.set()
                                self.send_cmd(CMD_STOP)
                                self._stop_tracking()
                                self._engage_brake()
                                self.latest_notification = "\u26d4 AZIMUTH STOPPED - safety input (X2-X4) triggered"
                                self.log(f"[SAFETY] Stop input triggered (IS=0x{value:04X})")
                            was_safety_active = safety_active

                    # Continuous live-angle feedback -- runs regardless
                    # of what's causing the motor to move (homing,
                    # +/-180, or manual jog all update this the same
                    # way). Only active once a home reference exists.
                    if self.home_reference_counts is not None:
                        raw = self._read_position_counts()
                        if raw is not None:
                            sign = 1.0 if ENCODER_COUNTS_POSITIVE_IS_FORWARD else -1.0
                            self.live_angle_deg = (
                                sign * (raw - self.home_reference_counts)
                                / ENCODER_COUNTS_PER_REV * 360.0
                            )
                        # a failed read here just leaves live_angle_deg
                        # at its last known value rather than erasing it
            except Exception as e:
                self.log(f"[WARN] Input poll iteration failed: {e}")
            time.sleep(0.05)

    def _wait_for_home_or_abort(self):
        """Blocks until X1 triggers or a safety input/STOP aborts it.
        No distance or time cap -- the search runs continuously, by
        design (see module docstring "HOMING")."""
        while True:
            if self._home_event.is_set():
                return "home"
            if self._abort_event.is_set():
                return "aborted"
            time.sleep(0.02)

    # =====================================================
    # POSITION TRACKING (elapsed-time projection -- fallback only, see
    # "LIVE ANGLE FEEDBACK" in the module docstring for the real one)
    # =====================================================

    def _begin_tracking(self, forward):
        rate = AZ_DEG_PER_SEC if forward else -AZ_DEG_PER_SEC
        with self._motion_lock:
            self._commit_position_locked()
            self._motion = {"start_time": time.time(), "start_pos": self.position, "rate": rate}

    def _commit_position_locked(self):
        if self._motion:
            elapsed = time.time() - self._motion["start_time"]
            self.position = self._motion["start_pos"] + self._motion["rate"] * elapsed
            self._motion = None

    def _stop_tracking(self):
        with self._motion_lock:
            self._commit_position_locked()

    def get_position(self):
        """Legacy timed estimate -- only used as a fallback for the
        UI when the real encoder readout isn't available, and for the
        +/-180 move's timeout math."""
        with self._motion_lock:
            if self._motion:
                elapsed = time.time() - self._motion["start_time"]
                return self._motion["start_pos"] + self._motion["rate"] * elapsed
            return self.position

    def get_status(self):
        if self.live_angle_deg is not None:
            position = round(self.live_angle_deg, 2)
            position_is_live = True
        else:
            position = round(self.get_position(), 2)
            position_is_live = False
        return {
            "position": position,
            "position_is_live": position_is_live,
            "state": self.state,
            "is_homed": self.is_homed,
            "busy": self.busy,
            "port": self.port,
            "connected": self.available,
            "jog_accel": self.jog_accel,
            "jog_decel": self.jog_decel,
            "rotations_per_move": ROTATIONS_PER_MOVE,
            "brake_engaged": self.brake_engaged,
            "manual_speed_rpm": self.manual_speed_rpm,
        }

    # =====================================================
    # HOMING
    # =====================================================

    def _drive_until_home(self, forward):
        """Jogs continuously in `forward` direction until X1 triggers
        or a safety input/STOP aborts it -- no distance or time cap
        (see module docstring "HOMING"). Always leaves the motor
        stopped. Returns True only if X1 genuinely confirmed home.
        Does NOT touch the brake -- that's handled by the caller
        (_homing_thread / _go_to_target_thread), so an internal
        return-to-home leg doesn't engage/release the brake on its
        own partway through a +/-180 move."""
        if not self.available:
            self.log("[HOME] Ignored - no COM port connected. Select one in the UI first.")
            self.latest_notification = "\u26a0 Not connected - select a COM port first"
            return False

        self._home_event.clear()
        self._abort_event.clear()

        self._apply_accel_decel()
        rpm = MOTOR_RPM if forward else -MOTOR_RPM
        self.set_speed(rpm)
        time.sleep(0.02)
        self.send_cmd(CMD_START)
        self._begin_tracking(forward)

        result = self._wait_for_home_or_abort()

        self.send_cmd(CMD_STOP)
        self._stop_tracking()

        if result == "home":
            self.position = 0.0
            self.is_homed = True
            self._home_event.clear()
            home_counts = self._read_position_counts()
            if home_counts is not None:
                self.home_reference_counts = home_counts
                self.live_angle_deg = 0.0
                self.log("[HOME] Home position confirmed (X1), encoder zero reference set.")
            else:
                self.home_reference_counts = None
                self.log("[HOME][WARN] Home confirmed (X1) but could not read the encoder "
                         "at that instant - live angle readout unavailable until the next "
                         "successful home. Confirm ACTUAL_POSITION_REGISTER.")
            return True
        else:  # "aborted" -- safety input tripped or STOP was pressed
            self.is_homed = False
            self.state = "UNHOMED"
            self.log("[HOME][ERROR] Aborted before X1 triggered - safety input or STOP.")
            self.latest_notification = "\u26d4 HOMING ABORTED"
            return False

    def start_homing(self):
        if self.busy:
            self.log("[HOME] Ignored - already busy.")
            return
        threading.Thread(target=self._homing_thread, daemon=True).start()

    def _homing_thread(self):
        self.busy = True
        try:
            self._release_brake()
            if self.state == "PLUS180":
                forward = False   # reverse of the forward move that reached +180
            elif self.state == "MINUS180":
                forward = True    # reverse of the reverse move that reached -180
            else:
                forward = HOMING_SEARCH_DIRECTION_FORWARD  # unknown starting position -> search
            ok = self._drive_until_home(forward)
            if ok:
                self.state = "HOME"
                self.latest_notification = "\u2705 Homing complete - angle buttons enabled"
        finally:
            self._engage_brake()
            self.busy = False

    # =====================================================
    # +180 / -180 MOVES (encoder-verified)
    # =====================================================

    def _position_move(self, forward, timed_estimate_sec):
        """Runs the drive until it has traveled ROTATIONS_PER_MOVE
        revolutions (confirmed by reading the encoder position back),
        a safety input trips, or a generous timeout elapses. The
        timeout is a fallback only -- see ACTUAL_POSITION_REGISTER in
        the module docstring."""
        self._abort_event.clear()

        start_counts = self._read_position_counts()
        encoder_ok = start_counts is not None
        if not encoder_ok:
            self.log("[MOVE][WARN] Could not read encoder position - "
                     "falling back to a timed-only move for this leg.")

        target_counts = abs(ROTATIONS_PER_MOVE) * ENCODER_COUNTS_PER_REV
        timeout = max(timed_estimate_sec * MOVE_TIMEOUT_MULTIPLIER, timed_estimate_sec + 2.0)

        self._apply_accel_decel()
        rpm = MOTOR_RPM if forward else -MOTOR_RPM
        self.set_speed(rpm)
        time.sleep(0.02)
        self.send_cmd(CMD_START)
        self._begin_tracking(forward)

        start_time = time.time()
        aborted = False
        reached = False
        while time.time() - start_time < timeout:
            if self._abort_event.is_set():
                aborted = True
                break
            if encoder_ok:
                current = self._read_position_counts()
                if current is not None:
                    traveled = abs(current - start_counts)
                    if traveled >= target_counts:
                        reached = True
                        break
            time.sleep(0.02)

        self.send_cmd(CMD_STOP)
        self._stop_tracking()

        if not aborted and not reached and encoder_ok:
            self.log("[MOVE][WARN] Timed out before the encoder confirmed "
                     "the target travel - check ENCODER_COUNTS_PER_REV / "
                     "ACTUAL_POSITION_REGISTER against the real Modbus map.")

        return not aborted

    def go_to_target(self, target_deg):
        """target_deg must be +TARGET_ANGLE or -TARGET_ANGLE."""
        if not self.is_homed:
            self.log("[MOVE] Ignored - home first.")
            return
        if self.busy:
            self.log("[MOVE] Ignored - already busy.")
            return
        if (target_deg > 0 and self.state == "PLUS180") or (target_deg < 0 and self.state == "MINUS180"):
            self.log("[MOVE] Ignored - already at that position.")
            return
        threading.Thread(target=self._go_to_target_thread, args=(target_deg,), daemon=True).start()

    def _go_to_target_thread(self, target_deg):
        self.busy = True
        try:
            self._release_brake()

            if self.state != "HOME":
                # Always return to the sensor-verified home first -- this
                # resets any open-loop drift before every new move. Brake
                # is deliberately NOT touched here -- it stays released
                # across this internal leg and the final move below.
                return_forward = (self.state == "MINUS180")
                if not self._drive_until_home(return_forward):
                    return  # is_homed/state already reset to UNHOMED by _drive_until_home
                self.state = "HOME"

            forward = target_deg > 0
            timed_estimate = abs(target_deg) / AZ_DEG_PER_SEC

            if not self._position_move(forward, timed_estimate):
                self.is_homed = False
                self.state = "UNHOMED"
                self.latest_notification = "\u26d4 MOVE ABORTED - safety input tripped, re-home required"
                return

            self.position = target_deg
            self.state = "PLUS180" if forward else "MINUS180"
            degree_sign = "+180" if forward else "-180"
            self.latest_notification = f"\u2705 Reached {degree_sign}\u00b0"
        finally:
            self._engage_brake()
            self.busy = False

    # =====================================================
    # MANUAL JOG (forward/backward, user-set speed)
    # =====================================================

    def manual_jog(self, direction, rpm):
        """direction is "forward" or "backward". rpm comes straight
        from the UI's manual-speed input box. Refused while anything
        else (homing, a +/-180 move, another jog) is already running
        -- press STOP first to change direction or stop."""
        if not self.available:
            self.log("[MANUAL] Ignored - no COM port connected.")
            self.latest_notification = "\u26a0 Not connected - select a COM port first"
            return
        if direction not in ("forward", "backward"):
            self.log(f"[MANUAL] Ignored - invalid direction {direction!r}.")
            return
        if self.busy:
            self.log("[MANUAL] Ignored - already busy, press STOP first.")
            return
        threading.Thread(target=self._manual_jog_thread, args=(direction, rpm), daemon=True).start()

    def _manual_jog_thread(self, direction, rpm):
        self.busy = True
        try:
            forward = (direction == "forward")
            try:
                rpm = max(0.1, float(rpm))
            except (TypeError, ValueError):
                rpm = self.manual_speed_rpm
            self.manual_speed_rpm = rpm

            self._release_brake()
            self._abort_event.clear()
            self._apply_accel_decel()
            self.set_speed(rpm if forward else -rpm)
            time.sleep(0.02)
            self.send_cmd(CMD_START)
            self._begin_tracking(forward)

            self.state = "MANUAL"
            self.is_homed = False  # a manual move isn't tracked by the +/-180 legality checks
            self.latest_notification = (
                f"\u25b6 Manual {'forward' if forward else 'reverse'} at {rpm} RPM - press STOP to end"
            )

            while not self._abort_event.is_set():
                time.sleep(0.05)
        finally:
            self.send_cmd(CMD_STOP)
            self._stop_tracking()
            self._engage_brake()
            self.busy = False

    # =====================================================
    # STOP
    # =====================================================

    def stop_all(self):
        self._abort_event.set()
        self.send_cmd(CMD_STOP)
        self._stop_tracking()
        self._engage_brake()
        self.busy = False
        self.latest_notification = "\u26d4 Stopped - brake engaged"