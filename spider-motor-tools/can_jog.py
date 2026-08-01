#!/usr/bin/env python3
"""Interactive CNC-style jog pendant for the ODrive joints over CAN.

Pick a node from the bus, jog it with the arrow keys in fixed angular steps
(default 5 deg), and capture the travel limits (min / max) at the poses you
jogged to.

  arrows / PgUp / PgDn    jog -/+ one step        + -   step size
  a arm      i idle       SPACE  EMERGENCY IDLE   q quit (always idles)
  [ set min  ] set max    c clear limits          w write limits file
  g goto min G goto max   t type target angle
  n / p next / prev node  e clear errors          r re-read limits file
  A apply limits to the board over CAN            S save board config to flash

Units are exactly what the node reports over `Get_Encoder_Estimates`: on a
split-feedback geared joint (the local `can_simple.cpp` patch, load_encoder_axis
!= this axis) that is TRUE JOINT/OUTPUT degrees with direction+zero applied; on a
plain node it is motor-shaft degrees. The tool neither knows nor needs the gear
ratio.

SAFETY (these joints have cooked a motor before -- see the repo notes):
  * never arms by itself: `a` is explicit, and the setpoint is seeded from the
    measured position first, so arming can never jump the joint
  * the setpoint is SLEWED to the jog target at --rate, not stepped
  * |Iq| over --iq-cap for a few samples in a row  -> auto IDLE
  * setpoint running away from the joint by more than --max-lag (a stall, e.g.
    jogging into a hard stop) -> jogging that way is blocked, 2x that -> IDLE
  * no keypress for --idle-timeout seconds while armed -> auto IDLE, because a
    geared joint holding against gravity is a CONTINUOUS-current duty that
    overheats the winding (most of these motors have no thermistor)
  * any fault, quit, Ctrl+C or exception -> IDLE

The captured min/max are stored in `configs/jog_limits.json`, which clamps this
tool's setpoint. On firmware >= 0.5.6 `A` pushes them into the board's real
endstops over CAN (`encoder.config.min_position` / `max_position`, via
can_config.py) and `S` commits them to flash -- no USB. On older firmware those
keys report that the board does not answer, and `w` still prints the odrivetool
snippet to apply them by hand.
"""
import argparse
import json
import os
import struct
import sys
import termios
import threading
import time
import tty

from can_goto import (find_bridge,
                      CMD_HEARTBEAT, CMD_GET_ENCODER_EST, CMD_GET_IQ,
                      CMD_SET_AXIS_STATE, CMD_SET_INPUT_POS,
                      CMD_SET_CONTROLLER_MODES, CMD_CLEAR_ERRORS,
                      AXIS_STATE_IDLE, AXIS_STATE_CLOSED_LOOP,
                      CONTROL_MODE_POSITION, INPUT_MODE_PASSTHROUGH,
                      STATE_NAMES)

import serial

CMD_GET_VBUS = 0x017
CMD_CONFIG_ACCESS = 0x01C   # local fw >= 0.5.6, see can_config.py
CMD_CONFIG_COMMIT = 0x01D

HERE = os.path.dirname(os.path.abspath(__file__))
LIMITS_PATH = os.path.join(HERE, "configs", "jog_limits.json")

JOINT_NAMES = {1: "coxa", 2: "femur", 3: "knee"}


def node_label(node):
    """CAN_NODE_ID_MAP convention: can_node_id = leg*10 + joint."""
    leg, joint = divmod(node, 10)
    if 1 <= leg <= 6 and joint in JOINT_NAMES:
        return "leg%d %s" % (leg, JOINT_NAMES[joint])
    return "node %d" % node


# ---------------------------------------------------------------- bus access

class Bus:
    """Multi-node, non-blocking view of the slcan bridge.

    Unlike can_goto.Bridge (bound to one node, blocking read_until) this drains
    every frame on the bus into a per-node cache, so the worker loop can send
    RTR polls and pick the answers up on a later tick without ever stalling.
    """

    def __init__(self, port):
        self.ser = serial.Serial(port, 115200, timeout=0)
        self.buf = b""
        self.seen = {}           # node -> last frame time
        self.hb = {}             # node -> (err, state, t)
        self.est = {}            # node -> (pos, vel, t)
        self.iq = {}             # node -> (iq_meas, t)
        self.vbus = {}           # node -> (v, t)
        # Config replies are request/response, not state: they are queued for
        # whoever asked rather than folded into a latest-value cache.
        self.cfg = []            # [(node, cmd, data)]

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

    def send(self, node, cmd, data=b"", rtr=False):
        arb = (node << 5) | cmd
        line = ("r%03X%X" % (arb, 8)) if rtr else \
               ("t%03X%X%s" % (arb, len(data), data.hex().upper()))
        try:
            self.ser.write((line + "\n").encode())
            self.ser.flush()
        except Exception:
            pass

    def drain(self):
        try:
            chunk = self.ser.read(4096)
        except Exception:
            chunk = b""
        if chunk:
            self.buf += chunk
        now = time.time()
        while True:
            i = self.buf.find(b"\r")
            if i < 0:
                i = self.buf.find(b"\n")
                if i < 0:
                    break
            raw, self.buf = self.buf[:i].strip(), self.buf[i + 1:]
            f = self._parse(raw)
            if f is None:
                continue
            node, cmd, data = f
            self.seen[node] = now
            try:
                if cmd == CMD_HEARTBEAT and len(data) >= 8:
                    err, state = struct.unpack("<II", data[0:8])
                    self.hb[node] = (err, state, now)
                elif cmd == CMD_GET_ENCODER_EST and len(data) >= 8:
                    pos, vel = struct.unpack("<ff", data[0:8])
                    self.est[node] = (pos, vel, now)
                elif cmd == CMD_GET_IQ and len(data) >= 8:
                    _, meas = struct.unpack("<ff", data[0:8])
                    self.iq[node] = (meas, now)
                elif cmd == CMD_GET_VBUS and len(data) >= 4:
                    self.vbus[node] = (struct.unpack("<f", data[0:4])[0], now)
                elif cmd in (CMD_CONFIG_ACCESS, CMD_CONFIG_COMMIT):
                    self.cfg.append((node, cmd, data))
                    del self.cfg[:-32]   # bounded: drop replies nobody claimed
            except struct.error:
                pass

    def take_config_frames(self, node):
        """Pop and return this node's queued config replies."""
        mine = [(c, d) for n, c, d in self.cfg if n == node]
        self.cfg = [f for f in self.cfg if f[0] != node]
        return mine

    @staticmethod
    def _parse(raw):
        try:
            s = raw.decode(errors="ignore").strip()
            j = 0
            while j < len(s) and s[j] not in "tr":
                j += 1
            s = s[j:]
            if len(s) < 5 or s[0] not in "tr":
                return None
            arb = int(s[1:4], 16)
            dlc = int(s[4], 16)
            data = bytes.fromhex(s[5:5 + dlc * 2]) if dlc else b""
            return (arb >> 5, arb & 0x1F, data)
        except Exception:
            return None

    def scan(self, seconds=3.0):
        """Passively listen for heartbeats and report the nodes on the bus."""
        end = time.time() + seconds
        while time.time() < end:
            self.drain()
            time.sleep(0.01)
        return sorted(self.hb.keys())


# ------------------------------------------------------------ limits storage

def load_limits():
    try:
        with open(LIMITS_PATH) as f:
            raw = json.load(f)
    except (OSError, ValueError):
        return {}
    out = {}
    for k, v in raw.items():
        try:
            out[int(k)] = {"min": v.get("min"), "max": v.get("max")}
        except (TypeError, ValueError):
            pass
    return out


def save_limits(limits):
    os.makedirs(os.path.dirname(LIMITS_PATH), exist_ok=True)
    blob = {}
    for node, lim in sorted(limits.items()):
        if lim.get("min") is None and lim.get("max") is None:
            continue
        blob[str(node)] = {"label": node_label(node),
                           "min": lim.get("min"), "max": lim.get("max"),
                           "units": "turn (as reported by Get_Encoder_Estimates)"}
    tmp = LIMITS_PATH + ".tmp"
    with open(tmp, "w") as f:
        json.dump(blob, f, indent=2, sort_keys=True)
        f.write("\n")
    os.replace(tmp, LIMITS_PATH)
    return LIMITS_PATH


# ------------------------------------------------------------------- pendant

class Jogger:
    """Owns the serial port. All CAN traffic happens in its worker thread."""

    TICK = 0.05  # 20 Hz setpoint stream

    def __init__(self, bus, node, args):
        self.bus = bus
        self.node = node
        self.a = args
        self.step_deg = args.step
        self.rate = args.rate
        self.limits = load_limits()

        self.lock = threading.Lock()
        self.queue = []
        self.stop = False
        self.armed = False
        self.arming = False
        self.target = None       # commanded target, turns
        self.cmd = None          # slewed setpoint actually sent, turns
        self.msg = "select a motor, then `a` to arm. Nothing moves until you do."
        self.msg_warn = False
        self.armed_since = None
        self.last_key = time.time()
        self.iq_over = 0
        self.blocked = 0         # direction currently blocked by stall guard

        self.worker = threading.Thread(target=self._run, daemon=True)

    # --- state helpers (read by the UI thread; plain reads are atomic enough)

    def pos(self):
        e = self.bus.est.get(self.node)
        return e[0] if e else None

    def vel(self):
        e = self.bus.est.get(self.node)
        return e[1] if e else None

    def iq_now(self):
        v = self.bus.iq.get(self.node)
        return v[0] if v else None

    def hb_now(self):
        v = self.bus.hb.get(self.node)
        return (v[0], v[1]) if v else (None, None)

    def lim(self, node=None):
        return self.limits.get(node if node is not None else self.node,
                               {"min": None, "max": None})

    def set_msg(self, text, warn=False):
        self.msg = text
        self.msg_warn = warn

    def post(self, cmd, arg=None):
        with self.lock:
            self.queue.append((cmd, arg))
        self.last_key = time.time()

    def _abort_requested(self):
        """Peek for a stop/idle while a blocking sequence (arming) is running."""
        with self.lock:
            return any(c in ("stop", "idle") for c, _ in self.queue)

    # --------------------------------------------------------------- worker

    def start(self):
        self.worker.start()

    def shutdown(self):
        self.stop = True
        self.worker.join(timeout=3.0)
        self.go_idle("shutdown")
        self.bus.drain()

    def go_idle(self, why=""):
        self.bus.send(self.node, CMD_SET_AXIS_STATE, struct.pack("<i", AXIS_STATE_IDLE))
        time.sleep(0.05)
        self.bus.send(self.node, CMD_SET_AXIS_STATE, struct.pack("<i", AXIS_STATE_IDLE))
        self.armed = False
        self.armed_since = None
        self.target = self.cmd = None
        if why:
            self.set_msg("IDLE (%s) -- gearbox holds the pose" % why, warn=True)

    def _run(self):
        tick = 0
        while not self.stop:
            t0 = time.time()
            self.bus.drain()

            with self.lock:
                pending, self.queue = self.queue, []
            for cmd, arg in pending:
                self._handle(cmd, arg)

            # Telemetry: estimates at 10 Hz, Iq at 5 Hz, vbus at 1 Hz.
            if tick % 2 == 0:
                self.bus.send(self.node, CMD_GET_ENCODER_EST, rtr=True)
            if tick % 4 == 1:
                self.bus.send(self.node, CMD_GET_IQ, rtr=True)
            if tick % 20 == 3:
                self.bus.send(self.node, CMD_GET_VBUS, rtr=True)

            if self.armed:
                self._guard()
            if self.armed:
                self._slew()

            tick += 1
            time.sleep(max(0.0, self.TICK - (time.time() - t0)))

    def _handle(self, cmd, arg):
        if cmd == "arm":
            self._arm()
        elif cmd == "idle":
            self.go_idle("requested")
        elif cmd == "stop":
            self.go_idle("EMERGENCY STOP")
        elif cmd == "clear_errors":
            self.bus.send(self.node, CMD_CLEAR_ERRORS)
            self.set_msg("clear_errors sent (only clears latched flags, not the cause)")
        elif cmd == "jog":
            self._jog(arg)
        elif cmd == "goto":
            self._goto(arg)
        elif cmd == "switch":
            self._switch(arg)
        elif cmd == "apply_limits":
            self._apply_limits()
        elif cmd == "save_flash":
            self._save_flash()

    # --- board-side configuration over CAN (firmware >= 0.5.6) -------------
    #
    # These run in the worker thread because it owns the serial port. They
    # borrow can_config's request/reply helpers, driving them through this
    # tool's own Bridge-compatible node view.

    def _cfg_bridge(self):
        """A can_config-compatible shim over this tool's Bus, bound to the node."""
        bus, node = self.bus, self.node

        class _Shim:
            def __init__(self):
                self.node = node

            def send(self, cmd, data=b"", rtr=False):
                bus.send(node, cmd, data, rtr)

            def poll(self):
                bus.drain()
                out, frames = [], bus.take_config_frames(node)
                for cmd, data in frames:
                    out.append((cmd, data))
                return out

        return _Shim()

    def _apply_limits(self):
        lim = self.lim()
        if lim["min"] is None or lim["max"] is None:
            self.set_msg("set BOTH min and max first", warn=True)
            return
        if self.armed:
            self.set_msg("go IDLE (`i`) before writing config to the board", warn=True)
            return
        try:
            import can_config as cc
        except ImportError as e:
            self.set_msg("can_config unavailable: %s" % e, warn=True)
            return
        br = self._cfg_bridge()
        try:
            # Order matters: never leave the board with the enable on while the
            # range is half-written. Disable, set the range, verify, re-enable.
            cc.access(br, "limit_enable", 0)
            lo = cc.access(br, "min", lim["min"] * 360.0)
            hi = cc.access(br, "max", lim["max"] * 360.0)
            en = cc.access(br, "limit_enable", 1)
            self.set_msg("board endstops: min=%.2f max=%.2f enabled=%d "
                         "(RAM only -- press S to save to flash)" % (lo, hi, en))
        except Exception as e:
            self.set_msg("apply failed: %s" % e, warn=True)

    def _save_flash(self):
        if self.armed:
            self.set_msg("go IDLE (`i`) before saving -- flash erase stalls the loop",
                         warn=True)
            return
        try:
            import can_config as cc
        except ImportError as e:
            self.set_msg("can_config unavailable: %s" % e, warn=True)
            return
        self.set_msg("saving to flash...")
        try:
            st = cc.commit(self._cfg_bridge(), cc.ACTION_SAVE)
            self.set_msg("save: %s" % cc.STATUS.get(st, "status 0x%X" % st),
                         warn=(st != 0x10))
        except Exception as e:
            self.set_msg("save failed: %s" % e, warn=True)

    def _arm(self):
        err, state = self.hb_now()
        if err is None:
            self.set_msg("no heartbeat from node %d -- not arming" % self.node, warn=True)
            return
        if err:
            self.set_msg("axis error 0x%X -- clear it (`e`) and fix the cause first" % err,
                         warn=True)
            return
        pos = self.pos()
        if pos is None:
            self.set_msg("no encoder estimate -- not arming", warn=True)
            return

        self.arming = True
        self.set_msg("arming...")
        try:
            # Seed the setpoint with the MEASURED position before closing the
            # loop, so entry can never jump the joint.
            self.bus.send(self.node, CMD_SET_CONTROLLER_MODES,
                          struct.pack("<ii", CONTROL_MODE_POSITION, INPUT_MODE_PASSTHROUGH))
            self.bus.send(self.node, CMD_SET_INPUT_POS, struct.pack("<fhh", pos, 0, 0))
            time.sleep(0.15)

            # An axis that has just powered up sits in UNDEFINED and silently
            # ignores a direct CLOSED_LOOP request -- go through IDLE first.
            if state != AXIS_STATE_IDLE:
                self.bus.send(self.node, CMD_SET_AXIS_STATE, struct.pack("<i", AXIS_STATE_IDLE))
                end = time.time() + 2.0
                while time.time() < end:
                    self.bus.drain()
                    if self.hb_now()[1] == AXIS_STATE_IDLE or self._abort_requested():
                        break
                    time.sleep(0.02)

            self.bus.send(self.node, CMD_SET_AXIS_STATE, struct.pack("<i", AXIS_STATE_CLOSED_LOOP))
            end = time.time() + 3.0
            while time.time() < end:
                self.bus.drain()
                if self._abort_requested():
                    self.go_idle("aborted during arming")
                    return
                err, state = self.hb_now()
                if err:
                    self.go_idle("error 0x%X on arm" % err)
                    return
                if state == AXIS_STATE_CLOSED_LOOP:
                    here = self.pos()
                    self.target = self.cmd = here if here is not None else pos
                    self.armed = True
                    self.armed_since = time.time()
                    self.iq_over = 0
                    self.blocked = 0
                    self.set_msg("ARMED -- arrows jog %.1f deg, SPACE stops" % self.step_deg)
                    return
                time.sleep(0.02)
            self.go_idle("did not reach CLOSED_LOOP")
        finally:
            self.arming = False

    def _guard(self):
        err, state = self.hb_now()
        if err:
            self.go_idle("FAULT 0x%X" % err)
            return
        if state is not None and state != AXIS_STATE_CLOSED_LOOP:
            self.go_idle("axis left closed loop (state %s)" % STATE_NAMES.get(state, state))
            return

        iq = self.iq_now()
        if iq is not None and abs(iq) > self.a.iq_cap:
            self.iq_over += 1
            if self.iq_over >= 3:
                self.go_idle("Iq %.1f A over cap %.1f A" % (iq, self.a.iq_cap))
                return
        else:
            self.iq_over = 0

        pos = self.pos()
        if pos is not None and self.cmd is not None:
            lag_deg = (self.cmd - pos) * 360.0
            if abs(lag_deg) > 2 * self.a.max_lag:
                self.go_idle("setpoint ran %.1f deg away from the joint (stall?)" % lag_deg)
                return
            if abs(lag_deg) > self.a.max_lag:
                self.blocked = 1 if lag_deg > 0 else -1
                self.set_msg("STALL GUARD: joint is %.1f deg behind -- jog %s blocked"
                             % (lag_deg, "+" if self.blocked > 0 else "-"), warn=True)
            else:
                self.blocked = 0

        if self.a.idle_timeout > 0 and time.time() - self.last_key > self.a.idle_timeout:
            self.go_idle("idle %.0f s -- holding a geared joint is continuous-current duty"
                         % self.a.idle_timeout)

    def _slew(self):
        if self.target is None or self.cmd is None:
            return
        step = (self.rate / 360.0) * self.TICK
        if abs(self.target - self.cmd) <= step:
            self.cmd = self.target
        else:
            self.cmd += step if self.target > self.cmd else -step
        self.bus.send(self.node, CMD_SET_INPUT_POS, struct.pack("<fhh", self.cmd, 0, 0))

    def _clamp(self, want):
        lim = self.lim()
        note = ""
        if lim["min"] is not None and want < lim["min"]:
            want, note = lim["min"], " (at MIN)"
        if lim["max"] is not None and want > lim["max"]:
            want, note = lim["max"], " (at MAX)"
        return want, note

    def _jog(self, sign):
        if not self.armed:
            self.set_msg("not armed -- press `a` first", warn=True)
            return
        if self.blocked and sign == self.blocked:
            self.set_msg("stall guard: cannot jog further that way until the joint catches up",
                         warn=True)
            return
        want, note = self._clamp(self.target + sign * self.step_deg / 360.0)
        self.target = want
        self.set_msg("target %.2f deg%s" % (want * 360.0, note), warn=bool(note))

    def _goto(self, turn):
        if not self.armed:
            self.set_msg("not armed -- press `a` first", warn=True)
            return
        want, note = self._clamp(turn)
        self.target = want
        self.set_msg("moving to %.2f deg%s" % (want * 360.0, note), warn=bool(note))

    def _switch(self, node):
        if node == self.node:
            return
        if self.armed:
            self.go_idle("switching motor")
        self.node = node
        self.target = self.cmd = None
        self.blocked = 0
        self.set_msg("selected node %d (%s) -- press `a` to arm" % (node, node_label(node)))


# ---------------------------------------------------------------------- tty

class RawTTY:
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        sys.stdout.write("\x1b[?25l")   # hide cursor
        sys.stdout.flush()
        return self

    def __exit__(self, *exc):
        sys.stdout.write("\x1b[?25h")
        sys.stdout.flush()
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)


ESC_KEYS = {b"[A": "UP", b"[B": "DOWN", b"[C": "RIGHT", b"[D": "LEFT",
            b"[5~": "PGUP", b"[6~": "PGDN"}

# Read the tty through the RAW fd, never sys.stdin: a buffered TextIOWrapper
# swallows the rest of an escape sequence into its own buffer, so a subsequent
# select() on the fd reports "nothing to read" and every arrow key is lost (and
# its leftover "[C" surfaces later, misread as a plain keystroke).
_stdin_buf = bytearray()


def _pump_stdin(wait=0.0):
    import select
    if select.select([sys.stdin.fileno()], [], [], wait)[0]:
        try:
            _stdin_buf.extend(os.read(sys.stdin.fileno(), 256))
        except OSError:
            pass


def read_keys():
    """Non-blocking: return a list of logical key names."""
    _pump_stdin(0.0)
    keys = []
    while _stdin_buf:
        b = _stdin_buf[0]
        if b != 0x1B:
            del _stdin_buf[0]
            keys.append(chr(b))
            continue
        # Escape: the rest of the sequence may not have arrived yet.
        if len(_stdin_buf) == 1:
            _pump_stdin(0.005)
            if len(_stdin_buf) == 1:
                del _stdin_buf[0]
                keys.append("ESC")
                continue
        for n in (4, 3, 2):
            seq = bytes(_stdin_buf[1:1 + n])
            if seq in ESC_KEYS:
                del _stdin_buf[:1 + n]
                keys.append(ESC_KEYS[seq])
                break
        else:
            del _stdin_buf[0]
            keys.append("ESC")
    return keys


def prompt(text):
    """Blocking line input. The tty stays in cbreak mode, so echo by hand."""
    sys.stdout.write("\x1b[?25h\n" + text)
    sys.stdout.flush()
    try:
        line = ""
        while True:
            if not _stdin_buf:
                _pump_stdin(0.05)
                continue
            ch = chr(_stdin_buf[0])
            del _stdin_buf[0]
            if ch in ("\n", "\r"):
                break
            if ch in ("\x7f", "\b"):
                line = line[:-1]
                sys.stdout.write("\b \b")
            elif ch == "\x03":
                raise KeyboardInterrupt
            elif ch == "\x1b":
                _stdin_buf.clear()
                return ""
            else:
                line += ch
                sys.stdout.write(ch)
            sys.stdout.flush()
        return line.strip()
    finally:
        sys.stdout.write("\x1b[?25l")
        sys.stdout.flush()


# ------------------------------------------------------------------ display

def fmt_deg(turn):
    return "  --  " if turn is None else "%7.2f" % (turn * 360.0)


def render(j, nodes, port):
    err, state = j.hb_now()
    pos, vel, iq = j.pos(), j.vel(), j.iq_now()
    lim = j.lim()
    vb = j.bus.vbus.get(j.node)

    lag = None
    if pos is not None and j.cmd is not None:
        lag = (j.cmd - pos) * 360.0

    others = " ".join(("[%d]" % n) if n == j.node else str(n) for n in nodes)
    armed_s = ("%.0f s" % (time.time() - j.armed_since)) if j.armed_since else "-"

    L = []
    L.append("\x1b[1mODrive CAN jog\x1b[0m   %s   node \x1b[1m%d\x1b[0m (%s)"
             % (port, j.node, node_label(j.node)))
    L.append("bus: %s" % (others or "(none)"))
    L.append("-" * 68)
    L.append("  pos   \x1b[1m%s deg\x1b[0m   vel %6.2f t/s      %s"
             % (fmt_deg(pos), vel if vel is not None else float("nan"),
                ("\x1b[7m ARMED %s \x1b[0m" % armed_s) if j.armed else " idle "))
    L.append("  cmd   %s deg   lag %s deg" %
             (fmt_deg(j.cmd), "  --  " if lag is None else "%7.2f" % lag))
    L.append("  Iq    %7.2f A     cap %.1f A          vbus %s"
             % (iq if iq is not None else float("nan"), j.a.iq_cap,
                "%.2f V" % vb[0] if vb else " -- "))
    L.append("  state %-12s err 0x%s"
             % (STATE_NAMES.get(state, "?" if state is None else state),
                "%X" % err if err is not None else "--"))
    L.append("-" * 68)
    L.append("  step  \x1b[1m%.2f deg\x1b[0m      rate %.1f deg/s" % (j.step_deg, j.rate))
    L.append("  min   %s deg     max %s deg     %s"
             % (fmt_deg(lim["min"]), fmt_deg(lim["max"]),
                "travel %.1f deg" % ((lim["max"] - lim["min"]) * 360.0)
                if lim["min"] is not None and lim["max"] is not None else ""))
    L.append("-" * 68)
    L.append("  \x1b[2mLEFT/DOWN -step   RIGHT/UP +step   +/- step size   </> rate\x1b[0m")
    L.append("  \x1b[2ma arm   i idle   SPACE STOP   e clear errors   q quit\x1b[0m")
    L.append("  \x1b[2m[ set min   ] set max   c clear   w write file   g/G goto min/max\x1b[0m")
    L.append("  \x1b[2mt type target   n/p switch node\x1b[0m")
    L.append("  \x1b[2mA apply limits to board (CAN)   S save board config to flash\x1b[0m")
    L.append("-" * 68)
    L.append(("\x1b[33m> %s\x1b[0m" if j.msg_warn else "> %s") % j.msg)

    # Home + per-line erase-to-EOL + erase-below: redraw in place, no flicker.
    out = "\x1b[H" + "\n".join(line + "\x1b[K" for line in L) + "\n\x1b[J"
    sys.stdout.write(out)
    sys.stdout.flush()


def print_apply_snippet(j, path):
    lim = j.lim()
    print("\nlimits for node %d (%s) written to %s" % (j.node, node_label(j.node), path))
    if lim["min"] is None or lim["max"] is None:
        print("  (set BOTH min and max before applying them to the board)")
        return
    print("""
  These clamp this tool only. CAN Simple has no generic parameter write, so to
  make them real firmware endstops, connect over USB and run:

    e = odrv0.axis1.encoder            # load-side joint encoder (split feedback)
    # sanity: the linear pos_estimate can silently lose a whole turn -- re-seed
    e.config.zero_offset = e.config.zero_offset
    print(e.pos_estimate)              # must match the angle you jogged to

    e.config.min_position = %.6f     # %.2f deg
    e.config.max_position = %.6f     # %.2f deg
    e.config.enable_position_limit = True     # enable DEAD LAST
    odrv0.save_configuration()
""" % (lim["min"], lim["min"] * 360.0, lim["max"], lim["max"] * 360.0))


# --------------------------------------------------------------------- main

def choose_node(bus, nodes, preset):
    if preset is not None:
        return preset
    if not nodes:
        raise SystemExit("no heartbeats on the bus -- nothing to jog "
                         "(unbooted boards look exactly like missing wiring)")
    if len(nodes) == 1:
        return nodes[0]
    print("\nnodes on the bus:")
    for i, n in enumerate(nodes):
        err, state, _ = bus.hb[n]
        print("  %d) node %-3d %-12s state=%-11s err=0x%X"
              % (i + 1, n, node_label(n), STATE_NAMES.get(state, state), err))
    while True:
        s = input("select [1-%d]: " % len(nodes)).strip()
        if s.isdigit() and 1 <= int(s) <= len(nodes):
            return nodes[int(s) - 1]


def main():
    p = argparse.ArgumentParser(description="CNC-style CAN jog pendant for ODrive joints")
    p.add_argument("--port", default=None, help="slcan bridge serial port")
    p.add_argument("--node", type=int, default=None, help="skip the picker, jog this node")
    p.add_argument("--step", type=float, default=5.0, help="jog step, deg (default 5)")
    p.add_argument("--rate", type=float, default=10.0, help="slew rate, deg/s")
    p.add_argument("--iq-cap", type=float, default=8.0, help="auto-IDLE above this |Iq|, A")
    p.add_argument("--max-lag", type=float, default=12.0,
                   help="block jogging when the joint falls this far behind, deg")
    p.add_argument("--idle-timeout", type=float, default=45.0,
                   help="auto-IDLE after this long with no keypress while armed "
                        "(0 disables -- thermal risk)")
    p.add_argument("--scan", type=float, default=3.0, help="seconds to listen for heartbeats")
    a = p.parse_args()

    port = a.port or find_bridge()
    print("bridge port:", port)
    bus = Bus(port)
    print("listening %.1f s for heartbeats..." % a.scan)
    nodes = bus.scan(a.scan)
    print("found: %s" % (", ".join("%d (%s)" % (n, node_label(n)) for n in nodes) or "none"))

    node = choose_node(bus, nodes, a.node)
    if node not in nodes:
        nodes = sorted(set(nodes) | {node})

    j = Jogger(bus, node, a)
    j.start()
    steps = [0.5, 1.0, 2.0, 5.0, 10.0, 20.0]

    try:
        with RawTTY():
            sys.stdout.write("\x1b[2J")
            last_draw = 0.0
            while True:
                for k in read_keys():
                    j.last_key = time.time()
                    if k in ("q", "Q"):
                        raise KeyboardInterrupt
                    elif k == " ":
                        j.post("stop")
                    elif k in ("LEFT", "DOWN"):
                        j.post("jog", -1)
                    elif k in ("RIGHT", "UP"):
                        j.post("jog", +1)
                    elif k == "PGUP":
                        j.post("jog", +1)
                    elif k == "PGDN":
                        j.post("jog", -1)
                    elif k == "a":  # lowercase only: `A` applies limits to the board
                        j.post("arm")
                    elif k in ("i", "I"):
                        j.post("idle")
                    elif k in ("e", "E"):
                        j.post("clear_errors")
                    elif k in ("+", "="):
                        nxt = [s for s in steps if s > j.step_deg]
                        j.step_deg = nxt[0] if nxt else steps[-1]
                        j.set_msg("step %.2f deg" % j.step_deg)
                    elif k in ("-", "_"):
                        prv = [s for s in steps if s < j.step_deg]
                        j.step_deg = prv[-1] if prv else steps[0]
                        j.set_msg("step %.2f deg" % j.step_deg)
                    elif k == ">":
                        j.rate = min(60.0, j.rate + 2.0)
                        j.set_msg("rate %.1f deg/s" % j.rate)
                    elif k == "<":
                        j.rate = max(1.0, j.rate - 2.0)
                        j.set_msg("rate %.1f deg/s" % j.rate)
                    elif k == "[":
                        pos = j.pos()
                        if pos is None:
                            j.set_msg("no position yet", warn=True)
                        else:
                            lim = dict(j.lim())
                            lim["min"] = pos
                            if lim["max"] is not None and lim["max"] < pos:
                                lim["max"] = None
                                j.set_msg("MIN = %.2f deg (old max was below it, cleared)"
                                          % (pos * 360), warn=True)
                            else:
                                j.set_msg("MIN = %.2f deg" % (pos * 360))
                            j.limits[j.node] = lim
                    elif k == "]":
                        pos = j.pos()
                        if pos is None:
                            j.set_msg("no position yet", warn=True)
                        else:
                            lim = dict(j.lim())
                            lim["max"] = pos
                            if lim["min"] is not None and lim["min"] > pos:
                                lim["min"] = None
                                j.set_msg("MAX = %.2f deg (old min was above it, cleared)"
                                          % (pos * 360), warn=True)
                            else:
                                j.set_msg("MAX = %.2f deg" % (pos * 360))
                            j.limits[j.node] = lim
                    elif k in ("c", "C"):
                        j.limits[j.node] = {"min": None, "max": None}
                        j.set_msg("limits cleared for node %d (file unchanged until `w`)" % j.node)
                    elif k == "w":
                        path = save_limits(j.limits)
                        j.set_msg("written to %s" % os.path.relpath(path, HERE))
                    elif k in ("r", "R"):
                        j.limits = load_limits()
                        j.set_msg("limits reloaded from file")
                    elif k == "g":
                        lo = j.lim()["min"]
                        if lo is None:
                            j.set_msg("no MIN set", warn=True)
                        else:
                            j.post("goto", lo)
                    elif k == "G":
                        hi = j.lim()["max"]
                        if hi is None:
                            j.set_msg("no MAX set", warn=True)
                        else:
                            j.post("goto", hi)
                    elif k in ("t", "T"):
                        s = prompt("target angle, deg: ")
                        try:
                            j.post("goto", float(s) / 360.0)
                        except ValueError:
                            j.set_msg("not a number: %r" % s, warn=True)
                    elif k == "A":
                        j.post("apply_limits")
                    elif k == "S":
                        j.post("save_flash")
                    elif k in ("n", "p", "N", "P"):
                        live = sorted(set(nodes) | set(bus.hb.keys()))
                        nodes = live
                        if live:
                            i = live.index(j.node) if j.node in live else 0
                            i = (i + (1 if k in ("n", "N") else -1)) % len(live)
                            j.post("switch", live[i])

                now = time.time()
                if now - last_draw > 0.1:
                    render(j, sorted(set(nodes) | set(bus.hb.keys())), port)
                    last_draw = now
                time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        j.shutdown()
        sys.stdout.write("\x1b[?25h\n")
        print("-> IDLE, disconnected.")
        lim = j.lim()
        if lim["min"] is not None or lim["max"] is not None:
            print_apply_snippet(j, save_limits(j.limits))
        bus.close()


if __name__ == "__main__":
    main()
