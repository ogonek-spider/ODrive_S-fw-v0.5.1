#!/usr/bin/env python3
"""Gently drive one ODrive joint to a target joint angle over CAN (split-feedback).

Safety-first position move for the geared legs:
  * passive heartbeat check first; abort on any axis error
  * read current joint pos, set input_pos = current (NO jump) BEFORE closed loop
  * enter CLOSED_LOOP, verify, abort on fault
  * ramp input_pos from current -> target at a slow, fixed deg/s rate
  * every step: watch heartbeat for faults and abort if |Iq| exceeds a cap
    (protects against ramming a hard endstop)
  * on arrival: hold briefly, then IDLE (non-backdrivable gearbox holds pose)

Coordinates are joint/output turns as reported by Get_Encoder_Estimates
(load_encoder_axis, with direction+zero already applied). target 0 == upper
endstop for the knee joints.
"""
import argparse
import glob
import struct
import time

import serial

CMD_HEARTBEAT = 0x001
CMD_SET_AXIS_STATE = 0x007
CMD_GET_ENCODER_EST = 0x009
CMD_SET_CONTROLLER_MODES = 0x00B
CMD_SET_INPUT_POS = 0x00C
CMD_GET_IQ = 0x014
CMD_CLEAR_ERRORS = 0x018

AXIS_STATE_IDLE = 1
AXIS_STATE_CLOSED_LOOP = 8
CONTROL_MODE_POSITION = 3
INPUT_MODE_PASSTHROUGH = 1

STATE_NAMES = {0: "UNDEFINED", 1: "IDLE", 3: "CALIB", 8: "CLOSED_LOOP"}


def find_bridge():
    ports = sorted(glob.glob("/dev/cu.usbmodem*"))
    if not ports:
        raise RuntimeError("no /dev/cu.usbmodem* bridge found")
    return ports[-1]


class Bridge:
    def __init__(self, port, node):
        self.node = node
        self.base = node << 5
        self.ser = serial.Serial(port, 115200, timeout=0.05)
        self.buf = b""

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

    def send(self, cmd, data=b"", rtr=False):
        arb = self.base | cmd
        if rtr:
            line = "r%03X%X" % (arb, 8)
        else:
            line = "t%03X%X%s" % (arb, len(data), data.hex().upper())
        self.ser.write((line + "\n").encode())
        self.ser.flush()

    def poll(self):
        out = []
        chunk = self.ser.read(512)
        if chunk:
            self.buf += chunk
        while True:
            i = self.buf.find(b"\r")
            if i < 0:
                i = self.buf.find(b"\n")
                if i < 0:
                    break
            raw = self.buf[:i].strip()
            self.buf = self.buf[i + 1:]
            f = self._parse(raw)
            if f is not None:
                out.append(f)
        return out

    def _parse(self, raw):
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
            if (arb >> 5) != self.node:
                return None
            return (arb & 0x1F, data)
        except Exception:
            return None

    def read_until(self, want_cmd, timeout, collect=None):
        end = time.time() + timeout
        got = None
        while time.time() < end:
            for cmd, data in self.poll():
                if collect is not None:
                    collect[cmd] = data
                if cmd == want_cmd:
                    got = data
            if got is not None:
                return got
            time.sleep(0.005)
        return got

    def get_pos(self, timeout=0.4):
        self.send(CMD_GET_ENCODER_EST, rtr=True)
        d = self.read_until(CMD_GET_ENCODER_EST, timeout)
        if d and len(d) >= 8:
            return struct.unpack("<ff", d[0:8])
        return None, None

    def get_iq(self, timeout=0.3):
        self.send(CMD_GET_IQ, rtr=True)
        d = self.read_until(CMD_GET_IQ, timeout)
        if d and len(d) >= 8:
            return struct.unpack("<ff", d[0:8])[1]
        return None

    def hb(self, timeout=0.4):
        d = self.read_until(CMD_HEARTBEAT, timeout)
        if d and len(d) >= 8:
            return struct.unpack("<II", d[0:8])
        return None, None


def goto(port, node, target_turn, rate_deg_s, iq_cap, tol_deg, hold_s, keep_closed, settle_s=4.0):
    br = Bridge(port, node)
    print("node %d -> target %.4f turn (%.1f deg)" %
          (node, target_turn, target_turn * 360), flush=True)

    err, state = br.hb(timeout=3.0)
    if err is None:
        print("  NO HEARTBEAT -> abort", flush=True)
        return False
    print("  heartbeat: err=0x%X state=%s(%s)" % (err, STATE_NAMES.get(state, "?"), state), flush=True)
    if err:
        print("  axis error set -> abort (clear the fault first)", flush=True)
        return False

    pos, _ = br.get_pos()
    if pos is None:
        print("  no encoder estimate -> abort", flush=True)
        return False
    print("  current pos: %.4f turn (%.1f deg)" % (pos, pos * 360), flush=True)
    start = pos

    # No jump: seed input_pos with current, set mode, then close loop.
    br.send(CMD_CLEAR_ERRORS)
    time.sleep(0.2)
    br.send(CMD_SET_CONTROLLER_MODES, struct.pack("<ii", CONTROL_MODE_POSITION, INPUT_MODE_PASSTHROUGH))
    br.send(CMD_SET_INPUT_POS, struct.pack("<fhh", pos, 0, 0))
    time.sleep(0.15)

    br.send(CMD_SET_AXIS_STATE, struct.pack("<i", AXIS_STATE_CLOSED_LOOP))
    entered = False
    end = time.time() + 3.0
    while time.time() < end:
        err, state = br.hb(timeout=0.5)
        if err:
            print("  err=0x%X on entry -> abort" % err, flush=True)
            br.send(CMD_SET_AXIS_STATE, struct.pack("<i", AXIS_STATE_IDLE))
            return False
        if state == AXIS_STATE_CLOSED_LOOP:
            entered = True
            break
    if not entered:
        print("  did not reach CLOSED_LOOP -> abort", flush=True)
        br.send(CMD_SET_AXIS_STATE, struct.pack("<i", AXIS_STATE_IDLE))
        return False
    print("  in CLOSED_LOOP, ramping...", flush=True)

    # Ramp input_pos start->target at rate_deg_s.
    rate_turn_s = rate_deg_s / 360.0
    step_dt = 0.05
    step = rate_turn_s * step_dt
    cmd = start
    ok = True
    while True:
        if abs(target_turn - cmd) <= step:
            cmd = target_turn
        else:
            cmd += step if target_turn > cmd else -step
        br.send(CMD_SET_INPUT_POS, struct.pack("<fhh", cmd, 0, 0))
        time.sleep(step_dt)

        err, state = br.hb(timeout=0.06)
        if err:
            print("  FAULT err=0x%X mid-move -> stop" % err, flush=True)
            ok = False
            break
        iq = br.get_iq(timeout=0.08)
        if iq is not None and abs(iq) > iq_cap:
            p, _ = br.get_pos(timeout=0.15)
            print("  Iq=%.2f A > cap %.2f A (likely endstop) at %.1f deg -> stop" %
                  (iq, iq_cap, (p or cmd) * 360), flush=True)
            ok = False
            break
        if cmd == target_turn:
            break

    # Settle: the ramp only finishes COMMANDING the target -- a heavy geared
    # joint still has to converge to it. Judging 0.3 s later reports a false
    # miss (and, with the abort path, skips the requested hold). Poll until it
    # lands inside tolerance or settle_s expires, still watching for faults and
    # the Iq cap so a genuine stall is not mistaken for slow convergence.
    p, v = br.get_pos()
    if ok:
        end = time.time() + settle_s
        while time.time() < end:
            if p is not None and abs(p - target_turn) * 360 <= tol_deg:
                break
            err, state = br.hb(timeout=0.06)
            if err:
                print("  FAULT err=0x%X while settling -> stop" % err, flush=True)
                ok = False
                break
            iq = br.get_iq(timeout=0.08)
            if iq is not None and abs(iq) > iq_cap:
                print("  Iq=%.2f A > cap %.2f A while settling -> stop" % (iq, iq_cap), flush=True)
                ok = False
                break
            time.sleep(0.1)
            p, v = br.get_pos(timeout=0.15)

    iq = br.get_iq()
    print("  reached: pos=%.4f turn (%.1f deg)  Iq=%.2f A" %
          (p if p is not None else float('nan'),
           (p * 360) if p is not None else float('nan'),
           iq if iq is not None else float('nan')), flush=True)
    if ok and p is not None and abs(p - target_turn) * 360 <= tol_deg:
        print("  within %.1f deg tolerance." % tol_deg, flush=True)
    elif ok:
        print("  NOT within tolerance after %.1f s settle." % settle_s, flush=True)
        ok = False

    # SAFETY: an abort (fault, Iq cap, missed target) means the joint is stalled
    # or faulted -- it must NEVER be left energized pushing into whatever stopped
    # it. --keep-closed only applies to a clean, in-tolerance arrival. Holding a
    # stalled geared joint is the continuous-current condition that cooked an
    # earlier motor.
    if not ok:
        br.send(CMD_SET_AXIS_STATE, struct.pack("<i", AXIS_STATE_IDLE))
        print("  -> ABORTED: forced IDLE (gearbox holds pose), --keep-closed ignored", flush=True)
        br.close()
        return ok

    if hold_s > 0:
        time.sleep(hold_s)
    if not keep_closed:
        br.send(CMD_SET_AXIS_STATE, struct.pack("<i", AXIS_STATE_IDLE))
        print("  -> IDLE (gearbox holds pose)", flush=True)
    else:
        print("  -> holding in CLOSED_LOOP", flush=True)
    br.close()
    return ok


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--port", default=None)
    p.add_argument("--node", type=int, required=True)
    p.add_argument("--target", type=float, default=0.0, help="target joint pos in turns")
    p.add_argument("--rate", type=float, default=10.0, help="ramp rate deg/s (output)")
    # 12 A: same reasoning as can_jog.py -- a 1:6 coxa/knee needs 8-12 A just to
    # move a limb, so a 6 A cap aborts legitimate moves. current_lim (15 A) is
    # the hard backstop; the danger is CONTINUOUS current, not the peak.
    p.add_argument("--iq-cap", type=float, default=12.0, help="abort if |Iq| exceeds this (A)")
    p.add_argument("--tol", type=float, default=1.5, help="arrival tolerance deg")
    p.add_argument("--hold", type=float, default=0.5, help="hold seconds after arrival")
    p.add_argument("--settle", type=float, default=4.0, help="max seconds to wait for convergence before judging tolerance")
    p.add_argument("--keep-closed", action="store_true", help="stay in closed loop (default: idle)")
    args = p.parse_args()
    port = args.port or find_bridge()
    print("bridge port:", port, flush=True)
    ok = goto(port, args.node, args.target, args.rate, args.iq_cap,
              args.tol, args.hold, args.keep_closed, args.settle)
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
