#!/usr/bin/env python3
"""Diagnostic slow move on ONE ODrive joint over CAN, logging the split-feedback
pair every tick: commanded joint angle vs actual joint pos (MT6701) vs motor
velocity (AS5047P) vs Iq. Reveals stick-slip / slip / commutation issues.

Gentle + guarded. Ramps input_pos low->high->low slowly and streams telemetry.
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

IDLE, CLOSED_LOOP = 1, 8
CONTROL_POSITION, INPUT_PASSTHROUGH = 3, 1


def find_bridge():
    return sorted(glob.glob("/dev/cu.usbmodem*"))[-1]


class B:
    def __init__(self, port, node):
        self.node = node
        self.ser = serial.Serial(port, 115200, timeout=0.02)
        self.buf = b""

    def send(self, cmd, data=b"", rtr=False):
        arb = (self.node << 5) | cmd
        line = ("r%03X%X" % (arb, 8)) if rtr else ("t%03X%X%s" % (arb, len(data), data.hex().upper()))
        self.ser.write((line + "\n").encode()); self.ser.flush()

    def _pump(self):
        c = self.ser.read(512)
        if c: self.buf += c
        out = []
        while True:
            i = self.buf.find(b"\r")
            if i < 0:
                i = self.buf.find(b"\n")
                if i < 0: break
            raw = self.buf[:i].strip(); self.buf = self.buf[i+1:]
            try:
                s = raw.decode(errors="ignore").strip()
                j = 0
                while j < len(s) and s[j] not in "tr": j += 1
                s = s[j:]
                if len(s) < 5 or s[0] not in "tr": continue
                arb = int(s[1:4], 16); dlc = int(s[4], 16)
                data = bytes.fromhex(s[5:5+dlc*2]) if dlc else b""
                if (arb >> 5) == self.node: out.append((arb & 0x1F, data))
            except Exception:
                pass
        return out

    def rtr(self, cmd, timeout=0.2):
        self.send(cmd, rtr=True)
        end = time.time() + timeout; got = None
        while time.time() < end:
            for c, d in self._pump():
                if c == cmd: got = d
            if got is not None: return got
            time.sleep(0.003)
        return got

    def hb(self, timeout=2.0):
        end = time.time() + timeout
        while time.time() < end:
            for c, d in self._pump():
                if c == CMD_HEARTBEAT and len(d) >= 8:
                    return struct.unpack("<II", d[0:8])
            time.sleep(0.004)
        return None, None


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--node", type=int, required=True)
    p.add_argument("--low", type=float, default=5.0)
    p.add_argument("--high", type=float, default=40.0)
    p.add_argument("--rate", type=float, default=15.0, help="deg/s")
    p.add_argument("--port", default=None)
    args = p.parse_args()
    br = B(args.port or find_bridge(), args.node)

    err, st = br.hb(3.0)
    if err is None:
        print("no heartbeat"); return 2
    print("start: err=0x%X state=%d" % (err, st))
    est = br.rtr(CMD_GET_ENCODER_EST)
    pos0 = struct.unpack("<ff", est[0:8])[0] if est else 0.0
    print("start joint pos=%.1f deg" % (pos0 * 360))

    br.send(CMD_CLEAR_ERRORS); time.sleep(0.15)
    br.send(CMD_SET_CONTROLLER_MODES, struct.pack("<ii", CONTROL_POSITION, INPUT_PASSTHROUGH))
    br.send(CMD_SET_INPUT_POS, struct.pack("<fhh", pos0, 0, 0)); time.sleep(0.1)
    br.send(CMD_SET_AXIS_STATE, struct.pack("<i", CLOSED_LOOP))
    end = time.time() + 3.0; entered = False
    while time.time() < end:
        e, s = br.hb(0.4)
        if e: print("err 0x%X on entry" % e); br.send(CMD_SET_AXIS_STATE, struct.pack("<i", IDLE)); return 3
        if s == CLOSED_LOOP: entered = True; break
    if not entered: print("no closed loop"); return 3
    print("closed loop. logging (t, cmd_deg, joint_deg, motor_vel_ts, iq_A):")

    lo, hi = args.low/360.0, args.high/360.0
    rate = args.rate/360.0; dt = 0.05; step = rate*dt
    segs = [(pos0, lo), (lo, hi), (hi, lo)]  # settle to low, down, up
    t0 = time.time(); maxlag = 0.0
    for a, b in segs:
        cmd = a
        while True:
            cmd = b if abs(b-cmd) <= step else cmd + (step if b > cmd else -step)
            br.send(CMD_SET_INPUT_POS, struct.pack("<fhh", cmd, 0, 0))
            time.sleep(dt)
            est = br.rtr(CMD_GET_ENCODER_EST, 0.08)
            iqd = br.rtr(CMD_GET_IQ, 0.06)
            jp, mv = struct.unpack("<ff", est[0:8]) if est and len(est) >= 8 else (float('nan'),)*2
            iq = struct.unpack("<ff", iqd[0:8])[1] if iqd and len(iqd) >= 8 else float('nan')
            lag = (cmd - jp) * 360
            maxlag = max(maxlag, abs(lag))
            print("  %5.2f  cmd=%5.1f  joint=%5.1f  (lag=%+5.1f)  motor_vel=%+5.2f t/s  iq=%+5.2f" %
                  (time.time()-t0, cmd*360, jp*360, lag, mv, iq), flush=True)
            e, s = br.hb(0.01)
            if e: print("  FAULT 0x%X" % e); br.send(CMD_SET_AXIS_STATE, struct.pack("<i", IDLE)); return 4
            if cmd == b: break
    print("max |lag| = %.1f deg" % maxlag)
    br.send(CMD_SET_AXIS_STATE, struct.pack("<i", IDLE))
    print("idled.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
