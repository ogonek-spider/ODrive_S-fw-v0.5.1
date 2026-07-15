#!/usr/bin/env python3
"""Synchronized cycling of multiple ODrive knee joints over CAN.

Every motion drives ALL nodes at once (never sequentially): each 20 Hz tick a
new commanded angle is streamed to every node in the same loop, so the legs move
together. For a demo/video: a slow synchronized validation swing first (both legs
together, watch them move), then N fast cycles low<->high, then park + idle.

Coordinates are joint/output turns from Get_Encoder_Estimates (load_encoder_axis,
direction+zero applied). Collision is at the leg-down (high) end, so keep high
under the confirmed-safe cap.
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
STATE_NAMES = {0: "UNDEF", 1: "IDLE", 3: "CALIB", 8: "CLOSED_LOOP"}


def find_bridge():
    ports = sorted(glob.glob("/dev/cu.usbmodem*"))
    if not ports:
        raise RuntimeError("no /dev/cu.usbmodem* bridge found")
    return ports[-1]


class Bus:
    def __init__(self, port):
        self.ser = serial.Serial(port, 115200, timeout=0.02)
        self.buf = b""
        self.hb = {}  # node -> (err,state)

    def send(self, node, cmd, data=b"", rtr=False):
        arb = (node << 5) | cmd
        line = ("r%03X%X" % (arb, 8)) if rtr else ("t%03X%X%s" % (arb, len(data), data.hex().upper()))
        self.ser.write((line + "\n").encode()); self.ser.flush()

    def pump(self):
        c = self.ser.read(512)
        if c: self.buf += c
        out = []
        while True:
            i = self.buf.find(b"\r")
            if i < 0:
                i = self.buf.find(b"\n")
                if i < 0: break
            raw = self.buf[:i].strip(); self.buf = self.buf[i+1:]
            f = self._parse(raw)
            if f is None: continue
            n, cmd, data = f
            if cmd == CMD_HEARTBEAT and len(data) >= 8:
                self.hb[n] = struct.unpack("<II", data[0:8])
            out.append(f)
        return out

    def _parse(self, raw):
        try:
            s = raw.decode(errors="ignore").strip()
            j = 0
            while j < len(s) and s[j] not in "tr": j += 1
            s = s[j:]
            if len(s) < 5 or s[0] not in "tr": return None
            arb = int(s[1:4], 16); dlc = int(s[4], 16)
            data = bytes.fromhex(s[5:5+dlc*2]) if dlc else b""
            return (arb >> 5, arb & 0x1F, data)
        except Exception:
            return None

    def rtr(self, node, cmd, timeout=0.2):
        self.send(node, cmd, rtr=True)
        end = time.time() + timeout; got = None
        while time.time() < end:
            for n, c, d in self.pump():
                if n == node and c == cmd: got = d
            if got is not None: return got
            time.sleep(0.003)
        return got

    def get_pos(self, node, retries=6):
        for _ in range(retries):
            d = self.rtr(node, CMD_GET_ENCODER_EST)
            if d and len(d) >= 8:
                return struct.unpack("<ff", d[0:8])[0]
        return None

    def get_iq(self, node):
        d = self.rtr(node, CMD_GET_IQ, 0.12)
        return struct.unpack("<ff", d[0:8])[1] if d and len(d) >= 8 else None

    def wait_hb(self, node, timeout=3.0):
        end = time.time() + timeout
        while time.time() < end:
            self.pump()
            if node in self.hb: return self.hb[node]
            time.sleep(0.005)
        return None


def any_fault(bus):
    bus.pump()
    for n, (e, s) in bus.hb.items():
        if e: return n, e
    return None


def idle_all(bus, nodes):
    for n in nodes:
        bus.send(n, CMD_SET_AXIS_STATE, struct.pack("<i", IDLE))


def setup(bus, node):
    hb = bus.wait_hb(node)
    if hb is None:
        print("  node %d: NO HEARTBEAT -> abort" % node, flush=True); return None
    err, state = hb
    print("  node %d: err=0x%X state=%s" % (node, err, STATE_NAMES.get(state, state)), flush=True)
    if err:
        print("  node %d: axis error set -> abort" % node, flush=True); return None
    pos = bus.get_pos(node)
    if pos is None:
        print("  node %d: no encoder est -> abort" % node, flush=True); return None
    bus.send(node, CMD_CLEAR_ERRORS); time.sleep(0.12)
    bus.send(node, CMD_SET_CONTROLLER_MODES, struct.pack("<ii", CONTROL_POSITION, INPUT_PASSTHROUGH))
    bus.send(node, CMD_SET_INPUT_POS, struct.pack("<fhh", pos, 0, 0)); time.sleep(0.1)
    bus.send(node, CMD_SET_AXIS_STATE, struct.pack("<i", CLOSED_LOOP))
    end = time.time() + 3.0
    while time.time() < end:
        bus.pump()
        e, s = bus.hb.get(node, (0, 0))
        if e:
            print("  node %d: err=0x%X on entry -> abort" % (node, e), flush=True); return None
        if s == CLOSED_LOOP:
            print("  node %d: CLOSED_LOOP at %.1f deg" % (node, pos * 360), flush=True); return pos
        time.sleep(0.01)
    print("  node %d: closed-loop entry timeout -> abort" % node, flush=True); return None


def move_all(bus, nodes, target, rate_deg_s, iq_cap=None, label="", log=False):
    """Simultaneously ramp every node from its own current pos to a shared target
    (turns) at rate_deg_s. All nodes commanded each tick -> they move together.
    Returns True on success, False on fault/Iq-trip."""
    rate = rate_deg_s / 360.0
    dt = 0.05
    step = rate * dt
    cmd = {}
    for n in nodes:
        p = bus.get_pos(n)
        cmd[n] = p if p is not None else target
    done = {n: False for n in nodes}
    tick = 0
    while not all(done.values()):
        for n in nodes:
            if abs(target - cmd[n]) <= step:
                cmd[n] = target; done[n] = True
            else:
                cmd[n] += step if target > cmd[n] else -step
            bus.send(n, CMD_SET_INPUT_POS, struct.pack("<fhh", cmd[n], 0, 0))
            time.sleep(0.004)  # space frames so the slcan bridge doesn't drop the 2nd node
        time.sleep(dt)
        f = any_fault(bus)
        if f:
            print("  %s FAULT node %d err=0x%X -> stop" % (label, f[0], f[1]), flush=True)
            return False
        if iq_cap is not None:
            for n in nodes:
                iq = bus.get_iq(n)
                if iq is not None and abs(iq) > iq_cap:
                    print("  %s node %d Iq=%.2f>%.2f (bind/endstop) -> stop"
                          % (label, n, iq, iq_cap), flush=True)
                    return False
        tick += 1
        if log and tick % 6 == 0:
            ps = "  ".join("n%d=%.0f" % (n, (bus.get_pos(n) or 0) * 360) for n in nodes)
            print("    %s  cmd=%.0f  actual: %s" % (label, target * 360, ps), flush=True)
    return True


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--port", default=None)
    p.add_argument("--nodes", default="13,23")
    p.add_argument("--low", type=float, default=5.0)
    p.add_argument("--high", type=float, default=85.0)
    p.add_argument("--cycles", type=int, default=15)
    p.add_argument("--rate", type=float, default=40.0)
    p.add_argument("--probe-rate", type=float, default=25.0)
    p.add_argument("--iq-cap", type=float, default=6.0)
    args = p.parse_args()

    port = args.port or find_bridge()
    nodes = [int(x) for x in args.nodes.split(",")]
    print("bridge:", port, " nodes:", nodes, flush=True)
    bus = Bus(port)

    print("\n[setup: closed loop on all]", flush=True)
    for n in nodes:
        if setup(bus, n) is None:
            idle_all(bus, nodes); return 2

    lowt, hight = args.low / 360.0, args.high / 360.0

    # home BOTH to the low start first (legs may start at different angles;
    # this brings them together before the synchronized demo strokes)
    print("\n[home -> %.0f deg (align both)]" % args.low, flush=True)
    if not move_all(bus, nodes, lowt, args.probe_rate, iq_cap=args.iq_cap, label="home", log=True):
        idle_all(bus, nodes); return 3

    # slow SYNCHRONIZED validation swing DOWN to high, both legs together
    print("\n[sync validation swing -> %.0f deg @ %.0f deg/s, Iq cap %.1f A]"
          % (args.high, args.probe_rate, args.iq_cap), flush=True)
    if not move_all(bus, nodes, hight, args.probe_rate, iq_cap=args.iq_cap, label="down", log=True):
        move_all(bus, nodes, lowt, args.probe_rate, label="recover")
        idle_all(bus, nodes); return 4
    print("  both reached %.0f deg together" % args.high, flush=True)
    if not move_all(bus, nodes, lowt, args.probe_rate, label="up", log=True):
        idle_all(bus, nodes); return 4
    print("  both back at %.0f deg" % args.low, flush=True)

    # fast synchronized cycles
    print("\n[%d cycles  %.0f<->%.0f deg @ %.0f deg/s]"
          % (args.cycles, args.low, args.high, args.rate), flush=True)
    for c in range(1, args.cycles + 1):
        t0 = time.time()
        if not move_all(bus, nodes, hight, args.rate, label="down"):
            idle_all(bus, nodes); return 5
        if not move_all(bus, nodes, lowt, args.rate, label="up"):
            idle_all(bus, nodes); return 5
        print("  cycle %2d/%d  (%.1fs)" % (c, args.cycles, time.time() - t0), flush=True)

    # park at low + idle
    print("\n[park -> %.0f deg + idle]" % args.low, flush=True)
    move_all(bus, nodes, lowt, args.probe_rate, label="park")
    time.sleep(0.3)
    for n in nodes:
        print("  node %d final %.1f deg" % (n, (bus.get_pos(n) or 0) * 360), flush=True)
    idle_all(bus, nodes)
    print("  idled. done.", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
