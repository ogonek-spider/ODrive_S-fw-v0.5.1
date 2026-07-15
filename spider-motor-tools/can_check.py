#!/usr/bin/env python3
"""Read-only CAN health probe for one or more ODrive joints via the slcan bridge.

Passive/observational only -- it never commands motion or state changes. For
each node it listens for the heartbeat (axis error + state), then RTR-polls
vbus, encoder estimates (joint pos / motor vel via the split-feedback patch),
and Iq. Safe to run on a live robot.
"""
import argparse
import glob
import struct
import time

import serial

CMD_HEARTBEAT = 0x001
CMD_GET_ENCODER_EST = 0x009
CMD_GET_IQ = 0x014
CMD_GET_TEMP = 0x015
CMD_GET_VBUS = 0x017

STATE_NAMES = {0: "UNDEFINED", 1: "IDLE", 3: "CALIB", 8: "CLOSED_LOOP"}


def find_bridge():
    ports = sorted(glob.glob("/dev/cu.usbmodem*"))
    if not ports:
        raise RuntimeError("no /dev/cu.usbmodem* bridge found")
    return ports[-1]


class Bridge:
    def __init__(self, port):
        self.ser = serial.Serial(port, 115200, timeout=0.05)
        self.buf = b""

    def send(self, node, cmd, rtr=True, data=b""):
        arb = (node << 5) | cmd
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
            return (arb >> 5, arb & 0x1F, data)
        except Exception:
            return None

    def get(self, node, cmd, timeout=0.6):
        """RTR-poll a node/cmd, return latest matching data or None."""
        self.send(node, cmd, rtr=True)
        end = time.time() + timeout
        got = None
        while time.time() < end:
            for n, c, data in self.poll():
                if n == node and c == cmd:
                    got = data
            if got is not None:
                return got
            time.sleep(0.005)
        return got

    def listen(self, node, cmd, timeout=3.0):
        end = time.time() + timeout
        got = None
        while time.time() < end:
            for n, c, data in self.poll():
                if n == node and c == cmd:
                    got = data
            if got is not None:
                return got
            time.sleep(0.005)
        return got


def probe(br, node, label):
    print("\n=== node %d  (%s) ===" % (node, label), flush=True)
    hb = br.listen(node, CMD_HEARTBEAT, timeout=3.0)
    if hb is None or len(hb) < 8:
        print("  NO HEARTBEAT -- node not on bus / unpowered / wrong id", flush=True)
        return False
    err, state = struct.unpack("<II", hb[0:8])
    print("  heartbeat: axis_error=0x%X  state=%s(%d)" %
          (err, STATE_NAMES.get(state, "?"), state), flush=True)

    vb = br.get(node, CMD_GET_VBUS)
    if vb and len(vb) >= 4:
        print("  vbus=%.2f V" % struct.unpack("<f", vb[0:4])[0], flush=True)

    est = br.get(node, CMD_GET_ENCODER_EST)
    if est and len(est) >= 8:
        pos, vel = struct.unpack("<ff", est[0:8])
        print("  encoder: pos=%.4f turn (%.1f deg)  vel=%.3f t/s" %
              (pos, pos * 360.0, vel), flush=True)

    iq = br.get(node, CMD_GET_IQ)
    if iq and len(iq) >= 8:
        iq_sp, iq_meas = struct.unpack("<ff", iq[0:8])
        print("  Iq: setpoint=%.3f A  measured=%.3f A" % (iq_sp, iq_meas), flush=True)

    tp = br.get(node, CMD_GET_TEMP)
    if tp and len(tp) >= 8:
        fet, motor = struct.unpack("<ff", tp[0:8])
        print("  temp: fet=%.1f C  motor=%.1f C" % (fet, motor), flush=True)
    return err == 0


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--port", default=None)
    p.add_argument("--nodes", default="13:motor#8 leg1-knee,23:motor#3 leg2-knee",
                   help="comma list of node:label")
    args = p.parse_args()
    port = args.port or find_bridge()
    print("bridge port:", port, flush=True)
    br = Bridge(port)
    allok = True
    for item in args.nodes.split(","):
        nid, _, label = item.partition(":")
        ok = probe(br, int(nid), label or "?")
        allok = allok and ok
    print("\n%s" % ("ALL NODES OK (error=0)" if allok else "SOME NODES HAVE ERRORS / MISSING"), flush=True)


if __name__ == "__main__":
    main()
