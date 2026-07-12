#!/usr/bin/env python3
"""Pure-CAN gentle spin test for an ODrive axis via the ESP32 slcan bridge.

No USB to the ODrive. Talks ODrive CAN Simple over the central-firmware USB-CAN
bridge (a dumb slcan `t`/`r` pass-through on a /dev/cu.usbmodem* serial port).

Sequence (all over CAN):
  1. Passive-listen for the axis heartbeat -> validates bridge + bus + node_id
     addressing and prints the current axis error/state BEFORE commanding.
  2. Clear errors.
  3. Set VELOCITY control + VEL_RAMP input, raise vel_limit.
  4. Enter CLOSED_LOOP_CONTROL; abort if the heartbeat reports any axis error.
  5. Gentle velocity steps in both directions, RTR-polling encoder estimates to
     confirm real motion; watches heartbeat for faults each step.
  6. Zero velocity, back to IDLE.

Bench context: bitrate 250 kbps (matches TWAI_BITRATE and odrv.can baud), node
id 13 (leg 1 knee, board 367836893335). Motor's thermistor is disabled in flash
(shorted), so there is NO over-temp protection -- keep runs gentle and no-load.
"""
import argparse
import glob
import struct
import sys
import time

import serial  # pyserial

# ODrive CAN Simple command IDs (5-bit cmd, 6-bit node).
CMD_HEARTBEAT = 0x001
CMD_SET_AXIS_STATE = 0x007
CMD_GET_ENCODER_EST = 0x009
CMD_SET_CONTROLLER_MODES = 0x00B
CMD_SET_INPUT_VEL = 0x00D
CMD_SET_VEL_LIMIT = 0x00F
CMD_GET_IQ = 0x014
CMD_GET_VBUS = 0x017
CMD_CLEAR_ERRORS = 0x018

AXIS_STATE_IDLE = 1
AXIS_STATE_CLOSED_LOOP = 8
CONTROL_MODE_VELOCITY = 2
INPUT_MODE_PASSTHROUGH = 1
INPUT_MODE_VEL_RAMP = 2

STATE_NAMES = {1: "IDLE", 3: "CALIB", 8: "CLOSED_LOOP"}


def find_bridge():
    ports = sorted(glob.glob("/dev/cu.usbmodem*"))
    if not ports:
        raise RuntimeError("no /dev/cu.usbmodem* bridge found")
    return ports[-1]


class Bridge:
    def __init__(self, port, node):
        self.node = node
        self.base = node << 5
        # native USB-CDC: baud is ignored. Don't toggle DTR/RTS on open.
        self.ser = serial.Serial(port, 115200, timeout=0.05)
        self.buf = b""

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

    def _arb(self, cmd):
        return self.base | cmd

    def send(self, cmd, data=b"", rtr=False):
        arb = self._arb(cmd)
        if rtr:
            line = "r%03X%X" % (arb, 8)
        else:
            line = "t%03X%X%s" % (arb, len(data), data.hex().upper())
        self.ser.write((line + "\n").encode())
        self.ser.flush()

    def poll(self):
        """Read available bytes, yield parsed frames as (cmd, data_bytes)."""
        out = []
        chunk = self.ser.read(512)
        if chunk:
            self.buf += chunk
        while True:
            # bridge terminates received frames with '\r'
            i = self.buf.find(b"\r")
            if i < 0:
                # tolerate '\n' too
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
        """Poll until a frame with want_cmd arrives or timeout. Returns data or None.
        collect: optional dict cmd->latest data to keep updating (e.g. heartbeat)."""
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


def decode_heartbeat(data):
    if len(data) < 8:
        return None, None
    err = struct.unpack("<I", data[0:4])[0]
    state = struct.unpack("<I", data[4:8])[0]
    return err, state


def decode_pair_f(data):
    if len(data) < 8:
        return None, None
    return struct.unpack("<ff", data[0:8])


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--port", default=None, help="serial port (default: last /dev/cu.usbmodem*)")
    p.add_argument("--node", type=int, default=13, help="ODrive CAN node id")
    p.add_argument("--speeds", type=float, nargs="+", default=[1.0, -1.0, 2.0, -2.0],
                   help="motor turns/s targets to step through")
    p.add_argument("--dwell", type=float, default=2.0, help="seconds to hold each speed")
    p.add_argument("--vel-limit", type=float, default=8.0)
    p.add_argument("--passthrough", action="store_true",
                   help="use INPUT_MODE_PASSTHROUGH (direct vel) instead of VEL_RAMP")
    args = p.parse_args()
    input_mode = INPUT_MODE_PASSTHROUGH if args.passthrough else INPUT_MODE_VEL_RAMP

    port = args.port or find_bridge()
    print("bridge port:", port, "  node:", args.node, "  base id: 0x%03X" % (args.node << 5), flush=True)
    br = Bridge(port, args.node)

    hb = {}
    try:
        # 1. passive listen -----------------------------------------------------
        print("\n[1] listening for heartbeat (validates bridge + bus + node addr)...", flush=True)
        data = br.read_until(CMD_HEARTBEAT, timeout=3.0, collect=hb)
        if data is None:
            print("  NO HEARTBEAT. Check: bridge powered, CANH/CANL + termination, "
                  "node id, ODrive powered. Aborting.", flush=True)
            return 2
        err, state = decode_heartbeat(data)
        print("  heartbeat OK: axis_error=0x%X state=%s(%d)" % (err, STATE_NAMES.get(state, "?"), state), flush=True)

        # vbus over CAN as an extra link check
        vb = br.read_until(CMD_GET_VBUS, timeout=0.6) or (br.send(CMD_GET_VBUS, rtr=True) or
                                                          br.read_until(CMD_GET_VBUS, timeout=0.6))
        if vb:
            print("  vbus=%.2f V" % struct.unpack("<f", vb[0:4])[0], flush=True)

        # 2. clear errors -------------------------------------------------------
        print("\n[2] clearing errors...", flush=True)
        br.send(CMD_CLEAR_ERRORS)
        time.sleep(0.3)
        data = br.read_until(CMD_HEARTBEAT, timeout=1.5, collect=hb)
        err, state = decode_heartbeat(data) if data else (None, None)
        print("  after clear: axis_error=0x%X state=%s" % (err, STATE_NAMES.get(state, state)), flush=True)

        # 3. controller mode + vel limit ---------------------------------------
        print("\n[3] set VELOCITY / %s, vel_limit=%.1f..." %
              ("PASSTHROUGH" if args.passthrough else "VEL_RAMP", args.vel_limit), flush=True)
        br.send(CMD_SET_CONTROLLER_MODES,
                struct.pack("<ii", CONTROL_MODE_VELOCITY, input_mode))
        br.send(CMD_SET_VEL_LIMIT, struct.pack("<f", args.vel_limit))
        br.send(CMD_SET_INPUT_VEL, struct.pack("<ff", 0.0, 0.0))
        time.sleep(0.2)

        # 4. closed loop --------------------------------------------------------
        print("\n[4] requesting CLOSED_LOOP_CONTROL...", flush=True)
        br.send(CMD_SET_AXIS_STATE, struct.pack("<i", AXIS_STATE_CLOSED_LOOP))
        entered = False
        end = time.time() + 3.0
        while time.time() < end:
            data = br.read_until(CMD_HEARTBEAT, timeout=0.5, collect=hb)
            if data:
                err, state = decode_heartbeat(data)
                if err:
                    print("  axis_error=0x%X during entry -> ABORT" % err, flush=True)
                    br.send(CMD_SET_AXIS_STATE, struct.pack("<i", AXIS_STATE_IDLE))
                    return 3
                if state == AXIS_STATE_CLOSED_LOOP:
                    entered = True
                    break
        if not entered:
            print("  did not reach CLOSED_LOOP (state=%s) -> ABORT" % STATE_NAMES.get(state, state), flush=True)
            br.send(CMD_SET_AXIS_STATE, struct.pack("<i", AXIS_STATE_IDLE))
            return 3
        print("  in CLOSED_LOOP.", flush=True)

        # 5. gentle spins -------------------------------------------------------
        print("\n[5] gentle spins (RTR-polling encoder estimates)...", flush=True)
        for spd in args.speeds:
            print("  target %+.2f t/s for %.1fs:" % (spd, args.dwell), flush=True)
            br.send(CMD_SET_INPUT_VEL, struct.pack("<ff", spd, 0.0))
            t_end = time.time() + args.dwell
            vels = []
            while time.time() < t_end:
                br.send(CMD_GET_ENCODER_EST, rtr=True)
                est = br.read_until(CMD_GET_ENCODER_EST, timeout=0.2, collect=hb)
                if est:
                    pos, vel = decode_pair_f(est)
                    vels.append(vel)
                # fault watch
                if CMD_HEARTBEAT in hb:
                    err, state = decode_heartbeat(hb[CMD_HEARTBEAT])
                    if err:
                        print("    axis_error=0x%X -> STOP" % err, flush=True)
                        br.send(CMD_SET_INPUT_VEL, struct.pack("<ff", 0.0, 0.0))
                        br.send(CMD_SET_AXIS_STATE, struct.pack("<i", AXIS_STATE_IDLE))
                        return 4
                time.sleep(0.05)
            if vels:
                print("    vel_estimate: mean %+.2f  min %+.2f  max %+.2f  (n=%d)" %
                      (sum(vels) / len(vels), min(vels), max(vels), len(vels)), flush=True)
            else:
                print("    (no encoder estimate replies)", flush=True)

        # 6. stop ---------------------------------------------------------------
        print("\n[6] ramping to 0 and IDLE...", flush=True)
        br.send(CMD_SET_INPUT_VEL, struct.pack("<ff", 0.0, 0.0))
        time.sleep(1.0)
        br.send(CMD_SET_AXIS_STATE, struct.pack("<i", AXIS_STATE_IDLE))
        time.sleep(0.3)
        data = br.read_until(CMD_HEARTBEAT, timeout=1.5, collect=hb)
        if data:
            err, state = decode_heartbeat(data)
            print("  final: axis_error=0x%X state=%s" % (err, STATE_NAMES.get(state, state)), flush=True)
        print("\nDONE.", flush=True)
        return 0
    except KeyboardInterrupt:
        print("\ninterrupted -> IDLE", flush=True)
        br.send(CMD_SET_INPUT_VEL, struct.pack("<ff", 0.0, 0.0))
        br.send(CMD_SET_AXIS_STATE, struct.pack("<i", AXIS_STATE_IDLE))
        return 130
    finally:
        br.close()


if __name__ == "__main__":
    sys.exit(main())
