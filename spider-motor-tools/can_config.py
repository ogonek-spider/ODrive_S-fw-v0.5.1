#!/usr/bin/env python3
"""Read and write ODrive joint configuration over CAN -- no USB, no replugging.

Talks to the local firmware addition `MSG_CONFIG_ACCESS` (0x01C) /
`MSG_CONFIG_COMMIT` (0x01D), added in fw 0.5.6. Requires that firmware; older
boards simply ignore the frames and every call times out.

  # read everything the joint cares about
  can_config.py --node 12

  # set the endstops in JOINT degrees, verify, then commit to flash
  can_config.py --node 12 --set min=58.2 max=160 --save
  can_config.py --node 12 --set limit_enable=1 --save

  # joint coordinate bring-up, all over CAN
  can_config.py --node 13 --set direction=1
  can_config.py --node 13 --set set_zero=1        # capture current pose as 0
  can_config.py --node 13 --set reseed=1          # fix a lost-turn pos_estimate

Angle-valued parameters (`min`, `max`, `pos`) are given and printed in
DEGREES here; the wire format is turns. Everything else is in its native unit.

Parameters live on the *motor* axis's node id even when they physically belong
to the joint encoder on axis1: the firmware resolves them through
`controller.config.load_encoder_axis`. That is deliberate -- axis1 has its CAN
heartbeat muted on the robot (otherwise every board claims node 1) and is not
addressable on the bus.
"""
import argparse
import struct
import time

from can_goto import Bridge, find_bridge, STATE_NAMES

CMD_CONFIG_ACCESS = 0x01C
CMD_CONFIG_COMMIT = 0x01D

OP_READ, OP_WRITE = 0, 1
ERROR_FLAG = 0x80

COMMIT_MAGIC = 0x0DC0FFEE
ACTION_SAVE, ACTION_REBOOT = 1, 2

STATUS = {
    0x00: "ok",
    0x01: "unknown parameter",
    0x02: "read-only",
    0x03: "value rejected",
    0x04: "requires IDLE",
    0x05: "no target",
    0x06: "bad magic",
    0x07: "busy",
    0x10: "saved to flash",
    0x11: "SAVE FAILED",
}

T_FLOAT, T_INT32, T_BOOL, T_UINT32 = 0, 1, 2, 3

# name -> (param id, type, unit, read-only, "degrees on the wire as turns")
PARAMS = {
    # joint / load encoder (resolved through load_encoder_axis)
    "min":            (0x01, T_FLOAT,  "deg",   False, True),
    "max":            (0x02, T_FLOAT,  "deg",   False, True),
    "limit_enable":   (0x03, T_BOOL,   "",      False, False),
    "direction":      (0x04, T_INT32,  "",      False, False),
    "zero_offset":    (0x05, T_INT32,  "count", False, False),
    "set_zero":       (0x06, T_INT32,  "count", False, False),
    "reseed":         (0x07, T_FLOAT,  "deg",   False, True),
    "pos":            (0x08, T_FLOAT,  "deg",   True,  True),
    "count_in_cpr":   (0x09, T_INT32,  "count", True,  False),
    "cpr":            (0x0A, T_INT32,  "count", True,  False),
    "encoder_error":  (0x0B, T_UINT32, "",      True,  False),
    "turn_snaps":     (0x0C, T_UINT32, "",      True,  False),
    # controller
    "pos_gain":       (0x10, T_FLOAT,  "",      False, False),
    "vel_gain":       (0x11, T_FLOAT,  "",      False, False),
    "vel_int_gain":   (0x12, T_FLOAT,  "",      False, False),
    "vel_limit":      (0x13, T_FLOAT,  "t/s",   False, False),
    "pos_direction":  (0x14, T_INT32,  "",      False, False),
    "load_axis":      (0x15, T_INT32,  "",      False, False),
    "vel_axis":       (0x16, T_INT32,  "",      False, False),
    "input_filter_bw": (0x17, T_FLOAT, "1/s",   False, False),
    # motor
    "current_lim":    (0x20, T_FLOAT,  "A",     False, False),
    "torque_constant": (0x21, T_FLOAT, "Nm/A",  False, False),
    # telemetry
    "fet_temp":       (0x30, T_FLOAT,  "C",     True,  False),
    "motor_temp":     (0x31, T_FLOAT,  "C",     True,  False),
    "motor_therm_en": (0x32, T_BOOL,   "",      False, False),
    # node / bus
    "node_id":        (0x40, T_INT32,  "",      False, False),
    "heartbeat_ms":   (0x41, T_INT32,  "ms",    False, False),
    "other_axis_hb":  (0x42, T_INT32,  "ms",    False, False),
}

BY_ID = {v[0]: k for k, v in PARAMS.items()}

# What `--node N` with no --set prints, in a useful reading order.
SUMMARY = ["pos", "count_in_cpr", "cpr", "encoder_error", "turn_snaps",
           "direction", "zero_offset",
           "min", "max", "limit_enable",
           "pos_gain", "vel_gain", "vel_int_gain", "vel_limit", "pos_direction",
           "load_axis", "vel_axis", "input_filter_bw",
           "current_lim", "torque_constant",
           "fet_temp", "motor_temp", "motor_therm_en",
           "node_id", "heartbeat_ms", "other_axis_hb"]


class ConfigError(RuntimeError):
    pass


def _encode(name, value):
    _, typ, _, _, as_turns = PARAMS[name]
    if typ == T_FLOAT:
        v = float(value) / 360.0 if as_turns else float(value)
        return struct.pack("<f", v)
    return struct.pack("<i", int(float(value)))


def _decode(name_or_id, typ, raw):
    name = name_or_id if isinstance(name_or_id, str) else BY_ID.get(name_or_id)
    as_turns = PARAMS[name][4] if name in PARAMS else False
    if typ == T_FLOAT:
        v = struct.unpack("<f", raw)[0]
        return v * 360.0 if as_turns else v
    if typ == T_UINT32:
        return struct.unpack("<I", raw)[0]
    return struct.unpack("<i", raw)[0]


def access(br, name, value=None, timeout=0.6, retries=3):
    """Read (value=None) or write one parameter. Returns the value AFTER the op."""
    if name not in PARAMS:
        raise ConfigError("unknown parameter %r" % name)
    pid = PARAMS[name][0]
    op = OP_READ if value is None else OP_WRITE
    payload = _encode(name, value) if value is not None else b"\x00\x00\x00\x00"
    frame = bytes([op, pid, 0, 0]) + payload

    for _ in range(retries):
        br.send(CMD_CONFIG_ACCESS, frame)
        end = time.time() + timeout
        while time.time() < end:
            for cmd, data in br.poll():
                if cmd != CMD_CONFIG_ACCESS or len(data) < 8:
                    continue
                if data[1] != pid:
                    continue  # a reply to some other in-flight request
                status, typ = data[2], data[3]
                if status:
                    raise ConfigError("%s: %s" % (name, STATUS.get(status, "status %d" % status)))
                return _decode(name, typ, data[4:8])
            time.sleep(0.005)
    raise ConfigError("%s: no reply from node %d "
                      "(needs fw >= 0.5.6 with the CAN config patch)" % (name, br.node))


def commit(br, action=ACTION_SAVE, timeout=8.0):
    """Save to flash (or reboot). Returns the final status byte."""
    br.send(CMD_CONFIG_COMMIT, struct.pack("<IB", COMMIT_MAGIC, action) + b"\x00\x00\x00")
    end = time.time() + timeout
    seen = None
    while time.time() < end:
        for cmd, data in br.poll():
            if cmd != CMD_CONFIG_COMMIT or len(data) < 2:
                continue
            status = data[1]
            seen = status
            if status in (0x10, 0x11):      # terminal: save finished
                return status
            if status not in (0x00,):       # a rejection is terminal too
                return status
        time.sleep(0.01)
    if seen == 0x00:
        raise ConfigError("save was accepted but never reported completion")
    raise ConfigError("no reply to commit (needs fw >= 0.5.6)")


def dump(br, names=SUMMARY):
    rows = []
    for n in names:
        try:
            rows.append((n, access(br, n), PARAMS[n][2], None))
        except ConfigError as e:
            rows.append((n, None, PARAMS[n][2], str(e)))
    return rows


def fmt_value(v, unit):
    if v is None:
        return "--"
    if isinstance(v, float):
        return "%.4f %s" % (v, unit) if unit else "%.4f" % v
    return "%d %s" % (v, unit) if unit else "%d" % v


def main():
    p = argparse.ArgumentParser(
        description="read/write ODrive joint config over CAN (fw >= 0.5.6)",
        epilog="parameters: " + ", ".join(sorted(PARAMS)))
    p.add_argument("--port", default=None)
    p.add_argument("--node", type=int, required=True)
    p.add_argument("--set", nargs="+", metavar="NAME=VALUE", default=[],
                   help="assignments, applied in order (angles in DEGREES)")
    p.add_argument("--get", nargs="+", metavar="NAME", default=[],
                   help="read only these parameters")
    p.add_argument("--save", action="store_true", help="commit to flash afterwards")
    p.add_argument("--reboot", action="store_true", help="reboot the board afterwards")
    a = p.parse_args()

    port = a.port or find_bridge()
    br = Bridge(port, a.node)
    print("bridge %s, node %d" % (port, a.node), flush=True)

    err, state = br.hb(timeout=3.0)
    if err is None:
        raise SystemExit("no heartbeat from node %d" % a.node)
    print("heartbeat: err=0x%X state=%s" % (err, STATE_NAMES.get(state, state)), flush=True)

    rc = 0
    for item in a.set:
        name, _, val = item.partition("=")
        try:
            got = access(br, name.strip(), val.strip())
            unit = PARAMS[name.strip()][2]
            print("  set %-16s -> %s" % (name.strip(), fmt_value(got, unit)), flush=True)
        except ConfigError as e:
            print("  set %-16s !! %s" % (name.strip(), e), flush=True)
            rc = 1

    if a.save and rc == 0:
        try:
            st = commit(br, ACTION_SAVE)
            print("  save: %s" % STATUS.get(st, "status 0x%X" % st), flush=True)
            if st != 0x10:
                rc = 1
        except ConfigError as e:
            print("  save: %s" % e, flush=True)
            rc = 1

    names = a.get or (SUMMARY if not a.set else [i.partition("=")[0].strip() for i in a.set])
    print("", flush=True)
    for name, value, unit, err_txt in dump(br, names):
        print("  %-16s %-18s %s" % (name, fmt_value(value, unit), err_txt or ""), flush=True)

    if a.reboot:
        try:
            commit(br, ACTION_REBOOT, timeout=2.0)
        except ConfigError:
            pass
        print("\n  reboot requested", flush=True)

    br.close()
    raise SystemExit(rc)


if __name__ == "__main__":
    main()
