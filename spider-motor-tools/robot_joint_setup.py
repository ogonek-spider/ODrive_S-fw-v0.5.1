#!/usr/bin/env python3
"""Bring a bench-characterized motor onto the robot: flash, CAN, joint encoder.

    robot_joint_setup.py --motor 13 --position 6-3

Runs the whole mount-on-robot sequence for one board, in the order that has to
be respected:

  1. identify board + preflight the build
  2. BACK UP config to configs/ (before anything can destroy it)
  3. flash the current build          (skipped if the board already runs it)
  4. RESTORE config + verify field-by-field against the JSON
  5. CAN: can_node_id = leg*10 + joint, and MUTE axis1's heartbeat
  6. encoders: axis0 commutation health (FATAL if dead), then axis1 joint
     encoder (MT6701) config + health triage
  7. split feedback + the predefined position-loop gains for this joint
  8. save to flash
  9. reboot and re-verify everything from NVM

Order matters. The flash wipes NVM whenever config_version changed, so the
backup must precede it and the restore must follow it. CAN and the joint
encoder are configured AFTER the restore because the backup JSON carries the
OLD can_node_id (usually 0) and a blank axis1 encoder -- restoring after
setting them would silently undo both.

This tool NEVER arms the motor and never commands motion. It is safe to run on
a mounted leg. It does not calibrate; commutation (offset/Kt) is carried over
from the bench via the config restore.

Step 7 writes the per-joint gains from JOINT_TUNING below. Gains only take
effect once something else arms the axis, so writing them here is still
motionless -- but see the position_direction warning in that table.

Position is the physical joint label `<leg>-<joint>`, leg 1..6, joint 1=coxa /
2=femur / 3=knee, giving can_node_id = leg*10 + joint (see CAN_NODE_ID_MAP.md).

NOTE: strips cwd from sys.path so ./odrive does not shadow the installed pkg.
"""
import argparse
import datetime
import json
import math
import os
import subprocess
import sys
import time

sys.path = [p for p in sys.path if p not in ("", ".", os.getcwd())]

import odrive
from odrive.device_manager import close_device_manager

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
ODRIVETOOL = os.path.join(REPO, ".venv", "bin", "odrivetool")
CONFIG_DIR = os.path.join(HERE, "configs")

# Joint-side MT6701 after the gearbox, on axis1.
MT6701_MODE = 261
MT6701_CPR = 16384
MT6701_CS_PIN = 6

JOINT_NAMES = {1: "coxa", 2: "femur", 3: "knee"}

ENCODER_ERROR_ABS_SPI_COM_FAIL = 0x80
ENCODER_ERROR_ABS_SPI_NOT_READY = 0x100

CONTROL_MODE_POSITION = 3
INPUT_MODE_PASSTHROUGH = 1

# ---------------------------------------------------------------------------
# Predefined per-joint position-loop tuning.
#
# With split feedback (load_encoder_axis = 1) the position error is measured in
# OUTPUT turns, so pos_gain is a JOINT-side gain and therefore scales with the
# gearbox ratio: a motor-side gain of ~20-23 becomes ~23*ratio at the joint.
# That is why a value tuned on a 108:1 femur must NEVER be copied onto a joint
# with a different box -- on a 35:1 knee the same number is ~3x too stiff and
# the joint oscillates.
#
# Keyed by joint number (1 = coxa, 2 = femur, 3 = knee). A position with an
# unmeasured gearbox would be None, and the script then REFUSES to write gains
# unless you pass --gearbox-ratio (gain is derived) or --pos-gain (explicit).
#
# vel_gain / vel_integrator_gain are motor-side (vel_encoder_axis = 0), so they
# do NOT scale with the ratio -- but "leave them at the ODrive defaults" is NOT
# safe on a low-ratio joint. On the leg6 coxa (1:6) the default vel_gain 0.1667
# overshot a 4 deg step by 94%; raising it to 2.0 cut that to 8-20%, while
# pos_gain changes did almost nothing. On a backdrivable box, DAMPING is the
# parameter that matters, so vel_gain is now a per-joint value too.
#
# There are two ways to damp a backdrivable joint and they are alternatives:
#   * low vel_limit  (joint 3 / knee: 0.6 t/s) -- simple, but the joint then
#     faults CONTROLLER_ERROR_OVERSPEED (0x1) whenever gravity backdrives it
#     faster than the limit. Seen on the coxa at vel_limit 1.0.
#   * high vel_gain  (joint 1 / coxa: 2.0 at vel_limit 2.0) -- keeps enough
#     speed headroom that gravity never trips overspeed, and damps with torque
#     instead. Prefer this where a descent has to be fault-free.
#
# Do NOT lower vel_integrator_gain to damp overshoot on a gravity-loaded joint.
# The integrator is what supplies the steady torque holding the limb up: on the
# leg6 knee, dropping it from 0.3333 to 0.1 stopped the joint moving at ALL
# (1.6 A, zero motion) at a pos_gain that worked fine with the default. Damp
# with vel_limit instead.
MOTOR_SIDE_POS_GAIN = 23.0  # pos_gain per unit of gearbox ratio

_DEFAULT_VEL_GAIN = 0.1666666716337204
_DEFAULT_VEL_INTEGRATOR_GAIN = 0.3333333432674408

JOINT_TUNING = {
    1: dict(
        ratio=6.02,  # measured motor-vs-joint on leg6; matches the 1:6 spec
        pos_gain=140.0,
        # DAMPING, not pos_gain, is what tunes this joint -- see the vel_gain
        # note above. 2.0 is ~12x the ODrive default and that is deliberate.
        vel_gain=2.0,
        vel_integrator_gain=0.8,
        # Kept HIGH on purpose: enough headroom that a gravity backdrive never
        # trips CONTROLLER_ERROR_OVERSPEED. Damping comes from vel_gain instead.
        vel_limit=2.0,
        verified=True,
        source="leg6 coxa, motor #2 / node 61, USB step sweep 2026-08-01: ratio "
               "measured 6.02:1 (motor +0.2261 turn -> joint +0.0375 turn, "
               "sign +1). pos_gain 140 with vel_gain 2.0 / vel_i 0.8 / "
               "vel_limit 2.0 gives 7-20% overshoot, <=0.23 deg residual and "
               "0.4-1.4 s settle on +-4 and +-8 deg steps. vel_gain dominates: "
               "0.17 (ODrive default) overshot 94% and blew the test guard, "
               "0.5 -> 45%, 1.0 -> 26%, 2.0 -> 8-20%, 3.0 -> 33% (worse). "
               "pos_gain 300 was worse than 140 (37-62% overshoot) with no "
               "accuracy gain, so do NOT reach for pos_gain here",
    ),
    2: dict(
        ratio=108.0,  # 6 x 6 x 3; measured 112.9:1 by motor-vs-joint sweep
        pos_gain=2500.0,
        vel_gain=_DEFAULT_VEL_GAIN,
        vel_integrator_gain=_DEFAULT_VEL_INTEGRATOR_GAIN,
        vel_limit=10.0,
        verified=True,
        source="leg6 femur, motor #9 / node 62, USB step sweep 2026-08-01: "
               "gain 300/700/1500 all settled in 3.3-3.9 s with 0.3-0.5 deg "
               "residual, 2500 settled in 0.27 s with -0.04 deg, 5000 only "
               "added overshoot",
    ),
    3: dict(
        ratio=6.0,
        pos_gain=140.0,
        vel_gain=_DEFAULT_VEL_GAIN,
        vel_integrator_gain=_DEFAULT_VEL_INTEGRATOR_GAIN,
        # A KNEE IS BACKDRIVABLE at 1:6, so vel_limit is a tuning parameter here,
        # not a formality: measured on leg6, 0.6 t/s settles with no overshoot
        # while 1.2 gives 78% and 2.0 gives 59%. The joint slams through its
        # target at anything faster.
        vel_limit=0.6,
        verified=True,
        source="leg6 knee, motor #13 / node 63, USB step sweep 2026-08-01: ratio "
               "measured 6.06:1 (motor 0.0905 turn per 5.38 deg of joint). At "
               "vel_limit 0.6 / vel_i 0.3333, pos_gain 140 settled a +5 deg step "
               "to -0.02 deg with 0% overshoot (5.3 s, peak 3.6 A); 60 could not "
               "move the joint at all. leg1 knee runs 130 on a box that measured "
               "5.6-7.4:1 VARYING WITH ANGLE, so re-check both ends of travel",
    ),
}

# +motor velocity -> +joint on every joint measured so far (leg1 coxa/femur/knee,
# leg6 coxa/femur/knee), so +1 is the default. It is still an ASSUMPTION about how this
# particular leg was assembled, and a wrong sign is a runaway the first time the
# axis is armed -- verify it with a small guarded step before trusting it.
DEFAULT_POSITION_DIRECTION = 1


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--motor", required=True,
                   help="Physical motor number, for file names and logging (e.g. 13).")
    p.add_argument("--position", required=True,
                   help="Joint position <leg>-<joint>, e.g. 6-3 (leg 6, knee) -> node 63.")
    p.add_argument("--serial-number",
                   help="Select a specific board (default: the only one on USB).")
    p.add_argument("--elf", default=os.path.join(REPO, "build", "ODriveFirmware.elf"),
                   help="Firmware .elf to flash (NOT the .hex).")
    p.add_argument("--skip-flash", action="store_true",
                   help="Leave the running firmware alone; do config/CAN/encoder only.")
    p.add_argument("--force-flash", action="store_true",
                   help="Flash even if the board already reports the target version.")
    p.add_argument("--no-mt6701", action="store_true",
                   help="Joint encoder not fitted yet: skip axis1 entirely.")
    p.add_argument("--allow-bad-encoder", action="store_true",
                   help="Save the axis1 config even if the MT6701 health check fails.")
    p.add_argument("--encoder-seconds", type=float, default=15.0,
                   help="MT6701 sampling window (default 15).")
    p.add_argument("--heartbeat-ms", type=int, default=100,
                   help="axis0 CAN heartbeat period (default 100).")
    p.add_argument("--no-gains", action="store_true",
                   help="Leave the controller config alone: no split feedback, no gains.")
    p.add_argument("--pos-gain", type=float,
                   help="Override the joint's predefined pos_gain (JOINT-side units).")
    p.add_argument("--gearbox-ratio", type=float,
                   help="This joint's real ratio; pos_gain is derived as "
                        f"{MOTOR_SIDE_POS_GAIN:.0f} x ratio unless --pos-gain is given.")
    p.add_argument("--position-direction", type=int, choices=(1, -1),
                   default=DEFAULT_POSITION_DIRECTION,
                   help="Sign mapping motor velocity to joint position (default "
                        f"{DEFAULT_POSITION_DIRECTION:+d}). VERIFY before arming.")
    p.add_argument("--dry-run", action="store_true",
                   help="Back up and report, but do not flash, write or save.")
    return p.parse_args()


def parse_position(text):
    try:
        leg_s, joint_s = text.split("-")
        leg, joint = int(leg_s), int(joint_s)
    except ValueError:
        raise SystemExit(f"--position must look like 6-3, got {text!r}")
    if not 1 <= leg <= 6:
        raise SystemExit(f"leg must be 1..6, got {leg}")
    if not 1 <= joint <= 3:
        raise SystemExit(f"joint must be 1..3 (1=coxa 2=femur 3=knee), got {joint}")
    node = leg * 10 + joint
    # CAN Simple's node field is 6 bits. 6-3 -> 63 is exactly the ceiling.
    if node > 63:
        raise SystemExit(f"node id {node} exceeds the 6-bit CAN Simple limit of 63")
    return leg, joint, node


def hdr(text):
    print(f"\n{'=' * 70}\n{text}\n{'=' * 70}", flush=True)


def disconnect():
    """Release the USB device before shelling out to odrivetool.

    Only one process can own the device. While this script holds a connection,
    `odrivetool backup-config` / `dfu` / `restore-config` fail with 'device is
    in use by another program'. close_device_manager() invalidates every device
    handle in this process; reconnect with connect() afterwards.
    """
    try:
        close_device_manager()
    except Exception:
        pass
    time.sleep(1.0)


def run_odrivetool(cmd_args, check=True):
    """Release the device, run an odrivetool subcommand, and stay disconnected.

    stdin is closed: odrivetool's prompts (e.g. 'file exists, override?') would
    otherwise die on EOF *after* half-doing the work.
    """
    disconnect()
    return subprocess.run([ODRIVETOOL] + cmd_args, cwd=REPO, check=check,
                          stdin=subprocess.DEVNULL)


def unique_path(path):
    """Never overwrite an existing backup -- a pre-flash config is not reproducible
    once the flash has wiped NVM."""
    if not os.path.exists(path):
        return path
    stem, ext = os.path.splitext(path)
    for n in range(2, 100):
        candidate = f"{stem}-{n}{ext}"
        if not os.path.exists(candidate):
            return candidate
    raise SystemExit(f"too many existing backups for {path}")


def connect(serial=None, timeout=30):
    # A reboot/DFU leaves a stale background handle that makes find_any time out
    # even though the device enumerated fine. Clear it before every reconnect.
    subprocess.run(["pkill", "-f", "odrivetool"], capture_output=True)
    time.sleep(0.5)
    # find_any(serial_number=) is case-sensitive; the board advertises upper case.
    return odrive.find_any(serial_number=serial.upper() if serial else None,
                           timeout=timeout)


def fw_of(dev):
    return (dev.fw_version_major, dev.fw_version_minor, dev.fw_version_revision)


def target_version():
    """The numeric version the build will report, from tools/odrive/version.txt."""
    path = os.path.join(REPO, "tools", "odrive", "version.txt")
    try:
        with open(path) as f:
            raw = f.read().strip()
    except OSError:
        return None
    # e.g. "fw-v0.5.6-mt6701" -> (0, 5, 6). Only the numeric part reaches the board.
    digits = raw.lstrip("fw-").lstrip("v").split("-")[0].split(".")
    try:
        return tuple(int(d) for d in digits[:3])
    except ValueError:
        return None


def check_build_fresh(elf):
    """Warn if any firmware source is newer than the .elf (stale build)."""
    if not os.path.exists(elf):
        raise SystemExit(f"firmware not found: {elf}\nBuild it with ./dockerbuild.sh build")
    elf_mtime = os.path.getmtime(elf)
    newer = []
    for root, dirs, files in os.walk(os.path.join(REPO, "Firmware")):
        dirs[:] = [d for d in dirs if d not in ("build", "autogen", ".git")]
        for name in files:
            if name.endswith((".c", ".cpp", ".h", ".hpp", ".yaml")):
                path = os.path.join(root, name)
                if os.path.getmtime(path) > elf_mtime:
                    newer.append(os.path.relpath(path, REPO))
    if newer:
        print(f"  !! {len(newer)} source file(s) are NEWER than {os.path.relpath(elf, REPO)}:")
        for path in newer[:5]:
            print(f"       {path}")
        print("     The build is STALE -- run ./dockerbuild.sh build first.")
        return False
    print(f"  build is up to date with Firmware/ sources")
    return True


def values_match(a, b):
    """Compare a live property against its JSON value.

    inf == inf must compare EQUAL: a naive abs(a-b) gives nan and reports a
    false mismatch on torque_lim / dc_max_positive_current, which default to inf.
    """
    if isinstance(a, bool) or isinstance(b, bool):
        return bool(a) == bool(b)
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        if isinstance(a, float) or isinstance(b, float):
            if math.isnan(a) and math.isnan(b):
                return True
            if math.isinf(a) or math.isinf(b):
                return a == b
            return math.isclose(a, b, rel_tol=1e-6, abs_tol=1e-9)
    return a == b


def get_path(dev, dotted):
    obj = dev
    for part in dotted.split("."):
        obj = getattr(obj, part)
    return obj


def verify_against_json(dev, path):
    """Field-by-field compare. odrivetool restore-config's exit code is NOT
    trustworthy: on this firmware it raises 'ODrive did not reboot' AFTER a
    successful save, so the restore must be verified by reading it back."""
    with open(path) as f:
        saved = json.load(f)
    missing, mismatched = [], []
    for key, want in saved.items():
        try:
            got = get_path(dev, key)
        except AttributeError:
            missing.append(key)
            continue
        if not values_match(got, want):
            mismatched.append((key, want, got))
    print(f"  compared {len(saved)} fields: {len(mismatched)} mismatched, "
          f"{len(missing)} absent in this firmware")
    for key, want, got in mismatched:
        print(f"    MISMATCH {key}: json={want} board={got}")
    if missing:
        # Read-only/renamed properties (anticogging, can.config.baud_rate) are
        # expected here; they are not restorable and not a failure.
        print(f"    (absent/read-only, expected: {', '.join(missing[:6])}"
              f"{' ...' if len(missing) > 6 else ''})")
    return not mismatched


def circular_spread(counts, cpr):
    """Peak-to-peak spread in counts, correct across the 0/cpr wrap.

    A plain max-min is wrong for an encoder resting on the seam: samples of
    16380 and 4 are 8 counts apart, not 16376, and the naive number then
    false-fails the at-rest check. Offsets are measured signed against the
    first sample, so this is valid for spreads under cpr/2.
    """
    if not counts:
        return 0
    ref = counts[0]
    offs = [((c - ref + cpr // 2) % cpr) - cpr // 2 for c in counts]
    return max(offs) - min(offs)


def mt6701_status(raw24):
    """MT6701 SSI status nibble, bits [9:6] of the 24-bit frame.

    Layout per the reference driver (servo-firmware/lib/mt6701/mt6701.cpp):
    bits 0-1 = field status, bit 2 = button pushed, bit 3 = TRACK LOSS. The
    track-loss bit is the sensor itself reporting that it sees no magnetic
    track -- far stronger evidence of a missing magnet than a wandering angle.
    """
    st = (raw24 >> 6) & 0xF
    return {"nibble": st, "field": st & 0x03,
            "pushed": bool(st & 0x04), "track_loss": bool(st & 0x08)}


def sample_counts(enc, n=30, dt=0.02):
    counts = [enc.count_in_cpr]
    for _ in range(n - 1):
        time.sleep(dt)
        counts.append(enc.count_in_cpr)
    return counts


def check_commutation_encoder(enc, dry_run=False):
    """Is axis0's absolute encoder -- the COMMUTATION encoder -- delivering?

    Judge on DELIVERY, never on error == 0. While axis1 hammers the shared SPI
    bus, axis0 routinely latches ABS_SPI_NOT_READY (0x100), which is a benign
    transient. ABS_SPI_COM_FAIL (0x80) with an all-zero count is not.
    """
    if not dry_run:
        enc.error = 0
        time.sleep(0.5)
    counts = sample_counts(enc)
    spread = circular_spread(counts, enc.config.cpr or 16384)
    ok = (not (enc.error & ENCODER_ERROR_ABS_SPI_COM_FAIL)
          and counts[-1] != 0 and spread < 100)
    note = ""
    if enc.error & ENCODER_ERROR_ABS_SPI_NOT_READY:
        note = "  [ABS_SPI_NOT_READY latched -- benign, axis1 shares the SPI bus]"
    line = (f"  axis0 commutation encoder (mode {enc.config.mode}, "
            f"CS {enc.config.abs_spi_cs_gpio_pin}): count {counts[-1]}, "
            f"spread {spread}, error {hex(enc.error)}"
            f"  -> {'delivering live data' if ok else 'NOT DELIVERING'}{note}")
    return ok, line, counts[-1]


def sample_mt6701(enc, seconds):
    """Sample the joint encoder and return raw health numbers.

    Uses sample_count/bad_crc_count deltas -- NOT mt6701_debug_sample(), which
    does not actually force a transfer (it only bumps a request counter), and
    NOT start_ok_count/start_fail_count, which are never incremented at all.
    """
    enc.error = 0
    time.sleep(0.5)
    s0, b0 = enc.mt6701_debug_sample_count, enc.mt6701_debug_bad_crc_count
    counts = []
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        counts.append(enc.count_in_cpr)
        time.sleep(0.05)
    samples = enc.mt6701_debug_sample_count - s0
    bad = enc.mt6701_debug_bad_crc_count - b0
    raw24 = enc.mt6701_debug_raw24
    return {
        "samples": samples,
        "bad": bad,
        "bad_pct": 100.0 * bad / max(samples, 1),
        "raw24": raw24,
        "status": mt6701_status(raw24),
        "spread": circular_spread(counts, MT6701_CPR),
        "count": counts[-1] if counts else 0,
        "error": enc.error,
    }


def triage_mt6701(health, axis0_ok):
    """Turn the raw numbers into a verdict + the specific thing to go check."""
    pct, raw24, spread = health["bad_pct"], health["raw24"], health["spread"]
    if health["samples"] == 0:
        # No samples at all means the mode was never set (e.g. --dry-run), so
        # 0 bad CRC and 0 spread are meaningless -- never read that as healthy.
        # It also covers the wedge below, which reports a plausible count and a
        # plausible raw24 while transacting nothing at all.
        if health["error"] & ENCODER_ERROR_ABS_SPI_COM_FAIL:
            return False, (f"SAMPLING HAS STOPPED: not one transaction completed, yet "
                           f"count reads {health['count']} and raw24 0x{health['raw24']:06x} "
                           "-- both STALE.\n     This is the abs-SPI wedge, not a wiring "
                           "fault (a bad wire keeps sampling and fails CRC).\n     "
                           "POWER-CYCLE the board; clearing the error, rewriting mode and "
                           "a soft reboot all fail to recover it.")
        return False, ("no samples were taken -- axis1 is not in MT6701 mode, so "
                       "there is nothing to judge.")
    if pct > 99.0:
        if raw24 == 0x000000:
            why = ("MISO reads all ZEROS -- the line is held low. Encoder "
                   "unpowered, or DO shorted to GND.")
        elif raw24 == 0xFFFFFF:
            why = ("MISO reads all ONES -- nothing is driving the line. CSN not "
                   "reaching IO6, DO not reaching MISO, or encoder unpowered.")
        else:
            why = f"frames arrive (raw24=0x{raw24:06x}) but every CRC fails -- check SPI mode/clock."
        board = ("The BOARD side is proven good: axis0's encoder reads live data "
                 "over the SAME SCK/MISO, so the fault is on CS6's side."
                 if axis0_ok else
                 "axis0's encoder is ALSO not delivering -- with BOTH dead the shared "
                 "SCK/MISO or the board itself is suspect, not just this cable.")
        return False, f"WIRING. {why}\n     {board}\n     Expect: DO->MISO, CLK->SCK, CSN->IO6, VCC->3.3V, GND->GND."
    if pct > 20.0:
        return False, (f"{pct:.1f}% of samples fail CRC. That is high even for PWM EMI "
                       "(~0.5% steady is normal on flying leads). Route the encoder "
                       "cable away from the phase wires and add a ferrite.")
    # The sensor's own track-loss bit beats inferring 'no magnet' from a
    # wandering angle: a magnet-less MT6701 still answers with a valid CRC.
    if health["status"]["track_loss"]:
        return False, (f"CRC is perfect but the MT6701 reports TRACK LOSS "
                       f"(status nibble 0x{health['status']['nibble']:x}) -- it sees no "
                       "magnetic track. Fit the diametric magnet centred over the die, "
                       "1-2 mm away.")
    if spread > 50:
        return False, (f"CRC is fine but the count wanders {spread} counts at rest -- "
                       "the magnet is loose, off-centre, or too far from the chip.")
    verdict = (f"HEALTHY ({pct:.4f}% bad CRC, {spread}-count spread at rest, "
               f"status 0x{health['status']['nibble']:x}).")
    if pct > 1.0:
        verdict += " CRC misses are scattered PWM EMI, not consecutive -- tolerated."
    return True, verdict


def resolve_tuning(joint, args):
    """Pick the position-loop gains for this joint. Returns (tuning, why).

    tuning is None when there is nothing trustworthy to write -- an unmeasured
    gearbox with no --gearbox-ratio / --pos-gain. Guessing a joint-side gain is
    not a safe default: it is only right to within the ratio, and the ratio is
    exactly the thing that is unknown.
    """
    table = JOINT_TUNING.get(joint)
    tuning = dict(table) if table else dict(
        ratio=None, pos_gain=None, vel_gain=_DEFAULT_VEL_GAIN,
        vel_integrator_gain=_DEFAULT_VEL_INTEGRATOR_GAIN, vel_limit=10.0,
        verified=False, source="no table entry for this joint")
    notes = []

    if args.gearbox_ratio is not None:
        tuning["ratio"] = args.gearbox_ratio
        tuning["pos_gain"] = MOTOR_SIDE_POS_GAIN * args.gearbox_ratio
        tuning["verified"] = False
        notes.append(f"--gearbox-ratio {args.gearbox_ratio:g} -> pos_gain "
                     f"{tuning['pos_gain']:.0f} ({MOTOR_SIDE_POS_GAIN:.0f} x ratio)")
    if args.pos_gain is not None:
        tuning["pos_gain"] = args.pos_gain
        tuning["verified"] = False
        notes.append(f"--pos-gain {args.pos_gain:g} (explicit override)")

    if tuning["pos_gain"] is None:
        return None, (f"joint {joint} has no measured gearbox ratio in JOINT_TUNING. "
                      "pos_gain is a JOINT-side gain, so it cannot be guessed without "
                      "the ratio -- pass --gearbox-ratio or --pos-gain, or measure the "
                      "ratio first.")
    tuning["position_direction"] = args.position_direction
    return tuning, "; ".join(notes) if notes else "from JOINT_TUNING"


def main():
    args = parse_args()
    leg, joint, node = parse_position(args.position)
    joint_name = JOINT_NAMES[joint]
    today = datetime.datetime.now().strftime("%Y-%m-%d")

    hdr(f"motor #{args.motor}  ->  leg {leg} {joint_name} (position {args.position})"
        f"  ->  can_node_id {node}")
    if args.dry_run:
        print("  DRY RUN: the board is only read. Nothing is flashed, written to")
        print("  the board, or saved. The config backup IS still written to disk.")

    # ---- 1. identify -------------------------------------------------------
    hdr("1/9  identify board")
    dev = connect(args.serial_number)
    serial = format(dev.serial_number, "x")
    want_fw = target_version()
    print(f"  serial            {serial}")
    print(f"  firmware          {'.'.join(map(str, fw_of(dev)))}")
    print(f"  target firmware   {'.'.join(map(str, want_fw)) if want_fw else '(unknown)'}")
    print(f"  user_config_loaded {dev.user_config_loaded}")
    print(f"  vbus              {dev.vbus_voltage:.2f} V")
    print(f"  Kt {dev.axis0.motor.config.torque_constant:.4f}   "
          f"offset {dev.axis0.encoder.config.offset}   "
          f"harmonic {dev.axis0.encoder.config.enable_harmonic_compensation}")
    if not dev.user_config_loaded:
        print("  !! user_config_loaded is False -- this board has NO usable saved config.")
        print("     Restoring a bench backup is the only way to get commutation back.")

    do_flash = not args.skip_flash and (args.force_flash or fw_of(dev) != want_fw)
    if not args.skip_flash:
        check_build_fresh(args.elf)
        if not do_flash:
            print(f"  board already runs {'.'.join(map(str, want_fw))} -- skipping flash "
                  "(use --force-flash to override)")

    # ---- 2. backup ---------------------------------------------------------
    hdr("2/9  back up configuration")
    tag = "%s.%s.%s" % want_fw if want_fw else "flash"
    backup = os.path.join(CONFIG_DIR,
                          f"motor{args.motor}-{serial}-before-{tag}-flash-{today}.json")
    if not dev.user_config_loaded:
        print("  SKIPPED: no config is loaded, a backup would capture only defaults.")
        backup = None
    else:
        os.makedirs(CONFIG_DIR, exist_ok=True)
        backup = unique_path(backup)
        run_odrivetool(["backup-config", backup])
        with open(backup) as f:
            print(f"  saved {len(json.load(f))} fields -> {os.path.relpath(backup, REPO)}")
        dev = connect(args.serial_number)

    # ---- 3. flash ----------------------------------------------------------
    hdr("3/9  flash firmware")
    if args.dry_run or not do_flash:
        print("  skipped")
    else:
        print(f"  flashing {os.path.relpath(args.elf, REPO)} ...")
        run_odrivetool(["dfu", args.elf])
        dev = connect(args.serial_number)
        got = fw_of(dev)
        print(f"  board now reports {'.'.join(map(str, got))}")
        if want_fw and got != want_fw:
            raise SystemExit(f"flash verify FAILED: wanted {want_fw}, got {got}")
        # The flash wipes NVM whenever config_version changed between versions.
        print(f"  user_config_loaded {dev.user_config_loaded}"
              f"{'  (NVM wiped by config_version bump, as expected)' if not dev.user_config_loaded else ''}")

    # ---- 4. restore --------------------------------------------------------
    hdr("4/9  restore + verify configuration")
    if args.dry_run or not do_flash or not backup:
        print("  skipped")
    else:
        # Exit code is unreliable here: restore-config raises DeviceException
        # ('ODrive did not reboot') AFTER successfully saving. Verify instead.
        run_odrivetool(["restore-config", backup], check=False)
        dev = connect(args.serial_number)
        if not verify_against_json(dev, backup):
            raise SystemExit("restore verify FAILED -- fix before continuing.")
        print(f"  user_config_loaded {dev.user_config_loaded}")
        print(f"  Kt {dev.axis0.motor.config.torque_constant:.4f}   "
              f"offset {dev.axis0.encoder.config.offset}   "
              f"harmonic {dev.axis0.encoder.config.enable_harmonic_compensation}")

    # ---- 5. CAN ------------------------------------------------------------
    hdr(f"5/9  CAN: node {node}, mute axis1")
    if args.dry_run:
        print(f"  would set axis0.config.can_node_id = {node}, "
              f"axis1.config.can_heartbeat_rate_ms = 0")
    else:
        dev.axis0.config.can_node_id = node
        dev.axis0.config.can_node_id_extended = False
        dev.axis0.config.can_heartbeat_rate_ms = args.heartbeat_ms
        # Every board's unused axis1 defaults to can_node_id=1 with a 100 ms
        # heartbeat and the firmware TXes it unconditionally -- so every
        # un-muted board on the bus claims node 1 at once. Mute it here, in the
        # same USB session, because reaching a mounted board later means
        # unplugging it. Muting does not change can_node_id: axis1 still ANSWERS
        # RTR on node 1, so keep node 1 reserved and never poll it.
        dev.axis1.config.can_heartbeat_rate_ms = 0
        dev.axis1.config.can_node_id_extended = False
    print(f"  axis0  node {dev.axis0.config.can_node_id}  "
          f"heartbeat {dev.axis0.config.can_heartbeat_rate_ms} ms")
    print(f"  axis1  node {dev.axis1.config.can_node_id}  "
          f"heartbeat {dev.axis1.config.can_heartbeat_rate_ms} ms  <- 0 = muted")
    print(f"  baud   {dev.can.config.baud_rate}")

    # ---- 6. encoders -------------------------------------------------------
    hdr("6/9  encoders: axis0 commutation + axis1 joint (MT6701)")
    encoder_ok = None

    # 6a. axis0's absolute encoder is the COMMUTATION encoder. It is checked
    # FIRST and UNCONDITIONALLY, because without it this joint cannot be armed
    # at all -- a dead axis1 only costs joint position, a dead axis0 costs the
    # motor. It doubles as the control that tells a bad axis1 cable apart from a
    # bad board: both encoders hang off the SAME SCK/MISO, differing only in CS.
    axis0_ok, a0_line, _ = check_commutation_encoder(dev.axis0.encoder, args.dry_run)
    print(a0_line)

    if args.no_mt6701:
        print("  axis1 skipped (--no-mt6701)")
    else:
        enc = dev.axis1.encoder
        if not args.dry_run:
            enc.config.cpr = MT6701_CPR
            enc.config.abs_spi_cs_gpio_pin = MT6701_CS_PIN
            enc.config.enable_phase_interpolation = False
            enc.config.pre_calibrated = True
            enc.error = 0
            # mode LAST: its setter re-inits the CS pin + SPI and starts sampling.
            enc.config.mode = MT6701_MODE
            time.sleep(1.0)
        print(f"  axis1 MT6701: mode {enc.config.mode}  cpr {enc.config.cpr}  "
              f"CS {enc.config.abs_spi_cs_gpio_pin}")

        print(f"  sampling axis1 for {args.encoder_seconds:.0f} s ...")
        health = sample_mt6701(enc, args.encoder_seconds)
        st = health["status"]
        print(f"  samples {health['samples']}  bad_crc {health['bad']} "
              f"({health['bad_pct']:.4f}%)  raw24 0x{health['raw24']:06x}  "
              f"status 0x{st['nibble']:x}"
              f"{'  TRACK LOSS' if st['track_loss'] else ''}")
        print(f"  count_in_cpr {health['count']}  spread {health['spread']}  "
              f"angle {360.0 * health['count'] / MT6701_CPR:.2f} deg  "
              f"error {hex(health['error'])}")
        encoder_ok, why = triage_mt6701(health, axis0_ok)
        print(f"  {'OK  ' if encoder_ok else 'FAIL'} {why}")
        if encoder_ok and not args.dry_run:
            # Re-seed the LINEAR position accumulator from the live absolute
            # count. pos_estimate is seeded once at startup and can be a whole
            # turn out after dropped samples; writing zero_offset onto itself
            # calls reset_user_position() and re-seeds it without a reboot.
            enc.config.zero_offset = enc.config.zero_offset
            time.sleep(0.3)
            print(f"  pos_estimate re-seeded from the absolute count: "
                  f"{enc.pos_estimate:.4f} turn")
        if not encoder_ok and not args.allow_bad_encoder:
            print("\n  axis1 will be left DISABLED (mode 0) rather than saved in a")
            print("  faulted state. The firmware, config restore and CAN setup below")
            print("  are still saved and complete. Fix the wiring, then re-run:")
            print(f"    {os.path.relpath(__file__, REPO)} --motor {args.motor} "
                  f"--position {args.position} --skip-flash")
            if not args.dry_run:
                enc.config.mode = 0

    if not axis0_ok:
        print("\n  !! axis0's COMMUTATION encoder is not delivering data. This joint")
        print("     cannot be armed at all until that is fixed -- FOC has no rotor")
        print("     angle, so it is not a question of tuning or of joint position.")
        print("     FIRST, before suspecting any wiring: POWER-CYCLE the board.")
        print("     On a board running BOTH absolute encoders, one of them can stop")
        print("     transacting altogether -- typically right after save_configuration()")
        print("     + reboot. The two axes' SPI transfers are interleaved onto opposite")
        print("     ADC phases (low_level.cpp, 'prevent the SPI transfers of axis0 and")
        print("     axis1 from conflicting'), and when that scheduling wedges the other")
        print("     encoder keeps working, so the bus looks fine. Neither clearing the")
        print("     error, nor rewriting mode, nor a soft reboot() recovers it -- only")
        print("     removing power. Observed on node 63 in both directions (axis0 dead")
        print("     with axis1 healthy, then the reverse).")
        print("     TELL: sample_count FROZEN (no new samples at all), with count and")
        print("     raw24 stuck at stale values -- not all-zero frames. A cut DO wire")
        print("     keeps sampling and fails CRC; this stops sampling entirely.")
        if encoder_ok:
            print("     axis1's MT6701 reads clean over the SAME SCK/MISO, so the SPI")
            print("     peripheral, clock and MISO line are all proven good: the fault")
            print("     is on axis0's own side -- its CS pin "
                  f"(IO{dev.axis0.encoder.config.abs_spi_cs_gpio_pin}), its supply, or")
            print("     its DO wire. Suspect this first if the other encoder's cable was")
            print("     recently reworked; they share one header.")
        print("     Nothing below writes commutation config, so it is safe to finish")
        print("     this run and re-check afterwards.")

    # ---- 7. gains ----------------------------------------------------------
    hdr(f"7/9  split feedback + position gains ({joint_name})")
    gains_applied = None
    if args.no_gains:
        print("  skipped (--no-gains)")
    elif not axis0_ok:
        # Split feedback is a position-loop setting; with no rotor angle there
        # is no loop to configure, and writing gains now would make the joint
        # look tuned in NVM when it cannot run.
        print("  SKIPPED: axis0's commutation encoder is dead (see step 6). Fix it,")
        print(f"  then re-run with --skip-flash. Controller config left as-is "
              f"(load_encoder_axis={dev.axis0.controller.config.load_encoder_axis}).")
    elif args.no_mt6701 or encoder_ok is False:
        # load_encoder_axis=1 points the position loop at axis1. Pointing it at
        # an encoder that is absent or faulted would close the loop on garbage
        # the moment somebody arms the axis, so leave the controller untouched.
        print("  SKIPPED: the joint encoder is not usable, so split feedback would")
        print("  aim the position loop at a dead axis1. Fix the encoder, then re-run")
        print(f"  with --skip-flash. Controller config left as-is "
              f"(load_encoder_axis={dev.axis0.controller.config.load_encoder_axis}).")
    else:
        tuning, why = resolve_tuning(joint, args)
        if tuning is None:
            print(f"  SKIPPED: {why}")
        else:
            ratio_txt = f"{tuning['ratio']:g}:1" if tuning["ratio"] else "unknown"
            print(f"  gearbox ratio     {ratio_txt}")
            print(f"  pos_gain          {tuning['pos_gain']:.1f}   "
                  f"({why})")
            print(f"  vel_gain          {tuning['vel_gain']:.4f}")
            print(f"  vel_integrator    {tuning['vel_integrator_gain']:.4f}")
            print(f"  vel_limit         {tuning['vel_limit']:.1f} motor turns/s")
            print(f"  split feedback    load_encoder_axis=1 (joint MT6701), "
                  f"vel_encoder_axis=0 (motor AS5047P)")
            print(f"  position_direction {tuning['position_direction']:+d}")
            print(f"  source: {tuning['source']}")
            if not tuning.get("verified"):
                print("  !! these gains are DERIVED, not step-tested on this joint type.")
                print("     Verify with small guarded steps before running a gait.")
            print("  !! position_direction is an assumption about this leg's assembly.")
            print("     A wrong sign is a RUNAWAY on the first arm -- confirm with a")
            print("     small guarded step (can_jog.py) before any real motion.")
            if args.dry_run:
                print("  would write the above to axis0.controller.config")
            else:
                c = dev.axis0.controller.config
                c.load_encoder_axis = 1
                c.vel_encoder_axis = 0
                c.position_direction = tuning["position_direction"]
                c.pos_gain = tuning["pos_gain"]
                c.vel_gain = tuning["vel_gain"]
                c.vel_integrator_gain = tuning["vel_integrator_gain"]
                c.vel_limit = tuning["vel_limit"]
                c.control_mode = CONTROL_MODE_POSITION
                c.input_mode = INPUT_MODE_PASSTHROUGH
                time.sleep(0.3)
                wrote = {
                    "load_encoder_axis": (c.load_encoder_axis, 1),
                    "vel_encoder_axis": (c.vel_encoder_axis, 0),
                    "position_direction": (c.position_direction,
                                           tuning["position_direction"]),
                    "pos_gain": (c.pos_gain, tuning["pos_gain"]),
                    "vel_gain": (c.vel_gain, tuning["vel_gain"]),
                    "vel_integrator_gain": (c.vel_integrator_gain,
                                            tuning["vel_integrator_gain"]),
                    "vel_limit": (c.vel_limit, tuning["vel_limit"]),
                    "control_mode": (c.control_mode, CONTROL_MODE_POSITION),
                    "input_mode": (c.input_mode, INPUT_MODE_PASSTHROUGH),
                }
                bad = [k for k, (got, want) in wrote.items()
                       if not values_match(got, want)]
                if bad:
                    raise SystemExit("controller config did not take: "
                                     + ", ".join(f"{k}={wrote[k][0]} (wanted "
                                                 f"{wrote[k][1]})" for k in bad))
                print(f"  written and read back OK ({len(wrote)} fields)")
                gains_applied = tuning

    # ---- 8. save -----------------------------------------------------------
    hdr("8/9  save to flash")
    if args.dry_run:
        print("  skipped (--dry-run)")
    else:
        try:
            dev.save_configuration()
        except Exception as exc:
            # save_configuration resets the USB transport; the exception is
            # normal on this firmware and does not mean the save failed.
            print(f"  save_configuration raised {type(exc).__name__} "
                  "(expected -- USB transport resets); verifying after reboot")
        print("  saved")

    # ---- 9. reboot + verify ------------------------------------------------
    hdr("9/9  reboot and verify from NVM")
    if args.dry_run:
        print("  skipped (--dry-run)")
        return 0
    try:
        dev.reboot()
    except Exception:
        pass  # reboot always drops the USB link mid-call
    time.sleep(4)
    dev = connect(args.serial_number, timeout=40)

    ok = True
    fw = fw_of(dev)

    # Any encoder error -- including the benign ABS_SPI_NOT_READY transient that
    # a shared SPI bus throws at boot -- also latches AXIS_ERROR_ENCODER_FAILED
    # (0x100) on the axis. Judging the raw boot errors therefore false-fails a
    # healthy board. Report what booted, then clear once and judge what comes
    # BACK, with axis0 judged on whether it DELIVERS data.
    boot_errors = (dev.axis0.error, dev.axis0.motor.error,
                   dev.axis0.encoder.error, dev.axis0.controller.error)
    print(f"  at boot: axis/motor/encoder/controller errors "
          f"{'/'.join(hex(e) for e in boot_errors)} (cleared, re-checking below)")
    for ax in (dev.axis0, dev.axis1):
        ax.encoder.error = 0
        ax.motor.error = 0
        ax.controller.error = 0
        ax.error = 0
    a0_boot_ok, a0_boot_line, _ = check_commutation_encoder(dev.axis0.encoder)
    print(a0_boot_line)
    time.sleep(1.5)
    # ABS_SPI_NOT_READY alone is tolerated; ABS_SPI_COM_FAIL and every other
    # error is not.
    enc_err = dev.axis0.encoder.error & ~ENCODER_ERROR_ABS_SPI_NOT_READY
    residual = (dev.axis0.motor.error, enc_err, dev.axis0.controller.error)

    checks = [
        ("axis0 commutation enc", "delivering" if a0_boot_ok else "NOT DELIVERING",
         a0_boot_ok),
        ("firmware", ".".join(map(str, fw)), fw == want_fw if want_fw else True),
        ("user_config_loaded", dev.user_config_loaded, bool(dev.user_config_loaded)),
        ("axis0 can_node_id", dev.axis0.config.can_node_id,
         dev.axis0.config.can_node_id == node),
        ("axis0 heartbeat_ms", dev.axis0.config.can_heartbeat_rate_ms,
         dev.axis0.config.can_heartbeat_rate_ms == args.heartbeat_ms),
        ("axis1 heartbeat_ms", dev.axis1.config.can_heartbeat_rate_ms,
         dev.axis1.config.can_heartbeat_rate_ms == 0),
        ("can baud", dev.can.config.baud_rate, dev.can.config.baud_rate == 250000),
        ("axis0 torque_constant", round(dev.axis0.motor.config.torque_constant, 4),
         dev.axis0.motor.config.torque_constant > 0.05),
        ("axis0 encoder offset", dev.axis0.encoder.config.offset,
         dev.axis0.encoder.config.offset != 0),
        ("axis0 pre_calibrated", dev.axis0.encoder.config.pre_calibrated,
         bool(dev.axis0.encoder.config.pre_calibrated)),
        ("axis0 state", dev.axis0.current_state, dev.axis0.current_state == 1),
        ("residual errors", "motor/encoder/controller "
         + "/".join(hex(e) for e in residual), not any(residual)),
    ]
    if not args.no_mt6701 and encoder_ok:
        checks.append(("axis1 encoder mode", dev.axis1.encoder.config.mode,
                       dev.axis1.encoder.config.mode == MT6701_MODE))
        a1 = dev.axis1.encoder
        a1_counts = sample_counts(a1)
        checks.append(("axis1 encoder", f"count {a1_counts[-1]}, spread "
                       f"{circular_spread(a1_counts, MT6701_CPR)}, "
                       f"error {hex(a1.error)}",
                       not (a1.error & ENCODER_ERROR_ABS_SPI_COM_FAIL)
                       and a1_counts[-1] != 0))
    if gains_applied:
        c = dev.axis0.controller.config
        checks.append(("ctrl load/vel enc axis",
                       f"{c.load_encoder_axis}/{c.vel_encoder_axis}",
                       (c.load_encoder_axis, c.vel_encoder_axis) == (1, 0)))
        checks.append(("ctrl pos_gain", round(c.pos_gain, 1),
                       values_match(c.pos_gain, gains_applied["pos_gain"])))
        checks.append(("ctrl vel_gain", round(c.vel_gain, 4),
                       values_match(c.vel_gain, gains_applied["vel_gain"])))
        checks.append(("ctrl vel_integrator", round(c.vel_integrator_gain, 4),
                       values_match(c.vel_integrator_gain,
                                    gains_applied["vel_integrator_gain"])))
        checks.append(("ctrl position_direction", c.position_direction,
                       c.position_direction == gains_applied["position_direction"]))
    if dev.axis0.motor_thermistor.config.enabled:
        temp = dev.axis0.motor_thermistor.temperature
        checks.append(("motor thermistor", f"{temp:.1f} C", 0.0 < temp < 60.0))
    else:
        print("  note: motor thermistor DISABLED -- nothing protects this winding.")

    for name, value, good in checks:
        print(f"  {'ok  ' if good else 'FAIL'} {name:24s} {value}")
        ok = ok and good

    hdr(f"{'DONE' if ok else 'INCOMPLETE'}: motor #{args.motor} -> leg {leg} "
        f"{joint_name}, node {node}")
    if not a0_boot_ok:
        print("  axis0 COMMUTATION encoder is not reading after the reboot -- this "
              "joint cannot")
        print("  be armed. POWER-CYCLE the board and re-run with --skip-flash before")
        print("  suspecting wiring: a soft reboot does not clear an abs-SPI latch-up.")
    if not args.no_mt6701 and encoder_ok is False:
        print("  axis1 joint encoder is NOT configured -- fix wiring and re-run "
              "with --skip-flash.")
    print(f"  Record it in {os.path.relpath(os.path.join(HERE, 'CAN_NODE_ID_MAP.md'), REPO)}:")
    print(f"    | {leg} | {joint} | {joint_name:11s} | **{node}** | "
          f"{args.motor} | {serial} |")
    if gains_applied:
        print(f"  Position loop is configured: split feedback + pos_gain "
              f"{gains_applied['pos_gain']:.0f} "
              f"({'step-tested' if gains_applied.get('verified') else 'DERIVED, untested'} "
              f"for a {gains_applied['ratio']:g}:1 box).")
        print("  Still to do on this joint: VERIFY position_direction with a small")
        print("  guarded step, joint zero (axis1 direction + set_zero), travel")
        print("  measurement, endstops (min/max + enable_position_limit).")
    else:
        print("  Still to do on this joint: joint zero + direction, split feedback")
        print("  (load_encoder_axis=1, vel_encoder_axis=0, position_direction),")
        print("  travel measurement, endstops, pos_gain.")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
