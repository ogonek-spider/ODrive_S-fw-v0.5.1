#!/usr/bin/env python3
"""Bring a bench-characterized motor onto the robot: flash, CAN, joint encoder.

    robot_joint_setup.py --position 6-3
    robot_joint_setup.py --position 6-3 --motor 13   # motor number is optional

Runs the whole mount-on-robot sequence for one board, in the order that has to
be respected:

  1. identify board + preflight the build
  2. BACK UP config to configs/ (before anything can destroy it)
  3. flash the current build          (skipped if the board already runs it)
  4. RESTORE config (only if the flash wiped it) + verify field-by-field
  5. CAN: can_node_id = leg*10 + joint, and MUTE axis1's heartbeat
  6. encoders: axis0 commutation health (FATAL if dead), then axis1 joint
     encoder (MT6701) config + health triage
  7. split feedback + the predefined position-loop gains for this joint
  8. save to flash
  9. re-verify everything over a fresh connection -- WITHOUT rebooting
 10. ask the operator to POWER-CYCLE, then re-read the config from NVM

Step 9 deliberately does NOT reboot the board. save_configuration() followed by
reboot() is the one sequence on this hardware that leaves an absolute encoder
wedged: on a board running BOTH abs-SPI encoders (axis0 AS5047P + axis1 MT6701)
one of them stops transacting altogether, and nothing but REMOVING POWER brings
it back -- not clearing the error, not rewriting mode, not another reboot.

Step 10 exists because step 9 CANNOT prove the save. save_configuration() is
void and, when the store fails, only printf()s to the debug UART and leaves
user_config_loaded_ untouched (main.cpp:43-61) -- so a config that never
reached flash reads back perfectly over USB. `user_config_loaded` is readonly
over USB, so this tool cannot use the trick the CAN commit path uses (clear the
flag, save, see whether the firmware sets it back). Only a power cycle can tell
the two apart, and it is the operator who has to perform it. leg2 coxa passed a
full field-by-field verify three times and came up at factory defaults each
time; that then presented as a dead joint encoder (a board with no absolute
encoder in NVM at boot makes a healthy MT6701 read 0xFFFFFF) and cost an
unnecessary encoder swap. --no-power-cycle-check skips it and says so loudly.

Step 4 restores with direct property writes, NOT `odrivetool restore-config`.
That tool calls erase_configuration(), and it is the common factor in all three
of the losses above and in the two earlier "restore did not take" boards.

Order matters. The flash wipes NVM whenever config_version changed, so the
backup must precede it and the restore must follow it. CAN and the joint
encoder are configured AFTER the restore because the backup JSON carries the
OLD can_node_id (usually 0) and a blank axis1 encoder -- restoring after
setting them would silently undo both.

config_version only changes when a config STRUCT changes, so most firmware
bumps (0.5.6 -> 0.5.7 among them) leave the saved configuration intact. Step 4
notices that, compares the live config against the backup, and SKIPS the
restore when they already match -- a restore that is not needed is pure risk on
a mounted joint (fw 0.5.6 dropped axis1's encoder mode on one board and left
axis0 at defaults on another, both while reporting success).

This tool NEVER arms the motor and never commands motion. It is safe to run on
a mounted leg. It does not calibrate; commutation (offset/Kt) is carried over
from the bench via the config restore.

Step 7 writes the per-joint gains from JOINT_TUNING below. Gains only take
effect once something else arms the axis, so writing them here is still
motionless -- but see the position_direction warning in that table.

    robot_joint_setup.py --position 6-3 --gains-only

--gains-only is for a joint that is ALREADY brought up and only needs the
retuned numbers from JOINT_TUNING pushed to it. It writes the controller config
and nothing else: no flash, no backup/restore, no CAN node id, no encoder mode
write, no zero_offset reseed, no reboot. Both axes must be IDLE (writing
pos_gain/vel_gain to an ARMED axis changes the running loop and can kick the
joint), and both encoders are checked read-only before the write, so an already
faulted joint is reported rather than re-tuned. The same parameters can be
pushed over CAN with no USB at all -- see can_config.py (fw 0.5.6+):

    can_config.py --node 63 --set pos_gain=140 vel_gain=2.0 \
        vel_int_gain=0.8 vel_limit=2.0 --save

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
        # Same backdrivable recipe as the coxa above: DAMP WITH vel_gain and
        # leave vel_limit high. The earlier knee tuning did the opposite
        # (vel_limit 0.6, vel_gain at the ODrive default) and it settled fine
        # going up but made the joint UNUSABLE over CAN -- see the vel_limit
        # comment below.
        vel_gain=2.0,
        vel_integrator_gain=0.8,
        # DO NOT lower this to damp the joint. At 0.6 t/s overspeed trips at
        # 1.2 x 0.6 = 0.72 motor t/s = only 43 deg/s at the joint, so a gravity
        # descent sets CONTROLLER_ERROR_OVERSPEED; Controller::update() then
        # returns false, the loop misses its PWM deadline and the motor disarms
        # with CONTROL_DEADLINE_MISSED. The host sees motor.error 0x10 and reads
        # it as a board fault. 2.0 t/s = 143 deg/s of headroom instead.
        vel_limit=2.0,
        verified=True,
        source="leg6 knee, motor #13 / node 63, USB step sweep 2026-08-01: ratio "
               "measured 6.06:1 (motor 0.0905 turn per 5.38 deg of joint). "
               "pos_gain 140 with vel_gain 2.0 / vel_i 0.8 / vel_limit 2.0 ran "
               "+-5 and +-10 deg steps (both directions, incl. gravity descent) "
               "with ZERO faults and <=0.4 deg residual, 1.7-5.4 s settle. The "
               "previous vel_limit 0.6 / default-vel_gain tuning from the same "
               "day faulted every descent with OVERSPEED + "
               "CONTROL_DEADLINE_MISSED. pos_gain 60 could not move the joint at "
               "all. leg1 knee runs 130 on a box that measured 5.6-7.4:1 VARYING "
               "WITH ANGLE, so re-check both ends of travel",
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
    # OPTIONAL on purpose. The motor number labels nothing the script decides:
    # the node id and gains come from --position, and the board's identity comes
    # from its serial, which is in every backup file name anyway. It is also the
    # weakest of the three identifiers -- motor numbers get REUSED between
    # boards, and a board swap (leg1 coxa 2026-08-03: ex-bench board #17 now
    # driving physical motor #12) makes "the motor number" outright ambiguous.
    # Pass it when it helps a human find the file later; leave it out otherwise
    # and the position is used instead.
    p.add_argument("--motor",
                   help="Physical motor number, for file names and logging only "
                        "(e.g. 13). Optional: --position identifies the joint and "
                        "the board serial identifies the board.")
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
    p.add_argument("--gains-only", action="store_true",
                   help="Push ONLY the JOINT_TUNING gains + split feedback to an "
                        "already-configured joint: no flash, no backup/restore, no "
                        "CAN or encoder writes, no reboot. Requires both axes IDLE.")
    p.add_argument("--pos-gain", type=float,
                   help="Override the joint's predefined pos_gain (JOINT-side units).")
    p.add_argument("--gearbox-ratio", type=float,
                   help="This joint's real ratio; pos_gain is derived as "
                        f"{MOTOR_SIDE_POS_GAIN:.0f} x ratio unless --pos-gain is given.")
    p.add_argument("--position-direction", type=int, choices=(1, -1), default=None,
                   help="Sign mapping motor velocity to joint position (default "
                        f"{DEFAULT_POSITION_DIRECTION:+d}; with --gains-only, default "
                        "is whatever the board already has). VERIFY before arming.")
    p.add_argument("--no-power-cycle-check", action="store_true",
                   help="Skip the final step that asks you to power-cycle the "
                        "board and re-reads the config from NVM. ONLY the power "
                        "cycle proves the save reached flash -- see the note in "
                        "the module docstring before using this.")
    p.add_argument("--dry-run", action="store_true",
                   help="Back up and report, but do not flash, write or save.")
    return p.parse_args()


# Written by this tool and therefore worth proving survived a power cycle. The
# controller gains are added at runtime only when step 7 actually wrote them.
NVM_PROOF_PATHS = (
    "axis0.config.can_node_id",
    "axis0.config.can_heartbeat_rate_ms",
    "axis1.config.can_heartbeat_rate_ms",
    "axis0.motor.config.pole_pairs",
    "axis0.motor.config.torque_constant",
    "axis0.motor.config.pre_calibrated",
    "axis0.encoder.config.mode",
    "axis0.encoder.config.offset",
    "axis0.encoder.config.pre_calibrated",
    "axis1.encoder.config.mode",
    "axis1.encoder.config.cpr",
    "axis1.encoder.config.abs_spi_cs_gpio_pin",
    "axis1.encoder.config.direction",
    "axis1.encoder.config.zero_offset",
    "axis0.controller.config.load_encoder_axis",
    "axis0.controller.config.vel_encoder_axis",
)

_GAIN_PROOF_PATHS = (
    "axis0.controller.config.pos_gain",
    "axis0.controller.config.vel_gain",
    "axis0.controller.config.vel_integrator_gain",
    "axis0.controller.config.vel_limit",
    "axis0.controller.config.position_direction",
)


def snapshot(dev, paths):
    return {p: get_path(dev, p) for p in paths}


def power_cycle_proof(dev, expected, serial, skip):
    """Prove the save reached NVM. Only a power cycle can.

    save_configuration() is void and only printf()s to the debug UART on
    failure (main.cpp:43-61), and it leaves user_config_loaded_ at whatever it
    already was -- so a store that never landed is INVISIBLE over USB, and no
    readback can tell the two apart. `user_config_loaded` is readonly over USB,
    so this tool cannot use the trick the CAN commit path uses (clear the flag,
    save, see whether the firmware sets it back).

    That is not theoretical: leg2 coxa passed a full field-by-field verify three
    times and came up at factory defaults each time, which then presented as a
    dead joint encoder and cost an unnecessary module swap.
    """
    if skip:
        print("  SKIPPED (--no-power-cycle-check).")
        print("  !! Nothing here has proven the save reached FLASH. A silent store")
        print("     failure looks EXACTLY like success over USB. Power-cycle the")
        print("     board and re-read before trusting this joint.")
        return None
    print("  Everything above was read from RAM, which a failed save cannot be")
    print("  distinguished from. Remove power from the board, restore it, wait for")
    print("  it to enumerate, then press Enter (Ctrl-C to skip).")
    try:
        input("  power-cycled? [Enter] ")
    except (EOFError, KeyboardInterrupt):
        print("\n  SKIPPED -- no confirmation. NVM is UNPROVEN; re-read after the "
              "next power-up.")
        return None

    dev = connect(serial, timeout=60)
    ok = bool(dev.user_config_loaded)
    print(f"  {'ok  ' if ok else 'FAIL'} user_config_loaded       "
          f"{dev.user_config_loaded}")
    if not ok:
        print("     The board came up at FACTORY DEFAULTS: the save did not reach")
        print("     flash. Do NOT re-run blindly -- the backup JSON is the only copy")
        print("     of this joint's configuration.")
    for path, want in expected.items():
        try:
            got = get_path(dev, path)
        except Exception:                                   # noqa: BLE001
            continue
        good = values_match(got, want)
        # Keep the axis prefix: axis0 and axis1 share field names, and
        # "encoder.config.mode" twice in a row tells the operator nothing.
        label = path.replace(".config.", ".")
        print(f"  {'ok  ' if good else 'FAIL'} {label:38s} "
              f"{got}{'' if good else f'  (saved {want})'}")
        ok = ok and good
    return ok


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


def make_stepper(total):
    """Numbered section headers, e.g. '3/9  flash firmware'.

    --gains-only runs a short subset of the sequence, so the numbering is
    generated rather than hard-coded and always counts the steps that this run
    actually performs.
    """
    state = {"n": 0}

    def step(text):
        state["n"] += 1
        hdr(f"{state['n']}/{total}  {text}")

    return step


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


def run_odrivetool(cmd_args, check=True, capture=False):
    """Release the device, run an odrivetool subcommand, and stay disconnected.

    stdin is closed: odrivetool's prompts (e.g. 'file exists, override?') would
    otherwise die on EOF *after* half-doing the work.
    """
    disconnect()
    return subprocess.run([ODRIVETOOL] + cmd_args, cwd=REPO, check=check,
                          stdin=subprocess.DEVNULL,
                          stdout=subprocess.PIPE if capture else None,
                          stderr=subprocess.STDOUT if capture else None,
                          text=True if capture else None)


# Dumped by backup-config but not writable, so they can never be restored and
# their failure means nothing. Same list odrivetool trips over.
_READONLY_SUFFIXES = (".anticogging.index", ".anticogging.calib_anticogging",
                      ".anticogging.cogging_ratio")
_READONLY_EXACT = ("can.config.baud_rate",)


def apply_config_json(dev, path):
    """Write a backup JSON onto the board field by field, in THIS connection.

    Replaces `odrivetool restore-config`, which was the common factor in three
    configurations that read back perfectly and were GONE at the next power-up
    (leg2 coxa 2026-08-03), and in the two "restore did not take" boards before
    it. That tool calls erase_configuration(); nvm.c states that unless exactly
    one sector is marked valid the valid-sector choice is UNDEFINED, so an erase
    followed by a single store can land there. Direct writes never erase.

    Writes RAM only -- step 8 does the single save. If the run aborts in
    between, NVM is untouched, which is the safer failure.
    """
    with open(path) as f:
        cfg = json.load(f)
    applied, failed = 0, []
    for key, value in cfg.items():
        if key in _READONLY_EXACT or key.endswith(_READONLY_SUFFIXES):
            continue
        parts = key.split(".")
        try:
            obj = dev
            for p in parts[:-1]:
                obj = getattr(obj, p)
            setattr(obj, parts[-1], value)
            applied += 1
        except Exception as exc:                            # noqa: BLE001
            failed.append(f"{key}: {type(exc).__name__}")
    return applied, failed


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


def elf_version(elf):
    """The firmware version compiled INTO the artifact, read from its symbols.

    The mtime check below cannot see a version bump, and that is not a corner
    case -- it is how every release goes: `version.txt` lives outside
    `Firmware/`, and `autogen/` (where `version.c` is generated) is skipped by
    the walk. So bumping the version without re-running ./dockerbuild.sh leaves
    a stale .elf that the freshness check happily passes. The board then gets
    the OLD firmware, and the only thing that notices is the post-flash verify
    -- after an already-mounted joint has been reflashed with the wrong image.

    Returns None if pyelftools is missing or the symbols cannot be located; the
    caller treats that as "unknown", not as a failure.
    """
    try:
        from elftools.elf.elffile import ELFFile
    except ImportError:
        return None
    names = ("fw_version_major_", "fw_version_minor_", "fw_version_revision_")
    found = {}
    try:
        with open(elf, "rb") as f:
            e = ELFFile(f)
            symtab = e.get_section_by_name(".symtab")
            if symtab is None:
                return None
            for sym in symtab.iter_symbols():
                if sym.name not in names:
                    continue
                shndx = sym.entry.st_shndx
                if not isinstance(shndx, int):
                    continue  # SHN_ABS/SHN_UNDEF: no section data to read
                sec = e.get_section(shndx)
                off = sym.entry.st_value - sec.header.sh_addr
                data = sec.data()[off:off + max(1, sym.entry.st_size)]
                if data:
                    found[sym.name] = data[0]
    except Exception:
        return None
    if len(found) != len(names):
        return None
    return tuple(found[n] for n in names)


def check_build_fresh(elf, want_fw=None):
    """Warn if any firmware source is newer than the .elf (stale build).

    A version mismatch between the artifact and version.txt is FATAL rather
    than a warning: flashing is destructive to the board's running firmware and
    there is no reason to do it with an image that is known to be the wrong
    build.
    """
    if not os.path.exists(elf):
        raise SystemExit(f"firmware not found: {elf}\nBuild it with ./dockerbuild.sh build")
    got = elf_version(elf)
    rel = os.path.relpath(elf, REPO)
    if got is None:
        print(f"  !! could not read the firmware version out of {rel} "
              "(pyelftools missing?)")
        print("     Cannot confirm the artifact matches tools/odrive/version.txt.")
    elif want_fw and got != want_fw:
        raise SystemExit(
            f"{rel} was built as firmware {'.'.join(map(str, got))}, but "
            f"tools/odrive/version.txt says {'.'.join(map(str, want_fw))}.\n"
            "The build is STALE -- a version bump alone does not touch anything "
            "the freshness check below looks at.\nRun ./dockerbuild.sh build and "
            "try again.")
    elif got:
        print(f"  {rel} is built as firmware {'.'.join(map(str, got))}")
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


def opt_path(dev, dotted, absent="n/a"):
    """Read a property that OLDER FIRMWARE may not expose.

    A board that has not been flashed yet is exactly the board this script is
    pointed at, and it can be running anything -- stock 0.5.1 has no
    `enable_harmonic_compensation`, no `position_direction`, no
    `vel_encoder_axis`. Reading one for a status line must never abort the run
    before the flash that would add it.
    """
    try:
        return get_path(dev, dotted)
    except AttributeError:
        return absent


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
    bits 0-1 = field status, bit 2 = button pushed, bit 3 = track loss.

    Which bit means "no magnet" was got WRONG once, and it cost a joint: the
    track-loss bit ALONE is not proof. Two measured data points --

      no magnet (bench stand): nibble 0xa -> field 0b10, track_loss set,
                               count wandering 900-11000 cts
      healthy   (node 52)    : nibble 0x8 -> field 0b00, track_loss set,
                               0 bad CRC in 104k samples, 6-count spread

    -- differ in the FIELD bits, not in track_loss. Treating track_loss as the
    verdict called a perfectly good module magnet-less, so the setup disabled
    axis1 (mode 0) and saved it that way. Judge on `field_weak` + at-rest
    spread; report track_loss only as a note.
    """
    st = (raw24 >> 6) & 0xF
    field = st & 0x03
    return {"nibble": st, "field": field,
            # 0b10 is the code observed with no magnet in front of the die.
            "field_weak": field == 0b10,
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
    # 'No magnet' is a magnetic-FIELD verdict, so read the field bits and the
    # at-rest spread -- NOT track_loss on its own (see mt6701_status). A
    # magnet-less MT6701 still answers with a valid CRC, so CRC cannot decide it.
    st = health["status"]
    if st["field_weak"]:
        return False, (f"CRC is perfect but the MT6701 reports a WEAK/ABSENT magnetic "
                       f"field (status nibble 0x{st['nibble']:x}, field bits 0b10"
                       f"{', track loss' if st['track_loss'] else ''}). Fit the "
                       "diametric magnet centred over the die, 1-2 mm away.")
    if spread > 50:
        return False, (f"CRC is fine but the count wanders {spread} counts at rest -- "
                       "the magnet is loose, off-centre, or too far from the chip."
                       + (" The track-loss bit is set too, which agrees."
                          if st["track_loss"] else ""))
    verdict = (f"HEALTHY ({pct:.4f}% bad CRC, {spread}-count spread at rest, "
               f"status 0x{st['nibble']:x}).")
    if pct > 1.0:
        verdict += " CRC misses are scattered PWM EMI, not consecutive -- tolerated."
    if st["track_loss"]:
        verdict += ("\n       NOTE: track_loss is set while the field reads normal and "
                    "the angle is steady.\n       Not a fault by itself (node 52 reads "
                    "this way permanently), but check magnet\n       distance/centring "
                    "before trusting the joint angle under fast motion.")
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
    if args.gains_only and args.no_gains:
        raise SystemExit("--gains-only and --no-gains contradict each other: "
                         "the gains are the only thing --gains-only writes.")
    leg, joint, node = parse_position(args.position)
    joint_name = JOINT_NAMES[joint]
    today = datetime.datetime.now().strftime("%Y-%m-%d")
    # --motor is optional; without it the joint labels the run and the files.
    who = f"motor #{args.motor} -> " if args.motor else ""
    file_label = f"motor{args.motor}" if args.motor else f"leg{leg}-{joint_name}"
    motor_arg = f"--motor {args.motor} " if args.motor else ""
    # gains-only runs: identify, encoders, gains, save, verify.
    step = make_stepper(6 if args.gains_only else 10)

    hdr(f"{who}leg {leg} {joint_name} (position {args.position})"
        f"  ->  can_node_id {node}")
    if args.gains_only:
        print("  GAINS ONLY: the controller config is the ONLY thing written.")
        print("  No flash, no backup/restore, no CAN node id, no encoder mode write,")
        print("  no zero_offset reseed, no reboot. Joint zero, endstops and")
        print("  commutation are left exactly as they are.")
    if args.dry_run:
        print("  DRY RUN: the board is only read. Nothing is flashed, written to")
        print("  the board, or saved. The config backup IS still written to disk.")

    # ---- 1. identify -------------------------------------------------------
    step("identify board")
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
          f"harmonic {opt_path(dev, 'axis0.encoder.config.enable_harmonic_compensation')}")
    if not dev.user_config_loaded:
        print("  !! user_config_loaded is False -- this board has NO usable saved config.")
        print("     Restoring a bench backup is the only way to get commutation back.")

    do_flash = False
    backup = None
    if args.gains_only:
        # Writing pos_gain / vel_gain to an ARMED axis re-tunes the loop that is
        # currently holding the limb up, which is motion. IDLE is also what
        # save_configuration() needs, so refuse anything else outright.
        states = (dev.axis0.current_state, dev.axis1.current_state)
        print(f"  axis states       {states[0]}/{states[1]}   (1 = IDLE)")
        if states != (1, 1):
            raise SystemExit(
                f"both axes must be IDLE to write gains; got axis0={states[0]}, "
                f"axis1={states[1]}. Disarm the joint first -- re-tuning a live "
                "position loop moves the leg.")
        if not dev.user_config_loaded:
            raise SystemExit(
                "user_config_loaded is False: this board has no saved commutation "
                "config, so it is not an already-configured joint. Run the full "
                "sequence (without --gains-only) to restore its bench backup.")
        # position_direction / vel_encoder_axis are local additions to this fork.
        # Probe for them rather than comparing version numbers: on stock or
        # pre-fork firmware the writes below would abort HALFWAY through the
        # controller config, leaving a joint with new gains and old feedback.
        missing = [f for f in ("vel_encoder_axis", "position_direction")
                   if opt_path(dev, f"axis0.controller.config.{f}", None) is None]
        if missing:
            raise SystemExit(
                f"this board runs firmware {'.'.join(map(str, fw_of(dev)))}, which has "
                f"no controller.config.{' / '.join(missing)}. --gains-only cannot "
                "configure split feedback on it -- run the full sequence to flash "
                f"{'.'.join(map(str, want_fw)) if want_fw else 'the current build'} first.")
        # The gains are joint-specific, so tuning the wrong board is the one
        # mistake worth blocking: a femur gain of 2500 on a 1:6 coxa oscillates.
        if dev.axis0.config.can_node_id != node:
            raise SystemExit(
                f"this board is can_node_id {dev.axis0.config.can_node_id}, but "
                f"--position {args.position} means node {node}. Refusing to tune "
                "the wrong joint -- check --serial-number / which board is plugged in.")
        print(f"  can_node_id       {dev.axis0.config.can_node_id}   "
              f"(matches --position {args.position})")
    else:
        do_flash = not args.skip_flash and (args.force_flash or fw_of(dev) != want_fw)
        if not args.skip_flash:
            check_build_fresh(args.elf, want_fw)
            if not do_flash:
                print(f"  board already runs {'.'.join(map(str, want_fw))} -- skipping "
                      "flash (use --force-flash to override)")

    if not args.gains_only:
        # ---- 2. backup -----------------------------------------------------
        step("back up configuration")
        tag = "%s.%s.%s" % want_fw if want_fw else "flash"
        backup = os.path.join(CONFIG_DIR,
                              f"{file_label}-{serial}-before-{tag}-flash-{today}.json")
        if not dev.user_config_loaded:
            print("  SKIPPED: no config is loaded, a backup would capture only defaults.")
            backup = None
        else:
            os.makedirs(CONFIG_DIR, exist_ok=True)
            backup = unique_path(backup)
            run_odrivetool(["backup-config", backup])
            with open(backup) as f:
                print(f"  saved {len(json.load(f))} fields -> "
                      f"{os.path.relpath(backup, REPO)}")
            dev = connect(args.serial_number)

        # ---- 3. flash ------------------------------------------------------
        step("flash firmware")
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
            # The flash wipes NVM only when config_version in nvm_config.hpp
            # changed, i.e. when a config STRUCT changed -- NOT on every version
            # bump. 0.5.6 -> 0.5.7 (CAN reset disabled) touches no struct, so
            # the saved configuration stays valid and loads straight back. Both
            # outcomes are normal here; step 4 decides what to do about it.
            print(f"  user_config_loaded {dev.user_config_loaded}"
                  f"{'  (NVM wiped -- config_version changed)' if not dev.user_config_loaded else '  (NVM survived -- config_version unchanged)'}")

        # ---- 4. restore ----------------------------------------------------
        step("restore + verify configuration")
        if args.dry_run or not do_flash or not backup:
            print("  skipped")
        else:
            # A restore is not free, and on fw 0.5.6 it was twice destructive:
            # one board came back with axis1.encoder.config.mode dropped (so the
            # joint MT6701 was never clocked while load_encoder_axis=1), another
            # with axis0 sitting at DEFAULTS -- both with user_config_loaded
            # still True, i.e. looking fine. So when the flash did not wipe NVM
            # (config_version unchanged, which is the case for 0.5.6 -> 0.5.7)
            # and the board already matches its own pre-flash backup
            # field-for-field, there is nothing to restore. Rewriting it would
            # only re-roll those dice on a joint that is already bolted to the
            # robot.
            need_restore = True
            if dev.user_config_loaded:
                print("  NVM survived the flash -- checking whether a restore is "
                      "needed at all")
                if verify_against_json(dev, backup):
                    need_restore = False
            if not need_restore:
                print("  board already matches its pre-flash backup field-for-field "
                      "-- restore SKIPPED")
                print("  (nothing was wiped, so rewriting the config would only add "
                      "risk).")
            else:
                # NOT `odrivetool restore-config`. That tool erases NVM first,
                # and it is the common factor in every configuration that has
                # silently failed to reach flash on this fleet. These writes go
                # straight into the live connection and are committed by the
                # single save in step 8.
                applied, failed = apply_config_json(dev, backup)
                print(f"  applied {applied} fields directly from the backup "
                      f"(no erase, no restore-config)")
                for line in failed:
                    print(f"    could not apply {line}")
                if failed:
                    raise SystemExit(f"{len(failed)} field(s) could not be applied "
                                     "-- fix before continuing.")
                if not verify_against_json(dev, backup):
                    raise SystemExit("restore verify FAILED -- fix before continuing.")
            print(f"  user_config_loaded {dev.user_config_loaded}")
            print(f"  Kt {dev.axis0.motor.config.torque_constant:.4f}   "
                  f"offset {dev.axis0.encoder.config.offset}   "
                  f"harmonic "
                  f"{opt_path(dev, 'axis0.encoder.config.enable_harmonic_compensation')}")

        # ---- 5. CAN --------------------------------------------------------
        step(f"CAN: node {node}, mute axis1")
        if args.dry_run:
            print(f"  would set axis0.config.can_node_id = {node}, "
                  f"axis1.config.can_heartbeat_rate_ms = 0")
        else:
            dev.axis0.config.can_node_id = node
            dev.axis0.config.can_node_id_extended = False
            dev.axis0.config.can_heartbeat_rate_ms = args.heartbeat_ms
            # Every board's unused axis1 defaults to can_node_id=1 with a 100 ms
            # heartbeat and the firmware TXes it unconditionally -- so every
            # un-muted board on the bus claims node 1 at once. Mute it here, in
            # the same USB session, because reaching a mounted board later means
            # unplugging it. Muting does not change can_node_id: axis1 still
            # ANSWERS RTR on node 1, so keep node 1 reserved and never poll it.
            dev.axis1.config.can_heartbeat_rate_ms = 0
            dev.axis1.config.can_node_id_extended = False
        print(f"  axis0  node {dev.axis0.config.can_node_id}  "
              f"heartbeat {dev.axis0.config.can_heartbeat_rate_ms} ms")
        print(f"  axis1  node {dev.axis1.config.can_node_id}  "
              f"heartbeat {dev.axis1.config.can_heartbeat_rate_ms} ms  <- 0 = muted")
        print(f"  baud   {dev.can.config.baud_rate}")

    # ---- 6. encoders -------------------------------------------------------
    step("encoders: axis0 commutation + axis1 joint (MT6701)"
         + ("  [read-only]" if args.gains_only else ""))
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
        if args.gains_only:
            # Never rewrite mode on an encoder that is already sampling. Taking a
            # live abs-SPI encoder down to mode 0 and back up in RAM leaves
            # ABS_SPI_COM_FAIL latched and the count frozen until a reboot -- and
            # on a mounted joint that is exactly what must not happen. If the
            # mode is wrong here, sample_mt6701 sees zero samples and the triage
            # below reports it instead of "fixing" it.
            print("  axis1 config left untouched (--gains-only), reading only")
        elif not args.dry_run:
            enc.config.cpr = MT6701_CPR
            enc.config.abs_spi_cs_gpio_pin = MT6701_CS_PIN
            enc.config.enable_phase_interpolation = False
            enc.error = 0
            # mode BEFORE pre_calibrated, and both before the readback below.
            # Encoder::check_pre_calibrated() clears the flag whenever the LIVE
            # mode_ is incremental-without-index, so writing pre_calibrated
            # first is silently rejected on a board whose axis1 is still at the
            # default mode 0 -- the write "succeeds", the read-back is False,
            # and nothing is raised. Setting mode first makes mode_ absolute, so
            # the flag sticks.
            enc.config.mode = MT6701_MODE
            time.sleep(1.0)
            enc.config.pre_calibrated = True
            time.sleep(0.2)
            if not enc.config.pre_calibrated:
                print("  !! pre_calibrated did NOT stick (silently rejected). The "
                      "encoder will not be\n     ready at boot -- let this run save, "
                      "then POWER-CYCLE (not reboot: save+reboot\n     wedges abs-SPI), "
                      "set it again and save.")
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
        if encoder_ok and args.gains_only:
            # No reseed either: it rewrites zero_offset (with the same value, but
            # still a write) and jumps pos_estimate by up to a turn. Harmless on
            # a bring-up, not something to do behind the user's back on a joint
            # whose zero and endstops are already set.
            print(f"  pos_estimate {enc.pos_estimate:.4f} turn  (not re-seeded, "
                  "--gains-only)")
        elif encoder_ok and not args.dry_run:
            # Re-seed the LINEAR position accumulator from the live absolute
            # count. pos_estimate is seeded once at startup and can be a whole
            # turn out after dropped samples; writing zero_offset onto itself
            # calls reset_user_position() and re-seeds it without a reboot.
            enc.config.zero_offset = enc.config.zero_offset
            time.sleep(0.3)
            print(f"  pos_estimate re-seeded from the absolute count: "
                  f"{enc.pos_estimate:.4f} turn")
        if not encoder_ok and args.gains_only:
            print("\n  The joint encoder is not usable, so the gains below are NOT")
            print("  written: split feedback would aim the position loop at it.")
            print("  axis1 is left exactly as found (mode not touched).")
            # --gains-only promises not to write encoder/controller config, so
            # this only warns -- but silence here would hide a live runaway on
            # an already-mounted joint.
            if dev.axis0.controller.config.load_encoder_axis == 1:
                print("  !! DANGER: load_encoder_axis is ALREADY 1 on this board, so the")
                print("     position loop is aimed at this unusable encoder RIGHT NOW.")
                print("     Do not arm this joint. Fix the encoder, or set")
                print("     axis0.controller.config.load_encoder_axis = 0 and save.")
        elif not encoder_ok and not args.allow_bad_encoder:
            print("\n  axis1 will be left DISABLED (mode 0) rather than saved in a")
            print("  faulted state. The firmware, config restore and CAN setup below")
            print("  are still saved and complete. Fix the wiring, then re-run:")
            print(f"    {os.path.relpath(__file__, REPO)} {motor_arg}"
                  f"--position {args.position} --skip-flash")
            if not args.dry_run:
                enc.config.mode = 0
                # Disabling axis1 is only half the job. The config restored in
                # step 4 comes from this board's PREVIOUS joint, which was very
                # likely split-feedback -- so load_encoder_axis is already 1 and
                # now points at an encoder that reports a FROZEN position. Arm
                # that in position mode and the error never shrinks: the motor
                # drives until something breaks. Point the loop back at axis0
                # (stock, non-split) so the saved config is merely untuned
                # rather than a runaway waiting for the next operator.
                c = dev.axis0.controller.config
                if c.load_encoder_axis != 0 or c.vel_encoder_axis != 0:
                    was = (c.load_encoder_axis, c.vel_encoder_axis)
                    c.load_encoder_axis = 0
                    c.vel_encoder_axis = 0
                    time.sleep(0.2)
                    print(f"  !! split feedback was load/vel={was[0]}/{was[1]} "
                          "(restored from this board's previous joint) and would")
                    print("     have aimed the position loop at the now-disabled "
                          "axis1 -- a frozen")
                    print("     position estimate is a RUNAWAY on the next arm. Reset "
                          f"to load/vel={c.load_encoder_axis}/{c.vel_encoder_axis} "
                          "(motor-side).")

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
    step(f"split feedback + position gains ({joint_name})")
    gains_applied = None
    if args.position_direction is None:
        # Only --gains-only can inherit it: on a bring-up there is nothing on the
        # board worth keeping, but on an already-tuned joint the saved sign was
        # verified against the real assembly and silently replacing it with the
        # +1 default is a runaway the next time the axis arms.
        if args.gains_only:
            args.position_direction = (dev.axis0.controller.config.position_direction
                                       or DEFAULT_POSITION_DIRECTION)
            print(f"  position_direction {args.position_direction:+d} kept from the "
                  "board (pass --position-direction to change it)")
        else:
            args.position_direction = DEFAULT_POSITION_DIRECTION
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
    step("save to flash")
    if args.dry_run:
        print("  skipped (--dry-run)")
    elif args.gains_only and gains_applied is None:
        # Saving here would only re-commit what is already in NVM, and it would
        # do it through the one operation on this board known to wedge an
        # abs-SPI encoder. Nothing was written, so nothing is saved.
        print("  NOT SAVED: nothing was written (see above). The board is exactly")
        print("  as it was found.")
        hdr(f"NOTHING WRITTEN: {who}leg {leg} {joint_name}, node {node}")
        return 1
    else:
        try:
            dev.save_configuration()
        except Exception as exc:
            # save_configuration resets the USB transport; the exception is
            # normal on this firmware and does not mean the save failed.
            print(f"  save_configuration raised {type(exc).__name__} "
                  "(expected -- USB transport resets); verifying below")
        print("  saved")

    # ---- 9. verify ---------------------------------------------------------
    if args.gains_only:
        step("verify the saved gains")
        if args.dry_run:
            print("  skipped (--dry-run) -- nothing was written")
            return 0
        # Deliberately NO reboot. save_configuration() + reboot is exactly the
        # sequence that has left one of the two absolute encoders wedged (only a
        # power cycle recovers it), and on a mounted leg that costs a power cycle
        # of the robot to undo. The written values are read back over a fresh
        # connection; NVM itself is proven at the next ordinary power-up.
        time.sleep(2)
        dev = connect(args.serial_number, timeout=40)
        c = dev.axis0.controller.config
        checks = [
            ("firmware", ".".join(map(str, fw_of(dev))), True),
            ("user_config_loaded", dev.user_config_loaded, bool(dev.user_config_loaded)),
            ("axis0 can_node_id", dev.axis0.config.can_node_id,
             dev.axis0.config.can_node_id == node),
            ("ctrl load/vel enc axis", f"{c.load_encoder_axis}/{c.vel_encoder_axis}",
             (c.load_encoder_axis, c.vel_encoder_axis) == (1, 0)),
            ("ctrl pos_gain", round(c.pos_gain, 1),
             values_match(c.pos_gain, gains_applied["pos_gain"])),
            ("ctrl vel_gain", round(c.vel_gain, 4),
             values_match(c.vel_gain, gains_applied["vel_gain"])),
            ("ctrl vel_integrator", round(c.vel_integrator_gain, 4),
             values_match(c.vel_integrator_gain, gains_applied["vel_integrator_gain"])),
            ("ctrl vel_limit", round(c.vel_limit, 3),
             values_match(c.vel_limit, gains_applied["vel_limit"])),
            ("ctrl position_direction", c.position_direction,
             c.position_direction == gains_applied["position_direction"]),
            ("axis0 state", dev.axis0.current_state, dev.axis0.current_state == 1),
        ]
        ok = True
        for name, value, good in checks:
            print(f"  {'ok  ' if good else 'FAIL'} {name:24s} {value}")
            ok = ok and good
        expected = snapshot(dev, _GAIN_PROOF_PATHS)
        step("prove the save reached NVM (power cycle)")
        proven = power_cycle_proof(dev, expected, args.serial_number,
                                   args.no_power_cycle_check)
        if proven is False:
            ok = False

        hdr(f"{'DONE' if ok else 'INCOMPLETE'} (gains only): {who}"
            f"leg {leg} {joint_name}, node {node}")
        print(f"  pos_gain {gains_applied['pos_gain']:.0f} / "
              f"vel_gain {gains_applied['vel_gain']:.4f} / "
              f"vel_i {gains_applied['vel_integrator_gain']:.4f} / "
              f"vel_limit {gains_applied['vel_limit']:.1f} written and saved.")
        print("  Joint zero, endstops, CAN node id and commutation were NOT touched.")
        if proven:
            print("  NVM PROVEN: the gains were re-read after a power cycle.")
        else:
            print("  NVM NOT PROVEN -- confirm the gains after the next power-up.")
        return 0 if ok else 1

    step("verify over a fresh connection (no reboot)")
    if args.dry_run:
        print("  skipped (--dry-run)")
        return 0
    # Deliberately NO reboot. save_configuration() + reboot() is exactly the
    # sequence that has left one of the two absolute encoders wedged on this
    # board (see the axis0 diagnosis below) and only removing power recovers it
    # -- on a mounted leg that means power-cycling the robot. save_configuration
    # sets user_config_loaded_ itself, so the checks below still prove the save
    # went through; what they cannot prove is that the config RELOADS cleanly,
    # and that is verified at the next ordinary power-up.
    print("  the board is NOT rebooted -- save + reboot is what wedges an "
          "abs-SPI encoder")
    time.sleep(2)
    dev = connect(args.serial_number, timeout=40)

    ok = True
    fw = fw_of(dev)

    # Any encoder error -- including the benign ABS_SPI_NOT_READY transient that
    # a shared SPI bus throws at startup -- also latches AXIS_ERROR_ENCODER_FAILED
    # (0x100) on the axis, and the writes in steps 5-7 leave their own transients
    # behind. Judging those raw errors would false-fail a healthy board. Report
    # what is latched, then clear once and judge what comes BACK, with axis0
    # judged on whether it DELIVERS data.
    latched_errors = (dev.axis0.error, dev.axis0.motor.error,
                      dev.axis0.encoder.error, dev.axis0.controller.error)
    print(f"  latched: axis/motor/encoder/controller errors "
          f"{'/'.join(hex(e) for e in latched_errors)} (cleared, re-checking below)")
    for ax in (dev.axis0, dev.axis1):
        ax.encoder.error = 0
        ax.motor.error = 0
        ax.controller.error = 0
        ax.error = 0
    a0_post_ok, a0_post_line, _ = check_commutation_encoder(dev.axis0.encoder)
    print(a0_post_line)
    time.sleep(1.5)
    # ABS_SPI_NOT_READY alone is tolerated; ABS_SPI_COM_FAIL and every other
    # error is not.
    enc_err = dev.axis0.encoder.error & ~ENCODER_ERROR_ABS_SPI_NOT_READY
    residual = (dev.axis0.motor.error, enc_err, dev.axis0.controller.error)

    checks = [
        ("axis0 commutation enc", "delivering" if a0_post_ok else "NOT DELIVERING",
         a0_post_ok),
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
    # The joint encoder's saved mode is checked even when the encoder failed,
    # because "mode 0" is the state that has to be READ BACK from NVM to be
    # believed. Node 52 was found months later with mode 0 and every other
    # axis1 field restored and plausible -- silently skipping this check is how
    # a disabled encoder gets mistaken for a configured one.
    if not args.no_mt6701 and encoder_ok is False:
        mode_now = dev.axis1.encoder.config.mode
        checks.append(("axis1 encoder mode",
                       f"{mode_now} (DELIBERATELY DISABLED - encoder failed step 6)",
                       mode_now == 0))
        c = dev.axis0.controller.config
        checks.append(("split feedback off", f"load/vel={c.load_encoder_axis}/"
                       f"{c.vel_encoder_axis}", c.load_encoder_axis == 0))
    if not args.no_mt6701 and encoder_ok:
        checks.append(("axis1 encoder mode", dev.axis1.encoder.config.mode,
                       dev.axis1.encoder.config.mode == MT6701_MODE))
        checks.append(("axis1 pre_calibrated", dev.axis1.encoder.config.pre_calibrated,
                       bool(dev.axis1.encoder.config.pre_calibrated)))
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

    # ---- 10. prove NVM -----------------------------------------------------
    proof_paths = NVM_PROOF_PATHS + (_GAIN_PROOF_PATHS if gains_applied else ())
    expected = snapshot(dev, proof_paths)
    step("prove the save reached NVM (power cycle)")
    proven = power_cycle_proof(dev, expected, args.serial_number,
                               args.no_power_cycle_check)
    if proven is False:
        ok = False

    hdr(f"{'DONE' if ok else 'INCOMPLETE'}: {who}leg {leg} "
        f"{joint_name}, node {node}")
    if not a0_post_ok:
        print("  axis0 COMMUTATION encoder is not reading after the save -- this "
              "joint cannot")
        print("  be armed. POWER-CYCLE the board and re-run with --skip-flash before")
        print("  suspecting wiring: nothing short of removing power clears an "
              "abs-SPI latch-up.")
    if not args.no_mt6701 and encoder_ok is False:
        print("  axis1 joint encoder is NOT configured -- fix wiring and re-run "
              "with --skip-flash.")
    print("  The board was NOT rebooted by this tool (save + reboot is what wedges")
    print("  an abs-SPI encoder); step 10's power cycle is operator-driven.")
    if proven:
        print("  NVM PROVEN: every field above was re-read after a power cycle.")
    elif proven is False:
        print("  !! NVM FAILED: the board did not come back with what was saved.")
    else:
        print("  !! NVM NOT PROVEN -- the save was never confirmed against a power")
        print("     cycle, and a silent store failure is invisible over USB.")
    print(f"  Record it in {os.path.relpath(os.path.join(HERE, 'CAN_NODE_ID_MAP.md'), REPO)}:")
    print(f"    | {leg} | {joint} | {joint_name:11s} | **{node}** | "
          f"{args.motor if args.motor else '?'} | {serial} |")
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
