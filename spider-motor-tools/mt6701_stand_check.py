#!/usr/bin/env python3
"""Test-stand checker for an MT6701 module.

Brings the module up on an ODrive axis in RAM (nothing is saved to NVM, no
motor is ever moved) and grades it:

    no samples             -> SPI never clocked; needs save_configuration()+reboot
    ~100 % bad CRC         -> wiring / power / module not in SSI mode
    CRC ok + count wanders -> no magnet, or magnet too far / misaligned
    CRC ok + tight spread  -> GOOD

It also reads the OTHER axis's absolute encoder as a board-side control: that
encoder shares the same SCK/MISO, so if it reads cleanly while this one does
not, the board and SPI bus are proven good and the fault is on the module side.

Usage:
    .venv/bin/python spider-motor-tools/mt6701_stand_check.py
    .venv/bin/python spider-motor-tools/mt6701_stand_check.py --spin
    .venv/bin/python spider-motor-tools/mt6701_stand_check.py --axis 1 --cs-pin 6
"""
import argparse
import sys
import time

import odrive

CPR = 16384
MODE_SPI_ABS_MT6701 = 261
CTS_PER_DEG = CPR / 360.0

# Bad-CRC percentage above which the link is called broken rather than noisy.
BROKEN_CRC_PCT = 50.0
# Above this it is a real EMI problem worth fixing, below it is normal jitter.
NOISY_CRC_PCT = 1.0
# At-rest spread (counts) above which the reading is noise, not an angle.
# A healthy module at rest sits within a handful of counts; the no-magnet
# case on this stand wandered ~1000 cts (22 deg).
REST_SPREAD_MAX = 40


def circular_spread(counts, cpr):
    """Peak-to-peak spread in counts, correct across the 0/cpr wrap.

    A plain max-min is wrong for an encoder resting on the seam: samples of
    16380 and 4 are 8 counts apart, not 16376. Offsets are measured signed
    against the first sample, so this is valid for spreads under cpr/2.
    """
    if not counts:
        return 0
    ref = counts[0]
    offs = [((c - ref + cpr // 2) % cpr) - cpr // 2 for c in counts]
    return max(offs) - min(offs)


def decode_status(raw24):
    """MT6701 SSI status nibble, bits [9:6] of the 24-bit frame.

    Layout per the reference driver (servo-firmware/lib/mt6701/mt6701.cpp):
    bits 0-1 = magnetic field status, bit 2 = button pushed, bit 3 = track loss.
    """
    st = (raw24 >> 6) & 0xF
    return {
        "nibble": st,
        "field": st & 0x03,
        "pushed": bool(st & 0x04),
        "track_loss": bool(st & 0x08),
    }


def read_debug(enc):
    """One snapshot of the MT6701 debug fields.

    NB: each attribute is a separate USB transaction, so these come from
    different control-loop samples. Never compare crc_calc against crc_recv
    from this dict -- use the bad_crc_count delta, which is atomic in firmware.
    """
    return {
        "samples": enc.mt6701_debug_sample_count,
        "bad_crc": enc.mt6701_debug_bad_crc_count,
        "raw24": enc.mt6701_debug_raw24,
        "word0": enc.mt6701_debug_word0,
        "word1": enc.mt6701_debug_word1,
        "pos": enc.mt6701_debug_pos,
    }


def check_other_axis(odrv, axis_num):
    """Read the opposite axis's abs encoder as a board/SPI-bus control."""
    other = odrv.axis0 if axis_num == 1 else odrv.axis1
    e = other.encoder
    if not (e.config.mode & 0x100):  # MODE_FLAG_ABS
        return "axis%d is not an absolute encoder (mode %d) - no control available" % (
            1 - axis_num, e.config.mode)
    counts = [e.count_in_cpr for _ in range(10)]
    spread = circular_spread(counts, e.config.cpr or CPR)
    # Judge the LINK by whether it DELIVERS data, not by error == 0:
    # ABS_SPI_NOT_READY (0x100) latches routinely when the other axis is
    # failing on the shared bus. Only ABS_SPI_COM_FAIL (0x80) means this
    # encoder is not reading at all.
    link_ok = counts[-1] != 0 and not (e.error & 0x80)
    # Spread is a separate question: a wandering angle means no magnet in
    # front of THIS encoder, which says nothing about the bus.
    if not link_ok:
        verdict = "also FAILING to read -> suspect the shared bus/board"
    elif spread < 200:
        verdict = "READS CLEAN -> board + SPI bus proven good"
    else:
        verdict = ("link OK (frames arriving) but angle wanders -> no magnet on "
                   "this encoder either; bus still proven good")
    return "axis%d (mode %d, CS %d): count=%d spread=%d err=0x%x\n   -> %s" % (
        1 - axis_num, e.config.mode, e.config.abs_spi_cs_gpio_pin,
        counts[-1], spread, e.error, verdict)


def main():
    p = argparse.ArgumentParser(description="Check an MT6701 module on a test stand")
    p.add_argument("--axis", type=int, default=1, choices=(0, 1),
                   help="axis to bring the module up on (default 1)")
    p.add_argument("--cs-pin", type=int, default=6, help="abs SPI CS GPIO pin (default 6)")
    p.add_argument("--seconds", type=float, default=5.0, help="CRC measurement window")
    p.add_argument("--rest-seconds", type=float, default=3.0, help="at-rest stability window")
    p.add_argument("--spin", action="store_true",
                   help="after the static test, stream angle so you can turn the "
                        "shaft by hand and confirm it tracks over a full turn")
    p.add_argument("--spin-seconds", type=float, default=30.0)
    p.add_argument("--no-config", action="store_true",
                   help="do not touch config; test whatever is already running")
    p.add_argument("--serial", default=None, help="board serial (lowercase hex)")
    args = p.parse_args()

    print("connecting...", flush=True)
    if args.serial:
        odrv = odrive.find_any(serial_number=args.serial, timeout=20)
    else:
        odrv = odrive.find_any(timeout=20)
    print("board %012x  fw %d.%d.%d  vbus %.2f V" % (
        odrv.serial_number, odrv.fw_version_major, odrv.fw_version_minor,
        odrv.fw_version_revision, odrv.vbus_voltage), flush=True)

    ax = odrv.axis1 if args.axis == 1 else odrv.axis0
    enc = ax.encoder

    if ax.current_state != 1:  # AXIS_STATE_IDLE
        print("\nERROR: axis%d is in state %d, not IDLE. Refusing to reconfigure a "
              "running axis." % (args.axis, ax.current_state))
        return 2

    if args.no_config:
        print("\n== using existing config: mode=%d cpr=%d CS=%d ==" % (
            enc.config.mode, enc.config.cpr, enc.config.abs_spi_cs_gpio_pin), flush=True)
    else:
        print("\n== configuring axis%d: mode %d, cpr %d, CS %d (RAM ONLY, not saved) ==" % (
            args.axis, MODE_SPI_ABS_MT6701, CPR, args.cs_pin), flush=True)
        enc.error = 0
        enc.config.cpr = CPR
        enc.config.abs_spi_cs_gpio_pin = args.cs_pin
        enc.config.enable_phase_interpolation = False
        enc.config.mode = MODE_SPI_ABS_MT6701   # set LAST: this starts SPI sampling
        time.sleep(1.0)
        print("   readback: mode=%d cpr=%d CS=%d" % (
            enc.config.mode, enc.config.cpr, enc.config.abs_spi_cs_gpio_pin), flush=True)

    # ---- SPI link quality -------------------------------------------------
    d0 = read_debug(enc)
    t0 = time.time()
    time.sleep(args.seconds)
    d1 = read_debug(enc)
    dt = time.time() - t0
    ds = d1["samples"] - d0["samples"]
    db = d1["bad_crc"] - d0["bad_crc"]
    bad_pct = (100.0 * db / ds) if ds else 100.0

    print("\n== SPI link, %.1f s window ==" % dt, flush=True)
    print("   samples : %d  (%.0f Hz)" % (ds, ds / dt if dt else 0))
    print("   bad CRC : %d  (%.3f %%)" % (db, bad_pct))
    print("   raw24   : 0x%06x   word0=0x%04x word1=0x%04x" % (
        d1["raw24"], d1["word0"], d1["word1"]))
    st = decode_status(d1["raw24"])
    print("   status  : nibble=0x%x  field_bits=0b%s  pushed=%s  track_loss=%s" % (
        st["nibble"], format(st["field"], "02b"), st["pushed"], st["track_loss"]))
    print("   enc.error=0x%x  is_ready=%s (is_ready needs pre_calibrated=True; "
          "False here is expected)" % (enc.error, enc.is_ready))

    # ---- at-rest stability ------------------------------------------------
    counts = []
    n = max(1, int(args.rest_seconds / 0.1))
    for _ in range(n):
        counts.append(enc.count_in_cpr)
        time.sleep(0.1)
    spread = circular_spread(counts, CPR)
    print("\n== at rest, %.1f s @10 Hz ==" % args.rest_seconds, flush=True)
    print("   count_in_cpr: spread=%d (%.2f deg, wrap-corrected)  now=%d" % (
        spread, spread / CTS_PER_DEG, counts[-1]))

    # ---- board-side control ----------------------------------------------
    print("\n== control: other axis on the same SCK/MISO ==", flush=True)
    print("   " + check_other_axis(odrv, args.axis), flush=True)

    # ---- verdict ----------------------------------------------------------
    print("\n== VERDICT ==", flush=True)
    ok = False
    if ds == 0:
        print("   NO SAMPLES - the control loop is not clocking this encoder.")
        print("   Try save_configuration() + reboot(); the CS pin may only bind at boot.")
    elif bad_pct > BROKEN_CRC_PCT:
        print("   BAD LINK: %.1f %% of frames fail CRC -> wiring, power, or the" % bad_pct)
        print("   module is not in SSI output mode.")
        if d1["raw24"] == 0x000000:
            print("   raw24=0x000000: MISO held LOW - module unpowered, or DO shorted to GND.")
        elif d1["raw24"] == 0xFFFFFF:
            print("   raw24=0xffffff: nothing DRIVES MISO - CSN not reaching the CS pin,")
            print("   DO not reaching MISO, or unpowered. MT6701 only drives DO while CSN is low.")
        else:
            print("   raw24=0x%06x: frames arrive but are corrupt - check CLK integrity/EMI." % d1["raw24"])
        print("   Check: DO->MISO, CLK->SCK, CSN->IO%d, VCC->3.3V, GND->GND" % args.cs_pin)
    elif bad_pct > NOISY_CRC_PCT:
        print("   MARGINAL: %.3f %% CRC failures. The link works but is noisy -" % bad_pct)
        print("   long/unshielded leads or motor PWM EMI. Separate the encoder cable")
        print("   from the phase wires and add a ferrite before trusting it under load.")
    elif spread > REST_SPREAD_MAX:
        print("   NO MAGNET (or magnet too far / misaligned).")
        print("   The module itself is FINE: %d frames, %.3f %% CRC errors - it is" % (ds, bad_pct))
        print("   talking perfectly. But the angle wanders %d cts (%.1f deg) at rest," % (
            spread, spread / CTS_PER_DEG))
        print("   which is noise, not a position. Fit the diametric magnet, centred")
        print("   over the die and ~1-2 mm away, then re-run.")
        if st["track_loss"]:
            print("   CONFIRMED by the module itself: the track_loss status bit is SET,")
            print("   i.e. the MT6701 reports it cannot see a magnetic track.")
    else:
        ok = True
        print("   GOOD: %d frames, %.3f %% CRC errors, at-rest spread %d cts (%.2f deg)." % (
            ds, bad_pct, spread, spread / CTS_PER_DEG))
        print("   Module and magnet are both healthy.")
        if st["track_loss"]:
            print("   NOTE: track_loss bit is set, but the field bits read 0b%s and the"
                  % format(st["field"], "02b"))
            print("   angle is steady - NOT the no-magnet case (that reads field 0b10")
            print("   plus a wandering count). Seen permanently on a healthy module;")
            print("   worth checking magnet distance/centring, not a fault by itself.")

    # ---- optional hand-turn check ----------------------------------------
    if args.spin:
        print("\n== SPIN CHECK: turn the shaft by hand through a full turn ==", flush=True)
        print("   (%.0f s; watching for full coverage and clean wrapping)" % args.spin_seconds)
        seen = set()
        lo, hi = None, None
        t0 = time.time()
        while time.time() - t0 < args.spin_seconds:
            c = enc.count_in_cpr
            seen.add(int(c * 36 / CPR))          # 10-degree buckets
            lo = c if lo is None else min(lo, c)
            hi = c if hi is None else max(hi, c)
            print("\r   count=%6d  %7.2f deg  covered %2d/36 of a turn" % (
                c, c / CTS_PER_DEG, len(seen)), end="", flush=True)
            time.sleep(0.1)
        db2 = read_debug(enc)["bad_crc"] - d1["bad_crc"]
        print("\n   coverage %d/36 buckets, range %d..%d cts, bad CRC during spin: %d" % (
            len(seen), lo, hi, db2))
        if len(seen) >= 34:
            print("   full turn tracked cleanly.")
        else:
            print("   incomplete turn - rotate further, or the magnet is losing the track.")

    print("\ndone")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
