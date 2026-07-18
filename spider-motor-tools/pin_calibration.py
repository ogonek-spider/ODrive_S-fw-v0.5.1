#!/usr/bin/env python3
"""Pin a motor's calibration to flash after health-check + Kt pass.

Runs a fresh motor calibration and encoder-offset calibration, writes the
measured Kt into torque_constant, sets motor + encoder pre_calibrated=True so
the drive boots straight into a usable state, then saves to flash and verifies
after reconnect.

This is the ONLY tool here that writes to flash. Run it only after
motor_health_check.py passes and measure_kt.py gives a clean Kt.

Assumes the encoder + thermistor are already configured on the board (this MKS
bench board carries onboard-AS5047P mode 257 / cpr 16384 / CS 7 and the GPIO4
motor thermistor across motor swaps). Use --check-thermistor to sanity the NTC.

NOTE: strips cwd from sys.path so ./odrive does not shadow the installed pkg.
"""
import argparse
import os
import sys
import time

sys.path = [p for p in sys.path if p not in ("", ".", os.getcwd())]

import odrive
from odrive.enums import (AXIS_STATE_ENCODER_OFFSET_CALIBRATION,
                          AXIS_STATE_IDLE, AXIS_STATE_MOTOR_CALIBRATION)


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--motor-id", help="Motor label (for logging).")
    p.add_argument("--serial-number")
    p.add_argument("--axis", type=int, default=0, choices=(0, 1))
    p.add_argument("--kt", type=float, required=True,
                   help="torque_constant (Nm/A) from measure_kt.py.")
    p.add_argument("--check-thermistor", action="store_true",
                   help="Print motor thermistor reading + verdict before saving.")
    p.add_argument("--dry-run", action="store_true",
                   help="Calibrate + set fields but DO NOT save to flash.")
    return p.parse_args()


def wait_idle(ax, t):
    d = time.monotonic() + t
    while ax.current_state != AXIS_STATE_IDLE:
        time.sleep(0.2)
        if time.monotonic() > d:
            return False
    return True


def run(ax, state, t):
    ax.error = ax.motor.error = ax.encoder.error = ax.controller.error = 0
    time.sleep(0.05)
    ax.requested_state = state
    time.sleep(0.3)
    wait_idle(ax, t)
    return (int(ax.error), int(ax.motor.error), int(ax.encoder.error), int(ax.controller.error))


def main():
    args = parse_args()
    print("Connecting to ODrive...", flush=True)
    dev = odrive.find_any(serial_number=args.serial_number, timeout=20)
    ax = getattr(dev, f"axis{args.axis}")
    m, e = ax.motor, ax.encoder
    sn = format(dev.serial_number, "x")
    print(f"motor #{args.motor_id}  serial={sn}  axis{args.axis}  vbus={dev.vbus_voltage:.1f}V", flush=True)

    e_mc = run(ax, AXIS_STATE_MOTOR_CALIBRATION, 15)
    print(f"motor cal        errs={e_mc}  R={m.config.phase_resistance:.4f}  "
          f"L={m.config.phase_inductance*1e3:.4f}mH", flush=True)
    e_oc = run(ax, AXIS_STATE_ENCODER_OFFSET_CALIBRATION, 25)
    print(f"encoder offs cal errs={e_oc}  offset={e.config.offset}  "
          f"offset_float={e.config.offset_float:.4f}  motor.dir={m.config.direction}", flush=True)
    if any(e_mc) or any(e_oc):
        print("ABORT: calibration reported errors — not saving.", flush=True)
        sys.exit(1)

    if args.check_thermistor:
        mt = ax.motor_thermistor
        t = mt.temperature
        ok = mt.config.enabled and -5 < t < 60
        print(f"thermistor GPIO{mt.config.gpio_pin}: {t:.1f}C enabled={mt.config.enabled} "
              f"lim={mt.config.temp_limit_lower:.0f}/{mt.config.temp_limit_upper:.0f}  "
              f"-> {'OK' if ok else 'CHECK (open NTC reads ~-63C)'}", flush=True)

    m.config.torque_constant = args.kt
    m.config.pre_calibrated = True
    e.config.pre_calibrated = True
    print(f"set torque_constant={args.kt:.4f}  motor.pre_cal={m.config.pre_calibrated}  "
          f"enc.pre_cal={e.config.pre_calibrated}", flush=True)

    errs = (int(ax.error), int(m.error), int(e.error), int(ax.controller.error))
    if any(errs):
        print(f"ABORT: errors present {errs} — not saving.", flush=True)
        sys.exit(1)

    if args.dry_run:
        print("dry-run: NOT saved to flash.", flush=True)
        return

    print("saving to flash ...", flush=True)
    try:
        dev.save_configuration()
    except Exception as ex:
        print(f"  (save_configuration returned, expected USB reset: {ex})", flush=True)

    time.sleep(2.0)
    print("reconnecting to verify ...", flush=True)
    # find_any(serial_number=) is case-sensitive; the board advertises its serial
    # UPPERCASE while format(sn,"x") is lowercase, so a serial containing a hex
    # letter (e.g. 367d...) would time out. Uppercase to match. See memory
    # odrivetool-serial-filter-case-2026-07-10.
    dev = odrive.find_any(serial_number=sn.upper(), timeout=25)
    ax = getattr(dev, f"axis{args.axis}")
    m, e = ax.motor, ax.encoder
    print(f"  user_config_loaded={dev.user_config_loaded}  is_calibrated={m.is_calibrated}", flush=True)
    print(f"  R={m.config.phase_resistance:.4f}  L={m.config.phase_inductance*1e3:.4f}mH  "
          f"Kt={m.config.torque_constant:.4f}  pole_pairs={m.config.pole_pairs}", flush=True)
    print(f"  enc mode={e.config.mode} cpr={e.config.cpr} cs={e.config.abs_spi_cs_gpio_pin} "
          f"offset={e.config.offset} pre_cal={e.config.pre_calibrated}", flush=True)
    print(f"  errors={ax.error},{m.error},{e.error},{ax.controller.error}", flush=True)
    print("DONE.", flush=True)


if __name__ == "__main__":
    main()
