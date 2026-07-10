#!/usr/bin/env python3
"""Measure torque constant Kt via no-load back-EMF.

Spins the motor at several steady no-load speeds and fits
    (Vq - R*Iq) = 2*pi*pole_pairs*lambda * vel      (vel in turns/s)
then reports Kt = 1.5*pole_pairs*lambda and Kv.

Vq is approximated by the applied stator-voltage-vector magnitude
|V| = hypot(final_v_alpha, final_v_beta); at no load Vd~=0 so |V| ~= Vq.

RAM-only: nothing is written to flash. The motor must already be calibrated
(healthy R/L + valid encoder offset) so closed-loop velocity control works.
Motor must be free to spin with NO load / NO gearbox.

NOTE: strips cwd from sys.path so the local ./odrive firmware dir does not
shadow the installed odrive package.
"""
import argparse
import json
import math
import os
import statistics
import sys
import time

sys.path = [p for p in sys.path if p not in ("", ".", os.getcwd())]

import odrive
from odrive.enums import (AXIS_STATE_CLOSED_LOOP_CONTROL, AXIS_STATE_IDLE,
                          CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_PASSTHROUGH)


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--motor-id", help="Motor label; records + auto-names JSON.")
    p.add_argument("--serial-number")
    p.add_argument("--axis", type=int, default=0, choices=(0, 1))
    p.add_argument("--speeds", type=float, nargs="+",
                   default=[4.0, 7.0, 10.0, 13.0, 16.0],
                   help="No-load velocities to sample (motor turns/s).")
    p.add_argument("--current-limit", type=float, default=10.0)
    p.add_argument("--json", help="Write report to this path.")
    return p.parse_args()


def main():
    args = parse_args()
    print("Connecting to ODrive...", flush=True)
    dev = odrive.find_any(serial_number=args.serial_number, timeout=20)
    ax = getattr(dev, f"axis{args.axis}")
    m, c, e = ax.motor, ax.controller, ax.encoder
    R = m.config.phase_resistance
    pp = m.config.pole_pairs
    if not m.is_calibrated:
        print("ABORT: motor not calibrated — run motor_health_check first.", flush=True)
        sys.exit(1)
    print(f"R={R:.4f} ohm  pole_pairs={pp}  vbus={dev.vbus_voltage:.1f}V", flush=True)

    saved = (m.config.current_lim, c.config.control_mode, c.config.input_mode,
             c.config.vel_limit, ax.config.enable_watchdog)
    data = []
    try:
        ax.error = m.error = e.error = c.error = 0
        ax.config.enable_watchdog = False
        m.config.current_lim = args.current_limit
        c.config.vel_limit = max(args.speeds) * 1.4
        c.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        c.config.input_mode = INPUT_MODE_PASSTHROUGH
        c.input_vel = 0.0
        ax.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        time.sleep(0.3)
        if ax.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
            print("closed-loop entry FAILED", ax.error, m.error, e.error, c.error, flush=True)
            sys.exit(1)
        for spd in args.speeds:
            c.input_vel = spd
            time.sleep(1.2)
            Vs, Iqs, Vels = [], [], []
            for _ in range(40):
                time.sleep(0.02)
                cc = m.current_control
                Vs.append(math.hypot(cc.final_v_alpha, cc.final_v_beta))
                Iqs.append(cc.Iq_measured)
                Vels.append(e.vel_estimate)
            V = statistics.fmean(Vs); Iq = statistics.fmean(Iqs); vel = statistics.fmean(Vels)
            bemf = V - R * Iq
            data.append({"cmd_turns_s": spd, "vel_turns_s": vel, "V_mag": V,
                         "Iq_a": Iq, "bemf_v": bemf})
            print(f"  cmd {spd:>4.1f} t/s: vel={vel:6.2f}  |V|={V:6.3f}  "
                  f"Iq={Iq:5.2f}  (V-R*Iq)={bemf:6.3f}", flush=True)
            c.input_vel = 0.0
            time.sleep(0.3)
    finally:
        c.input_vel = 0.0
        ax.requested_state = AXIS_STATE_IDLE
        time.sleep(0.2)
        (m.config.current_lim, c.config.control_mode, c.config.input_mode,
         c.config.vel_limit, ax.config.enable_watchdog) = saved

    n = len(data)
    xs = [d["vel_turns_s"] for d in data]
    ys = [d["bemf_v"] for d in data]
    sx, sy = sum(xs), sum(ys)
    sxx = sum(x * x for x in xs); sxy = sum(x * y for x, y in zip(xs, ys))
    slope = (n * sxy - sx * sy) / (n * sxx - sx * sx)
    intercept = (sy - slope * sx) / n
    lam = slope / (2 * math.pi * pp)
    Kt = 1.5 * pp * lam
    Kv = 60.0 / (2 * math.pi) / (1.5 * pp * lam)
    print(f"\nfit: bemf = {slope:.4f}*vel + {intercept:.4f}  (intercept should be ~0)", flush=True)
    print(f"lambda = {lam*1e3:.4f} mWb   Kt = {Kt:.4f} Nm/A   Kv ~ {Kv:.1f} RPM/V", flush=True)
    print(f"\n-> set  odrv.axis{args.axis}.motor.config.torque_constant = {Kt:.4f}", flush=True)
    print(f"   or   pin_calibration.py --motor-id {args.motor_id} --kt {Kt:.4f}", flush=True)

    report = {"motor_id": args.motor_id, "serial_number": format(dev.serial_number, "x"),
              "axis": args.axis, "R_ohm": R, "pole_pairs": pp,
              "slope": slope, "intercept": intercept, "lambda_wb": lam,
              "Kt_nm_per_a": Kt, "Kv_rpm_per_v": Kv, "samples": data}
    out = args.json
    if out is None and args.motor_id is not None:
        d = os.path.join(os.path.dirname(os.path.abspath(__file__)), "reports")
        os.makedirs(d, exist_ok=True)
        out = os.path.join(d, f"motor-{args.motor_id}-kt.json")
    if out:
        with open(out, "w", encoding="utf-8") as f:
            json.dump(report, f, indent=2); f.write("\n")
        print(f"report -> {out}", flush=True)


if __name__ == "__main__":
    main()
