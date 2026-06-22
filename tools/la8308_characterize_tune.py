#!/usr/bin/env python3
"""Characterize the LA8308 (direct-drive, no gearbox) and tune controller gains.

Two phases, selectable with --characterize / --tune (default: both):

  CHARACTERIZE (no flash writes unless --save-kt):
    - report stored R / L / pole_pairs / Kt
    - free-spin no-load current sweep  -> friction/iron-loss vs speed
    - back-EMF fit from steady-state Vq -> measured Kt / Kv
    Directly comparable to motor #10 free-spin numbers in
    spider-motor-tools/reports/.

  TUNE (no flash writes unless --save-gains):
    - velocity-loop step-response sweep over vel_gain candidates
    - pick largest stable vel_gain, set vel_integrator_gain by ratio
    - position-loop step-response sweep over pos_gain candidates

All motion is bench-safe for a FREE motor shaft (no gearbox / no load).
RAM config we touch is restored on exit; nothing is written to flash unless an
explicit --save flag is given.

NOTE: strips cwd from sys.path so the local ./odrive dir does not shadow the
installed odrive package (same guard as motor_health_check.py).
"""

import argparse
import json
import math
import os
import statistics
import sys
import time

# tools/ contains a local odrive/ package that shadows the installed one;
# drop cwd AND this script's own dir so `import odrive` finds the venv package.
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path = [p for p in sys.path if p not in ("", ".", os.getcwd(), _SCRIPT_DIR)]

import odrive
from odrive.enums import (
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    AXIS_STATE_IDLE,
    CONTROL_MODE_POSITION_CONTROL,
    CONTROL_MODE_VELOCITY_CONTROL,
    INPUT_MODE_PASSTHROUGH,
)

TWO_PI = 2.0 * math.pi


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--serial-number")
    p.add_argument("--axis", type=int, default=0, choices=(0, 1))
    p.add_argument("--current-limit", type=float, default=25.0)
    p.add_argument("--fet-temp-limit", type=float, default=70.0)
    p.add_argument("--spin-speeds", type=float, nargs="+",
                   default=[5.0, 10.0, 15.0, 20.0],
                   help="Free-spin / back-EMF speeds (motor turns/s).")
    p.add_argument("--characterize", action="store_true")
    p.add_argument("--spin-soak", action="store_true",
                   help="Sustained free-spin: cycle through --soak-speeds for "
                        "--soak-seconds total, logging vel/Iq/FET and watching "
                        "for magnet-slip (lost tracking / Iq saturation).")
    p.add_argument("--soak-seconds", type=float, default=60.0,
                   help="Total sustained-spin duration, split across soak speeds.")
    p.add_argument("--soak-speeds", type=float, nargs="+", default=None,
                   help="Speeds (motor turns/s) for --spin-soak; "
                        "defaults to --spin-speeds.")
    p.add_argument("--tune", action="store_true")
    p.add_argument("--vel-step", type=float, default=8.0,
                   help="Velocity step amplitude for vel-gain tuning (turns/s).")
    p.add_argument("--pos-step", type=float, default=2.0,
                   help="Position step amplitude for pos-gain tuning (turns).")
    p.add_argument("--vel-gains", type=float, nargs="+",
                   default=[0.05, 0.08, 0.12, 0.18, 0.27, 0.4])
    p.add_argument("--pos-gains", type=float, nargs="+",
                   default=[10.0, 20.0, 40.0, 60.0, 90.0])
    p.add_argument("--save-kt", action="store_true",
                   help="Write measured torque_constant to flash.")
    p.add_argument("--save-gains", action="store_true",
                   help="Write tuned gains to flash.")
    p.add_argument("--json", help="Write the full report to this JSON path.")
    return p.parse_args()


def err_tuple(axis):
    return (int(axis.error), int(axis.motor.error),
            int(axis.encoder.error), int(axis.controller.error))


def clear_errors(axis):
    axis.error = 0
    axis.motor.error = 0
    axis.encoder.error = 0
    axis.controller.error = 0


def enter_closed_loop(axis):
    clear_errors(axis)
    axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    time.sleep(0.3)
    return axis.current_state == AXIS_STATE_CLOSED_LOOP_CONTROL


def sample(axis, n, dt, fet_limit):
    """Collect (t, vel, iq, vq) samples; raise if FET too hot."""
    m = axis.motor
    rows = []
    t0 = time.monotonic()
    for _ in range(n):
        if axis.fet_thermistor.temperature > fet_limit:
            raise RuntimeError("FET temperature limit hit")
        rows.append((time.monotonic() - t0,
                     float(axis.encoder.vel_estimate),
                     float(m.current_control.Iq_measured),
                     float(m.current_control.v_current_control_integral_q)))
        time.sleep(dt)
    return rows


def characterize(axis, args, R, pole_pairs):
    e = axis.encoder; m = axis.motor; c = axis.controller
    print("\n=== free-spin no-load sweep + back-EMF ===", flush=True)
    c.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    c.config.input_mode = INPUT_MODE_PASSTHROUGH
    c.config.vel_limit = max(args.spin_speeds) * 1.4
    c.input_vel = 0.0
    if not enter_closed_loop(axis):
        return {"status": "FAIL", "reason": "closed-loop entry failed",
                "errors": err_tuple(axis)}
    runs = []
    try:
        for spd in args.spin_speeds:
            c.input_vel = spd
            time.sleep(1.2)                       # settle
            rows = sample(axis, 30, 0.02, args.fet_temp_limit)
            vels = [r[1] for r in rows]
            iqs = [abs(r[2]) for r in rows]
            vqs = [r[3] for r in rows]
            mean_vel = statistics.fmean(vels)
            mean_iq = statistics.fmean(iqs)
            mean_vq = statistics.fmean(vqs)
            reached = abs(mean_vel) >= 0.8 * spd
            # back-EMF: Vq - R*Iq = lambda * omega_elec ; omega_elec = 2pi*pp*vel
            omega_elec = TWO_PI * pole_pairs * mean_vel
            bemf = mean_vq - R * statistics.fmean([r[2] for r in rows])
            runs.append({"cmd_turns_s": spd, "mean_vel_turns_s": mean_vel,
                         "mean_abs_iq_a": mean_iq, "max_abs_iq_a": max(iqs),
                         "mean_vq_v": mean_vq, "omega_elec_rad_s": omega_elec,
                         "bemf_q_v": bemf, "reached": reached})
            print(f"  cmd {spd:4.1f} t/s: vel={mean_vel:6.2f}  |Iq|={mean_iq:5.2f}A "
                  f"max={max(iqs):5.2f}A  Vq={mean_vq:5.2f}V  bemf={bemf:5.2f}V  "
                  f"{'OK' if reached else 'DID NOT REACH'}", flush=True)
            c.input_vel = 0.0
            time.sleep(0.4)
    finally:
        c.input_vel = 0.0
        axis.requested_state = AXIS_STATE_IDLE
        time.sleep(0.2)

    # Linear fit bemf = lambda * omega_elec through valid reached points
    pts = [(r["omega_elec_rad_s"], r["bemf_q_v"]) for r in runs if r["reached"]]
    kt = kv = lam = None
    if len(pts) >= 2:
        sxx = sum(x * x for x, _ in pts)
        sxy = sum(x * y for x, y in pts)
        lam = sxy / sxx                            # flux linkage (Wb), zero-intercept fit
        kt = 1.5 * pole_pairs * lam                # Nm/A
        kv = 8.27 / kt if kt else None             # RPM/V
        print(f"\n  back-EMF fit: lambda={lam*1e3:.3f} mWb  "
              f"Kt={kt:.4f} Nm/A  Kv={kv:.1f} RPM/V", flush=True)
    return {"status": "PASS", "runs": runs,
            "flux_linkage_wb": lam, "kt_measured": kt, "kv_measured": kv}


def spin_soak(axis, args, R, pole_pairs):
    """Sustained free-spin across several speeds for ~--soak-seconds total.

    Unlike characterize (a ~0.6 s grab per speed) this holds each speed for a
    real stretch of time so you can watch current/temperature settle and catch
    intermittent faults. Given the LA8308's history of a slipping sensor magnet
    (commutation inverts under torque -> velocity collapses and Iq pins), each
    sample is checked for lost tracking and Iq saturation; the soak aborts
    safely on any axis error, FET over-temp, or slip signature.
    """
    e = axis.encoder; m = axis.motor; c = axis.controller
    speeds = args.soak_speeds or args.spin_speeds
    per = max(2.0, args.soak_seconds / len(speeds))
    print(f"\n=== sustained spin soak: {len(speeds)} speeds x {per:.1f}s "
          f"(~{args.soak_seconds:.0f}s total) ===", flush=True)
    c.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    c.config.input_mode = INPUT_MODE_PASSTHROUGH
    c.config.vel_limit = max(abs(s) for s in speeds) * 1.4
    c.input_vel = 0.0
    if not enter_closed_loop(axis):
        return {"status": "FAIL", "reason": "closed-loop entry failed",
                "errors": err_tuple(axis)}
    i_sat = 0.9 * args.current_limit
    runs = []
    aborted = None
    try:
        for spd in speeds:
            c.input_vel = spd
            time.sleep(1.2)                            # settle before logging
            fet0 = float(axis.fet_thermistor.temperature)
            vels, iqs = [], []
            t0 = time.monotonic()
            slip = False
            while time.monotonic() - t0 < per:
                fet = float(axis.fet_thermistor.temperature)
                if fet > args.fet_temp_limit:
                    aborted = f"FET temp {fet:.1f}C > {args.fet_temp_limit}C"
                    break
                if any(err_tuple(axis)):
                    aborted = f"axis error {err_tuple(axis)}"
                    break
                v = float(e.vel_estimate)
                iq = float(m.current_control.Iq_measured)
                vels.append(v); iqs.append(iq)
                # magnet-slip signature: commanded torque but rotor not tracking
                if abs(v) < 0.6 * abs(spd) or (v * spd < 0) or abs(iq) > i_sat:
                    slip = True
                    aborted = (f"slip/lost-tracking at {spd:.1f} t/s: "
                               f"vel={v:.2f} Iq={iq:.1f}A")
                    break
                time.sleep(0.1)
            absiq = [abs(x) for x in iqs]
            row = {"cmd_turns_s": spd, "seconds": round(time.monotonic() - t0, 1),
                   "mean_vel_turns_s": statistics.fmean(vels) if vels else 0.0,
                   "min_vel_turns_s": min(vels) if vels else 0.0,
                   "max_vel_turns_s": max(vels) if vels else 0.0,
                   "mean_abs_iq_a": statistics.fmean(absiq) if absiq else 0.0,
                   "max_abs_iq_a": max(absiq) if absiq else 0.0,
                   "fet_start_c": fet0,
                   "fet_end_c": float(axis.fet_thermistor.temperature),
                   "slip_detected": slip, "errors": err_tuple(axis)}
            runs.append(row)
            flag = "SLIP!" if slip else "ok"
            print(f"  {spd:5.1f} t/s {row['seconds']:4.1f}s: "
                  f"vel {row['min_vel_turns_s']:5.2f}..{row['max_vel_turns_s']:5.2f} "
                  f"|Iq| mean={row['mean_abs_iq_a']:4.2f}A max={row['max_abs_iq_a']:4.2f}A "
                  f"FET {fet0:.1f}->{row['fet_end_c']:.1f}C  {flag}", flush=True)
            c.input_vel = 0.0
            time.sleep(0.4)
            if aborted:
                break
    finally:
        c.input_vel = 0.0
        axis.requested_state = AXIS_STATE_IDLE
        time.sleep(0.2)
    status = "FAIL" if aborted else "PASS"
    if aborted:
        print(f"  ABORTED: {aborted}", flush=True)
    return {"status": status, "aborted": aborted, "runs": runs}


def step_metrics(rows, target, settle_frac=0.05):
    """Overshoot %, settling time, and post-settle ripple for a step to target."""
    vals = [r[1] for r in rows]
    ts = [r[0] for r in rows]
    peak = max(vals) if target >= 0 else min(vals)
    overshoot = (peak - target) / target * 100.0 if target else 0.0
    band = abs(target) * settle_frac
    settle_t = None
    for i in range(len(vals)):
        if all(abs(v - target) <= band for v in vals[i:]):
            settle_t = ts[i]
            break
    tail = vals[int(len(vals) * 0.6):]
    ripple = statistics.pstdev(tail) if len(tail) > 1 else 0.0
    return {"overshoot_pct": overshoot, "settle_s": settle_t,
            "ripple_std": ripple, "peak": peak}


def tune_velocity(axis, args):
    c = axis.controller
    print("\n=== velocity-loop step-response sweep ===", flush=True)
    c.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    c.config.input_mode = INPUT_MODE_PASSTHROUGH
    c.config.vel_limit = abs(args.vel_step) * 2.0
    results = []
    for vg in args.vel_gains:
        clear_errors(axis)
        c.config.vel_gain = vg
        c.config.vel_integrator_gain = 0.0       # isolate proportional response
        c.input_vel = 0.0
        if not enter_closed_loop(axis):
            results.append({"vel_gain": vg, "status": "no-closed-loop",
                            "errors": err_tuple(axis)})
            print(f"  vel_gain={vg:.3f}: closed-loop entry FAILED {err_tuple(axis)}",
                  flush=True)
            continue
        time.sleep(0.3)
        c.input_vel = args.vel_step
        rows = sample(axis, 40, 0.01, args.fet_temp_limit)   # 0.4 s capture
        c.input_vel = 0.0
        time.sleep(0.3)
        axis.requested_state = AXIS_STATE_IDLE
        time.sleep(0.2)
        mt = step_metrics(rows, args.vel_step)
        errs = err_tuple(axis)
        results.append({"vel_gain": vg, "status": "ok", **mt, "errors": errs})
        print(f"  vel_gain={vg:.3f}: overshoot={mt['overshoot_pct']:5.1f}%  "
              f"settle={mt['settle_s']}  ripple={mt['ripple_std']:.3f}  errs={errs}",
              flush=True)

    # pick largest vel_gain with overshoot<=20%, low ripple, no errors
    ok = [r for r in results if r.get("status") == "ok"
          and not any(r["errors"]) and r["overshoot_pct"] <= 20.0
          and r["ripple_std"] <= 0.4 and r["settle_s"] is not None]
    chosen = max(ok, key=lambda r: r["vel_gain"]) if ok else None
    return {"results": results, "chosen_vel_gain": chosen["vel_gain"] if chosen else None}


def tune_position(axis, args, vel_gain, vel_integrator_gain):
    c = axis.controller
    print("\n=== position-loop step-response sweep ===", flush=True)
    c.config.control_mode = CONTROL_MODE_POSITION_CONTROL
    c.config.input_mode = INPUT_MODE_PASSTHROUGH
    c.config.vel_gain = vel_gain
    c.config.vel_integrator_gain = vel_integrator_gain
    c.config.vel_limit = max(20.0, abs(args.pos_step) * 8.0)
    results = []
    for pg in args.pos_gains:
        clear_errors(axis)
        c.config.pos_gain = pg
        if not enter_closed_loop(axis):
            results.append({"pos_gain": pg, "status": "no-closed-loop",
                            "errors": err_tuple(axis)})
            continue
        start = float(axis.encoder.pos_estimate)
        c.input_pos = start
        time.sleep(0.2)
        target = start + args.pos_step
        c.input_pos = target
        t0 = time.monotonic()
        rows = []
        while time.monotonic() - t0 < 0.8:
            if axis.fet_thermistor.temperature > args.fet_temp_limit:
                break
            rows.append((time.monotonic() - t0,
                         float(axis.encoder.pos_estimate) - start,
                         float(axis.motor.current_control.Iq_measured), 0.0))
            time.sleep(0.01)
        c.input_pos = start
        time.sleep(0.4)
        axis.requested_state = AXIS_STATE_IDLE
        time.sleep(0.2)
        mt = step_metrics(rows, args.pos_step)
        errs = err_tuple(axis)
        results.append({"pos_gain": pg, "status": "ok", **mt, "errors": errs})
        print(f"  pos_gain={pg:5.1f}: overshoot={mt['overshoot_pct']:5.1f}%  "
              f"settle={mt['settle_s']}  ripple={mt['ripple_std']:.4f}  errs={errs}",
              flush=True)
    ok = [r for r in results if r.get("status") == "ok"
          and not any(r["errors"]) and r["overshoot_pct"] <= 15.0
          and r["settle_s"] is not None]
    chosen = max(ok, key=lambda r: r["pos_gain"]) if ok else None
    return {"results": results, "chosen_pos_gain": chosen["pos_gain"] if chosen else None}


def main():
    args = parse_args()
    if not args.characterize and not args.tune and not args.spin_soak:
        args.characterize = args.tune = True

    print("Connecting to ODrive...", flush=True)
    dev = odrive.find_any(serial_number=args.serial_number, timeout=25)
    axis = getattr(dev, f"axis{args.axis}")
    m = axis.motor; c = axis.controller
    R = float(m.config.phase_resistance)
    pole_pairs = int(m.config.pole_pairs)

    report = {
        "serial_number": format(dev.serial_number, "x"),
        "vbus_v": float(dev.vbus_voltage),
        "fet_temp_start_c": float(axis.fet_thermistor.temperature),
        "stored": {
            "R_ohm": R, "L_H": float(m.config.phase_inductance),
            "pole_pairs": pole_pairs, "torque_constant": float(m.config.torque_constant),
            "current_lim": float(m.config.current_lim),
            "pos_gain": float(c.config.pos_gain), "vel_gain": float(c.config.vel_gain),
            "vel_integrator_gain": float(c.config.vel_integrator_gain),
        },
        "pre_errors": err_tuple(axis),
    }
    print(f"serial={report['serial_number']} vbus={report['vbus_v']:.1f}V "
          f"FET={report['fet_temp_start_c']:.1f}C", flush=True)
    print(f"stored: R={R:.4f} ohm  L={m.config.phase_inductance*1e6:.1f} uH  "
          f"pole_pairs={pole_pairs}  Kt={m.config.torque_constant:.4f}", flush=True)

    saved = (m.config.current_lim, c.config.control_mode, c.config.input_mode,
             c.config.vel_limit, c.config.pos_gain, c.config.vel_gain,
             c.config.vel_integrator_gain, axis.config.enable_watchdog)
    try:
        clear_errors(axis)
        axis.config.enable_watchdog = False
        m.config.current_lim = args.current_limit

        if args.characterize:
            report["characterize"] = characterize(axis, args, R, pole_pairs)

        if args.spin_soak:
            report["spin_soak"] = spin_soak(axis, args, R, pole_pairs)

        if args.tune:
            vt = tune_velocity(axis, args)
            report["tune_velocity"] = vt
            vg = vt["chosen_vel_gain"]
            if vg is None:
                print("  no stable vel_gain found; skipping position tuning", flush=True)
            else:
                vig = 2.0 * vg          # keep ODrive default integrator/gain ratio
                pt = tune_position(axis, args, vg, vig)
                report["tune_position"] = pt
                report["recommended_gains"] = {
                    "vel_gain": vg, "vel_integrator_gain": vig,
                    "pos_gain": pt["chosen_pos_gain"]}
    finally:
        c.input_vel = 0.0
        axis.requested_state = AXIS_STATE_IDLE
        time.sleep(0.3)
        (m.config.current_lim, c.config.control_mode, c.config.input_mode,
         c.config.vel_limit, c.config.pos_gain, c.config.vel_gain,
         c.config.vel_integrator_gain, axis.config.enable_watchdog) = saved
        report["fet_temp_end_c"] = float(axis.fet_thermistor.temperature)
        report["post_errors"] = err_tuple(axis)

    # Optional flash writes
    wrote = []
    ch = report.get("characterize", {})
    if args.save_kt and ch.get("kt_measured"):
        m.config.torque_constant = float(ch["kt_measured"])
        wrote.append(f"torque_constant={ch['kt_measured']:.4f}")
    rg = report.get("recommended_gains")
    if args.save_gains and rg and rg.get("pos_gain"):
        c.config.vel_gain = rg["vel_gain"]
        c.config.vel_integrator_gain = rg["vel_integrator_gain"]
        c.config.pos_gain = rg["pos_gain"]
        wrote.append(f"gains pos={rg['pos_gain']} vel={rg['vel_gain']} "
                     f"vi={rg['vel_integrator_gain']}")
    if wrote:
        try:
            dev.save_configuration()
        except Exception:
            pass
        print(f"\nSAVED to flash: {', '.join(wrote)}", flush=True)
    else:
        print("\n(no flash writes; pass --save-kt / --save-gains to persist)", flush=True)

    if report.get("recommended_gains"):
        print(f"recommended gains: {report['recommended_gains']}", flush=True)

    if args.json:
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump(report, f, indent=2)
            f.write("\n")
        print(f"report -> {args.json}", flush=True)


if __name__ == "__main__":
    main()
