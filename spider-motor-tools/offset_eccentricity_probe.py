#!/usr/bin/env python3
"""
Commutation-offset eccentricity / slip probe v2 (bench, RAM-only).

Separates the two causes of run-to-run offset scatter that a single health
check cannot tell apart:

  offset(position, time) = C + Ecc(position) + Drift(time)

  * Ecc(position): STATIC eccentricity of an off-center but rigidly-fixed
    magnet -> a once-per-mechanical-rev function of starting position that is
    the SAME every time you revisit a position. Benign, correctable.
  * Drift(time): a SLIPPING magnet (motor #2 failure) -> the offset at a FIXED
    position marches over repeated energize/cal cycles. The pinned offset goes
    stale under load.

Method: interleave a cal at ONE fixed reference position between every map
point. The reference series holds position constant, so it isolates Drift(time).
The map series varies position; after removing the drift model it isolates
Ecc(position). Position is landed to an exact encoder count (not just a
commanded fraction) so the reference truly repeats.

ODrive's offset cal only sweeps ~0.5 mechanical turn, so it averages the
eccentricity over a different arc per starting position -> position dependence
is EXPECTED for any eccentric absolute encoder; that alone is benign. Only
time-drift at a fixed position indicts the magnet.

Nothing is written to flash. Saved offset untouched; a reboot restores it.
"""
import argparse, sys, time, math
import odrive
from odrive.enums import *


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--serial-number", default=None)
    p.add_argument("--axis", type=int, default=0)
    p.add_argument("--motor-id", default="?")
    p.add_argument("--map-points", type=int, default=8)
    p.add_argument("--ref-count", type=int, default=8192,
                   help="fixed encoder count used for the drift (reference) series")
    p.add_argument("--vel-limit", type=float, default=5.0)
    p.add_argument("--json", default=None)
    return p.parse_args()


def wait_idle(ax, t):
    d = time.monotonic() + t
    while ax.current_state != AXIS_STATE_IDLE:
        time.sleep(0.2)
        if time.monotonic() > d:
            return False
    return True


def clear(ax):
    ax.error = ax.motor.error = ax.encoder.error = ax.controller.error = 0
    time.sleep(0.05)


def signed_wrap(d, cpr):
    d %= cpr
    if d > cpr / 2:
        d -= cpr
    return d


def land_on_count(ax, target_count, cpr, vel_limit, tol=6, max_iter=14):
    """Closed-loop move until count_in_cpr is within `tol` of target, then idle.
    Returns the ACTUAL count_in_cpr after the idle relax (what the cal will see)."""
    clear(ax)
    c = ax.controller
    c.config.control_mode = CONTROL_MODE_POSITION_CONTROL
    c.config.input_mode = INPUT_MODE_POS_FILTER
    c.config.input_filter_bandwidth = 2.0
    c.config.vel_limit = vel_limit
    c.input_pos = ax.encoder.pos_estimate
    ax.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    time.sleep(0.3)
    for _ in range(max_iter):
        cur = int(ax.encoder.count_in_cpr)
        err = signed_wrap(target_count - cur, cpr)
        if abs(err) <= tol:
            break
        c.input_pos = ax.encoder.pos_estimate + err / cpr
        time.sleep(0.25)
    time.sleep(0.4)  # settle into the cogging detent
    ax.requested_state = AXIS_STATE_IDLE
    wait_idle(ax, 3)
    return int(ax.encoder.count_in_cpr)


def offset_cal(ax):
    clear(ax)
    ax.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
    time.sleep(0.3)
    ok = wait_idle(ax, 25)
    errs = (int(ax.error), int(ax.motor.error), int(ax.encoder.error), int(ax.controller.error))
    return ok, errs


def linfit(xs, ys):
    import numpy as np
    x = np.asarray(xs, float); y = np.asarray(ys, float)
    A = np.column_stack([x, np.ones_like(x)])
    (slope, intercept), *_ = np.linalg.lstsq(A, y, rcond=None)
    yhat = A @ [slope, intercept]
    ss_res = float(np.sum((y - yhat) ** 2))
    ss_tot = float(np.sum((y - y.mean()) ** 2)) or 1e-12
    return float(slope), float(intercept), 1 - ss_res / ss_tot


def fit_first_harmonic(thetas, ys):
    import numpy as np
    th = np.asarray(thetas); y = np.asarray(ys)
    M = np.column_stack([np.ones_like(th), np.cos(th), np.sin(th)])
    coef, *_ = np.linalg.lstsq(M, y, rcond=None)
    C, A, B = coef
    resid = y - M @ coef
    return math.hypot(A, B), float(C), float(np.sqrt(np.mean(resid ** 2)))


def main():
    args = parse_args()
    print("Connecting to ODrive...", flush=True)
    dev = odrive.find_any(serial_number=args.serial_number, timeout=25)
    ax = getattr(dev, f"axis{args.axis}")
    e = ax.encoder
    sn = format(dev.serial_number, "x")
    cpr = e.config.cpr
    pp = ax.motor.config.pole_pairs
    print(f"motor #{args.motor_id}  serial={sn}  axis{args.axis}  cpr={cpr}  pole_pairs={pp}", flush=True)
    print(f"saved offset (flash, untouched)={e.config.offset}  ref_count={args.ref_count}", flush=True)
    if not ax.motor.is_calibrated:
        print("ABORT: motor not calibrated.", flush=True)
        sys.exit(1)

    records = []
    seq = [0]

    def do(kind, target_count):
        actual = land_on_count(ax, target_count, cpr, args.vel_limit)
        ok, errs = offset_cal(ax)
        off = float(e.config.offset_float)
        idx = seq[0]; seq[0] += 1
        rec = dict(kind=kind, seq=idx, target=target_count, actual=actual,
                   angle_deg=actual / cpr * 360, offset_float=off, ok=ok, errs=errs)
        records.append(rec)
        flag = "" if (ok and not any(errs)) else "  <-- CAL ERROR"
        tc = "REF " if kind == "ref" else "map "
        print(f"  #{idx:02d} [{tc}] count={actual:5d} ({rec['angle_deg']:6.1f} deg)  "
              f"offset_float={off:7.4f}{flag}", flush=True)

    print("\n=== interleaved: REF (fixed count) before each MAP point ===", flush=True)
    for i in range(args.map_points):
        do("ref", args.ref_count)
        do("map", int((i + 0.5) / args.map_points * cpr))
    do("ref", args.ref_count)  # final reference to bracket the run

    ax.requested_state = AXIS_STATE_IDLE

    # ---------- analysis ----------
    refs = [r for r in records if r["kind"] == "ref" and r["ok"] and not any(r["errs"])]
    maps = [r for r in records if r["kind"] == "map" and r["ok"] and not any(r["errs"])]
    print("\n=== ANALYSIS ===", flush=True)
    print(f"clean cals: {len(refs)} reference + {len(maps)} map", flush=True)

    drift_slope = drift_total = None
    if len(refs) >= 3:
        actual_spread = max(r["actual"] for r in refs) - min(r["actual"] for r in refs)
        vals = [r["offset_float"] for r in refs]
        m = sum(vals) / len(vals); sd = (sum((v - m) ** 2 for v in vals) / len(vals)) ** 0.5
        drift_slope, b, r2 = linfit([r["seq"] for r in refs], vals)
        drift_total = drift_slope * (refs[-1]["seq"] - refs[0]["seq"])
        print(f"\nDRIFT (fixed position, isolates slip):", flush=True)
        print(f"  reference offset: mean={m:.4f} sd={sd:.4f} rad  "
              f"(landed count spread={actual_spread} cts / "
              f"{actual_spread/cpr*360:.1f} deg)", flush=True)
        print(f"  linear trend: slope={drift_slope:+.5f} rad/cal  "
              f"total over run={drift_total:+.4f} rad  R^2={r2:.2f}", flush=True)

    ecc_amp = ecc_res = None
    if len(maps) >= 4:
        # remove the time-drift model (from references) before fitting position
        thetas, ys = [], []
        for r in maps:
            detr = r["offset_float"]
            if drift_slope is not None:
                detr = r["offset_float"] - (drift_slope * r["seq"] + b)
            thetas.append(r["actual"] / cpr * 2 * math.pi); ys.append(detr)
        ecc_amp, C, ecc_res = fit_first_harmonic(thetas, ys)
        print(f"\nECCENTRICITY (position dep., drift-removed):", flush=True)
        print(f"  1/rev amplitude={ecc_amp:.4f} rad  residual_rms={ecc_res:.4f} rad", flush=True)
        print(f"  implied encoder eccentricity ~{ecc_amp/0.637:.4f} rad elec = "
              f"~{math.degrees(ecc_amp/0.637)/pp:.2f} deg mech", flush=True)

    # ---------- verdict ----------
    print("\n=== VERDICT ===", flush=True)
    if drift_total is not None:
        ad = abs(drift_total)
        if ad < 0.05:
            v = f"STABLE: fixed-position offset held within {ad:.3f} rad over the run -> magnet NOT slipping"
        elif ad < 0.12 and r2 < 0.4:
            v = f"noisy but no clear trend (total {drift_total:+.3f} rad, R^2={r2:.2f}) -> likely OK, borderline"
        elif r2 >= 0.4:
            v = (f"DRIFTING: fixed-position offset trends {drift_total:+.3f} rad over the run "
                 f"(R^2={r2:.2f}) -> MAGNET SLIPPING across energize cycles")
        else:
            v = f"scattered (total {drift_total:+.3f} rad) but no monotonic trend -> cal noise > drift; inconclusive"
        print(f"  slip: {v}", flush=True)
    if ecc_amp is not None:
        print(f"  eccentricity: {ecc_amp:.3f} rad 1/rev component present "
              f"(residual {ecc_res:.3f} rad). This part is static/benign.", flush=True)

    if args.json:
        import json
        with open(args.json, "w") as f:
            json.dump(dict(serial=sn, motor_id=args.motor_id, cpr=cpr, pole_pairs=pp,
                           saved_offset=int(e.config.offset), ref_count=args.ref_count,
                           records=records, drift_slope=drift_slope, drift_total=drift_total,
                           ecc_amplitude=ecc_amp, ecc_residual=ecc_res), f, indent=2)
        print(f"\nreport -> {args.json}", flush=True)
    print("\n(RAM-only: saved flash offset unchanged; reboot restores it.)", flush=True)


if __name__ == "__main__":
    main()
