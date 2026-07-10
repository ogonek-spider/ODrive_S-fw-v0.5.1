#!/usr/bin/env python3
"""
Passive cool-down logger for an ODrive S motor (radiator characterization).

Motor stays IDLE (no current, nothing moves). Logs motor-thermistor and FET
temperature as they decay toward ambient, so you can fit the radiator time
constant tau and thermal resistance. Pairs with heat_hold_test.py: heat, then
log the cool-down. Purely read-only on the drive — no state changes.

Stops at a near-ambient temp, a slow-rate settle, or a time cap.

Cool-down model: T(t) = T_amb + (T0 - T_amb) * exp(-t/tau).

NOTE: tools live alongside a local `odrive/` package dir in the firmware repo
that can shadow the installed package. This script strips cwd from sys.path.
"""

import argparse
import json
import math
import os
import sys
import time

sys.path = [p for p in sys.path if p not in ("", ".", os.getcwd())]

import odrive
from odrive.enums import AXIS_STATE_IDLE

try:
    from odrive.libodrive import DeviceLostException
except Exception:  # pragma: no cover - name varies across odrivetool versions
    class DeviceLostException(Exception):
        pass


def is_device_lost(exc):
    """True if the exception looks like a USB disconnect (name varies)."""
    return isinstance(exc, DeviceLostException) or "disconnect" in str(exc).lower()


def ntc_temp_from_adc(voltage, rfix, r0, beta, t0_c=25.0):
    """Divider voltage (3.3V--rfix--pin--NTC--GND) to degrees C (beta model).
    Returns None if the pin is railed (open/short)."""
    if voltage <= 0.002 or voltage >= 3.298:
        return None
    rntc = rfix * voltage / (3.3 - voltage)
    t0 = t0_c + 273.15
    tk = 1.0 / (1.0 / t0 + (1.0 / beta) * math.log(rntc / r0))
    return tk - 273.15


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--motor-id")
    p.add_argument("--serial-number")
    p.add_argument("--axis", type=int, default=0, choices=(0, 1))
    p.add_argument("--ambient", type=float, default=None,
                   help="Ambient temp (C) for the fit/stop. Default: FET temp at "
                        "the end (a decent room-temp proxy once settled).")
    p.add_argument("--stop-above-ambient", type=float, default=2.0,
                   help="Stop once motor temp is within this many C of ambient. "
                        "Default 2.")
    p.add_argument("--max-minutes", type=float, default=30.0)
    p.add_argument("--interval", type=float, default=3.0)
    p.add_argument("--json")
    p.add_argument("--jsonl")
    # Optional extra thermistor read via raw ADC (e.g. the radiator ring sensor).
    p.add_argument("--ring-gpio", type=int, default=None,
                   help="GPIO number of an extra NTC (raw ADC read), e.g. 3. "
                        "Logged as ring_c so you can watch the winding/rotor soak "
                        "and the radiator decay together.")
    p.add_argument("--ring-rfix", type=float, default=5000.0)
    p.add_argument("--ring-r0", type=float, default=10000.0)
    p.add_argument("--ring-beta", type=float, default=3950.0)
    return p.parse_args()


def read_ring(dev, args):
    """Median of 3 raw ADC reads of the ring NTC (PWM EMI glitches lone samples).
    Returns None if no ring configured or the pin is railed."""
    if args.ring_gpio is None:
        return None
    vs = sorted(float(dev.get_adc_voltage(args.ring_gpio)) for _ in range(3))
    return ntc_temp_from_adc(vs[1], args.ring_rfix, args.ring_r0, args.ring_beta)


def main():
    args = parse_args()
    print("Connecting to ODrive...", flush=True)
    dev = odrive.find_any(serial_number=args.serial_number, timeout=20)
    axis = getattr(dev, f"axis{args.axis}")
    mt = axis.motor_thermistor
    ft = axis.fet_thermistor

    # Make sure the motor is idle (no heating) for a clean passive cool-down.
    if axis.current_state != AXIS_STATE_IDLE:
        axis.requested_state = AXIS_STATE_IDLE
        time.sleep(0.3)

    t_start_motor = float(mt.temperature)
    t_start_fet = float(ft.temperature)
    report = {
        "test": "passive cool-down (motor idle, no current)",
        "motor_id": args.motor_id,
        "serial_number": format(dev.serial_number, "x"),
        "axis": args.axis,
        "motor_temp_start_c": t_start_motor,
        "fet_temp_start_c": t_start_fet,
        "samples": [],
    }

    if args.motor_id is not None:
        print(f"MOTOR #{args.motor_id}", flush=True)
    print(f"serial={report['serial_number']}  start MOTOR={t_start_motor:.1f}C "
          f"FET={t_start_fet:.1f}C", flush=True)
    amb_note = f"{args.ambient:.1f}C (fixed)" if args.ambient is not None else "FET-at-end proxy"
    print(f"idle cool-down; stop within {args.stop_above_ambient:.1f}C of ambient "
          f"({amb_note}) or {args.max_minutes:.0f} min", flush=True)

    out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "reports")
    json_path = args.json
    jsonl_path = args.jsonl
    if args.motor_id is not None:
        if json_path is None:
            json_path = os.path.join(out_dir, f"motor-{args.motor_id}-cooldown.json")
        if jsonl_path is None:
            jsonl_path = os.path.join(out_dir, f"motor-{args.motor_id}-cooldown.jsonl")
    jsonl_f = None
    if jsonl_path:
        os.makedirs(os.path.dirname(os.path.abspath(jsonl_path)), exist_ok=True)
        jsonl_f = open(jsonl_path, "w", encoding="utf-8")

    def reconnect():
        # Re-establish the USB link after a device drop. The motor is idle the
        # whole time (passive cool-down), so there is nothing to re-arm; just
        # refresh the handles. Returns True on success.
        nonlocal dev, axis, mt, ft
        try:
            dev = odrive.find_any(serial_number=args.serial_number, timeout=25)
        except Exception as e:
            print(f"    reconnect: find_any failed: {e}", flush=True)
            return False
        axis = getattr(dev, f"axis{args.axis}")
        mt = axis.motor_thermistor
        ft = axis.fet_thermistor
        return True

    stop_reason = "unknown"
    reconnects = 0
    t0 = time.monotonic()
    deadline = t0 + args.max_minutes * 60.0
    last_temp = t_start_motor
    last_t = t0
    slow = 0  # consecutive slow-rate samples (settle detection)
    try:
        while True:
            now = time.monotonic()
            elapsed = now - t0
            # Read all sensors; on a USB drop (this board's link is flaky)
            # reconnect and retry rather than aborting the whole cool-down.
            try:
                motor_t = float(mt.temperature)
                fet_t = float(ft.temperature)
                ring_t = read_ring(dev, args)
            except Exception as e:
                if not is_device_lost(e):
                    raise
                reconnects += 1
                print(f"    device lost (USB drop) - reconnecting "
                      f"(attempt {reconnects})", flush=True)
                if reconnect():
                    print("    reconnected", flush=True)
                else:
                    time.sleep(1.0)
                last_t = time.monotonic()  # avoid a bogus rate over the gap
                if now >= deadline:
                    stop_reason = f"time cap {args.max_minutes:.0f} min"
                    break
                continue
            dt = max(now - last_t, 1e-6)
            rate = (motor_t - last_temp) / dt * 60.0  # C/min (negative while cooling)
            amb = args.ambient if args.ambient is not None else fet_t
            sample = {"t_s": round(elapsed, 1), "motor_c": round(motor_t, 2),
                      "fet_c": round(fet_t, 2), "rate_c_min": round(rate, 2)}
            if args.ring_gpio is not None:
                sample["ring_c"] = round(ring_t, 2) if ring_t is not None else None
            report["samples"].append(sample)
            if jsonl_f:
                jsonl_f.write(json.dumps(sample) + "\n")
                jsonl_f.flush()
            ring_s = ""
            if args.ring_gpio is not None:
                ring_s = (f"  RING={ring_t:5.1f}C (m-r={motor_t - ring_t:+4.1f})"
                          if ring_t is not None else "  RING=OPEN")
            print(f"  t={elapsed:6.0f}s  MOTOR={motor_t:5.1f}C  FET={fet_t:5.1f}C{ring_s}  "
                  f"rate={rate:+5.2f}C/min  (motor-amb={motor_t - amb:+4.1f})", flush=True)
            last_temp, last_t = motor_t, now

            if motor_t <= amb + args.stop_above_ambient:
                stop_reason = f"within {args.stop_above_ambient:.1f}C of ambient"
                break
            # Settle: 4 consecutive samples cooling slower than 0.3 C/min.
            slow = slow + 1 if -rate < 0.3 else 0
            if slow >= 4:
                stop_reason = "cool-down settled (rate < 0.3 C/min)"
                break
            if now >= deadline:
                stop_reason = f"time cap {args.max_minutes:.0f} min"
                break
            time.sleep(args.interval)
    finally:
        # Final reads may hit a drop too; fall back to the last logged sample.
        try:
            report["motor_temp_end_c"] = float(mt.temperature)
            report["fet_temp_end_c"] = float(ft.temperature)
        except Exception:
            last = report["samples"][-1] if report["samples"] else {}
            report["motor_temp_end_c"] = last.get("motor_c", t_start_motor)
            report["fet_temp_end_c"] = last.get("fet_c", t_start_fet)
        report["reconnects"] = reconnects
        report["elapsed_s"] = round(time.monotonic() - t0, 1)
        report["stop_reason"] = stop_reason
        if jsonl_f:
            jsonl_f.close()

    # Fit tau from the exponential decay: ln(T - T_amb) vs t is a line of slope
    # -1/tau. Use the FET-at-end as ambient proxy unless overridden.
    samples = report["samples"]
    amb = args.ambient if args.ambient is not None else report["fet_temp_end_c"]
    xs, ys = [], []
    for s in samples:
        d = s["motor_c"] - amb
        if d > 1.0:  # keep well above ambient for a clean log fit
            xs.append(s["t_s"])
            ys.append(math.log(d))
    tau = None
    if len(xs) >= 3:
        n = len(xs)
        mx = sum(xs) / n
        my = sum(ys) / n
        sxx = sum((x - mx) ** 2 for x in xs)
        sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
        if sxx > 0 and sxy < 0:
            slope = sxy / sxx
            tau = -1.0 / slope
    report["ambient_used_c"] = round(amb, 2)
    report["tau_s"] = round(tau, 1) if tau else None

    print("\n=== SUMMARY ===", flush=True)
    print(f"  stop reason : {stop_reason}", flush=True)
    print(f"  MOTOR       : {t_start_motor:.1f}C -> {report['motor_temp_end_c']:.1f}C "
          f"in {report['elapsed_s'] / 60:.1f} min", flush=True)
    print(f"  ambient used: {amb:.1f}C  (FET end {report['fet_temp_end_c']:.1f}C)", flush=True)
    if tau:
        print(f"  time const  : tau ~= {tau:.0f}s ({tau / 60:.1f} min)", flush=True)
        print(f"                (tau = R_th * C_total; combine with a known R_th or C "
              f"to get the other)", flush=True)
    else:
        print("  time const  : not enough range above ambient to fit", flush=True)

    if json_path:
        os.makedirs(os.path.dirname(os.path.abspath(json_path)), exist_ok=True)
        with open(json_path, "w", encoding="utf-8") as f:
            json.dump(report, f, indent=2)
            f.write("\n")
        print(f"  report -> {json_path}", flush=True)
        if jsonl_path:
            print(f"  samples -> {jsonl_path}", flush=True)


if __name__ == "__main__":
    main()
