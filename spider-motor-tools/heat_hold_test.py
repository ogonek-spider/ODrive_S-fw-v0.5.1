#!/usr/bin/env python3
"""
Stationary no-load heater for an ODrive S motor (radiator / thermistor test).

Instead of spinning a free motor (which draws almost no current and barely
heats), this injects a *fixed* current vector via open-loop lockin. The rotor
snaps to one detent and holds still, so all electrical power dumps into the
windings as I^2 R heating with no load and no rotation. Logs winding
(motor thermistor) and FET temperature vs time so you can watch the thermistor
track and characterise how the radiator dissipates a known, steady power.

Dissipated power is roughly 1.5 * I^2 * R  (three-phase, R = phase resistance).

Safety:
- The firmware motor over-temp guard (motor_thermistor limits) stays active the
  whole run, so it cannot heat past the flashed trip; the loop also stops at a
  target temp well below it.
- On arming, the rotor twitches to a detent with real holding torque, then sits
  still. Make sure nothing is clamped to the shaft and a few degrees of motion
  is safe.
- All config changes are RAM-only and restored on exit; nothing is written to
  flash (no save_configuration), so a power cycle returns the saved state.

NOTE: tools live alongside a local `odrive/` package dir in the firmware repo
that can shadow the installed package. This script strips cwd from sys.path.
"""

import argparse
import json
import math
import os
import sys
import time

# Avoid shadowing the installed `odrive` package with a local ./odrive dir.
sys.path = [p for p in sys.path if p not in ("", ".", os.getcwd())]

import odrive
from odrive.enums import AXIS_STATE_IDLE, AXIS_STATE_LOCKIN_SPIN

try:
    from odrive.libodrive import DeviceLostException
except Exception:  # pragma: no cover - name varies across odrivetool versions
    class DeviceLostException(Exception):
        pass


def is_device_lost(exc):
    """True if the exception looks like a USB disconnect (name varies)."""
    return isinstance(exc, DeviceLostException) or "disconnect" in str(exc).lower()


def ntc_temp_from_adc(voltage, rfix, r0, beta, t0_c=25.0):
    """Convert a divider voltage (3.3V--rfix--pin--NTC--GND) to degrees C via the
    beta model. Returns None if the pin is railed (open/short)."""
    if voltage <= 0.002 or voltage >= 3.298:
        return None
    rntc = rfix * voltage / (3.3 - voltage)
    t0 = t0_c + 273.15
    tk = 1.0 / (1.0 / t0 + (1.0 / beta) * math.log(rntc / r0))
    return tk - 273.15


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--motor-id", help="Motor number/label. Recorded and used to "
                                       "auto-name the JSON/JSONL report.")
    p.add_argument("--serial-number")
    p.add_argument("--axis", type=int, default=0, choices=(0, 1))
    p.add_argument("--current", type=float, default=8.0,
                   help="Lockin holding current (A). Power ~= 1.5*I^2*R. Default 8.")
    p.add_argument("--target-temp", type=float, default=55.0,
                   help="Stop once the MOTOR thermistor reaches this temp (C). "
                        "Default 55 (kept below the flashed trip). Set high (e.g. "
                        "95) to run to plateau/trip instead.")
    p.add_argument("--plateau", action="store_true",
                   help="Also stop when the MOTOR temp flattens (steady state): "
                        "windowed rise rate below --plateau-rate.")
    p.add_argument("--plateau-rate", type=float, default=0.2,
                   help="Plateau threshold (C/min) over --plateau-window. Default 0.2.")
    p.add_argument("--plateau-window", type=float, default=90.0,
                   help="Window (s) for the plateau rate estimate. Default 90.")
    p.add_argument("--plateau-min-elapsed", type=float, default=300.0,
                   help="Ignore plateau before this many seconds (avoid early flat "
                        "spots). Default 300.")
    p.add_argument("--max-minutes", type=float, default=20.0,
                   help="Hard time cap (minutes). Default 20.")
    p.add_argument("--watchdog", type=float, default=3.0,
                   help="Axis watchdog timeout (s), fed each loop. If the host/USB "
                        "drops, the motor auto-idles after this. Default 3. Set 0 to "
                        "disable (NOT recommended).")
    p.add_argument("--reconnects", type=int, default=20,
                   help="Max USB reconnect-and-resume attempts on device drops. "
                        "Default 20.")
    p.add_argument("--interval", type=float, default=2.0,
                   help="Sample/log interval (seconds). Default 2.")
    p.add_argument("--json", help="Summary JSON path (default reports/motor-<id>-heat-hold.json).")
    p.add_argument("--jsonl", help="Per-sample JSONL log path (default alongside the JSON).")
    # Optional extra thermistor read via raw ADC (e.g. a radiator sensor).
    p.add_argument("--ring-gpio", type=int, default=None,
                   help="GPIO number of an extra NTC (raw ADC read, no firmware "
                        "config). Logged as an extra temp column, e.g. 3.")
    p.add_argument("--ring-rfix", type=float, default=5000.0,
                   help="Fixed divider resistor for --ring-gpio (ohm). Default 5000.")
    p.add_argument("--ring-r0", type=float, default=10000.0,
                   help="NTC nominal resistance at 25C for --ring-gpio. Default 10000.")
    p.add_argument("--ring-beta", type=float, default=3950.0,
                   help="NTC beta for --ring-gpio. Default 3950.")
    return p.parse_args()


def err_tuple(axis):
    return (int(axis.error), int(axis.motor.error),
            int(axis.encoder.error), int(axis.controller.error))


def clear_errors(axis):
    axis.error = 0
    axis.motor.error = 0
    axis.encoder.error = 0
    axis.controller.error = 0


# axis.error bits we tolerate: ENCODER_FAILED (0x100). The open-loop lockin
# heater does not use the encoder, but a magnetic SPI encoder on flying leads
# picks up PWM EMI and trips ABS_SPI_COM_FAIL, which propagates to axis.error
# and drops us out of lockin. Auto-recover from that so a noisy encoder cannot
# abort the heat-up; any motor/controller fault (incl. an over-temp trip) is
# NOT tolerated and still stops the run.
AXIS_ERROR_ENCODER_FAILED = 0x100


def is_encoder_only_fault(axis):
    a, mo, en, co = err_tuple(axis)
    return (mo == 0 and co == 0 and en != 0
            and (a & ~AXIS_ERROR_ENCODER_FAILED) == 0)


def main():
    args = parse_args()
    print("Connecting to ODrive...", flush=True)
    dev = odrive.find_any(serial_number=args.serial_number, timeout=20)
    axis = getattr(dev, f"axis{args.axis}")
    m = axis.motor
    lk = axis.config.general_lockin
    mt = axis.motor_thermistor
    ft = axis.fet_thermistor

    if not mt.config.enabled:
        print("REFUSING TO RUN: motor thermistor is NOT enabled - no winding "
              "over-temp guard. Enable it before a heat test.", flush=True)
        sys.exit(2)
    if m.config.direction == 0:
        print("REFUSING TO RUN: motor.config.direction == 0; lockin is rejected. "
              "Run offset calibration or set direction=+/-1 first.", flush=True)
        sys.exit(2)
    if args.current > m.config.current_lim:
        print(f"REFUSING TO RUN: --current {args.current} > current_lim "
              f"{m.config.current_lim}.", flush=True)
        sys.exit(2)

    R = float(m.config.phase_resistance)
    est_watts = 1.5 * args.current * args.current * R
    fw = getattr(odrive, "__version__", "?")
    t_start_motor = float(mt.temperature)
    t_start_fet = float(ft.temperature)
    report = {
        "test": "stationary lockin heater (no load, no rotation)",
        "motor_id": args.motor_id,
        "serial_number": format(dev.serial_number, "x"),
        "fw_version": fw,
        "axis": args.axis,
        "vbus_v": float(dev.vbus_voltage),
        "params": {
            "current_a": args.current,
            "est_watts": round(est_watts, 1),
            "target_temp_c": args.target_temp,
            "max_minutes": args.max_minutes,
            "interval_s": args.interval,
        },
        "stored": {
            "phase_resistance_ohm": R,
            "pole_pairs": m.config.pole_pairs,
            "torque_constant": m.config.torque_constant,
            "current_lim": m.config.current_lim,
            "motor_temp_limit_lower": mt.config.temp_limit_lower,
            "motor_temp_limit_upper": mt.config.temp_limit_upper,
            "fet_temp_limit_lower": ft.config.temp_limit_lower,
            "fet_temp_limit_upper": ft.config.temp_limit_upper,
        },
        "motor_temp_start_c": t_start_motor,
        "fet_temp_start_c": t_start_fet,
        "pre_errors": err_tuple(axis),
        "samples": [],
    }

    if args.motor_id is not None:
        print(f"MOTOR #{args.motor_id}", flush=True)
    print(f"serial={report['serial_number']} fw={fw} vbus={report['vbus_v']:.1f}V", flush=True)
    print(f"start: MOTOR={t_start_motor:.1f}C  FET={t_start_fet:.1f}C  R={R:.4f} ohm", flush=True)
    print(f"HOLD {args.current:.1f}A stationary (~{est_watts:.0f}W), stop at "
          f"MOTOR>={args.target_temp:.0f}C or {args.max_minutes:.0f} min "
          f"(firmware trips at {mt.config.temp_limit_lower:.0f}/{mt.config.temp_limit_upper:.0f}C)",
          flush=True)

    # Save the RAM config we touch; restored in finally.
    saved = {
        "current": lk.current, "ramp_time": lk.ramp_time,
        "ramp_distance": lk.ramp_distance, "accel": lk.accel, "vel": lk.vel,
        "finish_distance": lk.finish_distance,
        "finish_on_vel": lk.finish_on_vel,
        "finish_on_distance": lk.finish_on_distance,
        "finish_on_enc_idx": lk.finish_on_enc_idx,
        "enable_watchdog": axis.config.enable_watchdog,
        "watchdog_timeout": axis.config.watchdog_timeout,
    }

    jsonl_f = None
    out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "reports")
    json_path = args.json
    jsonl_path = args.jsonl
    if args.motor_id is not None:
        if json_path is None:
            json_path = os.path.join(out_dir, f"motor-{args.motor_id}-heat-hold.json")
        if jsonl_path is None:
            jsonl_path = os.path.join(out_dir, f"motor-{args.motor_id}-heat-hold.jsonl")
    if jsonl_path:
        os.makedirs(os.path.dirname(os.path.abspath(jsonl_path)), exist_ok=True)
        jsonl_f = open(jsonl_path, "w", encoding="utf-8")

    def apply_lockin_config():
        # Stationary lockin: ramp current up, then hold at vel=0 with no finish
        # condition so it dwells indefinitely as a heater.
        lk.current = args.current
        lk.ramp_time = 0.5
        lk.ramp_distance = 3.1415927
        lk.accel = 0.0
        lk.vel = 0.0
        lk.finish_distance = 0.0
        lk.finish_on_vel = False
        lk.finish_on_distance = False
        lk.finish_on_enc_idx = False

    def enable_watchdog():
        # Feed first, then enable, so the timer starts fresh (avoids an instant
        # trip). If the host/USB dies, the motor auto-idles after the timeout.
        if args.watchdog > 0:
            axis.config.watchdog_timeout = args.watchdog
            axis.watchdog_feed()
            axis.config.enable_watchdog = True

    def arm_lockin():
        clear_errors(axis)
        if args.watchdog > 0:
            axis.watchdog_feed()
        axis.requested_state = AXIS_STATE_LOCKIN_SPIN
        time.sleep(0.5)
        return axis.current_state == AXIS_STATE_LOCKIN_SPIN

    def reconnect_and_rearm():
        # Re-establish the USB link after a device drop and resume heating.
        # The board keeps its RAM config across these drops, but the watchdog
        # will have idled the motor, so we must re-arm. Returns True on success.
        nonlocal dev, axis, m, lk, mt, ft
        try:
            dev = odrive.find_any(serial_number=args.serial_number, timeout=25)
        except Exception as e:
            print(f"    reconnect: find_any failed: {e}", flush=True)
            return False
        axis = getattr(dev, f"axis{args.axis}")
        m = axis.motor
        lk = axis.config.general_lockin
        mt = axis.motor_thermistor
        ft = axis.fet_thermistor
        apply_lockin_config()
        enable_watchdog()
        return arm_lockin()

    stop_reason = "unknown"
    recoveries = 0
    reconnects = 0
    t0 = time.monotonic()
    try:
        clear_errors(axis)
        apply_lockin_config()
        enable_watchdog()

        if not arm_lockin():
            stop_reason = f"lockin entry failed state={axis.current_state} errs={err_tuple(axis)}"
            print("FAIL:", stop_reason, flush=True)
            return

        deadline = t0 + args.max_minutes * 60.0
        last_temp = t_start_motor
        last_t = t0
        recent = []  # rolling window to debounce the noisy thermistor
        hist = []    # (t, temp) history for windowed plateau rate

        def do_iteration():
            # One sample+control step. Returns a stop-reason string to end the
            # run, or None to keep going. Raises on a USB device-lost so the
            # driver loop can reconnect and resume.
            nonlocal last_temp, last_t, recent, hist, recoveries
            now = time.monotonic()
            elapsed = now - t0
            motor_t = float(mt.temperature)
            fet_t = float(ft.temperature)
            iq = float(m.current_control.Iq_measured)
            idm = float(m.current_control.Id_measured)
            errs = err_tuple(axis)
            dt = max(now - last_t, 1e-6)
            rate = (motor_t - last_temp) / dt * 60.0
            ring_t = None
            if args.ring_gpio is not None:
                # Median of 3 quick ADC reads: the raw NTC pin picks up motor
                # PWM EMI and glitches by several degrees on lone samples.
                vs = sorted(float(dev.get_adc_voltage(args.ring_gpio)) for _ in range(3))
                ring_t = ntc_temp_from_adc(vs[1], args.ring_rfix, args.ring_r0,
                                           args.ring_beta)
            sample = {"t_s": round(elapsed, 1), "motor_c": round(motor_t, 2),
                      "fet_c": round(fet_t, 2), "iq_a": round(iq, 2),
                      "id_a": round(idm, 2), "rate_c_min": round(rate, 2),
                      "errors": errs}
            if args.ring_gpio is not None:
                sample["ring_c"] = round(ring_t, 2) if ring_t is not None else None
            report["samples"].append(sample)
            if jsonl_f:
                jsonl_f.write(json.dumps(sample) + "\n")
                jsonl_f.flush()
            imag = (iq * iq + idm * idm) ** 0.5
            ring_s = ""
            if args.ring_gpio is not None:
                ring_s = (f"  RING={ring_t:5.1f}C" if ring_t is not None
                          else "  RING=OPEN")
                if ring_t is not None:
                    ring_s += f" (m-r={motor_t - ring_t:+4.1f})"
            print(f"  t={elapsed:6.0f}s  MOTOR={motor_t:5.1f}C ({motor_t - t_start_motor:+4.1f})  "
                  f"FET={fet_t:5.1f}C{ring_s}  |I|={imag:4.1f}A  rate={rate:+5.2f}C/min", flush=True)
            last_temp, last_t = motor_t, now

            if any(errs):
                # Tolerate a pure encoder EMI trip: clear and re-arm the heater.
                if is_encoder_only_fault(axis):
                    recoveries += 1
                    print(f"    encoder EMI fault {errs} - clearing & re-arming "
                          f"(recovery #{recoveries})", flush=True)
                    if not arm_lockin():
                        return f"re-arm after encoder fault failed errs={err_tuple(axis)}"
                    last_t = time.monotonic()
                    return None
                return f"axis fault errs={errs}"
            if axis.current_state != AXIS_STATE_LOCKIN_SPIN:
                return f"left lockin (state={axis.current_state}) - likely over-temp trip"
            # Debounce: motor thermistor glitches by a few degrees on single
            # samples, so trigger only when the MEDIAN of the last 3 crosses.
            recent.append(motor_t)
            recent = recent[-3:]
            if len(recent) == 3 and sorted(recent)[1] >= args.target_temp:
                return f"reached target {args.target_temp:.0f}C (median of last 3)"
            # Plateau: windowed rise rate has flattened (approaching steady state).
            if args.plateau:
                hist.append((now, motor_t))
                hist = [(ht, hv) for (ht, hv) in hist if now - ht <= args.plateau_window]
                if (elapsed >= args.plateau_min_elapsed and len(hist) >= 2
                        and now - hist[0][0] >= args.plateau_window * 0.8):
                    win_rate = (motor_t - hist[0][1]) / (now - hist[0][0]) * 60.0
                    if abs(win_rate) < args.plateau_rate:
                        return (f"plateau: {win_rate:+.2f}C/min over "
                                f"{now - hist[0][0]:.0f}s at {motor_t:.1f}C")
            if now >= deadline:
                return f"time cap {args.max_minutes:.0f} min"
            return None

        # Driver loop: feed the watchdog, run one iteration, and on a USB
        # device-lost reconnect & re-arm (the watchdog idles the motor during
        # the gap, so this can never run away). Any other stop ends the run.
        while True:
            try:
                if args.watchdog > 0:
                    axis.watchdog_feed()
                sr = do_iteration()
            except Exception as e:
                if not is_device_lost(e):
                    raise
                reconnects += 1
                print(f"    device lost (USB drop) - watchdog idles the motor; "
                      f"reconnecting (attempt {reconnects}/{args.reconnects})", flush=True)
                if reconnects > args.reconnects:
                    stop_reason = f"gave up after {reconnects} USB reconnects"
                    break
                if reconnect_and_rearm():
                    print("    reconnected & re-armed", flush=True)
                else:
                    print("    reconnect not ready; will retry", flush=True)
                    time.sleep(1.0)
                last_t = time.monotonic()  # avoid a bogus rate over the gap
                continue
            if sr is not None:
                stop_reason = sr
                break
            time.sleep(args.interval)
    finally:
        # Best-effort safe shutdown + config restore. If the board is gone,
        # reconnect once so we still idle it and undo our RAM changes; the
        # firmware watchdog has already idled it in the meantime.
        try:
            axis.requested_state = AXIS_STATE_IDLE
            time.sleep(0.3)
        except Exception as e:
            if is_device_lost(e):
                print("    (device lost at shutdown - reconnecting to idle+restore)",
                      flush=True)
                try:
                    reconnect_and_rearm()  # re-establishes handles
                    axis.requested_state = AXIS_STATE_IDLE
                    time.sleep(0.3)
                except Exception:
                    pass
        try:
            lk.current = saved["current"]
            lk.ramp_time = saved["ramp_time"]
            lk.ramp_distance = saved["ramp_distance"]
            lk.accel = saved["accel"]
            lk.vel = saved["vel"]
            lk.finish_distance = saved["finish_distance"]
            lk.finish_on_vel = saved["finish_on_vel"]
            lk.finish_on_distance = saved["finish_on_distance"]
            lk.finish_on_enc_idx = saved["finish_on_enc_idx"]
            axis.config.watchdog_timeout = saved["watchdog_timeout"]
            axis.config.enable_watchdog = saved["enable_watchdog"]
            report["motor_temp_end_c"] = float(mt.temperature)
            report["fet_temp_end_c"] = float(ft.temperature)
            report["post_errors"] = err_tuple(axis)
        except Exception as e:
            print(f"    (config restore incomplete: {e})", flush=True)
            report.setdefault("motor_temp_end_c", t_start_motor)
            report.setdefault("fet_temp_end_c", t_start_fet)
            report.setdefault("post_errors", (None, None, None, None))
        report["elapsed_s"] = round(time.monotonic() - t0, 1)
        report["stop_reason"] = stop_reason
        report["encoder_emi_recoveries"] = recoveries
        report["usb_reconnects"] = reconnects
        if jsonl_f:
            jsonl_f.close()

    samples = report["samples"]
    dT = report["motor_temp_end_c"] - t_start_motor
    mins = report["elapsed_s"] / 60.0
    avg_rate = dT / mins if mins > 0 else 0.0
    report["summary"] = {
        "delta_t_c": round(dT, 2),
        "avg_rate_c_min": round(avg_rate, 3),
        "peak_rate_c_min": round(max((s["rate_c_min"] for s in samples[1:]), default=0.0), 2),
        "fet_delta_c": round(report["fet_temp_end_c"] - t_start_fet, 2),
        "est_watts": round(est_watts, 1),
    }
    print("\n=== SUMMARY ===", flush=True)
    print(f"  stop reason     : {stop_reason}", flush=True)
    print(f"  MOTOR temp      : {t_start_motor:.1f}C -> {report['motor_temp_end_c']:.1f}C "
          f"(+{dT:.1f} in {mins:.1f} min)", flush=True)
    print(f"  FET temp        : {t_start_fet:.1f}C -> {report['fet_temp_end_c']:.1f}C", flush=True)
    print(f"  avg rise rate   : {avg_rate:.2f} C/min  (peak {report['summary']['peak_rate_c_min']:.2f})",
          flush=True)
    print(f"  est power       : {est_watts:.0f} W at {args.current:.1f} A", flush=True)
    print(f"  encoder recoveries: {recoveries}", flush=True)
    print(f"  USB reconnects  : {reconnects}", flush=True)
    print("  (config changes were RAM-only; nothing saved to flash)", flush=True)

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
