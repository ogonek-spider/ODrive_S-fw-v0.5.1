# Per-Motor Setup Runbook (Claude)

Repeatable procedure to bring up one bench motor of the standard spider-joint
type: **hoverboard-style, 15 pole-pairs, onboard AS5047P SPI-abs encoder on
`axis0` (mode 257, cpr 16384, CS pin 7), motor thermistor on GPIO4**. Written to
run the same way for every unit (target: ~20 motors) so results are comparable.

Run everything from the repo root with the repo venv:
`/Users/alarin/Documents/art/ogonek25-spider/ODrive_S-fw-v0.5.1/.venv/bin/python`.
No `timeout` on this macOS bench — use background runs + Monitor for long ones.

## What is per-motor vs constant

- **Per motor (MUST measure each unit):** phase R, phase L, encoder commutation
  offset, and **Kt** — Kt varies a lot across nominally-identical rotors
  (measured 0.16–0.27 Nm/A across bench units). Never copy Kt from another motor.
- **Constant on this board:** `motor_type=HIGH_CURRENT`, `pole_pairs=15`,
  encoder mode 257 / cpr 16384 / CS 7, GPIO4 thermistor + its coeffs
  (`temp_limit 70/90`). These carry across motor swaps on board `3482345a3034`.

## Step 0 — Firmware (usually SKIP)

Only flash if the board is NOT already on the current patched build. Check for
the local patches; if all present and `Firmware/` is unchanged since
`build/ODriveFirmware.elf`, **do not re-flash** — it only wipes NVM config.

```python
enc = odrv.axis0.encoder
all(hasattr(enc, a) for a in ("mt6701_debug_raw24",)) \
  and hasattr(enc.config, "zero_offset") and hasattr(enc.config, "direction") \
  and hasattr(odrv.axis0.controller.config, "vel_encoder_axis")
```

If a flash IS needed, follow AGENTS.md "Flashing Notes" (back up config first).

## Step 1 — Connect + back up + confirm starting state

```bash
.venv/bin/odrivetool backup-config spider-motor-tools/configs/motor<N>-before-setup-<DATE>.json
```

Read `axis0` errors, encoder mode/cpr/cs, pole_pairs. Expect stale R/L/offset/Kt
from the previous rotor — that's fine, they get overwritten.

## Step 2 — SAFETY GATE (ask the user, wait)

The next steps spin the bare motor to ~10–16 t/s. Per bench rule, **warn + ask
"free to spin? готов?" and wait** for confirmation before any motion. Shaft must
be clear, no load, nothing to be thrown or catch.

## Step 3 — Health check (RAM-only, non-destructive)

```bash
PYTHONUNBUFFERED=1 .venv/bin/python spider-motor-tools/motor_health_check.py \
  --motor-id <N> --json spider-motor-tools/reports/motor-<N>-health-<DATE>.json
```

Expect **OVERALL PASS**:
- Windings R/L tight scatter (bench units ~0.24–0.27 Ω, ~0.6 mH).
- Commutation offset spread small — good rotors ~0.09 rad; **>0.25 rad = FAIL**
  (loose/eccentric encoder magnet — re-bond before proceeding).
- Free-spin reaches commanded speed at low current (healthy ~0.5–1.5 A no-load).

If it does not pass, STOP and diagnose (magnet bond, phase clamps, encoder wiring)
before pinning anything.

## Step 4 — Thermistor basic check

Confirm the GPIO4 NTC reads a sane, stable temperature (not open-circuit ~−63 °C):

```bash
.venv/bin/python - <<'PY'
import odrive, statistics, time
ax = odrive.find_any(timeout=20).axis0
mt, ft = ax.motor_thermistor, ax.fet_thermistor
xs = [ (mt.temperature, ft.temperature) for _ in range(60) if not time.sleep(0.05) ]
m = [a for a,_ in xs]
print("motor therm mean=%.2f sd=%.3f  fet mean=%.2f  enabled=%s lim=%.0f/%.0f" % (
    statistics.fmean(m), statistics.pstdev(m),
    statistics.fmean([b for _,b in xs]), mt.config.enabled,
    mt.config.temp_limit_lower, mt.config.temp_limit_upper))
print("VERDICT:", "OK" if (-5 < statistics.fmean(m) < 60 and max(m)-min(m) < 5) else "CHECK")
PY
```

Sane at cold ambient: motor ~26–30 °C, dead stable, a few °C below the FET.
Raw ADC `odrv.get_adc_voltage(4)` should be mid-scale (~1.7 V), not rail-pinned.
If this is a **different board**, thermistor coeffs are resistor-specific —
recompute per the "Motor #1 thermistor 5k divider" note, don't trust carried coeffs.

## Step 5 — Measure Kt (back-EMF, RAM-only)

```bash
PYTHONUNBUFFERED=1 .venv/bin/python spider-motor-tools/measure_kt.py \
  --motor-id <N> --json spider-motor-tools/reports/motor-<N>-kt-<DATE>.json
```

Expect a clean linear fit with intercept ~0. Record `Kt` (Nm/A) and `Kv`.

## Step 6 — Pin calibration to flash (ONLY flash-writing step)

```bash
PYTHONUNBUFFERED=1 .venv/bin/python spider-motor-tools/pin_calibration.py \
  --motor-id <N> --kt <KT_FROM_STEP_5> --check-thermistor
```

This re-runs motor + encoder-offset cal, writes `torque_constant`, sets
`pre_calibrated` on motor + encoder, saves, then reconnects and prints the
persisted config. Confirm `user_config_loaded=True`, `is_calibrated=True`,
errors all 0. (Add `--dry-run` to rehearse without saving.)

## Step 7 — Post-save sanity

Reconnect, verify it boots straight into closed loop, do a short spin (e.g.
6 t/s), confirm zero errors. Optional deeper tests: `thermal_rise_test.py`,
`heat_hold_test.py`, `anticogging_calibration.py`.

## Step 8 — Optional: encoder harmonic (eccentricity) compensation

Only after Step 6 (motor must be pinned/commutating). Measures the encoder's
1st/2nd per-revolution error and, if the fit is good, subtracts it from the
position estimate + electrical phase. Defaults **off** in NVM; enable dead-last.
See AGENTS.md "Encoder Harmonic (Eccentricity) Compensation" for the firmware
side (config-gated, applied correction clamped to cpr/64).

**Always dry-run first and read the fit — do not blind-`--save`:**

```bash
# axis0 on-shaft AS5047P (ratio 1). Spins the motor at 12 t/s -> re-confirm the
# SAFETY GATE (Step 2) still holds before running.
PYTHONUNBUFFERED=1 .venv/bin/python spider-motor-tools/harmonic_calibration.py \
  --motor-id <N> --drive-axis 0
```

Judge the report, not just the trust gate (the gate is a floor):
- **Save-worthy:** 1st/2nd amplitudes repeatable across passes (spread well under
  25%), and the correction visibly cuts **error pk-pk**, not just RMS.
- **Leave OFF (common on a bare bench motor):** if error **pk-pk barely moves**,
  the encoder error is mostly **non-harmonic** and/or the fit is contaminated by
  cogging velocity ripple (no gearbox/load inertia to filter it at 12 t/s).
  Harmonic comp then buys almost nothing.

If save-worthy, apply + flash, then **power-cycle and re-verify a clean boot +
low-current arm** before trusting it:

```bash
.venv/bin/python spider-motor-tools/harmonic_calibration.py \
  --motor-id <N> --drive-axis 0 --save   # gated; --force to override the gate
```

Geared joint MT6701 (axis1, after the ~34:1 gearbox): use
`--encoder-axis 1 --ratio 34` and the joint must be free to turn full **output**
revolutions, or the fit is poor.

Reference (motor #6, 2026-07-17, board 367f36793335): dry-run clean but 1st/2nd
only 0.19°/0.26°, RMS 0.54°→0.48°, **pk-pk 5.1°→5.0° (unchanged)** → non-harmonic
error dominates → **left OFF**. Marginal benefit is the expected outcome for the
on-shaft AS5047P on a no-load bench; don't force it.

## Step 9 — Record

- Health + Kt JSON already saved under `spider-motor-tools/reports/`.
- Add a one-line memory entry (`motor<N>-setup-<DATE>`): board serial, R/L,
  offset spread, Kt/Kv, save status, any anomaly. Mirror the motor #8 entry.

## Reference values (motor #8, 2026-07-08, board 3482345a3034)

R=0.257 Ω, L=0.656 mH, offset spread 0.094 rad, offset=8469, **Kt=0.234 Nm/A**,
Kv≈40.7, thermistor 26.9 °C stable. Health + boot-to-closed-loop clean.
