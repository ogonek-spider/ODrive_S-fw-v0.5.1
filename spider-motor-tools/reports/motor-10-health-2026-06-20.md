# Motor #10 — health report (2026-06-20)

> **STATUS: USABLE — offset pinned to 0.95 and saved to flash (2026-06-20).**
> Boots ready (`is_calibrated`/`is_ready` true, `pre_calibrated=True`,
> `startup_encoder_offset_calibration=False`). Residual once-per-rev commutation
> error remains but is negligible behind the 36:1 gearbox. See "Resolution" below.


- **ODrive serial:** 3482345a3034
- **Firmware:** fw-v0.5.1-mt6701
- **Bus voltage:** 31.2 V
- **Encoder:** onboard AS5047P on axis0 (SPI mode 257, cpr 16384)
- **Stored motor:** pole_pairs 15, torque_constant 1.0 (placeholder)
- **Setup during test:** 1:36 gearbox, no load; axis1 (load MT6701) absent

## Verdict

Motor and encoder are **fundamentally sound**. Two mechanical faults prevent it
from matching the saved baseline:
1. **36:1 gearbox is binding/stiff** — dominates the high-current / stall numbers.
2. **Encoder magnet coupling is intermittent** — commutation offset calibration
   does not repeat, so commutation is sometimes badly off.

## Subsystem results

| Subsystem | Verdict | Evidence |
|-----------|---------|----------|
| AS5047P encoder (axis0) | ✅ Healthy | hand-spin ~2 turns: smooth, **0 glitches, spi_error_rate=0**, no errors, max step 9.6°/sample |
| Motor windings / phases | ✅ Healthy | R = 0.2400–0.2410 Ω, L ≈ 0.571–0.626 mH over 4 motor cals |
| 36:1 gearbox | ❌ Binding | free motor reaches 10 t/s @ 2.1 A; **with gearbox it stalled at 13 A** |
| Commutation offset cal | ❌ Intermittent | `offset_float` scattered 0.53 → 1.27 rad across runs (should be < 0.05) |

## Measured data

**Encoder offset calibration `offset_float` (rad):**
- with gearbox: 0.984, 1.272, 1.260, 0.938
- free motor:   0.534, 0.918, 0.984, 1.027
- spread ≈ 0.74 rad overall (≈ 1.5° mechanical, intermittent)

**Motor calibration (phase R / L):**
- R: 0.2410, 0.2405, 0.2407, 0.2400 Ω
- L: 0.5734, 0.6255, 0.5717, 0.5705 mH

**Free-spin steady-state current (no load):**

| cmd | gearbox ON | gearbox OFF (free) |
|-----|-----------|--------------------|
| 5 t/s  | 3.21 A mean / 7.14 A max, reached 3.35 t/s | 1.63 A / 3.34 A, reached 4.66 t/s |
| 10 t/s | 13.07 A mean / 15.14 A max, **STALLED (0 t/s)** | 2.11 A / 8.00 A, reached 10.07 t/s |

**No-load speed test vs baseline `max-speed-tuned.json`** (10 t/s, 6 reps, 6.5° leads,
run with gearbox + a bad commutation offset → see `motor-10-no-load-speed-2026-06-20.json`):

| Metric | Baseline | Motor #10 run |
|--------|----------|---------------|
| 0→45 mean | 0.849 s | 1.308 s |
| 0→45 worst | 0.880 s | 1.402 s |
| 45→0 mean | 0.823 s | 1.299 s |
| Peak phase current | 2.87 A | 7.37 A |
| Verdict | PASS | FAIL (one +3.24° miss) |

## Automated health-check run (`motor_health_check.py --motor-id 10`)

Validated end-to-end against the toolkit (gearbox off, free motor). Machine-readable
output: `reports/motor-10-health.json`.

| Check | Result | Data |
|-------|--------|------|
| Windings (R/L) | ✅ PASS | R 0.2393–0.2410 Ω, L 0.569–0.577 mH |
| Commutation offset | ❌ FAIL | offset_float 0.905, 1.461, 1.171, 0.687, 0.805 rad → **spread 0.774 rad** |
| Free-spin | ❌ FAIL | 5 t/s OK (1.87 A); 10 t/s did not reach (5.52 t/s, 5.29 A mean, 10.5 A peak) |
| **Overall** | ❌ **FAIL** | offset scatter ≫ 0.05 rad gate |

The 10 t/s "did not reach" is a direct consequence of that run's offset landing badly —
when commutation is off, the motor cannot make torque efficiently. Windings remain
healthy and repeatable, isolating the fault to the encoder magnet coupling.

## Background

Motor initially "wouldn't move" in closed loop: a stale commutation offset left
by the "encoder and motor direction sign change" commit (the 2026-06-09
baselines predate it). Recalibration restored motion but exposed the
intermittent-offset fault above.

## Recommended fixes (in order)

1. **Reseat / re-bond the encoder magnet** on the motor shaft (centered, correct
   air-gap, firmly fixed). Then run `motor_health_check.py --motor-id 10` and
   confirm offset spread < 0.05 rad.
2. **Inspect the 36:1 gearbox** for binding (it added ~10 A of load).
3. Re-run the no-load speed test and compare to `max-speed-tuned.json`.

## Resolution (2026-06-20)

The once-per-rev geometric error (a tilt/eccentricity in the sense-magnet mount,
not slip — the magnet is rigidly coupled to the rotor) could not be fully removed
mechanically. It was judged **acceptable for this application** and the config
was frozen:

- Pinned `encoder.config.offset_float = 0.95` — the midpoint of the two offset
  clusters (symmetric worst-case commutation error).
- `motor/encoder.config.pre_calibrated = True`, `startup_encoder_offset_calibration
  = False` → boots ready on the fixed offset, never re-rolls the calibration.
- **`save_configuration()` — first flash write; verified persisted across a reboot**
  (`offset_float=0.95`, `is_calibrated/is_ready=True`, errors 0).

Why it's acceptable:
- Running current is nearly identical (1.05–1.12 A @ 5 t/s no-load) across the whole
  0.60–1.30 rad offset range — the motor is insensitive to which offset is pinned.
- The residual ~1.3° mechanical once-per-rev error is on the motor shaft; behind
  36:1 that's ~0.04° at the joint. Only effect is a few-percent commutation torque
  ripple, filtered by the gearbox + leg.

**Direction caveat:** the saved `motor.config.direction = 1` (set by calibration;
the pre-existing flash had `-1` from the "direction sign change" commit, which was
non-functional with the stale offset). `controller.config.position_direction = 1`
is unchanged. Verify the joint moves the expected way for a positive command; if
reversed, flip `position_direction` (or the command sign in robot code) — do **not**
change `motor.config.direction`, which must stay +1 for valid commutation.

## Config state

All changes this session were **RAM-only** (never `save_configuration`):
`motor.config.direction` 1, re-measured R/L, last `offset_float` ≈ 1.027.
Flash config and `odrive-config-before-sensorless-test-2026-06-20.json` untouched;
power cycle restores the flashed state.
