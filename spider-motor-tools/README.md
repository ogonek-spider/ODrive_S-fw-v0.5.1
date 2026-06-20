# spider-motor-tools

Motor health diagnostics and reports for the spider's ODrive S joints.
Each motor is numbered; record results per motor with `--motor-id` (currently
testing **#10**).

## Setup

Use the firmware repo's virtualenv (has the custom `odrive` fw-v0.5.1-mt6701):

```bash
# from the repo root: /Users/alarin/Documents/art/ogonek25-spider/ODrive_S-fw-v0.5.1
.venv/bin/python3 spider-motor-tools/motor_health_check.py --motor-id 10
```

> Run from the **repo root** (not from `tools/`). The firmware repo contains a
> local `tools/odrive/` package dir that shadows the installed `odrive`. These
> scripts strip cwd from `sys.path` to be safe, but running from the root avoids
> surprises.

Safety: every script holds the axis IDLE between steps, restores any RAM config
it changes, and **never calls `save_configuration`** — a power cycle returns the
drive to its flashed state.

## Scripts

### `motor_health_check.py` — automated health battery
Runs and grades three checks, prints PASS/WARN/FAIL, writes a JSON report.

```bash
.venv/bin/python3 spider-motor-tools/motor_health_check.py --motor-id 10
# -> reports/motor-10-health.json
```

| Check | What it measures | Healthy result |
|-------|------------------|----------------|
| Motor calibration x4 | phase resistance / inductance scatter | R & L repeat tightly |
| Encoder offset cal x5 | commutation `offset_float` scatter | spread < 0.05 rad |
| Free-spin sweep | can it reach speed & at what current | reaches commanded speed, low current |

Useful flags: `--speeds 5 10`, `--current-limit 10`, `--offset-runs 5`,
`--motorcal-runs 4`, `--skip-spin`, `--json PATH`, `--serial-number SN`,
`--axis {0,1}`.

Interpreting results:
- **Windings WARN/FAIL** (R or L scatter) → suspect a motor phase / connector.
- **Commutation offset WARN/FAIL** (offset scatter) with a *clean encoder and
  healthy windings* → suspect the encoder magnet's mechanical coupling
  (loose / eccentric magnet on the shaft).
- **Free-spin DID NOT REACH / high current** → with a gearbox attached, suspect
  the gearbox (binding); free motor should reach speed at low current.

### `encoder_hand_test.py` — encoder signal integrity (hand-spin)
Axis held de-energised; spin the shaft by hand through several full turns.

```bash
.venv/bin/python3 spider-motor-tools/encoder_hand_test.py --motor-id 10 --duration 25
```
PASS = `spi_error_rate == 0`, zero glitches, smooth angle. Read-only on config.

## Related firmware-repo tools (not duplicated here)
- `tools/as5047p_hand_test.py` / `tools/mt6701_hand_test.py` — encoder bringup /
  reconfiguration.
- `tools/compare_motor_encoders.py` — Hall vs onboard AS5047P.
- `tools/robot_joint_characterization_test.py` — full stiction + speed sweep,
  produces the `*-characterization.json` baselines (see `tools/howtotest.md`).

## Baselines (repo root)
- `max-speed-tuned.json` — no-load motor-feedback speed reference
  (0.85 s/move @ 10 t/s, 2.87 A peak).
- `joint-characterization.json` — loaded (axis1 MT6701) full characterization.

Both dated 2026-06-09 and **predate** the "encoder and motor direction sign
change" commit, so a freshly recalibrated motor is required before comparing.

## Reports
Per-motor results live in `reports/`:
- `reports/motor-10-health-2026-06-20.md` — first full diagnosis of motor #10.
- `reports/motor-<id>-health.json` — machine-readable output from
  `motor_health_check.py`.
