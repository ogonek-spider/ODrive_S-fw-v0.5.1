# Joint Stiction And Speed Test

The current physical pose is treated as output position `0°`. The test range
is `0°` to `45°` after the gearbox.

Without a load-side encoder:

```bash
PYTHONUNBUFFERED=1 .venv/bin/python tools/robot_joint_characterization_test.py \
  --motor-feedback \
  --speed-repetitions 6 \
  --output no-load-characterization.json
```

With the load-side encoder connected on `axis1`:

```bash
PYTHONUNBUFFERED=1 .venv/bin/python tools/robot_joint_characterization_test.py \
  --load-feedback \
  --speed-repetitions 6 \
  --output loaded-characterization.json
```

For each speed, one repetition is:

1. `0° → 45°`
2. `45° → 0°`
3. `0° → 45°`

The default six repetitions produce twelve timed forward moves and six timed
reverse moves. The summary reports:

- Mean and worst `0° → 45°` time
- Mean and worst `45° → 0°` time
- Conservative safe time, using the worst move at the fastest passing speed
- Mean/minimum/maximum breakaway current

Detailed measurements are saved in the selected JSON output file.

Automatically find and validate the fastest repeatable speed:

```bash
PYTHONUNBUFFERED=1 .venv/bin/python tools/robot_joint_characterization_test.py \
  --motor-feedback \
  --skip-stiction \
  --auto-tune-speed \
  --speed-repetitions 6 \
  --output auto-tuned-speed.json
```

For each candidate speed, automatic tuning:

1. Runs two short forward/back/forward calibration repetitions.
2. Adjusts forward and reverse braking leads from mean signed endpoint error.
3. Repeats tuning up to four times.
4. Runs the full six-repetition validation after calibration passes.
5. Stops when a higher speed cannot meet the endpoint tolerance reliably.

Defaults search `2, 4, 6, 8, 10, 11, 12 motor turns/s`. Override with
`--speed-candidates`; supplied candidates are tested in ascending order.
The JSON `selected_speed` object contains the highest fully validated speed,
directional braking leads, conservative times, mean times, and peak current.

The default controller uses separate braking leads because forward and reverse
coast differently:

```bash
--forward-stop-lead-degrees 4.0 \
--reverse-stop-lead-degrees 4.0
```

Increase the corresponding lead if that direction overshoots. Decrease it if
that direction consistently stops short. A failed speed prints the exact
repetition, leg, final error, and overshoot that caused the failure.

## Tuned No-Load Maximum Speed

Validated unloaded 1:36 gearbox profile:

- Motor command: `10 turns/s`
- Nominal output speed: `100°/s`
- Forward/reverse braking lead: `6.5°`
- Mean `0° → 45°`: `0.849 s`
- Worst `0° → 45°`: `0.880 s`
- Mean `45° → 0°`: `0.823 s`
- Worst `45° → 0°`: `0.843 s`
- Peak measured phase current: `2.87 A`
- All 12 forward and 6 reverse moves passed within `±2°`

```bash
PYTHONUNBUFFERED=1 .venv/bin/python tools/robot_joint_characterization_test.py \
  --motor-feedback \
  --skip-stiction \
  --speed-candidates 10 \
  --speed-repetitions 6 \
  --forward-stop-lead-degrees 6.5 \
  --reverse-stop-lead-degrees 6.5 \
  --output max-speed-tuned.json
```

`11` and `12 motor turns/s` produced occasional endpoint errors above `2°`.
Therefore `10 motor turns/s` is the current robust no-load maximum. Retune
after adding the leg or switching to load-side position feedback.
