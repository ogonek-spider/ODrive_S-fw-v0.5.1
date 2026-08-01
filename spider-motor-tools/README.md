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

### `robot_joint_setup.py` — mount a bench motor onto the robot (one command)
Runs the whole flash → CAN → joint-encoder sequence for one board, in the order
that has to be respected. **Never arms the motor**, so it is safe on a mounted
leg.

```bash
.venv/bin/python spider-motor-tools/robot_joint_setup.py --motor 13 --position 6-3
```

`--position <leg>-<joint>` (leg 1..6; joint 1=coxa, 2=femur, 3=knee) gives
`can_node_id = leg*10 + joint` — see [CAN_NODE_ID_MAP.md](CAN_NODE_ID_MAP.md).

Nine stages: identify → **back up config** → flash `build/ODriveFirmware.elf` →
**restore + verify field-by-field** → CAN node id + mute `axis1`'s heartbeat →
`axis1` MT6701 config + health triage → **split feedback + predefined position
gains** → save → reboot and re-verify from NVM.

The order is not arbitrary. The flash wipes NVM whenever `config_version`
changed, so the backup has to precede it; and CAN + the joint encoder are set
*after* the restore because the backup JSON carries the OLD `can_node_id` and a
blank `axis1` — restoring last would silently undo both.

It refuses to save a faulted joint encoder (leaves `axis1` at mode 0) and tells
you which wire to check, using `axis0`'s AS5047P on the shared SCK/MISO as the
control that separates a bad encoder from a bad board.

**Predefined gains (`JOINT_TUNING` in the script).** Stage 7 writes split
feedback (`load_encoder_axis=1`, `vel_encoder_axis=0`) plus the position-loop
gains for that joint position, then reads them back and re-verifies them from
NVM after the reboot. With `load_encoder_axis=1` the error is in **output**
turns, so `pos_gain` is a joint-side gain and **scales with the gearbox ratio**
(≈ 23 × ratio) — which is why the table is keyed by joint and a femur value must
never be copied onto a knee:

| joint | ratio | `pos_gain` | status |
|:------|:-----:|:----------:|:-------|
| 1 coxa  | 6:1   | 140  | derived (23 × 6); leg1 knee runs 130 on a ~6:1 box |
| 2 femur | 108:1 | 2500 | **step-tested** on leg6 femur, node 62, 2026-08-01 |
| 3 knee  | 6:1   | 140  | derived; leg1's knee ratio varies 5.6–7.4:1 with angle |

Stage 7 is skipped when the joint encoder is missing or faulted — pointing the
position loop at a dead `axis1` would close the loop on garbage the moment
someone arms it. `position_direction` defaults to `+1` (every joint measured so
far), but that is an assumption about how the leg was assembled: **verify it
with a small guarded step before arming**, a wrong sign is a runaway.

Useful flags: `--skip-flash` (re-run after fixing encoder wiring),
`--force-flash`, `--no-mt6701` (joint encoder not fitted yet),
`--allow-bad-encoder`, `--dry-run`, `--serial-number SN`, `--encoder-seconds`,
`--no-gains`, `--pos-gain`, `--gearbox-ratio` (derives `pos_gain = 23 × ratio`),
`--position-direction {1,-1}`.

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

### `joint_hand_sweep.py` — ratio, sign and travel from a hand sweep (USB)
Axis held IDLE; move the joint by hand end to end while both encoders are read.
Reports at **zero current** whether the motor is actually coupled to the output,
the real gearbox ratio, whether that ratio is constant over the travel, the
sign, and the mechanical travel range.

```bash
.venv/bin/python3 spider-motor-tools/joint_hand_sweep.py --position 6-3
```

Run this **before** adding amps. A motor-driven ratio probe on a stiff joint
returns breakaway and backlash garbage, and a decoupled or seized joint reads
as a tuning problem until you turn it by hand. Read-only: nothing is written.

### `joint_step_tune.py` — guarded step tuning of `pos_gain` (USB)
Configures split feedback, establishes `position_direction` with a small step
before anything larger is commanded, then sweeps `pos_gain` candidates and
reports settle time, residual, overshoot and peak current.

```bash
.venv/bin/python3 spider-motor-tools/joint_step_tune.py --position 6-3
.venv/bin/python3 spider-motor-tools/joint_step_tune.py --position 6-3 --save
```

`pos_gain` is in **joint-side units** and scales with the gearbox ratio
(≈ 23 × ratio), so a high-ratio joint needs hundreds and a soft gain will not
move it at all. The sign check therefore escalates a `--sign-gains` ladder
rather than concluding "stuck" from one soft attempt, and when nothing moves it
uses the **motor** trace to separate the three causes: motor turns but joint
does not → decoupled; neither turns at near the current limit → seized; neither
turns at low current → gain too soft.

Safety: never arms without clearing errors first, seeds the setpoint from the
measured position, aborts the instant the joint moves the wrong way, on any
axis error, past `--abort-deg`, and on `Ctrl+C` — always leaving the axis IDLE.
Sampling is deliberately ~12 USB reads/s; **measured on this hardware, 18
reads/s trips `CONTROL_DEADLINE_MISSED`** in closed loop.

### `can_jog.py` — interactive CNC-style jog pendant + limit capture (CAN)
Pick a node off the bus, jog it with the arrow keys in fixed steps, and capture
`min` / `max` at the poses you jogged to.

```bash
.venv/bin/python3 spider-motor-tools/can_jog.py              # pick from the bus
.venv/bin/python3 spider-motor-tools/can_jog.py --node 12 --step 5 --iq-cap 8
```

Keys: `←/↓` and `→/↑` jog ∓step · `+ -` step size · `< >` slew rate · `a` arm ·
`i` idle · `SPACE` emergency idle · `[` set min · `]` set max · `g`/`G` go to
min/max · `t` type a target angle · `w` write limits · `n`/`p` switch node ·
`e` clear errors · `q` quit.

Angles are whatever the node reports over `Get_Encoder_Estimates` — **joint /
output degrees** on a split-feedback geared joint (`load_encoder_axis != axis`),
motor degrees otherwise. Limits go to `configs/jog_limits.json` and clamp this
tool only; on exit it prints the `odrivetool` snippet that turns them into real
firmware endstops (`encoder.config.min_position` / `max_position`).

Safety: never arms by itself, seeds the setpoint from the measured position
before closing the loop, slews instead of stepping, and auto-IDLEs on fault, on
`|Iq|` over the cap, on the setpoint running away from a stalled joint, and
after `--idle-timeout` (default 45 s) with no keypress — holding a geared joint
against gravity is a continuous-current duty that has destroyed a motor here.

### `can_config.py` — read/write joint config over CAN (needs fw >= 0.5.6)
No USB, no replugging. Talks to the `MSG_CONFIG_ACCESS` / `MSG_CONFIG_COMMIT`
firmware addition.

```bash
.venv/bin/python3 spider-motor-tools/can_config.py --node 12          # dump
.venv/bin/python3 spider-motor-tools/can_config.py --node 12 \
    --set min=58.2 max=160 limit_enable=1 --save
.venv/bin/python3 spider-motor-tools/can_config.py --node 13 --set set_zero=1
```

Angles (`min`, `max`, `pos`) are in **degrees** here; the wire format is turns.
Parameters that physically live on the joint encoder (`axis1`) are addressed
through the **motor** axis's node id — the firmware resolves them via
`load_encoder_axis`, because `axis1` has its CAN heartbeat muted on the robot.
Every write echoes the resulting value, so a rejected or clamped write is
visible immediately. To reorder a range you must disable `limit_enable` first:
the firmware refuses to invert a live range, since an inverted range silently
stops enforcing the endstops.

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
