# leg1 COXA (node 11) — commutation recalibration after board swap

Date: 2026-08-03. Board `367F36503335` (ex motor #17 board / ex MT6701 test-stand
rig), now carrying **motor #12** at leg 1 coxa. Motor **decoupled from the
gearbox** and on the bench for this work, so the shaft was free.

## Firmware: 0.5.6 → 0.5.7, flashed over SWD

`odrivetool dfu build/ODriveFirmware.elf` timed out mid `erase_sector` **again**
(second time on this board today), leaving it in ROM DFU with a partially erased
flash; `dfu-util` then failed at `Cannot set alternate interface:
LIBUSB_ERROR_OTHER` — the ROM bootloader was wedged.

**Recovered over ST-Link/SWD, no jumper needed.** Flash was still readable
(vector table intact, SP `0x20020000`) and RDP was Level 0 (`OPTCR 0x0fffaaed`,
RDP byte `0xAA`, left over from this morning's unlock).

```sh
openocd -f interface/stlink.cfg -c "transport select hla_swd" \
        -f target/stm32f4x.cfg -c "adapter speed 1800" \
        -c init -c "reset halt" \
        -c "program build/ODriveFirmware.elf verify" -c "reset run" -c exit
```

`** Verified OK **` (read-back compare — this is the check that catches the
silent-erase failure mode). Version bytes read back from flash at `0x08039A9C`:
`00 05 07` = **0.5.7**. Board booted and enumerated as fw 0.5.7 with
`user_config_loaded=True` — no `config_version` bump between 0.5.6 and 0.5.7, so
NVM survived and **no restore was needed**.

Pre-flash backup kept anyway:
`configs/leg1-coxa-367f36503335-before-0.5.7-flash-2026-08-03.json`.

**Takeaway: on this board, use SWD, not DFU.** Two `odrivetool dfu` attempts,
two wedged bootloaders. openocd `program … verify` took 7 s.

## Commutation calibration

The onboard AS5047P came with the board, so the old board's `offset 8399` and
this board's `offset 11689` (measured against motor #17's rotor) were both
meaningless. #17's harmonic fit was already zeroed and disabled beforehand so it
could not distort the offset fit.

Settings used: `pp=15`, `Kt=0.260` (motor #12's, written from the bench record —
Kt is a motor property), `calibration_current=5.0`,
`resistance_calib_max_voltage=4.0`, `calibration_lockin.current=5.0`.

`MOTOR_CALIBRATION` — all errors 0:

| | measured | motor #12 bench |
|---|---|---|
| phase_resistance | **0.267221 Ω** | 0.2632 Ω |
| phase_inductance | **0.00054884 H** | 0.000512 H |
| direction | 1 | 1 |

`ENCODER_OFFSET_CALIBRATION` ×3, all errors 0:

| pass | offset | offset_float | calib_scan_response |
|---|---|---|---|
| 1 | 8353 | 0.7053 | 8660.0 |
| 2 | 8354 | 0.6679 | 8659.0 |
| 3 | 8354 | 0.8593 | 8662.0 |

**Spread 1 count** over 3 passes = 0.3 electrical degrees of the 1092-count
electrical revolution. `calib_scan_response ≈ 8660` matches the expected
0.53-turn scan (0.53 × 16384 ≈ 8683), so the shaft really did sweep.

Saved: `offset=8354`, `pre_calibrated=True` on both motor and encoder, verified
back from NVM after reconnect.

Curiosity, not a shortcut: the new offset lands 45 counts (≈1° mechanical) from
the dead board's 8399 — the two AS5047Ps happen to sit within a degree of each
other. 45 counts is still 15 electrical degrees, so reusing the old value would
have been wrong; it just would not have been *obviously* wrong.

## Free-shaft closed-loop check (`scratch/coxa_bench_spin_check.py`)

RAM only, nothing saved: `load_encoder_axis` moved to 0 for the test (the
MT6701 is on the leg, so a position loop pointed at it cannot arm),
velocity control, `current_lim` 8 A, restored afterwards.

| target | measured | Iq mean | Iq peak | |
|---|---|---|---|---|
| 0.00 t/s | −0.010 | +0.050 A | 0.178 A | armed, at rest |
| +1.50 t/s | +1.459 | +1.370 A | 1.911 A | tracking error 2.8% |
| 0.00 t/s | −0.002 | +1.264 A | 1.534 A | integrator holding |
| −1.50 t/s | −1.396 | −1.502 A | 1.825 A | tracking error 6.9% |
| 0.00 t/s | +0.004 | −1.236 A | 1.344 A | integrator holding |

Armed first try, all four error registers 0 before and after, tracked both
directions, disarmed clean. **The new offset works under real torque** — this is
what the 1-count fit spread could not prove on its own.

Two things in the numbers are worth keeping:

- **The shaft is not free-running: ~1.3 A of drag** (≈0.33 Nm at Kt 0.26) at
  1.5 t/s, in both directions. A bare motor of this family draws ~0.46 A; ~1.8–2.0 A
  is the figure recorded with a 1:6 gearbox attached. So "decoupled" here looks
  like *decoupled from the leg*, with the gearbox input still on the shaft.
  That is a fine state to have calibrated in — the offset fit is unaffected —
  but it means this run does **not** double as a bare-motor drag measurement, and
  a mechanical rub could hide under it. What argues against a rub: offset spread
  of 1 count and an Iq peak/mean ratio of only 1.4 (a rub shows up as offset
  scatter and lumpy current — that is how motor #4's two rubs were caught).
- **The ±1.26 A at commanded zero is integrator hold, not a fault.** With
  `vel_integrator_gain = 0.8` the integrator has wound up to supply the drag
  torque, and at zero velocity error it has no reason to unwind. Expect the same
  residual current after a move on the robot.

## State left behind

- fw **0.5.7**, `user_config_loaded=True`, all axis0 errors 0.
- axis0: mode 257 / CS 7 / cpr 16384, offset 8354, pre_calibrated both True,
  harmonic compensation **off and zeroed**, Kt 0.260, current_lim 15.
- axis1 (MT6701): mode 261 / CS 6 / `direction=-1` / `zero_offset=1660` intact,
  but reads `ERROR_ABS_SPI_COM_FAIL (0x80)` — **expected**, the joint encoder
  stayed on the leg while the board is on the bench.
- controller: `load_encoder_axis=1`, `vel_encoder_axis=0`, pos_gain 140,
  vel_gain 2.0, vel_i 0.8, vel_limit 2.0.
- CAN: axis0 node 11 @100 ms, axis1 heartbeat muted.

## Still open

1. **`position_direction = 1` is UNVERIFIED and probably wrong.** The 2026-07-26
   on-robot correction found `direction=-1` **and** `position_direction=-1`, and
   recorded that both signs must flip together — flipping only one is a runaway.
   `coxa_joint_frame_and_split.py` wrote `+1` on 08-03 as an explicit
   "unverified" placeholder. Verify with a small guarded step **before** the
   first closed-loop arm after remounting.
2. **No cold-boot check yet.** The board has not been power-cycled since the
   save (deliberately — `reboot()` after a save wedges abs-SPI on dual-abs
   boards; only a power cycle recovers). Confirm a clean boot with both encoders
   erroring 0 once it is back on the joint.
3. Harmonic compensation is off. Re-fitting it needs the free shaft (≥12 motor
   t/s for many revolutions) — still possible while the motor is off the leg,
   but the chosen scope was offset-only.
4. Pre-test controller state, restored unchanged and recorded only so it is on
   file: `control_mode=3` (position), `input_mode=1` (passthrough),
   `vel_ramp_rate=1.0`, `current_lim=15.0`.
