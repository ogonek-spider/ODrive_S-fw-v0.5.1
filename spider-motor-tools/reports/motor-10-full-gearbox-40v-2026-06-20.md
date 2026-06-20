# Motor #10 — full 1:36 gearbox retest @ 40 V (2026-06-20)

**Headline: the 1:36 gearbox is HEALTHY. The earlier "gearbox binding" finding
was a misattribution — the real cause was the bad commutation offset, now pinned
to 0.95.** This retest was run after pinning the offset, on the fully reassembled
two-stage 1:36 gearbox, powered from a 40 V lab supply with the 2 Ω brake
resistor armed.

## Free-spin sweep (motor-shaft turns/s)

| cmd | original 36:1 (scattered offset) | full 1:36 @ 40 V (offset pinned 0.95) |
|---|---|---|
| 5  | 3.21 A / 7.14 A → 3.35 t/s | **1.36 A / 1.82 A → 5.14 t/s ✅** |
| 10 | 13.07 A / 15.14 A → **STALLED** | **1.50 A / 2.43 A → 10.23 t/s ✅** |
| 15 | — | **1.43 A / 2.39 A → 15.24 t/s ✅** |
| 20 | — | **1.46 A / 2.69 A → 20.37 t/s ✅** |

- Flat ~1.4–1.5 A across the whole range, reaching full commanded speed to 20 t/s.
- Report JSON: `motor-10-stage2-40v-2026-06-20.json`.

## Why the original diagnosis was wrong

The original 36:1 stall test ran with a *scattered* commutation offset
(`offset_float` was bimodal 0.5→1.3 rad). When commutation lands badly the motor
can't make torque efficiently, so it drew 13 A and stalled — and that was blamed
on gearbox binding. After pinning `offset_float = 0.95` (and confirming stage 1
and stage 2 each spin free), the fully assembled 1:36 gearbox spins at ~1.5 A.
So: encoder healthy, windings healthy, **gearbox healthy**, and the offset fix
resolved the only real fault.

## Measured full-drivetrain joint speed (no-load, 40 V)

- 20 t/s motor ÷ 36 = **~200 °/s at the joint** (~33 joint rpm) at ~1.5 A.
- The earlier ~16 / ~20 t/s "ceilings" were just bus voltage (30.8 V / 35.9 V);
  at 40 V it reaches 20 t/s cleanly. At a full 42 V pack, ~21 t/s → ~210 °/s.

## Setup

- ODrive 3482345a3034, fw-v0.5.1-mt6701, axis0 AS5047P (mode 257), offset 0.95.
- vbus 40.0 V (lab supply), 2 Ω brake resistor armed (`brake_resistor_armed=True`,
  `max_regen_current=0` → regen dumped to resistor, not back-fed to supply).
- Full two-stage 1:36 gearbox, no load. All config RAM-only; nothing flashed.
