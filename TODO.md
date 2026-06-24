# TODO

## Hardware

- [ ] **Add an RC low-pass at the motor-thermistor ADC input (GPIO4 / PA3).**
  The NTC divider (3.3V–10k–PIN–NTC(10k MF52 B3950)–GND) picks up motor PWM
  noise on its high-impedance node: raw ADC reads glitch by tens of degrees
  while the motor spins (seen 23.6–50.5 °C while the real winding was ~33 °C,
  stdev 3.3 °C). Fit **100 nF–1 µF from PA3 to GND**, optionally a **1 kΩ series
  resistor** between the divider midpoint and the pin to form a cleaner RC and
  protect the input. With the ~5 kΩ source impedance that gives τ ≈ 0.5–5 ms —
  far faster than any thermal transient but it crushes the PWM-band pickup.
  This is the root-cause fix; do it next time the board is open.
  - *Mitigation already in firmware:* a 1 s IIR low-pass in
    `Firmware/MotorControl/thermistor.cpp::update()` (flashed 2026-06-24)
    dropped the noise to stdev 0.59 °C / ~1 % glitches. The cap is still worth
    adding so the raw signal is clean regardless of firmware.

## Firmware

- [x] Port harmonic compensation
  https://docs.odriverobotics.com/v/latest/manual/hardware-config.html#harmonic-compensation
  - Per-encoder 1st/2nd-harmonic (eccentricity) correction added to
    `Encoder::update()`; config fields `enable_harmonic_compensation`,
    `harmonic_{cos,sin}_{1,2}` (in counts) + readonly `harmonic_error`.
    Coefficients are measured offline by
    `spider-motor-tools/harmonic_calibration.py` (constant-velocity sweep +
    least-squares fit). See AGENTS.md "Encoder Harmonic Compensation".
  - **Flashed + axis0 calibrated/saved on bench ODrive 3482345a3034 (motor #7),
    2026-06-24.** AS5047P intrinsic error is modest (~0.49° mech 1st / 0.25° 2nd);
    correction verified applied via common-mode A/B. Key lesson: calibrate at
    **≥12 motor t/s** — low speed is dominated by cogging velocity ripple and is
    not repeatable. axis1 MT6701 not yet calibrated (needs the joint free to
    rotate full output turns).
