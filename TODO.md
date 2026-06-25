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

- [ ] Port harmonic compensation
  https://docs.odriverobotics.com/v/latest/manual/hardware-config.html#harmonic-compensation
