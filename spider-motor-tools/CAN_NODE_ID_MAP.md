# Spider CAN node ID map

Per-joint CAN addressing for the hexapod (6 legs × 3 motors = 18 joints).

## Conventions

- **Position label:** `<leg>-<motor>` (e.g. `2-3` = leg 2, knee).
- **Leg number 1..6:** clockwise, starting at the **front-left** leg.
- **Motor number 1..3:** `1` = top, `2` = middle, `3` = bottom (knee).
- **`can_node_id = leg*10 + motor`** — readable decimal (id `23` = leg 2, motor 3).
- ODrive CAN Simple uses a 6-bit node field → range **0–63**. Leg 6 motor 3 = `63`
  is exactly the ceiling; extending past it needs extended (29-bit) frames.
- `can_node_id = 0` is the erased/default value, so no real joint uses it — an
  un-configured board is instantly visible on the bus.

Rows are keyed by leg position. `Motor #` (physical build/characterization order)
and `Serial` are filled in as boards get mounted and assigned.

## Assignment

| Leg | Motor | Location    | `can_node_id` | Motor # | Serial       |
|:---:|:-----:|:------------|:-------------:|:-------:|:-------------|
| 1   | 1     | top         | **11**        |         |              |
| 1   | 2     | middle      | **12**        |         |              |
| 1   | 3     | bottom/knee | **13**        | 2       | 367836893335 |
| 2   | 1     | top         | **21**        |         |              |
| 2   | 2     | middle      | **22**        |         |              |
| 2   | 3     | bottom/knee | **23**        |         |              |
| 3   | 1     | top         | **31**        |         |              |
| 3   | 2     | middle      | **32**        |         |              |
| 3   | 3     | bottom/knee | **33**        |         |              |
| 4   | 1     | top         | **41**        |         |              |
| 4   | 2     | middle      | **42**        |         |              |
| 4   | 3     | bottom/knee | **43**        |         |              |
| 5   | 1     | top         | **51**        |         |              |
| 5   | 2     | middle      | **52**        |         |              |
| 5   | 3     | bottom/knee | **53**        |         |              |
| 6   | 1     | top         | **61**        |         |              |
| 6   | 2     | middle      | **62**        |         |              |
| 6   | 3     | bottom/knee | **63**        |         |              |

## Setting it on a board (single-axis → use axis0)

```python
odrv0.axis0.config.can_node_id = 13          # leg 1, bottom/knee
odrv0.axis0.config.can_node_id_extended = False
odrv0.save_configuration()
```

Current bench board **serial `367836893335` (physical motor #2) → leg 1, bottom/knee → `can_node_id = 13`** (applied + saved).
