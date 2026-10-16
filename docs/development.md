# Development notes

## Stable hardware prototype: v0.4.5

The currently tested prototype uses DFRobot Gravity analog PPG, SSD1306 OLED, and a GY-521 module with unusual device identity (`WHO_AM_I=0x72`).

### Firmware progression

- **v0.4.4** — pulse processing, 50 Hz motion input, automatic gyro-bias calibration, motion-aware invalidation, OLED graph, serial telemetry and I²C recovery.
- **v0.4.5** — reduced motion sensitivity: trigger score `2.0`, four sequential detection samples, release score `0.9`, motion hold `350 ms`, settle period `700 ms`. This version was reported working during prototype testing.

### Useful tuning parameters

Inside the `.ino` file:

```cpp
constexpr float MOTION_TRIGGER = 2.0f;
constexpr float MOTION_RELEASE = 0.9f;
constexpr uint8_t MOTION_CONFIRM_SAMPLES = 4;
constexpr uint32_t MOTION_HOLD_MS = 350;
constexpr uint32_t SETTLE_MS = 700;
```

Larger `MOTION_TRIGGER` values are **less sensitive** to small movements, at the cost of potentially overlooking short artifacts. Do not interpret these thresholds as clinically validated cutoffs.

### Planned validation

Compare the measured BPM against an external reference and investigate timestamp accuracy of pulse threshold crossings. Evaluate performance with no movement, slight hand movement, and deliberate motion. Keep raw recordings to distinguish missed beats, false beats, and motion gating effects.
