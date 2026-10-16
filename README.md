# ESP32 Stress Monitor

**Experimental physiological monitoring prototype** using an ESP32, optical photoplethysmography (PPG), a motion sensor, and an OLED screen.

> [!IMPORTANT]
> This project does not measure psychological stress, and is not a medical device. The currently implemented features are pulse estimation and motion-aware rejection of potentially unreliable measurements.

## Features — firmware v0.4.5

- Analog PPG acquisition at a nominal 100 Hz with adaptive threshold beat detection.
- Heart rate (BPM) from the median of up to five accepted pulse intervals (IBI).
- BPM display updated every 2 seconds.
- 50 Hz acceleration/gyroscope monitoring and startup gyro-bias calibration.
- Gentle motion detection using four consecutive threshold crossings and hysteresis.
- Measurement pause during detected movement, followed by a 700 ms settling window.
- Live 128×64 OLED view with BPM, IBI, status and a scrolling PPG signal.
- 115200 baud Serial Monitor / Serial Plotter telemetry.
- Automatic attempts to reconnect to the motion sensor after communication loss.

## Hardware

| Component | Role |
| --- | --- |
| ESP32 30-pin Dev Board | Microcontroller |
| DFRobot Gravity Heart Rate Sensor SEN0203 | Analog PPG |
| SSD1306 128×64 OLED (I²C) | On-device display |
| GY-521 motion module | Motion detection; behaves partly like MPU6050 |

See [wiring instructions](docs/wiring.md).

## Quick start

1. Install **Arduino IDE** and the Espressif **esp32** board package.
2. Install **Adafruit GFX Library** and **Adafruit SSD1306** from Library Manager.
3. Wire the modules according to [docs/wiring.md](docs/wiring.md). Set the Gravity Heart Rate Sensor to analog **A** mode.
4. Open [`firmware/stress_monitor/stress_monitor.ino`](firmware/stress_monitor/stress_monitor.ino).
5. Choose **ESP32 Dev Module** and the board's serial port, then upload.
6. Keep the motion sensor **completely still for the initial three-second gyroscope calibration**.
7. Open Serial Monitor at **115200 baud**, or observe the on-device OLED. Put a finger against the optical pulse sensor and wait several beats for the first estimate.

## Serial output

The sketch prints labeled tab-separated values (every 200 ms): `Signal`, `Threshold`, `BPM`, `IBI`, `Motion`, `Moving`, `Stable`, `PulseValid`, `MotionOK`, and `I2CErrors`.

An invalid BPM is reported as `0` in the serial output and as `--` on the OLED. This is a **missing estimate**, not a heart rate of zero.

## How signal processing works

The PPG signal is detrended using a slowly updated baseline, smoothed, and compared against an adaptive threshold. Beat-to-beat intervals between 350 and 1500 ms may be accepted when motion is considered stable. Large deviations from an established median interval are rejected. The reported BPM is based on the median of up to five accepted intervals, not directly on raw waveform peaks.

The motion sensor uses an accelerometer-derived dynamic component and gyro rates corrected by a startup bias measurement. Motion must exceed the configured threshold for **four consecutive samples** before measurement is paused; hysteresis helps prevent rapid toggling. Once movement ends, the system waits before accepting fresh pulse intervals.

## Known limitations

- The motion module reports `WHO_AM_I = 0x72` despite being sold as an MPU6050/GY-521. Firmware accesses its registers directly; scaling assumptions and chip identity are **not independently verified**.
- The motion detector senses motion of the **MPU board**, not necessarily motion of the finger or PPG sensor when they are not mechanically coupled.
- Optical pulse timing is not equivalent to clinical ECG R–R timing; this firmware does **not** provide validated pulse-rate variability (PRV), heart-rate variability (HRV), SpO₂ or stress scores.
- The confirmation-based motion filter can miss brief movement artifacts. Measurements have not been clinically validated.
- The firmware's 100 Hz and 50 Hz values are nominal software sampling schedules, not separately measured performance guarantees.

## Planned experiments

- [ ] Add MAX30100 as a second optical sensor for comparison.
- [ ] Record synchronized data and evaluate waveform/beat detection quality.
- [ ] Add GSR/EDA measurements with appropriate electrical isolation and power precautions.
- [ ] Revisit a desktop dashboard when sensor acquisition is stable.
- [ ] Test repeatability against a suitable reference instrument.

## Safety

This is a non-medical experimental device. **Do not connect skin-contact GSR/EDA electrodes to a system powered by an ordinary computer USB connection.** Future electrode measurements require an appropriately designed, evaluated electrical safety setup. A battery alone does not make a DIY circuit medically safe.

## Repository

- Firmware: [`firmware/stress_monitor/`](firmware/stress_monitor/)
- Wiring: [`docs/wiring.md`](docs/wiring.md)
- Development notes: [`docs/development.md`](docs/development.md)

No software license has been selected yet; do not assume permission for redistribution or modification beyond applicable law.
