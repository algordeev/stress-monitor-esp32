# Wiring guide

Disconnect ESP32 power before changing connections. The ESP32's GPIO inputs are **3.3 V logic**; do not apply 5 V to them.

| Module | Module pin | ESP32 |
| --- | --- | --- |
| DFRobot Gravity SEN0203 | OUT / S (green signal wire) | GPIO34 |
| DFRobot Gravity SEN0203 | VCC / + (red wire) | 3V3 |
| DFRobot Gravity SEN0203 | GND / − (black wire) | GND |
| SSD1306 OLED | SDA | GPIO21 |
| SSD1306 OLED | SCL | GPIO22 |
| SSD1306 OLED | VCC | 3V3 |
| SSD1306 OLED | GND | GND |
| GY-521 motion module | SDA | GPIO21 |
| GY-521 motion module | SCL | GPIO22 |
| GY-521 motion module | VCC | 3V3 |
| GY-521 motion module | GND | GND |

Set the Gravity module selector to **A (analog)**. Verify wire labels against your particular module before powering it.

Both the OLED and GY-521 share the same two-wire I²C bus. The observed addresses in this prototype are:

- OLED: **0x3C**
- Motion module: **0x68**

The motion module's `WHO_AM_I` register reads **0x72**, which does not match the regular MPU6050 identity; the sketch uses direct I²C operations rather than `Adafruit_MPU6050` initialization. Accuracy of the assumed accelerometer/gyro scale factors is not independently established.

### Setup advice

- Place the motion module on a stable surface for the **3 s startup calibration**.
- Ensure breadboard connections are solid; loose I²C jumpers can cause intermittent errors.
- For useful motion-artifact detection, mechanically couple the MPU to the PPG sensing assembly. A stationary MPU cannot detect independent movement of the finger.
- GSR/EDA electrodes are **not part of the current wired setup**.
