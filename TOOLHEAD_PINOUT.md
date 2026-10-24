# Tool head connector pinout

[Русская версия](TOOLHEAD_PINOUT.ru.md)

All three Snapmaker Original modules (3D printing, laser, CNC) use the **same
8-pin connector**. The firmware tells them apart only by the resistance between
the thermistor pins, and reuses the heater and stepper lines in different ways.

The 3D printing column is the known module wiring. The laser column is derived
from the firmware source (`Firmware-Snapmaker-2.11/Marlin_ST_RC6_UDisk/CFile/Marline/`)
and checked against the 350 mW laser schematic in [`SMLaser.pdf`](SMLaser.pdf).
The CNC column is derived from the firmware only. Check with a multimeter before
connecting your own hardware.

## Pinout

| Pin | 3D printing | Laser | CNC |
|-----|-------------|-------|-----|
| 1 | VCC: heater socket pin 1, fan+ | VCC (24 V), laser driver supply | VCC: spindle motor power |
| 2 | Stepper coil A+ | not connected | not used¹ |
| 3 | Heater socket pin 2 (low-side switch to GND, PB0) | **/Enable** (active low), PWM from PB0 (TIM1_CH2N, ~100 Hz, 255 levels) | **Spindle on/off**, low-side switch to GND (PB0, no PWM) |
| 4 | Stepper coil A− | not connected | not used¹ |
| 5 | Thermistor socket pin 1 | **/Module present**: 150 Ω to GND (ID resistor) | **ID resistor** (see below) |
| 6 | Stepper coil B− | not connected | not used¹ |
| 7 | GND, fan−, thermistor socket pin 2 | GND | GND, other end of the ID resistor |
| 8 | Stepper coil B+ | not connected | not used¹ |

¹ In CNC mode the firmware never drives the E0 stepper driver, so these pins are
most likely not connected inside the CNC module. In the laser schematic they are
not connected.

## Controller side (`Pins.h`)

| Signal | MCU pin | Connector pin |
|--------|---------|---------------|
| `HEATER_0` = `LASER_PIN` (same pin) | PB0, drives the low-side MOSFET | 3 |
| `TEMP_0` | PC5, ADC | 5 (pin 7 is GND) |
| `E0_STEP` / `E0_DIR` / `E0_ENABLE` | PA4 / PA1 / PA0 | driver outputs on 2, 4, 6, 8 |

## Module detection

`loop()` in `Marlin_main.cpp` reads the thermistor input once at power-up:

```c
if(current_temperature[0] > 400)      CNCInit();                      // CNC
else if(current_temperature[0] > 340) MachineSelected = MA_LASER_350; // 350 mW laser
else if(current_temperature[0] > 260) MachineSelected = MA_LASER;     // 200 mW laser
else                                  PrintInit();                    // 3D printing
```

So the laser and CNC modules have a fixed resistor instead of a thermistor, which
the firmware reads as a very high "temperature". The firmware uses table 11
(`TEMP_SENSOR_0 11` in `thermistortables.h`), which Snapmaker rewrote for a 12-bit
ADC: it starts at `{4, 938}`, `{124, 314}`, so it is not a plain B = 3950 curve.

The ADC reading is `raw = 4095 × R / (R + R_pullup)`. The pull-up resistor on the
controller board is not in the firmware, so the table below gives the thresholds
for two likely values. These are calculated from table 11, not measured:

| Module | "Temperature" | R (pins 5–7), 4.7 kΩ pull-up | R (pins 5–7), 5.1 kΩ pull-up |
|--------|---------------|------------------------------|------------------------------|
| 3D printing | < 260 °C | > ~290 Ω (100 kΩ NTC) | > ~320 Ω (100 kΩ NTC) |
| Laser 200 mW | 260–340 °C | ~140–290 Ω | ~150–320 Ω |
| Laser 350 mW | 340–400 °C | ~127–140 Ω | ~137–153 Ω |
| CNC | > 400 °C | < ~127 Ω | < ~137 Ω |

The 350 mW schematic uses **150 Ω** (R2). With a 5.1 kΩ pull-up that reads as
about 350 °C, which is a 350 mW laser. With a 4.7 kΩ pull-up it reads as about
312 °C, and the firmware would pick the 200 mW laser. The 350 mW window is only
about 15 Ω wide, so measure the pull-up on your board (between PC5 and 3.3 V)
before choosing an ID resistor.

Detection runs only once at power-up, so modules cannot be swapped while the
machine is on.

## How pin 3 is driven

- **Laser** (`LaserTimerInit` in `Marlin_main.cpp`, `LaserInit` in `Laser.cpp`):
  PB0 is switched to the TIM1 alternate function (partial remap, channel CH2N).
  Power is set by the PWM duty cycle. In the 350 mW schematic the laser driver
  (XL3005 constant-current buck, 350 mA) is powered from pin 1 all the time.
  Pin 3 is only a logic input: a 10 k / 10 k divider (R4, R5) pulls it to about
  half of VCC, and through D1 this pushes the driver's CS pin high, which shuts
  the output off. When the controller pulls pin 3 to GND, D1 stops conducting
  and the laser turns on. So the laser is on while the switch on pin 3 conducts.
- **CNC** (`CNCInit` in `CNC.cpp`): PB0 is a plain push-pull output.
  `digitalWrite(LASER_PORT, LASER_PIN, 1)` turns the spindle on, `0` turns it off.
  The firmware does not control spindle speed.

## Features present in the code but disabled

`SnapmakerDefines.h` sets `SNAP_OLD_BASE 1`, `HAVE_CNC_MCU 0`,
`HAVE_LASER_EEPROM 0` and `LASER_USE_DAC 0`. So the following are not used:

- module detection by the `E_ID_0` / `E_ID_1` pins (PE15 / PE14);
- the laser module's I2C EEPROM on PB10 / PB11;
- UART3 link to a spindle controller;
- analog laser control through the DAC.

## See also

[`SMLaser.pdf`](SMLaser.pdf) is a third-party schematic of a 350 mW laser module
for the Snapmaker v1 (El Inverso Engineering, KiCad). The module connects through
an RJ45 jack with the same 8-pin numbering. The schematic labels the pins as
"1 Vcc (24V), 3 /Enable, 5 /Module present, 7 Ground", and the rest are not
connected. It shows no fan.
