# Tool head connector pinout

[Русская версия](TOOLHEAD_PINOUT.ru.md)

All three Snapmaker Original modules (3D printing, laser, CNC) use the **same
8-pin connector**. The firmware tells them apart only by the resistance between
the thermistor pins, and reuses the heater and stepper lines in different ways.

The 3D printing column is the known module wiring. The laser and CNC columns are
**derived from the firmware source** (`Firmware-Snapmaker-2.11/Marlin_ST_RC6_UDisk/CFile/Marline/`),
not from the module schematics. Check with a multimeter before connecting your
own hardware.

## Pinout

| Pin | 3D printing | Laser | CNC |
|-----|-------------|-------|-----|
| 1 | VCC: heater socket pin 1, fan+ | VCC: laser power (and likely its fan) | VCC: spindle motor power |
| 2 | Stepper coil A+ | not used¹ | not used¹ |
| 3 | Heater socket pin 2 (low-side switch to GND, PB0) | **Laser PWM**, low-side switch to GND (PB0, TIM1_CH2N, ~100 Hz, 255 levels) | **Spindle on/off**, low-side switch to GND (PB0, no PWM) |
| 4 | Stepper coil A− | not used¹ | not used¹ |
| 5 | Thermistor socket pin 1 | **ID resistor** (see below) | **ID resistor** (see below) |
| 6 | Stepper coil B− | not used¹ | not used¹ |
| 7 | GND, fan−, thermistor socket pin 2 | GND, other end of the ID resistor | GND, other end of the ID resistor |
| 8 | Stepper coil B+ | not used¹ | not used¹ |

¹ In laser and CNC mode the firmware never drives the E0 stepper driver, so these
pins are most likely not connected inside those modules.

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
the firmware reads as a very high "temperature". The thermistor type is 11
(`TEMP_SENSOR_0 11`, 100 kΩ, B = 3950). The resistances below are estimates from
the B equation, not measured:

| Module | "Temperature" | Resistance between pins 5 and 7 (estimate) |
|--------|---------------|--------------------------------------------|
| 3D printing | < 260 °C | 100 kΩ NTC thermistor |
| Laser 200 mW | 260–340 °C | ~110–290 Ω |
| Laser 350 mW | 340–400 °C | ~60–110 Ω |
| CNC | > 400 °C | < ~60 Ω |

Detection runs only once at power-up, so modules cannot be swapped while the
machine is on.

## How pin 3 is driven

- **Laser** (`LaserTimerInit` in `Marlin_main.cpp`, `LaserInit` in `Laser.cpp`):
  PB0 is switched to the TIM1 alternate function (partial remap, channel CH2N).
  Power is set by the PWM duty cycle. The module takes + from pin 1, and the
  low-side switch on pin 3 modulates it.
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

[`SMLaser.pdf`](SMLaser.pdf) is a schematic of a 350 mW laser module for the
Snapmaker v1. Use it to check pins 1, 3 and 5 against this page.
