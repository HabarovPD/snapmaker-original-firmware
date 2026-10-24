# Snapmaker Original (v1) firmware: GCC build

> [!WARNING]
> **Use at your own risk.** This is unofficial firmware, not affiliated with or endorsed by Snapmaker. Flashing it, replacing the controller MCU or opening the electronics may void your warranty. You alone are responsible for any loss of warranty and for any damage to your printer, laser or CNC module, controller board, programmer or other equipment, as well as for any injury or property damage resulting from its use.
>
> Back up the whole flash (`make backup`) before writing anything, check the wiring and supply voltage, and never leave the machine unattended while it runs, especially with the laser. The software is provided "as is", without warranty of any kind; see sections 15 and 16 of the GPLv3.

This repo has the source code of the official **Snapmaker Original firmware 2.11**,
which is Marlin 1.1.0-RC6 heavily modified by Snapmaker. It is ported from IAR
Embedded Workbench to **arm-none-eabi-gcc** and can be flashed with a cheap
**ST-Link V2** through OpenOCD.

The original IAR project file (`*.ewp`) was never published because the upstream
`.gitignore` excluded it, so the sources could not be built as distributed.
This repo makes them buildable again. It also adds support for controllers whose
STM32F105 was replaced with a **GD32F103 / STM32F103**.

## Status

| Target | Chip | State |
|--------|------|-------|
| `MCU=F103` (default) | GD32F103RE / STM32F103 high-density, 16 MHz crystal | Flashed and running on real hardware (3D printing) |
| `MCU=F105` | STM32F105RC / GD32F105RC (stock chip) | Builds, `Update.Bin` produced; not tested on hardware |

The F103 build has no USB host, because that chip has no USB OTG. Printing and
updating from a USB stick are disabled. The touchscreen, 3D printing, laser, CNC
and printing over the USB cable from a PC are unchanged.

## Quick start (Windows + MSYS2)

```sh
pacman -S mingw-w64-ucrt-x86_64-arm-none-eabi-toolchain mingw-w64-ucrt-x86_64-openocd make
export PATH=/c/msys64/ucrt64/bin:/c/msys64/usr/bin:$PATH

cd Firmware-Snapmaker-2.11/Marlin_ST_RC6_UDisk
make -j8            # build/F103/app.bin
make probe          # check the ST-Link connection and the chip
make backup         # save the whole flash first
make flash-blank    # chip without the Snapmaker bootloader: jump stub + app
make flash          # later updates: app only, settings are kept
```

Connect the board's SWD header to the ST-Link: SWDIO, SWCLK, GND. Power the
printer from its own supply. See
[`Firmware-Snapmaker-2.11/Marlin_ST_RC6_UDisk/BUILD.md`](Firmware-Snapmaker-2.11/Marlin_ST_RC6_UDisk/BUILD.md)
for the flash map, every make target and recovery options.

## What was changed for GCC

- All application code and the ST USB stack are compiled as C++, as IAR did,
  because cross-file calls rely on C++ linkage.
- The IAR inline-asm critical sections (`SaveSR`/`RestoreSR`) were replaced with
  CMSIS intrinsics. Their `bx lr` broke the caller once GCC inlined them.
- The busy-wait `delay_us` used by the laser module's I2C now has a `volatile`
  counter. Otherwise GCC removes the loop.
- F103 clocking: HSE 16 MHz / 2 × 9 = 72 MHz without PLL2, and ADC at 9 MHz.
- Interrupt vector tables were checked against the official 2.11 `Update.Bin`.

## Repository layout

| Path | Contents |
|------|----------|
| `Firmware-Snapmaker-2.11/Marlin_ST_RC6_UDisk/` | Firmware sources, `Makefile`, `gcc/` (linker script, startup, OpenOCD config, `Update.Bin` packer, boot stub) |
| `bins/Snapmaker-2.11-gcc/` | Prebuilt binaries of this GCC build (F103 and F105) with flashing instructions |
| `bins/Firmware GD32Base-2.11 bin/` | Official 2.11 `Update.Bin` with changelog and upgrade instructions |
| `bins/klipper bins/` | Prebuilt Klipper images for this board (`b<bootloader KiB>_c<crystal MHz>`) |
| `assets-20210120/` | CAM post-processors and tool libraries (ArtCAM, Aspire, FreeCAD, Fusion 360, VCarve) |
| `SMLaser.pdf` | Schematic of a 350 mW laser module for Snapmaker v1 (RJ45 pinout, XL3005 driver) |

## License

Marlin is licensed under GPLv3, and so is this firmware as a derivative work;
see [`LICENSE`](LICENSE). The ST libraries keep their own licenses
(see `Firmware-Snapmaker-2.11/Marlin_ST_RC6_UDisk/Libraries/`).
