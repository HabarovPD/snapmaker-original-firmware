# Snapmaker Original firmware 2.11: GCC build, release 2.11-gcc2

Prebuilt from commit `8b9dec6` of this repository. **Use at your own risk**: see
the warning in the [main README](../../README.md).

## F103: GD32F103 / STM32F103 high-density, 16 MHz crystal (tested on hardware)

This is for a controller whose STM32F105 was replaced with a F103-class chip.
It has no USB host, so printing and updating from a USB stick are disabled.

| File | Write at | Use |
|------|----------|-----|
| `full_0x08000000.bin` | `0x08000000` | Blank or new chip: jump stub + firmware in one file |
| `app_0x08009800.bin` | `0x08009800` | Update the firmware only (stub already on the chip) |
| `boot_stub_0x08000000.bin` | `0x08000000` | Jump stub only (32 bytes) |

## F105: STM32F105RC / GD32F105RC, stock chip (builds; not tested on hardware)

| File | Write at | Use |
|------|----------|-----|
| `Update.Bin` | via USB stick | Stock Snapmaker bootloader: copy to the root of a FAT32 stick, power on |
| `app_0x08009800.bin` | `0x08009800` | Firmware only, through ST-Link; keeps the stock bootloader |

## Flashing with ST-Link V2 + OpenOCD

Back up the whole flash first:

```sh
openocd -f interface/stlink.cfg -c "set CPUTAPID 0" -f target/stm32f1x.cfg \
  -c "init; reset halt; flash read_bank 0 flash_backup.bin; shutdown"
```

Then write the file you need at its address (example: blank F103 chip):

```sh
openocd -f interface/stlink.cfg -c "set CPUTAPID 0" -f target/stm32f1x.cfg \
  -c "program F103/full_0x08000000.bin verify reset exit 0x08000000"
```

`set CPUTAPID 0` is needed for GD32 chips. STM32CubeProgrammer and the
ST-Link Utility work too: open the file and set the start address from the
table above.

On first boot the settings are empty, so the firmware loads its defaults
(Z home offset -5.737). Calibrate, then save with `M500`.

Updating from 2.11-gcc1 keeps the settings, but the old 2x2 mesh does not
fit the new 7x7 grid and is cleared. The old Z home offset (usually -8) is
kept: set it with `G29 S5` / `G29 S6` or `M206 Z-5.737` + `M500`, then
redo the mesh with `G29 S1`. See [Bed leveling](../../README.md#bed-leveling).

Check the files with `sha256sum -c SHA256SUMS.txt`.
