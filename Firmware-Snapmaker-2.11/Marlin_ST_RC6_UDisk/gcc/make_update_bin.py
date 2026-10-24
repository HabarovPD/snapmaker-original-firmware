"""Pack app.bin into Update.Bin for the Snapmaker bootloader (USB-stick update).

Layout, taken from the official Snapmaker-GD32Base-2.11 Update.Bin:
  0x000        0x00
  0x001..      "Snapmaker_YYYY-MM-DD-HHMM" (ASCII version string)
  0x024        0xEA  (Main.c: DelUpdateFile() removes Update.bin when it sees this byte)
  ..0x7FF      zero padding
  0x800..      raw image linked at 0x08009800
"""
import datetime
import sys

HEADER_SIZE = 0x800
APP_MAX = 0x20000  # 0x08009800..0x08029800, settings page follows


def main():
    src, dst = sys.argv[1], sys.argv[2]
    image = open(src, "rb").read()
    if len(image) > APP_MAX:
        sys.exit(f"image is {len(image)} bytes, more than {APP_MAX}")

    header = bytearray(HEADER_SIZE)
    version = datetime.datetime.now().strftime("Snapmaker_%Y-%m-%d-%H%M").encode()
    header[1:1 + len(version)] = version
    header[0x24] = 0xEA

    open(dst, "wb").write(bytes(header) + image)
    print(f"{dst}: {version.decode()}, image {len(image)} bytes")


if __name__ == "__main__":
    main()
