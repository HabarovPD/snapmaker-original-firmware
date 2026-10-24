# GCC build for Snapmaker (Original) Marlin firmware.
# Replaces the lost IAR project (*.ewp was excluded by .gitignore).
#
#   MCU=F103 (default) - GD32F103/STM32F103 high-density (replacement chip), 16 MHz HSE,
#                        no USB host: printing/updating from a USB stick is disabled
#   MCU=F105           - original STM32F105RC / GD32F105RC, full feature set
#
#   make             - build/<MCU>/app.elf, app.bin (+ Update.Bin for F105)
#   make probe       - check ST-Link connection, flash size, readout protection
#   make backup      - read the whole 256K flash into backup/ (DO THIS FIRST)
#   make flash       - write build/app.bin at 0x08009800, bootloader and settings are kept
#   make restore     - write backup/flash_full.bin back
#   make flash-blank - new/blank chip (no Snapmaker bootloader): jump stub + app
#   make flash-stub  - only the jump stub at 0x08000000
#   make clean
#
# Flash map: 0x08000000 bootloader (32K, not in these sources) | 0x08008000 boot
# params | 0x08009800 app (128K max) | 0x08029800 Marlin settings.

TOOLCHAIN ?= /c/msys64/ucrt64/bin/
PREFIX    := $(TOOLCHAIN)arm-none-eabi-
CC        := $(PREFIX)gcc
CXX       := $(PREFIX)g++
OBJCOPY   := $(PREFIX)objcopy
SIZE      := $(PREFIX)size
OPENOCD   ?= $(TOOLCHAIN)openocd
PYTHON    ?= python

MCU ?= F103
BUILD := build/$(MCU)

APP_BASE := 0x08009800
APP_SIZE := 0x20000

# ---------------------------------------------------------------- sources
# IAR compiled the whole project as C++ (cross-file calls use C++ linkage),
# so the app *.c files and the ST USB stack are built with g++ too.
# Only CMSIS system + StdPeriph (they have extern "C" guards) stay plain C.
CXX_FROM_C := \
  CFile/Main.c \
  CFile/FATFS/ff.c \
  CFile/FATFS/diskio.c \
  CFile/FATFS/drivers/SDCardDriverSPI.c \
  CFile/FATFS/option/unicode.c

# USB OTG host (USB stick) - only the F105 has it
USB_HOST_SRC := \
  CFile/usb_bsp.c \
  CFile/usbh_usr.c \
  CFile/FATFS/drivers/fatfs_usb.c \
  Libraries/STM32_USB_OTG_Driver/src/usb_core.c \
  Libraries/STM32_USB_OTG_Driver/src/usb_hcd.c \
  Libraries/STM32_USB_OTG_Driver/src/usb_hcd_int.c \
  Libraries/STM32_USB_HOST_Library/Core/src/usbh_core.c \
  Libraries/STM32_USB_HOST_Library/Core/src/usbh_hcs.c \
  Libraries/STM32_USB_HOST_Library/Core/src/usbh_ioreq.c \
  Libraries/STM32_USB_HOST_Library/Core/src/usbh_stdreq.c \
  Libraries/STM32_USB_HOST_Library/Class/MSC/src/usbh_msc_bot.c \
  Libraries/STM32_USB_HOST_Library/Class/MSC/src/usbh_msc_core.c \
  Libraries/STM32_USB_HOST_Library/Class/MSC/src/usbh_msc_scsi.c

# SDCard/cardreader.cpp is not used: "card" is a UDisk (Marlin_main.cpp)
APP_CXX := $(wildcard CFile/Marline/*.cpp) \
  CFile/Marline/UDisk/UDisk.cpp

LIB_C := \
  Libraries/CMSIS/STM32F10x/Source/Templates/system_stm32f10x.c \
  $(wildcard Libraries/STM32F10x_StdPeriph_Driver/src/*.c)

ifeq ($(MCU),F105)
  CXX_FROM_C += $(USB_HOST_SRC)
  ASM := gcc/startup_stm32f10x_cl.s
  MCU_DEFS := STM32F10X_CL
else ifeq ($(MCU),F103)
  CXX_FROM_C += CFile/UDiskNone.c
  ASM := gcc/startup_stm32f10x_hd.s
  # FATFS_USE_USB: diskio.c keeps drive 0, UDiskNone.c reports "no disk"
  MCU_DEFS := STM32F10X_HD HSE_VALUE=16000000 FATFS_USE_USB=1
else
  $(error MCU must be F103 or F105)
endif

LDSCRIPT := gcc/stm32f105rc_app.ld

INCLUDES := \
  Includes \
  CFile \
  CFile/Marline \
  CFile/Marline/Configs \
  CFile/Marline/SDCard \
  CFile/Marline/UDisk \
  CFile/FATFS \
  CFile/FATFS/drivers \
  CFile/FATFS/option \
  Libraries/CMSIS/Include \
  Libraries/CMSIS/STM32F10x/Include \
  Libraries/STM32F10x_StdPeriph_Driver/inc \
  Libraries/STM32_USB_OTG_Driver/inc \
  Libraries/STM32_USB_HOST_Library/Core/inc \
  Libraries/STM32_USB_HOST_Library/Class/MSC/inc

# USE_USB_OTG_FS: USB headers are included everywhere via Includes.h
DEFS := $(MCU_DEFS) USE_STDPERIPH_DRIVER USE_USB_OTG_FS

# ---------------------------------------------------------------- flags
CPU    := -mcpu=cortex-m3 -mthumb
OPT    ?= -Os
COMMON := $(CPU) $(OPT) -g3 -ffunction-sections -fdata-sections \
          -fno-strict-aliasing -funsigned-char \
          $(addprefix -D,$(DEFS)) $(addprefix -I,$(INCLUDES)) \
          -MMD -MP -Wall -Wno-unused-variable -Wno-unused-function -Wno-unused-but-set-variable
CFLAGS   := $(COMMON) -std=gnu99
CXXFLAGS := $(COMMON) -std=gnu++11 -fno-exceptions -fno-rtti -fno-threadsafe-statics \
            -fpermissive -Wno-narrowing -Wno-write-strings
LDFLAGS  := $(CPU) -T$(LDSCRIPT) -Wl,--defsym=APP_BASE=$(APP_BASE) -Wl,--defsym=APP_SIZE=$(APP_SIZE) \
            -Wl,--gc-sections -Wl,-Map=$(BUILD)/app.map -Wl,--print-memory-usage \
            --specs=nano.specs --specs=nosys.specs
LIBS     := -lm

# ---------------------------------------------------------------- objects
obj = $(BUILD)/obj/$(basename $(1)).o
CXX_FROM_C_OBJS := $(foreach f,$(CXX_FROM_C),$(call obj,$(f)))
OBJS := $(CXX_FROM_C_OBJS) $(foreach f,$(LIB_C) $(APP_CXX) $(ASM),$(call obj,$(f)))

all: $(BUILD)/app.bin $(if $(filter F105,$(MCU)),$(BUILD)/Update.Bin)

$(BUILD)/obj/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(CXX_FROM_C_OBJS): $(BUILD)/obj/%.o: %.c
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -x c++ -c $< -o $@

$(BUILD)/obj/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BUILD)/obj/%.o: %.s
	@mkdir -p $(dir $@)
	$(CC) $(CPU) -c $< -o $@

$(BUILD)/app.elf: $(OBJS) $(LDSCRIPT)
	$(CXX) $(LDFLAGS) $(OBJS) $(LIBS) -o $@
	$(SIZE) $@

$(BUILD)/app.bin: $(BUILD)/app.elf
	$(OBJCOPY) -O binary $< $@

$(BUILD)/Update.Bin: $(BUILD)/app.bin gcc/make_update_bin.py
	$(PYTHON) -I gcc/make_update_bin.py $< $@

$(BUILD)/boot_stub.bin: gcc/boot_stub.s
	@mkdir -p $(BUILD)
	$(CC) $(CPU) -nostdlib -Wl,-Ttext=0x08000000 -Wl,-e,stub_reset $< -o $(BUILD)/boot_stub.elf
	$(OBJCOPY) -O binary $(BUILD)/boot_stub.elf $@

# ---------------------------------------------------------------- ST-Link
OOCD := $(OPENOCD) -f gcc/openocd_stlink_f105.cfg $(OOCD_EXTRA)

flash: $(BUILD)/app.bin
	$(OOCD) -c "program $(BUILD)/app.bin verify reset exit $(APP_BASE)"

backup:
	@mkdir -p backup
	$(OOCD) -c "init; reset halt; flash read_bank 0 backup/flash_full.bin; reset run; shutdown"

restore:
	$(OOCD) -c "program backup/flash_full.bin verify reset exit 0x08000000"

flash-stub: $(BUILD)/boot_stub.bin
	$(OOCD) -c "program $(BUILD)/boot_stub.bin verify reset exit 0x08000000"

# new / blank chip: jump stub + app in one session
flash-blank: $(BUILD)/boot_stub.bin $(BUILD)/app.bin
	$(OOCD) -c "init; reset halt" \
	  -c "flash write_image erase $(BUILD)/boot_stub.bin 0x08000000" \
	  -c "flash write_image erase $(BUILD)/app.bin $(APP_BASE)" \
	  -c "verify_image $(BUILD)/boot_stub.bin 0x08000000" \
	  -c "verify_image $(BUILD)/app.bin $(APP_BASE)" \
	  -c "reset run; shutdown"

probe:
	$(OOCD) -c "init; reset halt; flash info 0; stm32f1x options_read 0; reset run; shutdown"

clean:
	rm -rf $(BUILD)

.PHONY: all flash flash-stub flash-blank backup restore probe clean

-include $(OBJS:.o=.d)
