##
# @file   Makefile
# @author cy023@MVMC-Lab (cyyang023@gmail.com)
# @date   2022.07.23
# @brief  Makefile for SAME54_GCC project.

################################################################################
# User Settings
################################################################################

## Target
TARGET = main

# Upload Info.
COMPORT    ?=
# UPLOAD_HEX ?= main
# UPLOAD_HEX ?= test_00_uart
# UPLOAD_HEX ?= test_01_gpio
# UPLOAD_HEX ?= test_02_bootjump
# UPLOAD_HEX ?= test_03_delay
# UPLOAD_HEX ?= test_04_flash
UPLOAD_HEX ?= test_05_littlefs
# UPLOAD_HEX ?= test_06_fs_rw_image

## MCU Info.
CPU       = -mcpu=cortex-m4
MCU       = atsame54p20a
# FPU       = -mfpu=fpv4-sp-d16
# FLOAT-ABI = -mfloat-abi=softfp

## Warning Options
WARNINGS = -Wall -Werror -Wtype-limits -Wno-unused-function

## Debugging Options
DEBUG = -gdwarf-2

## Optimize Options
OPTIMIZE = -O0

## Libraries Path
LIBS    = -lc -lgcc -lm
LIBDIRS = 

## Include Path
C_INCLUDES  = -I.
C_INCLUDES += -ICore
C_INCLUDES += -ICore/boot
C_INCLUDES += -IDrivers/boot
C_INCLUDES += -IDrivers/CMSIS
C_INCLUDES += -IDrivers/SAME54_DFP
C_INCLUDES += -IDrivers/w25q128jv
C_INCLUDES += -IMiddleware/LittleFS

## Source Path
C_SOURCES += $(wildcard Device_Startup/*.c)
C_SOURCES += $(wildcard Drivers/boot/*.c)
C_SOURCES += $(wildcard Drivers/w25q128jv/*.c)
C_SOURCES += $(wildcard Middleware/LittleFS/*.c)

################################################################################
# Project Architecture
################################################################################

## Root Path
IPATH = .

## Build Output Path
BUILD_DIR = build

## Build Reference Path
VPATH  = $(sort $(dir $(C_SOURCES)))
VPATH += $(sort $(dir $(C_APPSRCS)))
VPATH += $(sort $(dir $(C_TESTSRC)))

## User App Path
C_APPSRCS = $(wildcard Core/*.c)
C_APPSRCS += $(wildcard Core/boot/*.c)

## Unit Test Path
C_TESTSRC = $(wildcard UnitTest/*.c)

## Object Files
C_OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
C_APPOBJS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_APPSRCS:.c=.o)))
C_TESTOBJ = $(addprefix $(BUILD_DIR)/,$(notdir $(C_TESTSRC:.c=.o)))

## Target and UnitTest HEX Files
TARGET_FILE = $(BUILD_DIR)/$(TARGET).hex
TEST_TARGET = $(C_TESTOBJ:.o=.hex)

### Linker script
LDSCRIPT = Device_Startup/same54p20a_flash.ld

################################################################################
# Toolchain
################################################################################
COMPILER_PATH ?= 
CROSS   := $(COMPILER_PATH)arm-none-eabi
CC      := $(CROSS)-gcc
AR      := $(CROSS)-ar
SIZE    := $(CROSS)-size
OBJDUMP := $(CROSS)-objdump
OBJCOPY := $(CROSS)-objcopy
NM      := $(CROSS)-nm

## Common Options for assemble, compile and link.
MCUFLAGS = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

## Compile Options
CFLAGS  = $(MCUFLAGS)
CFLAGS += -D__SAME54P20A__
CFLAGS += -std=gnu99 $(WARNINGS) $(DEBUG) $(OPTIMIZE)
# CFLAGS += -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums 
CFLAGS += -funsigned-char -funsigned-bitfields -fshort-enums -ffunction-sections
CFLAGS += -Wa,-a,-ad,-alms=$(@:%.o=%.lst)
CFLAGS += -Wp,-MM,-MP,-MT,$(BUILD_DIR)/$(*F).o,-MF,$(BUILD_DIR)/$(*F).d
CFLAGS += $(C_INCLUDES)

## Assembler Options
ASMFLAGS  = $(MCUFLAGS)
ASMFLAGS += -x assembler-with-cpp -Wa,-g$(DEBUG)
ASMFLAGS += $(CFLAGS)

## Link Options
LDFLAGS  = $(MCUFLAGS)
LDFLAGS += -Wl,-Map=$(BUILD_DIR)/$*.map,--cref -Wl,--gc-sections
LDFLAGS += -T$(LDSCRIPT)
LDFLAGS += -specs=nosys.specs -specs=nano.specs -flto
LDFLAGS += -u _printf_float
LDFLAGS += $(LIBDIRS) $(LIBS)

## Archiver Options
# ARFLAGS = rcs

## Intel Hex file production Options
HEX_FLASH_FLAGS   = -R .eeprom -R .fuse -R .lock -R .signature
HEX_EEPROM_FLAGS  = -j .eeprom
HEX_EEPROM_FLAGS += --set-section-flags=.eeprom=alloc,load
HEX_EEPROM_FLAGS += --change-section-lma .eeprom=0 --no-change-warnings

################################################################################
# User Command
################################################################################

all: $(C_OBJECTS) $(C_APPOBJS) $(TARGET_FILE)
	@echo "========== SIZE =========="
	$(SIZE) $^

test: $(C_OBJECTS) $(C_TESTOBJ) $(TEST_TARGET)
	@echo "========== SIZE =========="
	$(SIZE) $^

macro: $(C_OBJECTS:.o=.i) $(C_APPOBJS:.o=.i) $(C_TESTOBJ:.o=.i)

lib: $(BUILD_DIR)/libc4mrtos.a

size: $(TARGET_FILE)
	$(SIZE) $(TARGET_FILE)

clean:
	-rm -rf $(BUILD_DIR)

upload:
	asaloader prog -p $(COMPORT) -f build/$(UPLOAD_HEX).hex

terminal:
	putty -serial $(COMPORT) -sercfg 38400,1,N,N

systeminfo:
	@uname -a
	@$(CC) --version

.PHONY: all test macro lib size systeminfo clean upload terminal

################################################################################
# Build The Project
################################################################################

## Preprocess
$(BUILD_DIR)/%.i: %.c Makefile | $(BUILD_DIR)
	$(CC) -E -D__SAME54P20A__ $(C_INCLUDES) $< -o $@

## Compile
$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(CC) -c $(ASMFLAGS) $< -o $@

## Link
$(TARGET_FILE:.hex=.elf): $(C_OBJECTS) $(C_APPOBJS) Makefile
	$(CC) $(LDFLAGS) $(C_OBJECTS) $(C_APPOBJS) -o $@

$(TEST_TARGET:.hex=.elf): $(C_OBJECTS) $(C_TESTOBJ) Makefile
	$(CC) $(LDFLAGS) $(C_OBJECTS) $(@:%.elf=%.o) -o $@

## Intel HEX format
$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(OBJCOPY) -O ihex $(HEX_FLASH_FLAGS) $< $@

## Binary format
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(OBJCOPY) -O binary $< $@

## Archive
# $(BUILD_DIR)/libc4mrtos.a: $(C_OBJECTS)
# 	$(AR) rcs $@ $(C_OBJECTS)

## Eeprom format
$(BUILD_DIR)/%.eep: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(OBJCOPY) $(HEX_EEPROM_FLAGS) -O binary $< $@ || exit 0

## Disassemble
$(BUILD_DIR)/%.lss: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(OBJDUMP) -h -S $< > $@

## Symbol Table
$(BUILD_DIR)/%.sym: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(NM) -n $< > $@

## Make Directory
$(BUILD_DIR):
	mkdir $@

################################################################################
# dependencies
################################################################################

-include $(wildcard $(BUILD_DIR)/*.d)
