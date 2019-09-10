#===============================================================
#Makefile for building MSP Code Examples in command line
#environement using the GCC Open Source Compiler for MSP430
#
# Slightly modified version of the original Makefile
# --------------------------------------------------
# created: 15.8.2016
# author: Ondrej Hejda
# description:
# The original Makefile is modified to compile and flash (not
# only the example) code for exp430fr2311 launchpad using
# msp430-gcc and Msp430flasher.
# Makefile should be now located in the same dir as the source
# code. All you have to setup is the path of the msp430_gcc
# (GCC_DIR) and the MSP430Flasher program (FLASHER).
# Running "make" command will create the target directory,
# compile all *.c files into *.o files, link them into *.out
# file and translate the result into the *.hex file.
# Running "make program" will flash the *.hex file into the
# device.
# Running "make clean" will delete the target directory.
#===============================================================

DEVICE=MSP430FR2311
TARGET=target

###################### Windows OS ######################
ifeq ($(OS),Windows_NT)
	GCC_DIR ?= C:/ti/msp430_gcc
	FLASHER := C:/ti/MSPFlasher_1.3.10/MSP430Flasher
################### Linux or Mac OS ####################
else
	GCC_DIR ?= ~/ti/gcc
	FLASHER := MSP430Flasher
endif

GCC_MSP_INC_DIR ?= $(GCC_DIR)/include
RM := rm -rf

######################################
GCC_BIN_DIR     ?= $(GCC_DIR)/bin
GCC_INC_DIR     ?= $(GCC_DIR)/msp430-elf/include
######################################
CC              := $(GCC_BIN_DIR)/msp430-elf-gcc
GDB			    := $(GCC_BIN_DIR)/msp430-elf-gdb
OBJCOPY         := $(GCC_BIN_DIR)/msp430-elf-objcopy
SIZE            := $(GCC_BIN_DIR)/msp430-elf-size
######################################
CFLAGS          := -O2 -D__$(DEVICE)__ -mmcu=$(DEVICE) -g -ffunction-sections -fdata-sections -DDEPRECATED
LDFLAGS         := -L $(GCC_MSP_INC_DIR) -T $(DEVICE).ld -mmcu=$(DEVICE) -g -Wl,--gc-sections
INCLUDES        := -I $(GCC_MSP_INC_DIR) -I $(GCC_INC_DIR)
######################################
SRC             := $(wildcard *.c)
EXOBJECT        := $(patsubst %.c,$(TARGET)/%.o,$(SRC))
EXOUTPUT        := $(TARGET)/$(TARGET).out
EXOUTHEX        := $(TARGET)/$(TARGET).hex
######################################

all: $(EXOUTPUT) $(EXOUTHEX)

$(TARGET):
	@mkdir $(TARGET)

$(EXOBJECT): ${SRC}
	@echo ============================================
	@echo Compiling $(SRC)
	$(CC) $(INCLUDES) $(CFLAGS) -c $< -o $@

$(EXOUTPUT): $(TARGET) $(EXOBJECT)
	@echo ============================================
	@echo Linking objects and generating output binary
	$(CC) $(LDFLAGS) $(EXOBJECT) -o $@
	
$(EXOUTHEX): $(TARGET) $(EXOBJECT) $(EXOUTPUT)
	@echo ============================================
	@echo Translating .out file into .hex file
	$(OBJCOPY) -O ihex $(EXOUTPUT) $(EXOUTHEX)
	@echo ============================================
	@echo Compiled binary size
	$(SIZE) $(EXOUTPUT) $(EXOUTHEX)

debug: all
	$(GDB) $(EXOUTPUT)

clean:
	@$(RM) $(TARGET)
	
program: all
	@echo ============================================
	@echo Flashing the .hex binary into the device
	$(FLASHER) -i TIUSB -m SBW2 -g -n $(DEVICE) -e ERASE_ALL -w $(EXOUTHEX) -v -z [VCC]