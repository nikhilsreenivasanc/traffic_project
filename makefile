# Makefile for AVR-GCC projects

# MCU target (default: atmega328p)
#MCU = atmega328p
MCU = atmega2560      # Uncomment for ATmega2560

# F_CPU (CPU Frequency in Hz)
 F_CPU = 16000000UL
#F_CPU = 8000000UL  # Uncomment for 8 MHz clock

# Programmer settings
#PROGRAMMER = wiring
#PROGRAMMER = arduino	  // Uncomment for ATmega328p uno/nano
PROGRAMMER = stk500v2    // Uncomment for ATmega2560
#PROGRAMMER = usbtiny
#PROGRAMMER = AVRISP
#PROGRAMMER = avrisp2
#PROGRAMMER = usbasp
#PROGRAMMER = dragon_isp
#PROGRAMMER = jtag2updi
#PROGRAMMER = atmelice_isp
#PROGRAMMER = atmelice_dap

# Source and output names
SRC = main.c mfrc522.c uart.c
TARGET = main

# AVR-GCC options
CFLAGS = -mmcu=$(MCU) -DF_CPU=$(F_CPU) -Os -Wall
LDFLAGS = -mmcu=$(MCU)

# Tools
CC = avr-gcc
OBJCOPY = avr-objcopy

# Default target: build hex and elf
all: $(TARGET).hex $(TARGET).elf
&= ~(1 << UNO_INPUT_PIN);
$(TARGET).elf: $(SRC)
	$(CC) $(CFLAGS) $(LDFLAGS) $^ -o $@

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@

# Flash the device (optional)
flash: $(TARGET).hex
#replace COM3 with your port like /dev/ttyUSB0 on Linux or /dev/cu.usbmodemXXXX on Mac, use $ sudo dmesg.
#stk500v2 for atmega2560, arduino for atmega328p
#place -D in the avrdude command to disable auto erase for arduino MEGA 2560 as it fails to upload sometimes
#replace 115200 with 57600 for atmega328p arduino nano v3 older boards
	avrdude -D -v -p $(MCU) -c $(PROGRAMMER) -P /dev/ttyUSB0 -b 115200 -U flash:w:main.hex

# Clean build files
clean:
	rm -f $(TARGET).elf $(TARGET).hex

.PHONY: all flash clean

