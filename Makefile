
F_CPU=14745600UL
#F_CPU=18432000UL
USART_BAUD=19200UL
MCU=atmega88
CC=avr-gcc

DEFINES=-DF_CPU=$(F_CPU) -DUSART_BAUD=$(USART_BAUD) -DBOOTADDR=$(BOOTADDR)
CFLAGS= -mmcu=$(MCU) -Wall -funsigned-char
OPTFLAG=-Os -flto -fwhole-program



# Program for creating the lookup tables.
CREATETABLES=createtables

OBJECTS=\
	main.o\
	globals.o\
	init.o\
	serial.o\
	speed.o\
	interrupts.o

ifeq ($(DISPLAY),1)

OBJECTS += vfd.o display.o tables.o
CFLAGS += -DFM_DISPLAY

endif

BIN=pumpcontrol.bin
HEX=pumpcontrol.hex

# Memory sections
#SHAREDSECT=-Wl,--section-start=.blsect=0x008004FF
SHAREDSECT=


# Bootloader data
# # http://www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&t=35869
BOOTBIN=bootloader.bin
BOOTHEX=bootloader.hex

COMBHEX=combined.hex


# 128 words (256 bytes)
BOOTSIZE=512
# Size of the EEPROM, in bytes
EEPROM_SZ=256

ifeq ($(BOOTSIZE), 128)
# Word 0xF80
BOOTADDR = 0x1F00
BOOTAPPEND= 0x1EFE
endif
ifeq ($(BOOTSIZE), 256)
# Word 0xF00
BOOTADDR = 0x1E00
BOOTAPPEND=0x1DFE
endif
ifeq ($(BOOTSIZE), 512)
#Word 0xE00
BOOTADDR = 0x1C00
BOOTAPPEND=0x1BFE
endif
ifeq ($(BOOTSIZE), 1024)
#Word 0xC00
BOOTADDR = 0x1800
BOOTAPPEND=0x17FE
endif

# Datasheet, 27.5
# 	Page size 32 words (64 bytes)
# 	Mega88 has 128 Pages.
BOOTFLAGS=  -DBOOTAPPEND=$(BOOTAPPEND) -DEEPROM_SZ=$(EEPROM_SZ)

# AvrDude options
DUDE_PORT=/dev/stk500
DUDE_PROG=stk500v2
DUDE_PART=$(MCU)

.SUFFIXES: .o .c
.PHONY: program all clean progfuse boot comb dump

all: tables.c $(OBJECTS) $(HEX)

$(HEX): $(BIN)
	avr-objcopy -O ihex $(BIN) $(HEX)
	avr-size -C --mcu=$(MCU) $(BIN)

#avr-objcopy -j .bss -j .text -j .data -O ihex $(BIN) $(HEX)

$(BIN): $(OBJECTS)
	$(CC) $(DEFINES) $(CFLAGS) $(OPTFLAG) $(SHAREDSECT) -o $(BIN) $(OBJECTS)

.c.o:
	$(CC) $(DEFINES) $(CFLAGS) $(OPTFLAG) -c $< -o $@

# If this barfs with a can't open device error, you may need to add yourself
# to the user group for opening tty devices (In fedora, dialout)
program: $(HEX)
	avrdude -p $(DUDE_PART) -c $(DUDE_PROG) -P $(DUDE_PORT) -U flash:w:$(HEX)

boot: $(BOOTHEX)

# Build the bootloader
$(BOOTHEX): $(BOOTBIN)
	avr-objcopy -j .text -O ihex $(BOOTBIN) $(BOOTHEX)
	avr-size -C --mcu=$(MCU) $(BOOTBIN)

bootloader.o: bootloader.c
	$(CC) $(DEFINES) $(BOOTFLAGS) $(CFLAGS) -Os -c bootloader.c -o bootloader.o

$(BOOTBIN): bootloader.o
	$(CC) $(DEFINES) $(BOOTFLAGS) $(CFLAGS) $(SHAREDSECT) -Os -Wl,--section-start=.text=$(BOOTADDR) -o $(BOOTBIN) bootloader.o

$(COMBHEX): $(HEX) $(BOOTHEX)
	srec_cat  pumpcontrol.hex -intel bootloader.hex -intel -o combined.hex -intel

$(CREATETABLES): createtables.c
	gcc -lm -o $(CREATETABLES) -Wall -O2 createtables.c

tables.c: $(CREATETABLES)
	./$(CREATETABLES)

dump: all
	avr-objdump  -S -C -d $(BIN) > dump.s

# program the bootloader
progboot: $(COMBHEX)
	avrdude -p $(DUDE_PART) -c $(DUDE_PROG) -P $(DUDE_PORT) -U flash:w:$(COMBHEX)

# Program the fuses for the ATMEGA88
# Change accordingly if you are using a different part!
LFUSE=D6
HFUSE=DF
EFUSE=03
# http://www.frank-zhao.com/fusecalc/fusecalc.php?chip=atmega88&LOW=D6&HIGH=DF&EXTENDED=04&LOCKBIT=FF
progfuse:
	avrdude -p $(DUDE_PART) -c $(DUDE_PROG) -P $(DUDE_PORT) -U lfuse:w:0x$(LFUSE):m
	avrdude -p $(DUDE_PART) -c $(DUDE_PROG) -P $(DUDE_PORT) -U hfuse:w:0x$(HFUSE):m
	avrdude -p $(DUDE_PART) -c $(DUDE_PROG) -P $(DUDE_PORT) -U efuse:w:0x$(EFUSE):m

clean:
	rm -f $(OBJECTS) $(HEX) $(BIN) $(BOOTBIN) $(BOOTHEX) bootloader.o combined.hex $(CREATETABLES)
	rm -f createtables.o tables.h tables.c

dumpflash:
	avrdude -p $(DUDE_PART) -c $(DUDE_PROG) -P $(DUDE_PORT) -U flash:r:dump.hex:i

fuseurl:
	@echo "http://www.frank-zhao.com/fusecalc/fusecalc.php?chip=atmega88&LOW=$(LFUSE)&HIGH=$(HFUSE)&EXTENDED=$(EFUSE)&LOCKBIT=FF"
# misc

comb: all $(COMBHEX)
