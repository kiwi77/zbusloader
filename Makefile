CC=avr-gcc
OBJCOPY=avr-objcopy
STRIP=avr-strip
SIZE=avr-size
AVRDUDE=avrdude

#CPPFLAGS += -mmcu=atmega8 -DF_CPU=8000000UL 
#CFLAGS += -std=gnu99 -Os -g -Wall -W
#LDFLAGS += $(CFLAGS) -nostdlib -Wl,--section-start=.text=0x1E00
CPPFLAGS += -mmcu=atmega169 -DF_CPU=8000000UL 
CFLAGS += -std=gnu99 -Os -g -Wall -W -mmcu=atmega169
LDFLAGS += $(CFLAGS) -nostdlib -Wl,--section-start=.text=0x3C00


all: zbusloader.hex

zbusloader: zbusloader.o avr_init.o
	$(CC) -o $@ $(LDFLAGS) $^
	$(SIZE) $@

clean:
	rm -f zbusloader *.o *.s *.hex *~

%.hex: %
	$(OBJCOPY) -O ihex -R .eeprom $< $@

load: zbusloader.hex
	$(AVRDUDE) -p m8 -U flash:w:$<

fuse:
	$(AVRDUDE) -p m8 -U lfuse:w:0xa4:m
	$(AVRDUDE) -p m8 -U hfuse:w:0xdc:m

.PHONY:	fuse load clean
