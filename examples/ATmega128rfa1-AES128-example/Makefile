#CONTIKI_PROJECT = aes
all: aes-example
#broadcast-example broadcast-example-2 unicast-sender unicast-receiver
APPS=servreg-hack
 
ifndef TARGET
TARGET=avr-atmega128rfa1
endif

CONTIKI_SOURCEFILES += atmega128rfa1-aes.c
#=======================================================================
# AVRDUDE Settings
#=======================================================================

# custom avrdude command
AVRDUDE = sudo avrdude
AVRDUDE_PORT = usb
AVRDUDE_PROGRAMMER = dragon_jtag

#=======================================================================
# IPv6 Config
#=======================================================================

# variable for root Makefile.include
WITH_UIP6=1

# for some platforms
UIP_CONF_IPV6=1

CFLAGS+= -DUIP_CONF_IPV6_RPL

#all: $(CONTIKI_PROJECT)
CONTIKI = ../..
include $(CONTIKI)/Makefile.include
#=======================================================================
# RULES: make u | make eu | make program
#=======================================================================

# shortcuts to download the project to the microcontroller board
u: $(CONTIKI_PROJECT).$(TARGET).u
.PHONY: u
eu: $(CONTIKI_PROJECT).$(TARGET).eu
.PHONY: eu
program: u
.PHONY: program

# setup rs232 port in linux with cat
# use: make cat TTY=/dev/ttyUSB0 BAUD=9600
# or change default values and use: make cat
ifndef $(TTY)
TTY = /dev/ttyUSB0
endif
ifndef $(BAUD)
BAUD = 38400
endif
cat:
	@echo "TTY: $(TTY)    BAUD: $(BAUD)"
	@stty -F $(TTY) raw
	@stty -F $(TTY) -echo -echoe -echok
	@stty -F $(TTY) $(BAUD)
	@cat $(TTY)
.PHONY: cat
