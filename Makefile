PROJECT   = servo_test
EXTENSION = .bin
OUTPUTDIR =
LIBRARIES =
SOURCES   = $(PROJECT).cpp ArduinoPiGpio.cpp

#
# DEBUG 0=off 1=on
#
ifndef DEBUG
DEBUG = 1
endif

CXX = g++

CCFLAGS = -Wall -I./libraries/MCP23S17/ -I./
CRFLAGS = -O2
CDFLAGS = -DDEBUG

ifeq ($(DEBUG),0)
CFLAGS = $(CCFLAGS) $(CRFLAGS)
else
CFLAGS = $(CCFLAGS) $(CDFLAGS)
endif

LCFLAGS =
LRFLAGS =
LDFLAGS =

ifeq ($(DEBUG),0)
LFLAGS = $(LCFLAGS) $(LRFLAGS)
else
LFLAGS = $(LCFLAGS) $(LDFLAGS)
endif

ERROR_FILE = $(CXX)_error.log

help:
	@echo "To compile type: make all"
	@echo "To remove the executable type: make clean"
	@echo "To show the last compile error type: make error"

all: prepare $(PROJECT)$(EXTENSION)
	@cat $(ERROR_FILE) ||:

prepare:
	@echo "Compiling '$(PROJECT)'..."

$(PROJECT)$(EXTENSION):
	-$(CXX) $(CFLAGS) $(LFLAGS) $(SOURCES) -o $@ 2> $(ERROR_FILE)

clean:
	-@rm $(PROJECT)$(EXTENSION) ||:
	-@rm $(ERROR_FILE) ||:

error:
	@cat $(ERROR_FILE)
