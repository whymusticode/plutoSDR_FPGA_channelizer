# Cross-compile the on-Pluto channel apps with the firmware build's toolchain.
#   make                      -> build/<app> for every src/<app>.c
#   make BUILDROOT=<path>     -> another Buildroot output (default: this repository's)
BUILDROOT ?= $(abspath ../../buildroot/output)
CC := $(BUILDROOT)/host/bin/arm-linux-gnueabihf-gcc
SYSROOT := $(BUILDROOT)/staging
CFLAGS := -O2 -mcpu=cortex-a9 -mfpu=neon -mfloat-abi=hard -ffast-math -Wall --sysroot=$(SYSROOT)
LDLIBS := -liio -lm -lpthread

APPS := $(filter-out pfbapp,$(basename $(notdir $(wildcard src/*.c))))

all: $(addprefix build/,$(APPS))

build/%: src/%.c src/pfbapp.c src/pfbapp.h | build
	$(CC) $(CFLAGS) -o $@ $< src/pfbapp.c $(LDLIBS)

build:
	mkdir -p $@

clean:
	rm -rf build

.PHONY: all clean
