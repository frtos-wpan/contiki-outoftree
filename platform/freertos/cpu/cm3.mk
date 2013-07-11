#
# cpu/cm3.mk - CPU-specific Makefile for Cortex M3
#

ARCH_FLAGS = -mcpu=cortex-m3 -mthumb -march=armv7-m

STM32OCM3 = clock.c
TARGETLIBS = random.c

CONTIKI_TARGET_SOURCEFILES += $(TARGETLIBS)

PREFIX = arm-none-eabi-
