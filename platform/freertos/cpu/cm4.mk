#
# cpu/cm4.mk - CPU-specific Makefile for Cortex M4
#
# Items lifted from contiki/cpu/arm/stm32-ocm3/Makefile.stm32-ocm3
#

ARCH_FLAGS = -mcpu=cortex-m4 -mthumb -march=armv7e-m \
             -mfloat-abi=hard -mfpu=fpv4-sp-d16

TARGETLIBS = random.c

CONTIKI_TARGET_SOURCEFILES += $(STM32OCM3) $(TARGETLIBS)

PREFIX = arm-none-eabi
