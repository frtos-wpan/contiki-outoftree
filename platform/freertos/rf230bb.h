/*
 * rf230bb.h - Ugly hack to force use of our hal.h
 *
 * Developed by Werner Almesberger for Actility S.A., and
 * licensed under LGPLv2 by Actility S.A.
 */

#include <hal.h>
#define HAL_AVR_H
#include "contiki/cpu/avr/radio/rf230bb/rf230bb.h"
