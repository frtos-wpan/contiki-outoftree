/*
 * rtimer-arch.h - Architecture-specific rtimer support
 *
 * Developed by Werner Almesberger for Actility S.A., and
 * licensed under LGPLv2 by Actility S.A.
 */


#ifndef RTIMER_ARCH_H
#define RTIMER_ARCH_H

#include <stdint.h>


#define	RTIMER_ARCH_SECOND	1000 /* from contiki-conf.h */


typedef uint32_t rtimer_clock_t;


extern rtimer_clock_t rtimer_arch_second;


void rtimer_arch_init(void);
rtimer_clock_t rtimer_arch_now(void);
void rtimer_arch_schedule(rtimer_clock_t wakeup_time);

#endif /* RTIMER_ARCH_H */
