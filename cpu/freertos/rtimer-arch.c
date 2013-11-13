/*
 * rtimer-arch.c - Architecture-specific rtimer support
 *
 * Developed by Werner Almesberger for Actility S.A., and
 * licensed under LGPLv2 by Actility S.A.
 */


#include <stdint.h>
#include <stdio.h>

#include "sys/rtimer.h"
#include "rtimer-arch.h"

#include "freertos-task.h"
#include "freertos-timers.h"


#define	BLOCK_TICKS	100	/* @@@ */


rtimer_clock_t rtimer_arch_second;
static xTimerHandle rtimer;


/*
 * FreeRTOS introduces the novel concept of timers where all operations can
 * potentially fail. Since we don't even have a way to inform our caller when
 * something went wrong, we just add a general "panic" function for such cases.
 */

static void panic(void)
{
	printf("\nPANIC\n");
	while (1);
}


static void rtimer_handler(xTimerHandle xTimer)
{
	rtimer_run_next();
}


void rtimer_arch_init(void)
{
	rtimer_arch_second = xTaskGetTickCountFromISR();
	/*
	 * FreeRTOS has a weird concept of type for "pcTimerName", so we need a
	 * cast. "const char *" would have done nicely.
	 */
	rtimer = xTimerCreate((const signed char * const) "rtimer",
	    1, pdFALSE, NULL, rtimer_handler);
}


rtimer_clock_t rtimer_arch_now(void)
{
	return xTaskGetTickCountFromISR();
}


void rtimer_arch_schedule(rtimer_clock_t wakeup_time)
{
	int32_t d = wakeup_time - xTaskGetTickCountFromISR();

	if (xTimerStop(rtimer, BLOCK_TICKS) == pdFAIL)
		panic();
	if (d <= 0) {
		rtimer_run_next();
		return;
	}
	if (xTimerChangePeriod(rtimer, d, BLOCK_TICKS) == pdFAIL)
		panic();
	if (xTimerStart(rtimer, BLOCK_TICKS) == pdFAIL)
		panic();
}
