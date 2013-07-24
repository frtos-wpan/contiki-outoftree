/*
 * Copyright (c) 2013, Karl Palsson <karlp@tweak.net.au>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/**
 *
 * \file
 *         A set of tests for all the timer and clock routines.
 * Requires host side timestamping to help validate.  Partly based on the cc253x
 * timer test routines
 */
#include "contiki.h"
#include "sys/clock.h"
#include "sys/rtimer.h"

#include <stdio.h>

PROCESS(tests, "timer and clock test routines");
AUTOSTART_PROCESSES(&tests);

PROCESS_THREAD(tests, ev, data)
{
	static int tick_count = 10;
	static int ticks = 0;
	int i, k;
	static struct etimer et; // Define the timer
	PROCESS_BEGIN();


	/* etimer and ctimer are both built on the same base */
	printf("TESTSTART{\"expected\":%d}\n", tick_count * 100);
	etimer_set(&et, 100);
	for (ticks = 0; ticks < tick_count; ticks++) {
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
		printf("etimer tick %d/%d\n", ticks, tick_count);
		etimer_reset(&et);
	}
	printf("TESTEND\n");


	/* clock_delay_usec is important for many hardware drivers */
	printf("TESTSTART{\"expected\":%d}\n", tick_count * 100);
	for (ticks = 0; ticks < tick_count; ticks++) {
		for (i = 0; i < 100; i++) {
			clock_delay_usec(1000);
		}
		printf("clock_delay_usec: 100 * 1000usecs, %d/%d\n", ticks, tick_count);
	}
	printf("TESTEND\n");

	/*
	 * timer is built on clock_wait, no need to test it specially,
	 * test clock_seconds at the same time, used by stimers
	 */
	printf("TESTSTART{\"expected\":3000}\n");
	i = clock_seconds();
	clock_wait(3 * CLOCK_SECOND);
	k = clock_seconds();
	printf("TESTEND\n");
	printf("clock_seconds delta=%d, expected %d (before: %d, after: %d)\n",
		k - i, 3, i, k);
	
	PROCESS_END();
}

