#include <stdbool.h>
#include <unistd.h>

#include "dev/leds.h"


static ssize_t dummy; /* avoid complains about ignoring what write(2) returns */


void leds_toggle(unsigned char leds)
{
	static bool toggle = 0;

	toggle = !toggle;
	dummy = write(2, toggle ? "//" : "\\", 1);
}


void leds_blink(void)
{
	dummy = write(2, "*", 1);
}
