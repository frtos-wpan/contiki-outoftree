/*
 * shell-wpan.c - WPAN transceiver configuration
 *
 * Developed by Werner Almesberger for Actility S.A., and
 * licensed under LGPLv2 by Actility S.A.
 */

#include <stdio.h>

#include "contiki.h"
#include "radio.h" /* for struct radio_driver */
#include "rf230bb.h"
#include "shell.h"


extern uint8_t eui64[8];	/* defined by FreeRTOS */


PROCESS(shell_wpan_process, "wpan");
SHELL_COMMAND(wpan_command, "wpan",
    "wpan channel [pan [short-addr]]: configure WPAN transceiver",
    &shell_wpan_process);


PROCESS_THREAD(shell_wpan_process, ev, data)
{
	const char *args;
	unsigned chan, pan, saddr;
	int n;

	PROCESS_BEGIN();

	args = data;
	n = sscanf(args, "%u %x %x\n", &chan, &pan, &saddr);

	switch (n) {
	case 2:
		saddr = 0xffff;	/* unconfigured */
		/* fall through */
	case 3:
		rf230_set_pan_addr(pan, saddr, eui64);
		/* fall through */
	case 1:
		rf230_set_channel(chan);
		break;
	default:
		shell_output_str(&wpan_command, "usage error", "");
		PROCESS_EXIT();
	}

	PROCESS_END();
}


void shell_wpan_init(void)
{
	shell_register_command(&wpan_command);
}
