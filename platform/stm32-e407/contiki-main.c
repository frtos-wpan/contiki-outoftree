#include <stdint.h>
#include <stdio.h>
#include <debug-uart.h>
#include <sys/autostart.h>
#include <sys/clock.h>
#include <sys/ctimer.h>
#include <sys/etimer.h>
#include <sys/process.h>
#include <sys/procinit.h>
#include <dev/serial-line.h>
#include <dev/leds.h>

#include "contiki-conf.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/cortex.h>

#include "net/netstack.h"


#define	OSC	12000000	/* 12 MHz crystal */


static void
configure_mcu_clocks(void)
{
	rcc_clock_setup_hse_3v3(hse_12mhz_3v3+CLOCK_3V3_48MHZ);
}


static void
show_mcu_clocks(void)
{
	unsigned pllm = RCC_PLLCFGR & 63;
	unsigned plln = (RCC_PLLCFGR >> 6) & 511;
	unsigned pllp = (RCC_PLLCFGR >> 16) & 3;
	unsigned hpre = (RCC_CFGR >> 4) & 15;
	unsigned hdiv = hpre < 8 ? 1 : 1 << (hpre-7);
	unsigned osc = OSC;
	unsigned vco_in = osc/pllm;
	unsigned vco_out = vco_in*plln;
	unsigned pll_out = vco_out/(2*pllp+2);
	unsigned ahb = pll_out/hdiv;

	printf("PLL M %u N %u P %u HPRE %u (%u)\n",
	    pllm, plln, pllp, hpre, hdiv);
	printf("OSC %u VCOin %u VCOout %u sys %u AHB %u\n",
	    osc, vco_in, vco_out, pll_out, ahb);
}


int
main()
{
  configure_mcu_clocks();
  uart_init(115200);
  show_mcu_clocks();
  printf("Platform init complete\n");

  clock_init();
  leds_init();
  process_init();

//hal_test();
#if WITH_SERIAL_LINE_INPUT
  uart_set_input(serial_line_input_byte);
  serial_line_init();
#endif
  process_start(&etimer_process, NULL);
  ctimer_init();
  autostart_start(autostart_processes);

  /* see ../native/contiki-main.c */
  netstack_init();

  while(1) {
    do {
	// meant to do some sleeping here, if we want to save power...
	//
    } while(process_run() > 0);
  }
  return 0;
}
