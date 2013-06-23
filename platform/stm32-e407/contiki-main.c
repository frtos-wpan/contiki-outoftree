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

#include "net/netstack.h"


static void
configure_mcu_clocks(void)
{
	rcc_clock_setup_hse_3v3(hse_12mhz_3v3+CLOCK_3V3_48MHZ);
}

int
main()
{
  configure_mcu_clocks();
  uart_init(115200);
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
