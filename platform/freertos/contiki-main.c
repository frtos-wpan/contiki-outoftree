/*
 * Copyright (c) 2002, Adam Dunkels.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * This file is part of the Contiki OS
 *
 */

#include <stdio.h>
#include <string.h>

#include "contiki.h"
#include "contiki-net.h"

#include "dev/serial-line.h"

#include "rf230bb.h"


extern uint8_t eui64[8];	/* defined by FreeRTOS */

/*
 * We deliberately don't use defaults here. If someone doesn't set a value,
 * they get a compile-time error, which will make it clear that something is
 * missing, and where it is. This will probably save them a lot of time
 * trying to figure out where all these strange values come from.
 */
 
const static uint8_t rf_channel = RF_CHANNEL;
const static uint16_t pan_addr = PAN_ADDR;
const static uint16_t short_addr = SHORT_ADDR;

/*---------------------------------------------------------------------------*/
static void
set_rime_addr(void)
{
  rimeaddr_t addr;
  int i;

  memset(&addr, 0, sizeof(rimeaddr_t));
#if UIP_CONF_IPV6
  memcpy(addr.u8, eui64, sizeof(addr.u8));
#else
  if(node_id == 0) {
    for(i = 0; i < sizeof(rimeaddr_t); ++i) {
      addr.u8[i] = eui64[7 - i];
    }
  } else {
    addr.u8[0] = node_id & 0xff;
    addr.u8[1] = node_id >> 8;
  }
#endif
  rimeaddr_set_node_addr(&addr);
  printf("Rime started with address ");
  for(i = 0; i < sizeof(addr.u8) - 1; i++) {
    printf("%d.", addr.u8[i]);
  }
  printf("%d\n", addr.u8[i]);
}


/*---------------------------------------------------------------------------*/

void contiki_main(void)
{
#if UIP_CONF_IPV6
#if UIP_CONF_IPV6_RPL
  printf(CONTIKI_VERSION_STRING " started with IPV6, RPL\n");
#else
  printf(CONTIKI_VERSION_STRING " started with IPV6\n");
#endif
#else
  printf(CONTIKI_VERSION_STRING " started\n");
#endif

  process_init();
  process_start(&etimer_process, NULL);
  ctimer_init();

  set_rime_addr();

  queuebuf_init();

  netstack_init();

  printf("MAC %s RDC %s NETWORK %s\n", NETSTACK_MAC.name, NETSTACK_RDC.name,
    NETSTACK_NETWORK.name);

#ifdef HAVE_RF230
  rf230_set_channel(RF_CHANNEL);
  rf230_set_pan_addr(PAN_ADDR, short_addr, eui64);
#endif

#if WITH_UIP6
  memcpy(&uip_lladdr.addr, eui64, sizeof(uip_lladdr.addr));

  process_start(&tcpip_process, NULL);
  process_start(&resolv_process, NULL);
  printf("Tentative link-local IPv6 address ");
  {
    uip_ds6_addr_t *lladdr;
    int i;
    lladdr = uip_ds6_get_link_local(-1);
    for(i = 0; i < 7; ++i) {
      printf("%02x%02x:", lladdr->ipaddr.u8[i * 2],
             lladdr->ipaddr.u8[i * 2 + 1]);
    }
    /* make it hardcoded... */
    lladdr->state = ADDR_AUTOCONF;

    printf("%02x%02x\n", lladdr->ipaddr.u8[14], lladdr->ipaddr.u8[15]);
  }
#else
  process_start(&tcpip_process, NULL);
#endif

  serial_line_init();

  autostart_start(autostart_processes);

  while(1) {
    if (process_run())
      /* delay 1 us */;
    else
      /* delay 1 s */;
    etimer_request_poll();

  }
}
/*---------------------------------------------------------------------------*/
void
log_message(char *m1, char *m2)
{
  fprintf(stderr, "%s%s\n", m1, m2);
}
/*---------------------------------------------------------------------------*/
void
uip_log(char *m)
{
  fprintf(stderr, "%s\n", m);
}
/*---------------------------------------------------------------------------*/
