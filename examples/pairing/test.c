/*
 * Copyright (c) 2011, Swedish Institute of Computer Science.
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
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

#include "contiki.h"
#include "lib/random.h"
#include "sys/ctimer.h"
#include "sys/etimer.h"
#include "net/uip.h"
#include "net/uip-ds6.h"

...
#include "simple-udp.h"
#include "dev/button-sensor.h"
...
#define UDP_PORT 1234

#define SEND_INTERVAL		(5 * CLOCK_SECOND)
#define SEND_TIME		(random_rand() % (SEND_INTERVAL))
...

#include <stdio.h>
#include <string.h>

#include "dev/leds.h"

#define UDP_PORT 1234

#define SEND_INTERVAL		(5 * CLOCK_SECOND)
#define SEND_TIME		(random_rand() % (SEND_INTERVAL))

static struct simple_udp_connection broadcast_connection;

/*---------------------------------------------------------------------------*/
PROCESS(broadcast_example_process, "UDP broadcast example process");
AUTOSTART_PROCESSES(&broadcast_example_process);
/*---------------------------------------------------------------------------*/
static void
receiver(struct simple_udp_connection *c,
         const uip_ipaddr_t *sender_addr,
         uint16_t sender_port,
         const uip_ipaddr_t *receiver_addr,
         uint16_t receiver_port,
         const uint8_t *data,
         uint16_t datalen)
{
  /*printf("Data received from: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x (from port %d) on port %d  with length %d\n",
         sender_addr->u16[0],sender_addr->u16[1],sender_addr->u16[2],sender_addr->u16[3],sender_addr->u16[4],sender_addr->u16[5],sender_addr->u16[6],sender_addr->u16[7], 
	 sender_port, receiver_port, datalen);*/
 printf("Broadcast received from: %02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x on port %d\n", 
	sender_addr->u8[0],sender_addr->u8[1],sender_addr->u8[2],sender_addr->u8[3],sender_addr->u8[4],sender_addr->u8[5],sender_addr->u8[6],sender_addr->u8[7], sender_addr->u8[8],sender_addr->u8[9],sender_addr->u8[10],sender_addr->u8[11],sender_addr->u8[12],sender_addr->u8[13],sender_addr->u8[14],sender_addr->u8[15], sender_port);
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(broadcast_example_process, ev, data)
{
  static struct etimer periodic_timer;
  static struct etimer send_timer;
  uip_ipaddr_t addr;
  //uip_ip6addr(&addr, 0xfe80, 0, 0, 0, 0, 0, 0, 1);
  //uip_ip6addr(&addr, 0xfe01, 0, 0, 0, 0, 0, 0, 1);

  PROCESS_BEGIN();

#if PLATFORM_HAS_LEDS
  leds_off(LEDS_ALL);
#endif

  simple_udp_register(&broadcast_connection, UDP_PORT,
                      NULL, UDP_PORT,
                      receiver);

  etimer_set(&periodic_timer, SEND_INTERVAL);
  while(1) {
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&periodic_timer));
    etimer_reset(&periodic_timer);
    etimer_set(&send_timer, SEND_TIME);

    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&send_timer));
    uip_create_linklocal_allnodes_mcast(&addr);
#if PLATFORM_HAS_LEDS
  leds_on(LEDS_ALL);
#endif
    /*printf("Sending broadcast from: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x on port %d\n", 
	addr.u16[0],addr.u16[1],addr.u16[2],addr.u16[3],addr.u16[4],addr.u16[5],addr.u16[6],addr.u16[7], 
	UDP_PORT);*/
    printf("Sending broadcast from: %02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x on port %d\n", addr.u8[0],addr.u8[1],addr.u8[2],addr.u8[3],addr.u8[4],addr.u8[5],addr.u8[6],addr.u8[7], addr.u8[8],addr.u8[9],addr.u8[10],addr.u8[11],addr.u8[12],addr.u8[13],addr.u8[14],addr.u8[15], UDP_PORT);
    simple_udp_sendto(&broadcast_connection, "Test", 4, &addr);
#if PLATFORM_HAS_LEDS
  leds_off(LEDS_ALL);
#endif
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "contiki-net.h"

#include "net/rime.h"

#include "dev/button-sensor.h"
#include "dev/leds.h"

#include <stdio.h>
#include <string.h>

/*---------------------------------------------------------------------------*/
PROCESS(pairing_client_process, "Pairing client process");
AUTOSTART_PROCESSES(&pairing_client_process);
/*---------------------------------------------------------------------------*/
static struct unicast_conn unicast;
static struct broadcast_conn broadcast;

static void
unicast_recv(struct unicast_conn *c, const rimeaddr_t *from)
{
  printf("unicast message received from server with rime-address %d.%d: '%s'\n",
         from->u8[0], from->u8[1], (char *)packetbuf_dataptr());

  // Todo: Adresse korrekt uebergeben
  //memcpy(&uip_lladdr.addr, &from->u8, sizeof(uip_lladdr)); //example: uip_lladdr = {{0x00,0x06,0x98,0x00,0x02,0x32}};
  //memcpy(&uip_lladdr.addr, &from.u8, sizeof(rimeaddr_t));

  unicast_close(&unicast);
  broadcast_close(&broadcast);
}

static void
broadcast_recv(struct broadcast_conn *c, const rimeaddr_t *from)
{
  puts("broadcast received.");
  // clients shouldn't receive broadcast packets in this case
}

static const struct unicast_callbacks unicast_callbacks = {unicast_recv};
static const struct broadcast_callbacks broadcast_callbacks = {broadcast_recv};

static uint8_t pm_active;
static uint32_t pairing_mode_period = 10;
static uint32_t broadcast_period = 2;
static struct etimer et_pairing_mode; // Define the timer
static struct etimer et_broadcast; // Define the timer
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(pairing_client_process, ev, data)
{
  PROCESS_EXITHANDLER(goto exit)

  PROCESS_BEGIN();

  // puts("Client: pairing preparation"); //wird nicht angezeigt, weil nicht in der Prozessschleife

#if PLATFORM_HAS_BUTTON
  SENSORS_ACTIVATE(button_sensor);
  //puts("Press a button to start pairing-mode on client"); //wird nicht angezeigt, weil nicht in der Prozessschleife
#endif
#if PLATFORM_HAS_LEDS
  leds_off(LEDS_ALL);
  leds_on(LEDS_RED);
  //puts("LED will blink, if button pressed"); //wird nicht angezeigt, weil nicht in der Prozessschleife
#endif

pm_active = 0;

  while(1) {
    //PROCESS_WAIT_EVENT();
	PROCESS_PAUSE();

    //if(ev == sensors_event) {  // If the event it's provoked by the user button, then...
      //if(data == &button_sensor) {
	if(button_sensor.value(0) == 1 && pm_active == 0) {		
        leds_on(LEDS_GREEN);
        SENSORS_DEACTIVATE(button_sensor);
	unicast_open(&unicast, 26, &unicast_callbacks);
	broadcast_open(&broadcast, 26, &broadcast_callbacks);
        printf("rime address: %d.%d\n", rimeaddr_node_addr.u8[0], rimeaddr_node_addr.u8[1]);
	puts("Turning on Pairing-mode( green LED ;) ) on client for 10 seconds.");
 	
	// diverse Tests zur Adressermittlung
	/* geht nicht
	uip_ipaddr_t uip_ipaddr;
	uip_getdraddr(&uip_ipaddr);
	printf("ipv4 address: %d.%d.%d.%d\n", uip_ipaddr.u8[0], uip_ipaddr.u8[1], uip_ipaddr.u8[2], uip_ipaddr.u8[3]);
	*/

	//uip_ipaddr_t hostaddr, hostaddr2, hostaddr3;
	// geht nicht	
	//uip_gethostaddr(&hostaddr);
	//printf("ipv4 address: %d.%d.%d.%d\n", hostaddr.u8[0], hostaddr.u8[1], hostaddr.u8[2], hostaddr.u8[3]);

	/*
	puts("manueller Versuch --> lokales setzen einer Adresse: 192.168.1.2");
	uip_ipaddr(&hostaddr, 192,168,1,2);
	printf("ipv4 address: %d.%d.%d.%d\n", hostaddr.u8[0], hostaddr.u8[1], hostaddr.u8[2], hostaddr.u8[3]);
	uip_ipaddr(&hostaddr2, 192,168,1,5);
	uip_sethostaddr(&hostaddr2);
	uip_gethostaddr(&hostaddr3);
	printf("ipv4 address: %d.%d.%d.%d\n", hostaddr3.u8[0], hostaddr3.u8[1], hostaddr3.u8[2], hostaddr3.u8[3]);
	*/

	pm_active = 1;
	etimer_set(&et_pairing_mode, CLOCK_SECOND*pairing_mode_period);  // Set the timer
	etimer_set(&et_broadcast, CLOCK_SECOND*broadcast_period);
      }
    //}

    if(etimer_expired(&et_broadcast) && pm_active == 1) {
	leds_on(LEDS_YELLOW);
	packetbuf_copyfrom("Hello, I'm a node and seeking a new home", 42); // 42 = length of characters + 1 
    	broadcast_send(&broadcast);
	printf("%d.%d: broadcast_send\n", rimeaddr_node_addr.u8[0], rimeaddr_node_addr.u8[1]);
    	//puts("broadcast message sent");
        etimer_reset(&et_broadcast);
	leds_off(LEDS_YELLOW);
    }

    if(etimer_expired(&et_pairing_mode) && pm_active == 1) {  // If the event it's provoked by the timer expiration, then...
      leds_off(LEDS_GREEN);
      SENSORS_ACTIVATE(button_sensor);
      unicast_close(&unicast);
      broadcast_close(&broadcast);
      puts("Turning off Pairing-mode( green LED ;) ) on client.");
      pm_active = 0;
      etimer_stop(&et_broadcast);
      etimer_stop(&et_pairing_mode);
    }
  }

  exit:
  leds_off(LEDS_ALL);
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
