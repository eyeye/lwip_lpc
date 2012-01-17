/**********************************************************************
* $Id$		httprdaw_sa_app.c			2011-11-20
*//**
* @file		httprdaw_sa_app.c
* @brief	Standalone HTTP server app
* @version	1.0
* @date		20. Nov. 2011
* @author	NXP MCU SW Application Team
* 
* Copyright(C) 2011, NXP Semiconductor
* All rights reserved.
*
***********************************************************************
* Software that is described herein is for illustrative purposes only
* which provides customers with programming information regarding the
* products. This software is supplied "AS IS" without any warranties.
* NXP Semiconductors assumes no responsibility or liability for the
* use of the software, conveys no license or title under any patent,
* copyright, or mask work right to the product. NXP Semiconductors
* reserves the right to make changes in the software without
* notification. NXP Semiconductors also make no representation or
* warranty that such application will be suitable for the specified
* use without further testing or modification.
**********************************************************************/

#include "lwip/opt.h"
#include "lwip/sys.h"
#include "lwip/memp.h"
#include "lwip/tcpip.h"
#include "lwip/ip_addr.h"
#include "lwip/netif.h"
#include "lwip/timers.h"
#include "netif/etharp.h"

#if LWIP_DHCP
#include "lwip/dhcp.h"
#endif

#include "lpc177x_8x.h"
#include "lpc17_emac.h"
#include "lpc_arch.h"
#include "lpc_board.h"
#include "lpc_lwip_debug.h"
#include "application_config.h"
#include "lpc_phy.h" /* For the PHY monitor support */
#include "httpd.h"

/** @defgroup httpraw_sa_app	httpd raw server
 * @ingroup lwip_applications
 *
 * This example shows how to use a HTTP server. The example can
 * be built with a static or DHCP obtained IP addresses and copied
 * or zero-copy buffers. Once the system is initialized, a single
 * while(1) loop is used to handle the main system tasks including
 * timers and PHY status. The LED on the board module will indicate
 * if a cable is plugged into the board (on = cable attached).
 * @{
 */

/** \brief  SysTick IRQ user handler and timebase management

    This function is called by the Systick timer when the
	timer count is updated. It does nothing in this application,
	but can be used for the ARP timer or PHY monitoring. 

	\param[in]    ms    Number of milliSconds since the timer was started
 */
void SysTick_User(u32_t ms)
{
	;
}

/** \brief  Application entry point

	\return       Does not return
 */
int main (void)
{
	struct netif lpc_netif;
	ip_addr_t ipaddr, netmask, gw;
	err_t err;
	struct pbuf *p;

	/* Setup board including GPIOs and pin muxing */
	board_setup();
	led_set(0);

	/* Setup a 1mS sysTick for the primary time base */
	SysTick_Enable(1);

#ifdef UART_REDIRECT
	/* Re-direct ouput to selected UART */
	init_redirect();
#endif
#ifdef NULL_REDIRECT
	/* Re-direct ouput to NULL device (nowhere) */
	init_redirect();
#endif

	/* Initialize LWIP */
	err = lwip_init();
	if (err != ERR_OK) {
		if (err = ERR_CONN)
			APP_DEBUG("Ethernet link not yet detected, will continue\n");
		else {
			APP_DEBUG("Error %d initializing LWIP, stopping.\n", err);
			while (1);
		}
	}

	APP_DEBUG("Starting httpd...\n");

	/* Static IP assignment */
#if LWIP_DHCP
	IP4_ADDR(&gw, 0, 0, 0, 0);
	IP4_ADDR(&ipaddr, 0, 0, 0, 0);
	IP4_ADDR(&netmask, 0, 0, 0, 0);
#else
	IP4_ADDR(&gw, 10, 1, 10, 1);
	IP4_ADDR(&ipaddr, 10, 1, 10, 234);
	IP4_ADDR(&netmask, 255, 255, 255, 0);
	APP_PRINT_IP(&ipaddr);
#endif

	/* Add netif interface for lpc17xx_8x */
	netif_add(&lpc_netif, &ipaddr, &netmask, &gw, NULL, lpc_enetif_init,
		ethernet_input);
	netif_set_default(&lpc_netif);
	netif_set_up(&lpc_netif);

    /* Enable MAC interrupts */
    NVIC_SetPriority(ENET_IRQn, ((0x01 << 3) | 0x01));
    NVIC_EnableIRQ(ENET_IRQn);

#if LWIP_DHCP
	dhcp_start(&lpc_netif);
#endif

	/* Initialize and start application */
	httpd_init();

	/* This could be done in the sysTick ISR, but may stay in IRQ context
	   too long, so do this stuff with a background loop. */
	while (1) {
		/* Handle packets as part of this loop, not in the IRQ handler */
		lpc_enetif_input(&lpc_netif);

#if LPC_PBUF_RX_ZEROCOPY
		/* Re-queue RX buffers as needed */
		while (lpc_rx_queue(&lpc_netif));
#endif

#if LPC_PBUF_TX_ZEROCOPY
		/* Free TX buffers that are done sending */
		lpc_tx_reclaim(&lpc_netif);
#endif

		/* LWIP timers - ARP, DHCP, TCP, etc. */
		sys_check_timeouts();

		/* Call the PHY status update state machine once in a while
		   to keep the link status up-to-date */
		if (lpc_phy_sts_sm(&lpc_netif) != 0) {
			/* Set the state of the LED to on if the ethernet link is
			   active or off is disconnected. */
			if (lpc_netif.flags & NETIF_FLAG_LINK_UP)
				led_set(1);
			else
				led_set(0);
		}
	}

	return 1;
}

/**
 * @}
 */

/* --------------------------------- End Of File ------------------------------ */