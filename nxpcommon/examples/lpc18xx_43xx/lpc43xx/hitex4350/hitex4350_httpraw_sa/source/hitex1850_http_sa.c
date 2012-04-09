/**********************************************************************
* $Id$		hitex1850_http_sa.c			2011-11-20
*//**
* @file		hitex1850_http_sa.c
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

#include "lpc43xx.h"
#include "lpc18xx_43xx_emac.h"
#include "debug_frmwrk.h"
#include "lpc_arch.h"
#include "lpc_board.h"
#include "lpc_phy.h" /* For the PHY monitor support */
#include "httpd.h"

/** @defgroup hitex1850_http_sa	HTTP server (standalone)
 * @ingroup HITEX4350
 *
 * This example shows how to use a HTTP server without an RTOS.
 * @{
 */

/** \brief  NETIF data
 */
static struct netif lpc_netif;

/* Sets up system hardware */
static void prvSetupHardware(void)
{
	SystemInit();
	CGU_Init();
	fpuInit();
	fpuEnable();

	/* Setup board including GPIOs and pin muxing */
	board_setup();
	led_set(0);

	/* Initialize debug output via serial port */
	debug_frmwrk_init();

	/* Setup a 1mS sysTick for the primary time base */
	SysTick_Enable(1);
}

/** \brief  Application entry point
 *
 * \return       Does not return
 */
int main (void)
{
	ip_addr_t ipaddr, netmask, gw;

	prvSetupHardware();

	/* Initialize LWIP */
	lwip_init();

	LWIP_DEBUGF(LWIP_DBG_ON, ("Starting LWIP HTTP server...\n"));

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

		/* lpc_rx_queue will re-qeueu receive buffers. This normally occurs
		   automatically, but in systems were memory is constrained, pbufs
		   may not always be able to get allocated, so this function can be
		   optionally enabled to re-queue receive buffers. */
#if 0
		while (lpc_rx_queue(&lpc_netif));
#endif

		/* Free TX buffers that are done sending */
		lpc_tx_reclaim(&lpc_netif);

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

	/* Should never arrive here */
	return 1;
}

/**
 * @}
 */

/* --------------------------------- End Of File ------------------------------ */
