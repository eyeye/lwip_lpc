/**********************************************************************
* $Id$		hitex1850_tcpecho_freertos.c			2011-11-20
*//**
* @file		hitex1850_tcpecho_freertos.c
* @brief	RTOS based TCP echo app
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

#include "lpc18xx_libcfg_default.h"
#include "lpc18xx.h"
#include "debug_frmwrk.h"
#include "lpc18xx_43xx_emac.h"
#include "debug_frmwrk.h"
#include "lpc_arch.h"
#include "lpc_board.h"
#include "lpc_phy.h" /* For the PHY monitor support */
#include "tcpecho.h"

/** @defgroup hitex1850_tcpecho_freertos	TCP echo server with FreeRTOS
 * @ingroup HITEX1850
 *
 * This example shows how to use a TCP echo server integrated with FreeRTOS.
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
	//fpuInit();
	//fpuEnable();

	/* Setup board including GPIOs and pin muxing */
	board_setup();
	led_set(0);

	/* Initialize debug output via serial port */
	debug_frmwrk_init();
}

/* Callback for TCPIP thread to indicate TCPIP init is done */
static void tcpip_init_done_signal(void *arg)
{
	/* Tell main thread TCP/IP init is done */
	*(s32_t *) arg = 1;
}

/* LWIP kickoff and PHY link monitor thread */
static portTASK_FUNCTION( vSetupIFTask, pvParameters )
{
	ip_addr_t ipaddr, netmask, gw;
	volatile s32_t tcpipdone = 0;
	static int prt_ip = 0;

	/* Wait until the TCP/IP thread is finished before
	   continuing or wierd things may happen */
	//LWIP_DEBUGF(LWIP_DBG_ON, ("Waiting for TCPIP thread to initialize...\n"));
	lpc_printf("Waiting for TCPIP thread to initialize...\r\n");
	tcpip_init(tcpip_init_done_signal, &tcpipdone);
	while (!tcpipdone);

	//LWIP_DEBUGF(LWIP_DBG_ON, ("Starting LWIP TCP echo server...\n"));
	lpc_printf("Starting LWIP TCP echo server...\r\n");

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
	memset(lpc_netif, 0, sizeof(lpc_netif));
	if (!netif_add(&lpc_netif, &ipaddr, &netmask, &gw, NULL, lpc_enetif_init,
		tcpip_input))
		LWIP_ASSERT("Net interface failed to initialize\r\n", 0);

	netif_set_default(&lpc_netif);
	netif_set_up(&lpc_netif);

  /* Enable MAC interrupts only after LWIP is ready */
	NVIC_SetPriority(ETHERNET_IRQn, ((0x01<<3)|0x01));
  NVIC_EnableIRQ(ETHERNET_IRQn);

#if LWIP_DHCP
	dhcp_start(&lpc_netif);
#endif

	/* Initialize and start application */
	tcpecho_init();

	/* This loop monitors the PHY link and will handle cable events
	   via the PHY driver. */
	while (1)
	{
		/* Call the PHY status update state machine once in a while
		   to keep the link status up-to-date */
		if (lpc_phy_sts_sm(&lpc_netif) != 0) {
			/* Set the state of the LED to on if the ethernet link is
			   active or off is disconnected. */
			if (lpc_netif.flags & NETIF_FLAG_LINK_UP) {
				//LWIP_DEBUGF(LWIP_DBG_ON, ("Ethernet link up\n"));
				lpc_printf("Ethernet link up \r\n");
				led_set(1);
			} else {
				//LWIP_DEBUGF(LWIP_DBG_ON, ("Ethernet link down\n"));
				lpc_printf("Ethernet link down \r\n");
				led_set(0);
			}
		}

		/* Delay for link detection */
		msDelay(250);
		if(!prt_ip) {
					if(lpc_netif.ip_addr.addr) {
						lpc_printf("IP_ADDR 0x%x NETMSK 0x%x GW 0x%x \r\n", 
							(u32_t)lpc_netif.ip_addr.addr, (u32_t) lpc_netif.netmask.addr, (u32_t)lpc_netif.gw.addr);
							prt_ip = 1;
					}					
		}
	}
}

/** \brief  Application entry point
 *
 * \return       Does not return
 */
int main (void)
{
	prvSetupHardware();

	/* Add another thread for initializing physical interface. This
	   is delayed from the main LWIP initialization. */
	xTaskCreate( vSetupIFTask, ( signed char * ) "SetupIFx",
		configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),
		( xTaskHandle * ) NULL );

	/* Start the scheduler */
	vTaskStartScheduler();

	/* Should never arrive here */
	return 1;
}

/**
 * @}
 */

#ifdef  DEBUG
void check_failed(uint8_t *file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while(1);
}
#endif

/* --------------------------------- End Of File ------------------------------ */
