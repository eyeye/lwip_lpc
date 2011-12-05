/**********************************************************************
* $Id$		lpc_lwip_debug.h			2011-11-20
*//**
* @file		lpc_lwip_debug.h
* @brief	LPC LWIP port help functions
* @version	1.0
* @date		20 Nov. 2011
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

#ifndef __LPC_LWIP_DEBUG_H
#define __LPC_LWIP_DEBUG_H

#include "lwip/opt.h"
#include "lwip/ip_addr.h"

#ifdef __cplusplus
extern "C"
{
#endif

/** @defgroup lwip_lpc_debug	NXP LPC LWIP library port debug helper functions
 *  @ingroup lwip_applications
 *
 * These functions are not needed for LWIP to work, but are useful for
 * debugging LWIP and application functions.
 * @{
 */

#ifdef LWIP_DEBUG
/** \brief  APP_DEBUG is routed to printf
 */
 #define APP_DEBUG printf

#else
#define APP_DEBUG
#endif

 /** \brief  Print out an IP address with carriage return
 */
 #define APP_PRINT_IP(ip) APP_DEBUG("%d.%d.%d.%d\n", ip4_addr1(ip), \
	ip4_addr2(ip), ip4_addr3(ip), ip4_addr4(ip));
 
/**		  
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __LPC_LWIP_DEBUG_H */

/* --------------------------------- End Of File ------------------------------ */
