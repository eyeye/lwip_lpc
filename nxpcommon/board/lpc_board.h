/**********************************************************************
* $Id$		lpc_board.h			2011-11-20
*//**
* @file		lpc_board.h
* @brief	Board specific functions used with the LWIP examples
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

#ifndef __LPC_BOARD_H
#define __LPC_BOARD_H

#include "lwip/opt.h"

/** @ingroup lpc_board
 * @{
 */

#define LPC_EMAC_ADDR0 0x00 /**< Hardware MAC address field 0 */
#define LPC_EMAC_ADDR1 0x60 /**< Hardware MAC address field 1 */
#define LPC_EMAC_ADDR2 0x37 /**< Hardware MAC address field 2 */
#define LPC_EMAC_ADDR3 0x12 /**< Hardware MAC address field 3 */
#define LPC_EMAC_ADDR4 0x34 /**< Hardware MAC address field 4 */
#define LPC_EMAC_ADDR5 0x56 /**< Hardware MAC address field 5 */

void led_set(s32_t state);
void board_setup(void);
void board_get_macaddr(u8_t *macaddr);

/**
 * @}
 */

#endif /* __LPC_BOARD_H */

/* --------------------------------- End Of File ------------------------------ */
