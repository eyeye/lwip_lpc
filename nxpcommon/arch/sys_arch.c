/**********************************************************************
* $Id$		sys_arch.c			2011-11-20
*//**
* @file		sys_arch.c
* @brief	Required functions for standalone mode
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
#include "lpc_arch.h"

/** @defgroup lwip_sys LWIP SYS (arch) support functions
 * @ingroup lwip_lpc
 *
 * These functions are ony intended for standalone mode and are
 * needed by LWIP.
 * @{
 */

/** \brief  Returns the current time in mS. This is needed for the
 *          LWIP timers.
 */
u32_t sys_now(void)
{
  return (u32_t) SysTick_GetMS();
}

/**		  
 * @}
 */

/*-----------------------------------------------------------------------------------*/
